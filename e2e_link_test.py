"""End-to-end serial link test without hardware.

Device side: the firmware's own protocol.c loaded as a DLL (virtual_mc02.VirtualMcU).
Client side: pc_service.py's shipped crc8/Parser/unpack_telemetry.
Frames the client refuses to transmit (RATE/STOP/motion) are built raw here on
purpose: this test proves the *device* rejects them, while the client whitelist
is asserted separately.

Prints E2E_TESTS pass=<n> fail=<n> and exits non-zero on any failure.
"""
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import pc_service as client  # real client-side crc/framing/parsing
from virtual_mc02 import VirtualMcU, CAN_REPLY

PONG, ACK, TELEM, ERROR = 0x11, 0x12, 0x10, 0x7F
VERSION, VOLTAGE, STATE = 0, 1, 2
MOTION_CODES = [0x0A, 0x0E, 0x46, 0x4C, 0x93, 0x9A, 0x9C, 0xF3, 0xF6, 0xFD, 0xFE, 0xFF]

PASS = FAIL = 0


def check(name, condition, detail=''):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f'  ok   {name}')
    else:
        FAIL += 1
        print(f'  FAIL {name} {detail}')


def raw_frame(typ, payload=b''):
    """Frame builder for types the client whitelist forbids (device-side tests only)."""
    body = bytes([client.SOF, len(payload), typ]) + payload
    return body + bytes([client.crc8(body)])


class Decoder:
    """Persistent client-side parser over a byte stream."""

    def __init__(self):
        self.parser = client.Parser()

    def push(self, data):
        return self.parser.feed(data)


class Clock:
    def __init__(self, start=1000):
        self.now = start

    def advance(self, ms):
        self.now += ms


def query_roundtrip(mcu, clock, which):
    mcu.feed(client.build_frame(client.QUERY, bytes([which])), clock.now)
    clock.advance(2)
    mcu.step(clock.now)   # tx_complete observed
    clock.advance(10)
    mcu.step(clock.now)   # driver reply lands and matches
    clock.advance(110)    # telemetry period elapses and carries the matched state
    out = mcu.step(clock.now)
    frames, _ = Decoder().push(out)
    fields = None
    for frame in frames:
        if frame['type'] == TELEM:
            fields = client.unpack_telemetry(frame['payload'])
    return fields


def main():
    print('== protocol identity ==')
    check('crc8(123456789)==0xF4', client.crc8(b'123456789') == 0xF4, hex(client.crc8(b'123456789')))
    check('ping frame encoding', client.build_frame(client.PING).hex().upper() == 'A500018F',
          client.build_frame(client.PING).hex().upper())

    print('== device loop (firmware protocol.c) ==')
    mcu = VirtualMcU()
    clock = Clock()

    frames, _ = Decoder().push(mcu.feed(client.build_frame(client.PING), clock.now))
    pongs = [f for f in frames if f['type'] == PONG and f['payload'] == b'']
    check('PING -> PONG', len(pongs) == 1, str(frames))

    for which, fn in ((VERSION, 0x1F), (VOLTAGE, 0x24), (STATE, 0x3A)):
        fields = query_roundtrip(mcu, clock, which)
        reply = CAN_REPLY[fn]
        check(f'QUERY {which} -> real reply {reply.hex().upper()} in telemetry',
              fields is not None and bytes(fields['last_raw'][:fields['last_raw_len']]) == reply,
              str(fields))
        check(f'QUERY {which} matched once', fields is not None and fields['matched'] >= 1, str(fields))
        check(f'QUERY {which} fn recorded', fields is not None and fields['last_fn'] == fn, str(fields))

    print('== telemetry rate ==')
    mcu2 = VirtualMcU()
    clock2 = Clock(5000)
    mcu2.feed(raw_frame(client.RATE, bytes([50])), clock2.now)
    intervals = []
    last = None
    decoder = Decoder()
    for _ in range(300):
        clock2.advance(5)
        out = mcu2.step(clock2.now)
        if not out:
            continue
        frames, _ = decoder.push(out)
        for frame in frames:
            if frame['type'] == TELEM:
                if last is not None:
                    intervals.append(clock2.now - last)
                last = clock2.now
    check('rate 50Hz -> ~20ms period', bool(intervals) and all(15 <= i <= 25 for i in intervals), str(intervals[:5]))

    print('== stop and recovery ==')
    frames, _ = Decoder().push(mcu2.feed(raw_frame(client.STOP), clock2.now))
    check('STOP -> ACK', bool(frames) and frames[0]['type'] == ACK, str(frames))
    quiet = True
    for _ in range(50):
        clock2.advance(20)
        out = mcu2.step(clock2.now)
        if out:
            frames, _ = Decoder().push(out)
            if any(f['type'] == TELEM for f in frames):
                quiet = False
    check('no telemetry after STOP', quiet)
    frames, _ = Decoder().push(mcu2.feed(client.build_frame(client.PING), clock2.now))
    check('PING still works after STOP', bool(frames) and frames[0]['type'] == PONG)

    print('== motion rejection (device side) ==')
    mcu3 = VirtualMcU()
    clock3 = Clock(9000)
    ok = True
    for code in MOTION_CODES:
        frames, _ = Decoder().push(mcu3.feed(raw_frame(code, b'\x01'), clock3.now))
        errors = [f for f in frames if f['type'] == ERROR and f['payload'][0] == code and f['payload'][1] == 5]
        if not errors:
            ok = False
            print(f'    code {code:#04x} -> {frames}')
        clock3.advance(1)
    check('all 12 motion codes -> error 5', ok)
    check('no motion frame reached the virtual CAN driver',
          mcu3.refused == 0 and mcu3.can_requests == 0,
          f'refused={mcu3.refused} requests={mcu3.can_requests}')

    print('== resynchronisation ==')
    mcu4 = VirtualMcU()
    clock4 = Clock(12000)
    bad = bytearray(client.build_frame(client.PING))
    bad[-1] ^= 0xFF
    frames, _ = Decoder().push(mcu4.feed(bytes(bad), clock4.now))
    crc_errors = [f for f in frames if f['type'] == ERROR and f['payload'][1] == 1]
    check('bad CRC -> error 1', len(crc_errors) == 1, str(frames))
    frames, _ = Decoder().push(mcu4.feed(client.build_frame(client.PING), clock4.now))
    check('link recovers after bad CRC', bool(frames) and frames[0]['type'] == PONG)
    frames, _ = Decoder().push(mcu4.feed(client.build_frame(client.PING)[:2], clock4.now))
    check('half frame produces no reply', frames == [])
    clock4.advance(50)
    frames, _ = Decoder().push(mcu4.feed(client.build_frame(client.PING)[2:], clock4.now))
    check('split frame reassembles', bool(frames) and frames[0]['type'] == PONG)

    print('== client-side transmit whitelist ==')
    check('client only transmits PING/QUERY', client.TX_WHITELIST == {client.PING, client.QUERY},
          str(client.TX_WHITELIST))
    try:
        client.build_frame(0x46, b'\x01')
        check('client refuses to build a motion frame', False, 'build_frame accepted 0x46')
    except ValueError:
        check('client refuses to build a motion frame', True)

    print('== TCP stand-in (bridge transport) ==')
    proc = subprocess.Popen([sys.executable, str(HERE / 'virtual_mc02.py'), '--serve', '127.0.0.1:45111'],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        sock = None
        for _ in range(80):
            try:
                sock = socket.create_connection(('127.0.0.1', 45111), timeout=1)
                break
            except OSError:
                time.sleep(0.1)
        check('tcp stand-in accepts connection', sock is not None)
        if sock:
            sock.settimeout(3)
            decoder = Decoder()
            sock.sendall(client.build_frame(client.PING))
            frames, _ = decoder.push(sock.recv(256))
            check('tcp PING -> PONG', any(f['type'] == PONG for f in frames), str(frames))

            sock.sendall(client.build_frame(client.QUERY, bytes([VOLTAGE])))
            deadline = time.time() + 3
            fields = None
            while time.time() < deadline and fields is None:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                frames, _ = decoder.push(chunk)
                for frame in frames:
                    if frame['type'] == TELEM:
                        fields = client.unpack_telemetry(frame['payload'])
            check('tcp QUERY voltage -> real reply bytes in telemetry',
                  fields is not None and bytes(fields['last_raw'][:fields['last_raw_len']]) == CAN_REPLY[0x24],
                  str(fields))

            sock.sendall(raw_frame(0x46, b'\x01'))
            frames, _ = decoder.push(sock.recv(256))
            check('tcp motion code -> error 5',
                  any(f['type'] == ERROR and f['payload'][1] == 5 for f in frames), str(frames))
            sock.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print(f'E2E_TESTS pass={PASS} fail={FAIL}')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
