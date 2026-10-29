"""Host-side stand-in for the MC02 serial firmware.

The protocol state machine is the firmware's own protocol.c, compiled into
virtual_mc02.exe and driven over stdin/stdout with length-prefixed commands.
Python owns the clock and the transport; the protocol is never reimplemented.
The virtual CAN driver answers only the three read-only replies recorded from
the real bench (COMMUNICATION_TEST_RESULT.md) and refuses anything else.

Run as a TCP loopback stand-in for a serial port (used by the bridge):
    python virtual_mc02.py --serve 127.0.0.1:45111
"""
import argparse
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXE = HERE / 'build' / 'virtual_mc02.exe'

# Real reply bytes from the address-1 bench (ID 0x100, extended, classic, 0x6B last byte).
CAN_REPLY = {
    0x1F: bytes([0x1F, 0x00, 0xCA, 0x23, 0x14, 0x6B]),  # version, DLC 6
    0x24: bytes([0x24, 0x5F, 0x00, 0x6B]),              # voltage, DLC 4
    0x3A: bytes([0x3A, 0x03, 0x6B]),                    # state, DLC 3
}


class VirtualMcU:
    """One firmware instance: protocol.c driven by a subprocess."""

    def __init__(self, exe=None):
        self.exe = str(exe or EXE)
        self.proc = subprocess.Popen([self.exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL)
        self.refused = 0
        self.can_requests = 0

    def _read_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.proc.stdout.read(n - len(buf))
            if not chunk:
                raise EOFError('virtual device closed')
            buf += chunk
        return buf

    def _command(self, cmd, payload=b''):
        self.proc.stdin.write(cmd.encode() + struct.pack('<I', len(payload)) + payload)
        self.proc.stdin.flush()
        header = self._read_exact(4)
        (n,) = struct.unpack('<I', header)
        return self._read_exact(n) if n else b''

    def feed(self, data: bytes, now: int) -> bytes:
        """Device receives serial bytes; returns whatever it transmits back.

        The clock is synced first so the device never sees a stale `now`
        (the firmware reads HAL_GetTick() on every byte).
        """
        out = self._command('T', struct.pack('<I', now))
        out += self._command('F', data)
        self._sync_status()
        return out

    def step(self, now: int) -> bytes:
        """One firmware main-loop iteration at clock `now` (ms)."""
        return self._command('T', struct.pack('<I', now))

    def _sync_status(self):
        raw = self._command('S')
        if len(raw) == 8:
            self.refused, self.can_requests = struct.unpack('<II', raw)

    def close(self):
        try:
            self.proc.stdin.write(b'Q' + struct.pack('<I', 0))
            self.proc.stdin.flush()
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()


def serve(host: str, port: int, verbose: bool = False):
    """TCP loopback stand-in for a serial port. One client at a time."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f'virtual_mc02 listening on {host}:{port}', flush=True)
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=_session, args=(conn, verbose), daemon=True).start()


def _session(conn, verbose):
    mcu = VirtualMcU()
    conn.settimeout(0.05)
    start = time.monotonic()
    try:
        while True:
            now = int((time.monotonic() - start) * 1000)
            try:
                data = conn.recv(256)
                if data == b'':
                    break
                out = mcu.feed(data, now)
            except socket.timeout:
                out = b''
            out += mcu.step(now)
            if out:
                conn.sendall(out)
            if verbose:
                import json
                print(json.dumps({'tx': out.hex(), 'can_requests': mcu.can_requests,
                                  'refused': mcu.refused}), flush=True)
            time.sleep(0.001)
    except (OSError, EOFError):
        pass
    finally:
        mcu.close()
        conn.close()


def main():
    ap = argparse.ArgumentParser(description='Virtual MC02 serial stand-in (no hardware)')
    ap.add_argument('--serve', default='127.0.0.1:45111', help='loopback address:port (default 127.0.0.1:45111)')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()
    host, _, port = args.serve.partition(':')
    if host not in ('127.0.0.1', 'localhost'):
        raise SystemExit('refusing to bind a non-loopback address')
    serve(host, int(port), args.verbose)


if __name__ == '__main__':
    main()
