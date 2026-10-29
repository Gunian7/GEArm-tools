# -*- coding: utf-8 -*-
"""Localhost read-only bridge between a browser and a GEArm device link.

Reuses pc_service for CRC, frame construction, parsing and telemetry unpack.
Listens on 127.0.0.1 only. Never opens a serial port.
"""
import argparse
import base64
import hashlib
import json
import logging
import math
import queue
import socket
import sys
import threading
import time
from pathlib import Path

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

import pc_service as pc

log = logging.getLogger("serial_bridge")

WEB_ROOT = Path(__file__).resolve().parent / "web"
MODEL_ROOT = Path(__file__).resolve().parents[1] / "arm-model"
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
ALLOWED_CMDS = ("ping", "query", "rate", "stop")
CMD_TYPE = {
    "ping": pc.PING,
    "query": pc.QUERY,
    "rate": pc.RATE,
    "stop": pc.STOP,
}
# Leading byte for text blobs produced by the `arm` command. Must NOT equal
# pc.SOF (0xA5) so the binary frame validator can tell the two apart, and must
# not be printable ASCII so it can never collide with a real command line.
# It is stripped in UsbCdcDeviceLink.send() before the bytes hit the wire.
ARM_TEXT_MARKER = b"\x00"
TERMINAL = {"acked", "device_error", "timeout", "transport_lost", "cancelled"}


def wire_payload(blob: bytes) -> bytes:
    """Strip the bridge-internal text marker so only real wire bytes remain.

    The `arm` command's blob carries a leading 0x00 so _assert_safe_frame() can
    tell text apart from binary frames. That byte must never reach the device,
    so every link funnels its writes through here instead of each implementing
    the strip (which is how the ScriptedDevice test link ended up sending the
    marker on the wire).
    """
    payload = bytes(blob)
    if payload[:1] == ARM_TEXT_MARKER:
        return payload[1:]
    return payload
SAFE_TYPES = (pc.PING, pc.QUERY, pc.RATE, pc.STOP)
_ENCODE_LOCK = threading.Lock()
_STATIC = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
    "/control.html": ("control.html", "text/html; charset=utf-8"),
    "/control.js": ("control.js", "text/javascript; charset=utf-8"),
    "/viewer.html": ("viewer.html", "text/html; charset=utf-8"),
    "/viewer.js": ("viewer.js", "text/javascript; charset=utf-8"),
    "/joystick.html": ("joystick.html", "text/html; charset=utf-8"),
    "/joystick.js": ("joystick.js", "text/javascript; charset=utf-8"),
    "/vendor/three.module.js": ("vendor/three.module.js", "text/javascript; charset=utf-8"),
    "/vendor/addons/controls/OrbitControls.js": ("vendor/addons/controls/OrbitControls.js", "text/javascript; charset=utf-8"),
    "/vendor/addons/loaders/STLLoader.js": ("vendor/addons/loaders/STLLoader.js", "text/javascript; charset=utf-8"),
}

# Host-side kinematics. The URDF chain is the single source of truth for FK/IK
# and is what the web viewer uses, so bridge and page can never disagree. The
# DH-table exe (built from the firmware's robot.c) is loaded lazily and only
# serves as an independent cross-check.
_URDF_KIN = None
_DH_KIN = None
_KIN_LOCK = threading.Lock()

# Episode recording. Root is a local directory; nothing here touches the device.
_RECORDER = None
_RECORDER_LOCK = threading.Lock()
RECORD_ROOT = Path(__file__).resolve().parents[1] / "episodes"
RecordingError = Exception  # replaced by the real class on first recorder use


def _recorder():
    global _RECORDER
    with _RECORDER_LOCK:
        if _RECORDER is None:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            import episode_recorder as _ep
            globals()["RecordingError"] = _ep.RecordingError
            _RECORDER = _ep.EpisodeRecorder(_urdf_kin(), RECORD_ROOT)
        return _RECORDER


def _urdf_kin():
    global _URDF_KIN
    with _KIN_LOCK:
        if _URDF_KIN is None:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            from urdf_kinematics import UrdfKinematics
            _URDF_KIN = UrdfKinematics(Path(__file__).resolve().parents[1] / "arm-model" / "arm.urdf")
        return _URDF_KIN


def _dh_kin():
    """Cross-check only. Returns None when the DH exe was never built."""
    global _DH_KIN
    with _KIN_LOCK:
        if _DH_KIN is None:
            try:
                sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
                from kinematics import Kinematics
                _DH_KIN = Kinematics()
            except Exception:
                _DH_KIN = False
        return _DH_KIN or None


def _kinematics(body):
    """FK/IK only. No device access, no motion, no serial port."""
    if not isinstance(body, dict):
        raise CommandRejected("kinematics", "rejected: body must be an object")
    op = body.get("op")
    if op == "fk":
        joints = body.get("joints")
        if not isinstance(joints, list) or len(joints) != 6:
            raise CommandRejected("fk", "rejected: joints must be 6 numbers")
        try:
            joints = [float(v) for v in joints]
        except (TypeError, ValueError):
            raise CommandRejected("fk", "rejected: joints must be numbers")
        pos, _quat = _urdf_kin().fk(joints)
        return {"ok": True, "op": "fk", "tcp": list(pos),
                "chain": _urdf_kin().joint_positions(joints), "frame": "urdf-y-up"}
    if op == "ik":
        target = body.get("target")
        seed = body.get("seed")
        if not isinstance(target, list) or len(target) != 3:
            raise CommandRejected("ik", "rejected: target must be x y z")
        if not isinstance(seed, list) or len(seed) != 6:
            raise CommandRejected("ik", "rejected: seed must be 6 joint angles")
        try:
            target = [float(v) for v in target]
            seed = [float(v) for v in seed]
        except (TypeError, ValueError):
            raise CommandRejected("ik", "rejected: numbers required")
        solved, residual, iterations, converged = _urdf_kin().ik(target, seed_deg=seed)
        return {"ok": True, "op": "ik", "joints": list(solved),
                "residual": residual, "iterations": iterations,
                "converged": converged, "frame": "urdf-y-up"}
    if op == "crosscheck":
        # Compare the URDF chain against the firmware DH table at one pose.
        # They are different models, so a large gap is expected and must be
        # resolved before any joint angles are trusted for motion.
        joints = body.get("joints")
        if not isinstance(joints, list) or len(joints) != 6:
            raise CommandRejected("crosscheck", "rejected: joints must be 6 numbers")
        try:
            joints = [float(v) for v in joints]
        except (TypeError, ValueError):
            raise CommandRejected("crosscheck", "rejected: numbers required")
        urdf_pos, _ = _urdf_kin().fk(joints)
        dh = _dh_kin()
        if dh is None:
            return {"ok": True, "op": "crosscheck", "available": False,
                    "detail": "DH exe not built; URDF is the only model"}
        dh_pos = dh.fk(joints)["tcp"]
        gap = math.sqrt(sum((urdf_pos[i] - dh_pos[i]) ** 2 for i in range(3)))
        return {"ok": True, "op": "crosscheck", "available": True,
                "urdf_tcp": list(urdf_pos), "dh_tcp": list(dh_pos), "gap_mm": gap,
                "note": "URDF is authoritative; a large gap means the firmware DH table is stale"}
    raise CommandRejected(str(op), "rejected: unknown kinematics op")


class ConfigError(Exception):
    def __init__(self, message, code=2):
        super().__init__(message)
        self.message = message
        self.code = code


class CommandRejected(Exception):
    def __init__(self, cmd, detail):
        super().__init__(detail)
        self.cmd = cmd
        self.detail = detail


class DeviceLink:
    def send(self, data: bytes) -> None:
        raise NotImplementedError

    def recv(self) -> bytes:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class LoopbackDeviceLink(DeviceLink):
    def __init__(self):
        self.sent = []
        self.closed = False
        self.fail_send = False
        self.fail_recv = False
        self._rx = queue.Queue()

    def send(self, data: bytes) -> None:
        if self.closed or self.fail_send:
            raise ConnectionError("loopback send failed")
        self.sent.append(wire_payload(data))

    def recv(self) -> bytes:
        if self.closed or self.fail_recv:
            raise ConnectionError("loopback recv failed")
        try:
            data = self._rx.get(timeout=0.05)
        except queue.Empty:
            if self.closed or self.fail_recv:
                raise ConnectionError("loopback recv failed")
            return b""
        if self.closed or self.fail_recv:
            raise ConnectionError("loopback recv failed")
        return bytes(data)

    def push(self, data: bytes) -> None:
        self._rx.put(bytes(data))

    def close(self) -> None:
        self.closed = True
        self._rx.put(b"")


class TcpDeviceLink(DeviceLink):
    def __init__(self, host, port, timeout=0.2):
        if host == "localhost":
            host = "127.0.0.1"
        if host != "127.0.0.1":
            raise ValueError("refusing non-loopback tcp device")
        self.sock = socket.create_connection((host, int(port)), timeout=1.0)
        self.sock.settimeout(timeout)

    def send(self, data: bytes) -> None:
        self.sock.sendall(wire_payload(data))

    def recv(self) -> bytes:
        try:
            data = self.sock.recv(4096)
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionError("tcp device closed")
        return data

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class UsbCdcDeviceLink(DeviceLink):
    """Real device link over the board's USB CDC virtual COM port.

    The board enumerates as VID 0483 / PID 5740 (usbser), which Windows exposes
    as a COM port. That port is the primary link to the arm; the CH340/USART1
    wiring is only a fallback and is not used here. Baud is meaningless for
    USB CDC - the firmware ignores it.

    Port resolution: explicit ``port`` argument, else the registry lookup for
    the board's VID/PID (see pc.find_usb_port). Raises ConfigError when no
    such device can be found, so the bridge reports a clear reconnect reason.
    """

    def __init__(self, port=None, vid=None, pid=None):
        name = port
        if not name:
            lookup = {}
            if vid is not None:
                lookup["vid"] = vid
            if pid is not None:
                lookup["pid"] = pid
            name = pc.find_usb_port(**lookup)
        if not name:
            raise ConfigError(
                "no USB CDC device found (VID %04X/PID %04X); plug in the board's "
                "USB cable, or pass --port COMx" % (pc.USB_CDC_VID, pc.USB_CDC_PID)
            )
        self.name = name
        try:
            self._port = pc.Win32Port(name)
        except OSError as exc:
            raise ConfigError("cannot open %s: %s" % (name, exc))

    def send(self, data: bytes) -> None:
        try:
            self._port.write(wire_payload(data))
        except OSError as exc:
            raise ConnectionError("usb write failed on %s: %s" % (self.name, exc))

    def recv(self) -> bytes:
        try:
            return self._port.read(4096)
        except OSError as exc:
            raise ConnectionError("usb read failed on %s: %s" % (self.name, exc))

    def close(self) -> None:
        try:
            self._port.close()
        except OSError:
            pass


def _integral(arg):
    if isinstance(arg, bool) or arg is None:
        return None
    if isinstance(arg, int):
        return arg
    if isinstance(arg, float) and arg.is_integer():
        return int(arg)
    if isinstance(arg, str):
        text = arg.strip()
        if text.lstrip("-").isdigit():
            return int(text)
    return None


def _restore_whitelist(saved):
    pc.TX_WHITELIST.clear()
    pc.TX_WHITELIST.update(saved)


def _frame_for(typ, payload):
    saved = set(pc.TX_WHITELIST)
    if typ in (pc.RATE, pc.STOP):
        pc.TX_WHITELIST.add(typ)
    try:
        try:
            return pc.build_frame(typ, payload)
        except ValueError as exc:
            raise CommandRejected("0x%02X" % typ, "rejected: %s" % exc) from exc
    finally:
        _restore_whitelist(saved)


def _assert_safe_frame(blob):
    """Reject anything that is not a well-formed binary frame.

    Text blobs (the `arm` command) are exempt and bail out on the first line:
    they are already validated by pc.build_arm_command() against a whitelist
    with a length cap and no control characters, and they are not binary frames
    at all, so the SOF/length/CRC checks below make no sense for them.
    """
    if blob[:1] == ARM_TEXT_MARKER:
        return
    if len(blob) < 4 or blob[0] != pc.SOF or blob[2] not in SAFE_TYPES:
        raise CommandRejected("frame", "unsafe frame blocked before send")
    if blob[1] != len(blob) - 4:
        raise CommandRejected("frame", "frame length does not match payload")
    if blob[-1] != pc.crc8(blob[:-1]):
        raise CommandRejected("frame", "frame crc mismatch")


def build_command(cmd, arg):
    """Turn a public command into (name, norm, blob) where blob is what goes on
    the wire.

    Two encodings live side by side:
      * ping/query/rate/stop -> binary frames for the serial-test firmware
      * arm                 -> whitelisted text line for GEArm-H723-Firmware
    The `arm` command carries the text in `arg`, so `norm` stays None and the
    blob is already terminated -- nothing downstream needs to know the
    difference beyond sending the bytes.
    """
    if not isinstance(cmd, str):
        shown = "" if cmd is None else str(cmd)
        raise CommandRejected(shown, "command must be a string")
    name = cmd.strip().lower()
    if name == "arm":
        try:
            blob = pc.build_arm_command(arg if isinstance(arg, str) else "")
        except ValueError as exc:
            raise CommandRejected("arm", "rejected: %s" % exc) from exc
        return name, None, ARM_TEXT_MARKER + blob
    if name not in CMD_TYPE:
        raise CommandRejected(cmd.strip(), "rejected: not in whitelist ping, query, rate, stop, arm")
    with _ENCODE_LOCK:
        if name == "ping":
            if arg is not None:
                raise CommandRejected(name, "rejected: ping takes no argument")
            blob = _frame_for(pc.PING, b"")
            norm = None
        elif name == "stop":
            if arg is not None:
                raise CommandRejected(name, "rejected: stop takes no argument")
            blob = _frame_for(pc.STOP, b"")
            norm = None
        elif name == "query":
            value = _integral(arg)
            if value not in (0, 1, 2):
                raise CommandRejected(name, "rejected: query arg must be 0, 1 or 2")
            blob = _frame_for(pc.QUERY, bytes([value]))
            norm = value
        else:
            value = _integral(arg)
            if value is None or not 1 <= value <= 100:
                raise CommandRejected(name, "rejected: rate arg must be 1..100")
            blob = _frame_for(pc.RATE, bytes([value]))
            norm = value
    try:
        _assert_safe_frame(blob)
    except CommandRejected:
        raise
    return name, norm, blob


def _telemetry_snapshot(fields):
    length = int(fields["last_raw_len"])
    raw = bytes(fields["last_raw"][:length])
    return {
        "heartbeat": int(fields["heartbeat"]),
        "init_stage": int(fields["init_stage"]),
        "tx_enqueued": int(fields["tx_enqueued"]),
        "tx_complete": int(fields["tx_complete"]),
        "rx_count": int(fields["rx_count"]),
        "matched": int(fields["matched"]),
        "timeouts": int(fields["timeouts"]),
        "error": int(fields["error"]),
        "last_fn": int(fields["last_fn"]),
        "last_raw_len": length,
        "last_raw": raw.hex(),
    }


def ws_accept_value(key: str) -> str:
    digest = hashlib.sha1((key.strip() + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def ws_encode(payload: bytes, opcode: int) -> bytes:
    data = bytes(payload)
    size = len(data)
    header = bytearray([0x80 | (opcode & 0x0F)])
    if size < 126:
        header.append(size)
    elif size < 65536:
        header.append(126)
        header += size.to_bytes(2, "big")
    else:
        header.append(127)
        header += size.to_bytes(8, "big")
    return bytes(header) + data


def ws_decode(buf: bytes):
    frames = []
    view = bytes(buf)
    while True:
        if len(view) < 2:
            break
        length = view[1] & 0x7F
        masked = (view[1] & 0x80) != 0
        offset = 2
        if length == 126:
            if len(view) < 4:
                break
            length = int.from_bytes(view[2:4], "big")
            offset = 4
        elif length == 127:
            if len(view) < 10:
                break
            length = int.from_bytes(view[2:10], "big")
            offset = 10
        if length > 1_000_000:
            raise ValueError("websocket frame too large")
        mask_len = 4 if masked else 0
        total = offset + mask_len + length
        if len(view) < total:
            break
        if masked:
            mask = view[offset:offset + 4]
            offset += 4
            data = bytes(view[offset + i] ^ mask[i % 4] for i in range(length))
        else:
            data = view[offset:offset + length]
        frames.append((view[0] & 0x0F, data))
        view = view[total:]
    return frames, view


class _WsClient:
    def __init__(self, sock):
        self.sock = sock
        self.out = queue.Queue()

    def enqueue(self, frame: bytes) -> None:
        self.out.put(frame)


class Bridge:
    def __init__(self, link_factory, retry_interval=0.5, command_timeout=1.2):
        self.link_factory = link_factory
        self.retry_interval = float(retry_interval)
        self.command_timeout = float(command_timeout)
        self.lock = threading.Lock()
        self._ws_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="bridge-device", daemon=True)
        self.link = None
        self.connection = "disconnected"
        self.retry_count = 0
        self.last_error = None
        self.telemetry = None
        self.commands = []
        self.queue = []
        self.inflight = None
        self.rejections = []
        self._seq = 0
        self.parser = pc.Parser()
        self.ws_clients = []
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        link = self.link
        self.link = None
        if link is not None:
            try:
                link.close()
            except Exception:
                pass
        with self.lock:
            for item in self.commands:
                if item["state"] not in TERMINAL:
                    item["state"] = "transport_lost" if item["state"] == "sent" else "cancelled"
                    item["error"] = item["error"] or "bridge stopped"
                    item["event"].set()
            self.queue.clear()
            self.inflight = None
        with self._ws_lock:
            clients = list(self.ws_clients)
        for client in clients:
            try:
                client.sock.close()
            except OSError:
                pass
        if self._started:
            self._thread.join(timeout=3)

    def status(self) -> dict:
        with self.lock:
            return {
                "connection": self.connection,
                "retry_count": self.retry_count,
                "last_error": self.last_error,
                "telemetry": None if self.telemetry is None else dict(self.telemetry),
                "commands": [self._public(item) for item in self.commands[-50:]],
                "rejections": [dict(item) for item in self.rejections[-50:]],
            }

    def note_rejection(self, cmd, arg, detail) -> None:
        shown = cmd if isinstance(cmd, str) else ("" if cmd is None else str(cmd))
        if not isinstance(arg, (str, int, float, bool, type(None))):
            stored_arg = str(arg)
        else:
            stored_arg = arg
        entry = {"cmd": shown, "arg": stored_arg, "detail": detail}
        with self.lock:
            self.rejections.append(entry)
            if len(self.rejections) > 200:
                del self.rejections[:-200]
        log.warning("rejected command: %s (%s)", shown, detail)

    def submit(self, cmd, arg, wait=True) -> dict:
        try:
            name, norm, blob = build_command(cmd, arg)
        except CommandRejected as exc:
            self.note_rejection(exc.cmd, arg, exc.detail)
            raise
        item = {
            "seq": None,
            "cmd": name,
            "arg": norm,
            "state": "queued",
            "frame": blob,
            "response": None,
            "error": None,
            "event": threading.Event(),
            "sent_at": None,
        }
        with self.lock:
            self._seq += 1
            item["seq"] = self._seq
            self.commands.append(item)
            if len(self.commands) > 200:
                self.commands = self.commands[-200:]
            self.queue.append(item)
            public = self._public(item)
        if not wait:
            return public
        deadline = time.monotonic() + self.command_timeout + 0.5
        while time.monotonic() < deadline:
            if item["state"] in TERMINAL:
                break
            item["event"].wait(0.05)
        with self.lock:
            return self._public(item)

    def attach_ws(self, sock, handshake: bytes) -> _WsClient:
        client = _WsClient(sock)
        with self._ws_lock:
            self.ws_clients.append(client)
            sock.sendall(handshake)
        hello = self.status()
        self._broadcast({
            "kind": "status",
            "connection": hello["connection"],
            "retry_count": hello["retry_count"],
            "last_error": hello["last_error"],
        })
        return client

    def detach_ws(self, client) -> None:
        with self._ws_lock:
            if client in self.ws_clients:
                self.ws_clients.remove(client)

    def run_ws(self, client) -> None:
        sock = client.sock
        try:
            sock.settimeout(0.2)
        except OSError:
            return
        buf = b""
        while not self._stop.is_set():
            if not self._flush_ws(client):
                break
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            if len(buf) > 1_000_000:
                break
            try:
                frames, buf = ws_decode(buf)
            except ValueError:
                break
            if not self._handle_ws_frames(client, frames):
                break
        self._flush_ws(client)

    def _handle_ws_frames(self, client, frames) -> bool:
        for opcode, payload in frames:
            if opcode == 0x8:
                try:
                    client.sock.sendall(ws_encode(payload[:125], 0x8))
                except OSError:
                    pass
                return False
            if opcode == 0x9:
                client.enqueue(ws_encode(payload, 0xA))
        return True

    def _flush_ws(self, client) -> bool:
        while True:
            try:
                frame = client.out.get_nowait()
            except queue.Empty:
                return True
            try:
                client.sock.sendall(frame)
            except OSError:
                return False

    def _broadcast(self, obj) -> None:
        try:
            frame = ws_encode(json.dumps(obj, separators=(",", ":")).encode("utf-8"), 0x1)
        except (TypeError, ValueError):
            log.warning("rejected websocket payload")
            return
        with self._ws_lock:
            clients = list(self.ws_clients)
        for client in clients:
            client.enqueue(frame)

    def _public(self, item) -> dict:
        return {
            "seq": item["seq"],
            "cmd": item["cmd"],
            "arg": item["arg"],
            "state": item["state"],
            "response": item["response"],
            "error": item["error"],
        }

    def _set_connection(self, state) -> None:
        with self.lock:
            self.connection = state
            retry_count = self.retry_count
            last_error = self.last_error
        self._broadcast({
            "kind": "status",
            "connection": state,
            "retry_count": retry_count,
            "last_error": last_error,
        })

    def _loop(self) -> None:
        while not self._stop.is_set():
            if self.link is None:
                self._set_connection("connecting" if self.retry_count == 0 else "reconnecting")
                try:
                    link = self.link_factory()
                except Exception as exc:
                    self._fail_connect(exc)
                    continue
                if link is None:
                    self._fail_connect(ConnectionError("device link factory returned nothing"))
                    continue
                self.link = link
                self.parser = pc.Parser()
                with self.lock:
                    self.last_error = None
                self._set_connection("connected")
            try:
                if not self._pump_send():
                    continue
                data = self.link.recv()
            except Exception as exc:
                if self._stop.is_set():
                    break
                self._on_disconnect(str(exc))
                continue
            if data:
                self._on_bytes(data)
            self._check_timeout()
        self._set_connection("stopped")

    def _fail_connect(self, exc) -> None:
        with self.lock:
            self.retry_count += 1
            self.last_error = str(exc)
        self._set_connection("reconnecting")
        self._stop.wait(self.retry_interval)

    def _pump_send(self) -> bool:
        with self.lock:
            if self.inflight is not None or not self.queue or self.link is None:
                return True
            item = self.queue.pop(0)
        try:
            _assert_safe_frame(item["frame"])
            self.link.send(item["frame"])
        except Exception as exc:
            with self.lock:
                item["state"] = "queued"
                self.queue.insert(0, item)
            self._on_disconnect(str(exc))
            return False
        with self.lock:
            item["state"] = "sent"
            item["sent_at"] = time.monotonic()
            self.inflight = item
        return True

    def _on_disconnect(self, message) -> None:
        link = self.link
        self.link = None
        if link is not None:
            try:
                link.close()
            except Exception:
                pass
        with self.lock:
            item = self.inflight
            self.inflight = None
            if item is not None and item["state"] == "sent":
                item["state"] = "transport_lost"
                item["error"] = message or "device disconnected before response"
                item["event"].set()
            self.retry_count += 1
            self.last_error = message or "disconnected"
        self._set_connection("reconnecting")
        log.warning("device link lost: %s", message)
        self._stop.wait(self.retry_interval)

    def _check_timeout(self) -> None:
        with self.lock:
            item = self.inflight
            if item is None or item["state"] != "sent" or not item["sent_at"]:
                return
            if time.monotonic() - item["sent_at"] < self.command_timeout:
                return
            self.inflight = None
            item["state"] = "timeout"
            item["error"] = "no device response before timeout"
            item["event"].set()
            public = self._public(item)
        self._broadcast({"kind": "command", "command": public})

    def _on_bytes(self, data: bytes) -> None:
        frames, errors = self.parser.feed(data)
        for code in errors:
            log.warning("rejected device frame code=%s", code)
        for frame in frames:
            self._on_frame(frame)

    def _on_frame(self, frame) -> None:
        typ = frame["type"]
        payload = frame["payload"]
        if typ == pc.TELEM:
            try:
                snap = _telemetry_snapshot(pc.unpack_telemetry(payload))
            except ValueError as exc:
                log.warning("rejected telemetry: %s", exc)
                return
            with self.lock:
                self.telemetry = snap
                connection = self.connection
            self._broadcast({"kind": "telemetry", "connection": connection, "telemetry": snap})
            return
        if typ == pc.PONG:
            self._complete(lambda item: item["cmd"] == "ping", {"kind": "pong"}, "acked", None)
            return
        if typ == pc.ACK and len(payload) >= 2:
            ack_type = payload[0]
            arg = payload[1]
            self._complete(
                lambda item, ack_type=ack_type: CMD_TYPE.get(item["cmd"]) == ack_type,
                {"kind": "ack", "type": ack_type, "arg": arg},
                "acked",
                None,
            )
            return
        if typ == pc.ERROR and len(payload) >= 2:
            err_type = payload[0]
            code = payload[1]
            self._complete(
                lambda item, err_type=err_type: CMD_TYPE.get(item["cmd"]) == err_type,
                {"kind": "error", "type": err_type, "code": code},
                "device_error",
                "device error code %s" % code,
            )

    def _complete(self, pred, response, state, error) -> None:
        with self.lock:
            item = self.inflight
            if item is None or item["state"] != "sent" or not pred(item):
                return
            self.inflight = None
            item["state"] = state
            item["response"] = response
            item["error"] = error
            item["event"].set()
            public = self._public(item)
        self._broadcast({"kind": "command", "command": public})


from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class BridgeHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    rbufsize = 0
    wbufsize = 0

    def log_message(self, fmt, *args):
        log.debug("http " + fmt, *args)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/ws":
            self._upgrade()
            return
        if path == "/api/status":
            self._json(200, self.server.bridge.status())
            return
        if path == "/api/episodes":
            self._json(200, {"ok": True, "episodes": _recorder().list_episodes(),
                             "root": str(RECORD_ROOT)})
            return
        if path == "/api/episode/frames":
            # Read-only replay source: one episode's frames, so the viewer can
            # play a recording back without touching the device.
            try:
                index = int(self.path.split("index=", 1)[1].split("&", 1)[0])
            except (IndexError, ValueError):
                self._json(400, {"ok": False, "error": "index must be an integer"})
                return
            try:
                meta, frames = _recorder().load(index)
            except RecordingError as exc:
                self._json(404, {"ok": False, "error": str(exc)})
                return
            self._json(200, {"ok": True, "meta": meta, "frames": frames})
            return
        if path == "/api/model":
            kin = _urdf_kin()
            zero, _quat = kin.fk([0.0] * 6)
            self._json(200, {
                "ok": True,
                "frame": "urdf-y-up",
                "joints": [{"name": n, "index": i, "limit_deg": list(l) if l else None,
                            "limit_verified": l is not None}
                           for i, (n, l) in enumerate(zip(kin.names, kin.limits))],
                "zero_tcp_mm": list(zero),
                "state_dim": 9,
                "nominal_hz": 50.0,
            })
            return
        if path == "/favicon.ico":
            self._empty(204)
            return
        if path.startswith("/arm/"):
            self._serve_model(path[len("/arm/"):])
            return
        static = _STATIC.get(path)
        if static is None:
            self._json(404, {"ok": False, "error": "not_found"})
            return
        name, content_type = static
        file_path = WEB_ROOT / name
        if not file_path.is_file():
            self._json(404, {"ok": False, "error": "not_found"})
            return
        body = file_path.read_bytes()
        self._bytes(200, content_type, body)

    def _serve_model(self, relative):
        """Serve the robot model (URDF + STL meshes) from a fixed local root.

        Only exact files under MODEL_ROOT are served; no directory listing, no
        traversal, no path outside the model root.
        """
        parts = [p for p in relative.split("/") if p not in ("", ".", "..")]
        if not parts:
            self._json(404, {"ok": False, "error": "not_found"})
            return
        target = MODEL_ROOT.joinpath(*parts)
        try:
            resolved = target.resolve()
            root = MODEL_ROOT.resolve()
            if root not in resolved.parents:
                self._json(403, {"ok": False, "error": "forbidden"})
                return
        except OSError:
            self._json(404, {"ok": False, "error": "not_found"})
            return
        if not resolved.is_file():
            self._json(404, {"ok": False, "error": "not_found"})
            return
        suffix = resolved.suffix.lower()
        content_type = {".urdf": "application/xml; charset=utf-8",
                        ".stl": "model/stl"}.get(suffix, "application/octet-stream")
        self._bytes(200, content_type, resolved.read_bytes())

    def _read_json_body(self, limit=65536):
        """Read and decode a JSON request body. Returns (body, error_response).

        Kept separate because each POST branch reads the body itself; sharing one
        local across branches is how a NameError ends up dropping the socket.
        """
        try:
            length = int(self.headers.get("Content-Length", "0") or "0")
        except ValueError:
            return None, (400, {"ok": False, "error": "bad content length"})
        if length < 0 or length > limit:
            return None, (400, {"ok": False, "error": "bad content length"})
        raw = self.rfile.read(length) if length else b""
        try:
            return json.loads(raw.decode("utf-8")), None
        except (ValueError, UnicodeDecodeError):
            return None, (400, {"ok": False, "error": "body must be JSON"})

    def _episode_api(self, action):
        """Recording endpoints. Local only: no device access, no motion command."""
        body, error = self._read_json_body()
        if error is not None:
            self._json(error[0], error[1])
            return
        rec = _recorder()
        try:
            if action == "start":
                task = body.get("task") if isinstance(body, dict) else None
                index = rec.start(task)
                self._json(200, {"ok": True, "episode_index": index, "task": task,
                                 "state": "recording"})
                return
            if action == "stop":
                meta = rec.stop()
                self._json(200, {"ok": True, "meta": meta, "state": "idle"})
                return
            if action == "frame":
                joints = body.get("joints") if isinstance(body, dict) else None
                if not isinstance(joints, list) or len(joints) != 6:
                    self._json(400, {"ok": False, "error": "joints must be 6 numbers"})
                    return
                try:
                    joints = [float(v) for v in joints]
                except (TypeError, ValueError):
                    self._json(400, {"ok": False, "error": "joints must be numbers"})
                    return
                frames = rec.record(joints)
                self._json(200, {"ok": True, "frame_index": frames[0]["frame_index"]})
                return
            if action == "frames":
                # Batch path used by the 50 Hz recorder: one round trip carries
                # many samples, each with the client-measured gap to the previous.
                samples = body.get("samples") if isinstance(body, dict) else None
                if not isinstance(samples, list) or not samples or len(samples) > 500:
                    self._json(400, {"ok": False, "error": "samples must be 1..500 entries"})
                    return
                prepared = []
                for sample in samples:
                    if not isinstance(sample, dict):
                        self._json(400, {"ok": False, "error": "each sample must be an object"})
                        return
                    joints = sample.get("joints")
                    if not isinstance(joints, list) or len(joints) != 6:
                        self._json(400, {"ok": False, "error": "joints must be 6 numbers"})
                        return
                    try:
                        prepared.append(([float(v) for v in joints], sample.get("dt")))
                    except (TypeError, ValueError):
                        self._json(400, {"ok": False, "error": "joints must be numbers"})
                        return
                try:
                    written = rec.record_batch(prepared)
                except RecordingError as exc:
                    self._json(409, {"ok": False, "error": str(exc)})
                    return
                self._json(200, {"ok": True, "written": len(written),
                                 "last_frame_index": written[-1]["frame_index"]})
                return
            if action == "verify":
                index = body.get("index") if isinstance(body, dict) else None
                if not isinstance(index, int):
                    self._json(400, {"ok": False, "error": "index must be an integer"})
                    return
                self._json(200, {"ok": True, "report": rec.verify(index)})
                return
            self._json(404, {"ok": False, "error": "unknown episode action"})
        except RecordingError as exc:
            self._json(409, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self._json(500, {"ok": False, "error": type(exc).__name__})

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/kinematics":
            try:
                length = int(self.headers.get("Content-Length", "0") or "0")
            except ValueError:
                length = -1
            if length < 0 or length > 4096:
                self._json(400, {"ok": False, "error": "rejected", "detail": "bad content length"})
                return
            raw = self.rfile.read(length) if length else b""
            try:
                body = json.loads(raw.decode("utf-8"))
                self._json(200, _kinematics(body))
            except CommandRejected as exc:
                self._json(400, {"ok": False, "error": "rejected", "cmd": exc.cmd, "detail": exc.detail})
            except Exception as exc:
                self._json(500, {"ok": False, "error": type(exc).__name__})
            return
        if path != "/api/command":
            if path.startswith("/api/episode/"):
                self._episode_api(path[len("/api/episode/"):])
                return
            self._json(404, {"ok": False, "error": "not_found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0") or "0")
        except ValueError:
            length = -1
        if length < 0 or length > 4096:
            self.server.bridge.note_rejection("", None, "rejected: bad content length")
            self._json(400, {"ok": False, "error": "rejected", "cmd": None})
            return
        raw = self.rfile.read(length) if length else b""
        try:
            body = json.loads(raw.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError):
            self.server.bridge.note_rejection("", None, "rejected: bad json")
            self._json(400, {"ok": False, "error": "rejected", "cmd": None})
            return
        if not isinstance(body, dict):
            self.server.bridge.note_rejection("", None, "rejected: body must be an object")
            self._json(400, {"ok": False, "error": "rejected", "cmd": None})
            return
        extra = set(body) - {"cmd", "arg", "wait"}
        cmd = body.get("cmd")
        arg = body.get("arg", None)
        if extra or ("wait" in body and not isinstance(body["wait"], bool)):
            self.server.bridge.note_rejection(cmd, arg, "rejected: unknown field or bad wait flag")
            shown = cmd if isinstance(cmd, str) else None
            self._json(400, {"ok": False, "error": "rejected", "cmd": shown})
            return
        try:
            public = self.server.bridge.submit(cmd, arg, wait=body.get("wait", True))
        except CommandRejected as exc:
            shown = exc.cmd if isinstance(exc.cmd, str) else None
            self._json(400, {"ok": False, "error": "rejected", "cmd": shown})
            return
        self._json(200, {"ok": True, **public})

    def _upgrade(self):
        self.close_connection = True
        upgrade = (self.headers.get("Upgrade") or "").lower()
        key = self.headers.get("Sec-WebSocket-Key")
        if upgrade != "websocket" or not key:
            self._json(400, {"ok": False, "error": "rejected"})
            return
        accept = ws_accept_value(key)
        handshake = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: %s\r\n"
            "\r\n" % accept
        ).encode("ascii")
        client = self.server.bridge.attach_ws(self.connection, handshake)
        try:
            self.server.bridge.run_ws(client)
        finally:
            self.server.bridge.detach_ws(client)

    def _json(self, code, obj):
        self._bytes(code, "application/json; charset=utf-8", json.dumps(obj).encode("utf-8"))

    def _bytes(self, code, content_type, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)
        self.close_connection = True

    def _empty(self, code):
        self.send_response(code)
        self.send_header("Content-Length", "0")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True


class BridgeHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
    bridge = None


def serve(bridge, host="127.0.0.1", port=0):
    if host != "127.0.0.1":
        raise ValueError("HTTP listen host must be 127.0.0.1")
    httpd = BridgeHTTPServer((host, int(port)), BridgeHandler)
    httpd.bridge = bridge
    thread = threading.Thread(target=httpd.serve_forever, name="bridge-http", daemon=True)
    thread.start()
    httpd.serve_thread = thread
    return httpd


def parse_tcp(spec: str):
    if spec.count(":") != 1:
        raise ConfigError("tcp endpoint must be host:port")
    host, port_text = spec.split(":")
    if host == "localhost":
        host = "127.0.0.1"
    if host != "127.0.0.1":
        raise ConfigError("拒绝非回环设备地址 %s" % host)
    try:
        port = int(port_text)
    except ValueError as exc:
        raise ConfigError("bad tcp port") from exc
    if not 1 <= port <= 65535:
        raise ConfigError("bad tcp port")
    return host, port


def _usb_id(text, default, name):
    """Parse a USB VID/PID given as 0x1234 or decimal. None -> default."""
    if text is None:
        return default
    try:
        value = int(str(text), 0)
    except ValueError as exc:
        raise ConfigError("bad %s %r" % (name, text)) from exc
    if not 0 <= value <= 0xFFFF:
        raise ConfigError("bad %s %r" % (name, text))
    return value


def plan_runtime(argv):
    parser = argparse.ArgumentParser(description="GEArm localhost bridge (USB CDC device link)")
    parser.add_argument("--transport", choices=("usb", "tcp"), default="usb",
                        help="usb = the board's USB CDC COM port (default); "
                             "tcp = localhost software stand-in, no hardware")
    parser.add_argument("--tcp", default="127.0.0.1:45111", help="virtual device host:port, loopback only")
    parser.add_argument("--port", default=None,
                        help="COM port for --transport usb, e.g. COM10. Omit to auto-detect the board's USB CDC port.")
    parser.add_argument("--vid", default=None, help="board USB VID, default 0x%04X" % pc.USB_CDC_VID)
    parser.add_argument("--pid", default=None, help="board USB PID, default 0x%04X" % pc.USB_CDC_PID)
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--retry-interval", type=float, default=0.5)
    parser.add_argument("--command-timeout", type=float, default=1.2)
    args = parser.parse_args(list(argv))
    if args.transport not in ("usb", "tcp"):
        raise ConfigError("bad transport")
    if not 0 <= args.http_port <= 65535:
        raise ConfigError("bad http port")
    host, tcp_port = parse_tcp(args.tcp)
    return {
        "transport": args.transport,
        "serial": args.port,
        "tcp": (host, tcp_port),
        "vid": _usb_id(args.vid, None, "vid"),
        "pid": _usb_id(args.pid, None, "pid"),
        "http_host": "127.0.0.1",
        "http_port": args.http_port,
        "retry_interval": args.retry_interval,
        "command_timeout": args.command_timeout,
    }


def make_link_factory(plan):
    """Build the device link factory for a runtime plan."""
    if plan["transport"] == "usb":
        return lambda: UsbCdcDeviceLink(plan["serial"], plan["vid"], plan["pid"])
    return lambda: TcpDeviceLink(plan["tcp"][0], plan["tcp"][1])


def _write_stderr(message: str) -> None:
    try:
        sys.stderr.write(message + "\n")
        sys.stderr.flush()
    except UnicodeEncodeError:
        encoding = getattr(sys.stderr, "encoding", None) or "utf-8"
        buffer = getattr(sys.stderr, "buffer", None)
        if buffer is None:
            return
        buffer.write((message + "\n").encode(encoding, "replace"))
        buffer.flush()


def main(argv=None) -> int:
    try:
        plan = plan_runtime(sys.argv[1:] if argv is None else argv)
    except ConfigError as exc:
        _write_stderr(exc.message)
        return exc.code
    bridge = Bridge(
        make_link_factory(plan),
        retry_interval=plan["retry_interval"],
        command_timeout=plan["command_timeout"],
    )
    bridge.start()
    httpd = serve(bridge, plan["http_host"], plan["http_port"])
    bound = httpd.server_address
    if plan["transport"] == "usb":
        target = "usb cdc %s" % (plan["serial"] or "auto-detect (VID %04X/PID %04X)"
                                 % (pc.USB_CDC_VID, pc.USB_CDC_PID))
    else:
        target = "tcp %s:%d" % (plan["tcp"][0], plan["tcp"][1])
    _write_stderr("bridge http://%s:%d device %s" % (bound[0], bound[1], target))
    try:
        while not bridge._stop.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        httpd.shutdown()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
