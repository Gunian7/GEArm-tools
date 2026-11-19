#!/usr/bin/env python3
"""Stage-0 GEArm telemetry reader over the board's USB CDC virtual COM port.

The board (DM-MC02 / STM32H723) exposes a USB CDC device (VID 0483, PID 5740)
that the host turns into a virtual serial port: "COM10" on Windows (usbser),
"/dev/ttyACM0" on Linux (cdc_acm). That port is the primary link to the arm;
the old CH340/USART1 path is only a fallback.

Sends only PING and QUERY. Motion, enable, speed, position, homing and
parameter-write frames are refused before any byte is produced.
--selftest uses an in-process virtual transport and does not open a port.

Standard library only. Windows drives ctypes + kernel32 and POSIX drives os +
termios; each of those two imports exists on exactly one platform, so both are
pulled in only on the branch that needs them and this module stays importable
everywhere.
"""
import argparse
import os
import re
import sys
import time
import ctypes

try:  # ctypes.wintypes declares VARIANT_BOOL with the "v" field type, which
    # only Windows defines, so the import itself raises ValueError elsewhere.
    from ctypes import wintypes
except (ImportError, ValueError):  # pragma: no cover - POSIX only
    wintypes = None

SOF = 0xA5
POLY = 0x07
MAX_PAYLOAD = 48
PING = 0x01
QUERY = 0x02
RATE = 0x03
STOP = 0x04
TELEM = 0x10
PONG = 0x11
ACK = 0x12
ERROR = 0x7F
TX_WHITELIST = {PING, QUERY}
MOTION = {0x0A, 0x0E, 0x46, 0x4C, 0x93, 0x9A, 0x9C, 0xF3, 0xF6, 0xFD, 0xFE, 0xFF}
CAN_FN = {0: 0x1F, 1: 0x24, 2: 0x3A}
OPEN_COUNT = 0

# ---------------------------------------------------------------------------
# GEArm 固件的文本命令通路（2026-10-28 新增）
#
# GEArm-H723-Firmware 用的是文本命令（robot_cmd.c 的 robot_uart1_cmd_table），
# 不是上面那套二进制帧协议。USB CDC 搬进 GEArm 固件后，两边要对上，就得让
# bridge 也能发文本。
#
# 设计：只放行白名单内的命令，原样加上换行符发给设备。不引入任意字符串直通，
# 否则 bridge 就从"受控的命令转发器"退化成"任意注入口"了。
#
# 白名单与固件 Core/Src/robot_cmd.c 的 robot_uart1_cmd_table 一一对应。
# joint_id 是 0 起（固件里 joint_id + 1 才是电机 CAN 地址）。
ARM_CMDS = {
    "rel_rotate", "auto", "hard_reset", "soft_reset", "zero",
    "estop", "estop_clear",
    "remote_enable", "remote_disable", "remote_event",
    "qstat",                  # 只读诊断：查询驱动 3A/35/36/1F 状态
    "jstat",                  # 只读诊断：6 轴的当前角度/软件限位范围/减速比
    "probe",                  # 重新探测 CAN 总线成员（接好新电机后用）
}
ARM_MAX_LEN = 96          # 固件 USB 接收缓冲 128 字节，留足余量
ARM_TERMINATOR = bytes((0x0D, 0x0A))  # CR+LF：固件认 0x0A 结束、忽略 0x0D


def build_arm_command(cmd: str) -> bytes:
    """Encode a whitelisted GEArm text command for the USB CDC link.

    Format is "<name>[ arg ...]" + CRLF -- exactly what the firmware
    robot_uart1_handle() parses with sscanf.
    """
    if not isinstance(cmd, str):
        raise ValueError("arm command must be a string")
    text = cmd.strip()
    if not text:
        raise ValueError("arm command is empty")
    if len(text) > ARM_MAX_LEN:
        raise ValueError("arm command too long (%d > %d)" % (len(text), ARM_MAX_LEN))
    if any(ord(ch) < 0x20 or ord(ch) == 0x7F for ch in text):
        raise ValueError("arm command must not contain control characters")
    parts = text.split()
    name = parts[0].lower()
    if name not in ARM_CMDS:
        raise ValueError("arm command %r not in whitelist %s"
                         % (parts[0], sorted(ARM_CMDS)))
    # 参数必须是数字（rel_rotate 0 10 / auto 1 2 3 ...），避免把乱七八糟的东西注入固件
    for arg in parts[1:]:
        try:
            float(arg)
        except ValueError:
            raise ValueError("arm command argument %r is not a number" % arg)
    return text.encode("ascii") + ARM_TERMINATOR

# The board's USB CDC identity (see usb/App/usbd_desc.c: USBD_VID/USBD_PID_HS).
USB_CDC_VID = 0x0483
USB_CDC_PID = 0x5740


def _reg_query(key, name):
    import winreg
    try:
        value, _kind = winreg.QueryValueEx(key, name)
    except OSError:
        return None
    return value


def _find_usb_port_windows(vid, pid):
    """Windows: registry Enum\\USB\\VID_xxxx&PID_yyyy -> "COM10", or None.

    Works whether or not the port is currently open, and never opens it.
    """
    try:
        import winreg
    except ImportError:
        return None
    base = r"SYSTEM\CurrentControlSet\Enum\USB\VID_%04X&PID_%04X" % (vid, pid)
    try:
        root = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base)
    except OSError:
        return None
    try:
        index = 0
        while True:
            try:
                instance = winreg.EnumKey(root, index)
            except OSError:
                return None
            index += 1
            try:
                device = winreg.OpenKey(root, instance)
            except OSError:
                continue
            try:
                friendly = _reg_query(device, "FriendlyName")
                if friendly:
                    match = re.search(r"\(COM(\d+)\)", str(friendly))
                    if match:
                        return "COM" + match.group(1)
                try:
                    params = winreg.OpenKey(device, "Device Parameters")
                except OSError:
                    params = None
                if params is not None:
                    try:
                        port = _reg_query(params, "PortName")
                    finally:
                        winreg.CloseKey(params)
                    if port:
                        name = str(port).strip()
                        if name.upper().startswith("COM"):
                            return name
            finally:
                winreg.CloseKey(device)
    finally:
        winreg.CloseKey(root)


# sysfs / devfs mount points, used as the defaults of "injectable" parameters
# so the POSIX discovery can be exercised against a fake tree on any platform.
SYS_ROOT = "/sys"
DEV_ROOT = "/dev"


def _read_text(path):
    try:
        with open(path, "r") as handle:
            return handle.read().strip()
    except OSError:
        return None


def _sysfs_usb_ids(node, sys_root, levels=8):
    """Return (vid, pid) read from a sysfs node, walking up towards the device.

    /sys/class/tty/ttyACM0/device points at the USB *interface* (1-1:1.0);
    idVendor/idProduct live one level higher, on the USB *device* (1-1). The
    walk is capped at "levels" and never leaves sys_root.
    """
    stop = os.path.realpath(sys_root)
    current = os.path.realpath(node)
    for _ in range(levels):
        vendor = _read_text(os.path.join(current, "idVendor"))
        product = _read_text(os.path.join(current, "idProduct"))
        if vendor and product:
            try:
                return int(vendor, 16), int(product, 16)
            except ValueError:
                return None
        parent = os.path.dirname(current)
        if current == stop or parent == current:
            return None
        current = parent
    return None


def _tty_usb_ids(tty_name, sys_root):
    return _sysfs_usb_ids(os.path.join(sys_root, "class", "tty", tty_name, "device"), sys_root)


def _dev_node(dev_root, name):
    """Build a /dev-style path with a POSIX separator, whatever os.sep is."""
    return dev_root.rstrip("/") + "/" + name


def _find_usb_port_posix(vid, pid, sys_root=SYS_ROOT, dev_root=DEV_ROOT):
    """POSIX: return "/dev/ttyACM0" for the matching USB CDC device, or None.

    Pure path work -- the port is never opened. Two passes:

      1. /dev/serial/by-id/ holds stable symlinks; resolve each to its
         /dev/ttyACM<N> and compare that tty's sysfs USB ids.
      2. otherwise scan /sys/class/tty/ttyACM*/device, walk up to the USB
         device node and return /dev/ttyACM<N> on a VID/PID match.
    """
    by_id = os.path.join(dev_root, "serial", "by-id")
    try:
        entries = sorted(os.listdir(by_id))
    except OSError:
        entries = []
    for entry in entries:
        try:
            target = os.path.realpath(os.path.join(by_id, entry))
        except OSError:
            continue
        name = os.path.basename(target)
        if not name.startswith("ttyACM"):
            continue
        if _tty_usb_ids(name, sys_root) == (vid, pid):
            return _dev_node(dev_root, name)

    tty_root = os.path.join(sys_root, "class", "tty")
    try:
        names = sorted(os.listdir(tty_root))
    except OSError:
        return None
    for name in names:
        if not name.startswith("ttyACM"):
            continue
        if _tty_usb_ids(name, sys_root) == (vid, pid):
            return _dev_node(dev_root, name)
    return None


def find_usb_port(vid=USB_CDC_VID, pid=USB_CDC_PID, sys_root=SYS_ROOT, dev_root=DEV_ROOT):
    """Return the board's USB CDC serial port name, or None.

    Windows: registry lookup under Enum\\USB\\VID_xxxx&PID_yyyy -> "COM10".
    POSIX:   /dev/serial/by-id, then /sys/class/tty/ttyACM* -> "/dev/ttyACM0".

    Neither branch opens the port, and None means the device has never been
    seen (or, on Windows, that winreg is unavailable).
    """
    if os.name == "nt":
        return _find_usb_port_windows(vid, pid)
    return _find_usb_port_posix(vid, pid, sys_root, dev_root)


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(typ: int, payload: bytes = b"") -> bytes:
    if typ not in TX_WHITELIST:
        raise ValueError("refused type 0x%02X" % typ)
    if typ == PING and payload != b"":
        raise ValueError("PING payload is empty")
    if typ == QUERY and (len(payload) != 1 or payload[0] not in CAN_FN):
        raise ValueError("QUERY parameter must be 0, 1 or 2")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([SOF, len(payload), typ]) + payload
    return body + bytes([crc8(body)])


class Parser:
    def __init__(self):
        self.state = 0
        self.buf = bytearray()
        self.length = 0

    def feed(self, data: bytes):
        frames = []
        errors = []
        for byte in data:
            item = self._byte(byte)
            if item is None:
                continue
            kind, value = item
            if kind == "err":
                errors.append(value)
            else:
                frames.append(value)
        return frames, errors

    def _byte(self, byte):
        if self.state == 0:
            if byte == SOF:
                self.buf = bytearray([SOF])
                self.state = 1
            return None
        if self.state == 1:
            if byte > MAX_PAYLOAD:
                self.state = 1 if byte == SOF else 0
                self.buf = bytearray([SOF]) if byte == SOF else bytearray()
                return ("err", 2)
            self.length = byte
            self.buf.append(byte)
            self.state = 2
            return None
        if self.state == 2:
            self.buf.append(byte)
            self.state = 3 if self.length else 4
            return None
        if self.state == 3:
            self.buf.append(byte)
            if len(self.buf) == 3 + self.length:
                self.state = 4
            return None
        expect = crc8(bytes(self.buf))
        if byte != expect:
            self.state = 1 if byte == SOF else 0
            self.buf = bytearray([SOF]) if byte == SOF else bytearray()
            return ("err", 1)
        frame = {"type": self.buf[2], "payload": bytes(self.buf[3:])}
        self.state = 0
        self.buf = bytearray()
        return ("frame", frame)


def unpack_telemetry(payload: bytes):
    if len(payload) != 42:
        raise ValueError("telemetry length")
    def u32(off):
        return int.from_bytes(payload[off:off + 4], "little")
    return {
        "heartbeat": u32(0),
        "init_stage": u32(4),
        "tx_enqueued": u32(8),
        "tx_complete": u32(12),
        "rx_count": u32(16),
        "matched": u32(20),
        "timeouts": u32(24),
        "error": u32(28),
        "last_fn": payload[32],
        "last_raw_len": payload[33],
        "last_raw": payload[34:42],
    }


def format_telemetry(fields) -> str:
    raw = fields["last_raw"][:fields["last_raw_len"]].hex()
    return ("telemetry heartbeat=%u init_stage=%u tx_enqueued=%u tx_complete=%u "
            "rx=%u matched=%u timeouts=%u error=%u last_fn=0x%02X raw=%s" % (
                fields["heartbeat"], fields["init_stage"], fields["tx_enqueued"], fields["tx_complete"],
                fields["rx_count"], fields["matched"], fields["timeouts"], fields["error"],
                fields["last_fn"], raw))


class VirtualMcu:
    def __init__(self):
        self.can = []
        self.rate = 10
        self.state = "idle"
        self.parser = Parser()
        self.heartbeat = 7
        self.init_stage = 4
        self.tx_enqueued = 0
        self.tx_complete = 0
        self.rx_count = 0
        self.matched = 0
        self.timeouts = 0
        self.error = 0
        self.last_fn = 0
        self.last_raw = b""

    def feed(self, data: bytes) -> bytes:
        out = bytearray()
        frames, errors = self.parser.feed(data)
        for err in errors:
            out += self._error(0, err)
        for frame in frames:
            out += self._command(frame)
        return bytes(out)

    def _error(self, typ, code):
        self.error = code
        body = bytes([SOF, 2, ERROR, typ, code])
        return body + bytes([crc8(body)])

    def _ack(self, typ, arg):
        body = bytes([SOF, 2, ACK, typ, arg])
        return body + bytes([crc8(body)])

    def _pong(self):
        body = bytes([SOF, 0, PONG])
        return body + bytes([crc8(body)])

    def _telemetry(self):
        raw = bytearray(42)
        def put(off, value):
            raw[off:off + 4] = int(value).to_bytes(4, "little")
        put(0, self.heartbeat)
        put(4, self.init_stage)
        put(8, self.tx_enqueued)
        put(12, self.tx_complete)
        put(16, self.rx_count)
        put(20, self.matched)
        put(24, self.timeouts)
        put(28, self.error)
        raw[32] = self.last_fn
        raw[33] = len(self.last_raw)
        raw[34:34 + len(self.last_raw)] = self.last_raw
        body = bytes([SOF, 42, TELEM]) + bytes(raw)
        return body + bytes([crc8(body)])

    def _command(self, frame):
        typ = frame["type"]
        payload = frame["payload"]
        if typ in MOTION:
            return self._error(typ, 5)
        if typ == PING:
            if payload:
                return self._error(typ, 4)
            self.error = 0
            return self._pong()
        if typ == QUERY:
            if len(payload) != 1 or payload[0] not in CAN_FN:
                return self._error(typ, 4)
            if self.state != "idle":
                return self._error(typ, 6 if self.state == "wait" else 7)
            fn = CAN_FN[payload[0]]
            self.can.append(bytes([fn, 0x6B]))
            self.tx_enqueued += 1
            self.tx_complete += 1
            self.rx_count += 1
            self.matched += 1
            self.last_fn = fn
            self.last_raw = bytes([fn, 0x00, 0x6B]) if fn == 0x3A else bytes([fn, 0x11, 0x22, 0x6B])
            self.state = "idle"
            self.error = 0
            return self._ack(typ, payload[0]) + self._telemetry()
        if typ == RATE:
            if len(payload) != 1 or not 1 <= payload[0] <= 100:
                return self._error(typ, 4)
            self.rate = payload[0]
            return self._ack(typ, payload[0])
        if typ == STOP:
            if payload:
                return self._error(typ, 4)
            self.rate = 0
            self.state = "idle"
            return self._ack(typ, 0)
        return self._error(typ, 3)


class VirtualPort:
    def __init__(self):
        self.mcu = VirtualMcu()
        self.rx = bytearray()

    def write(self, data: bytes):
        self.rx += self.mcu.feed(data)

    def read(self, n: int) -> bytes:
        take = bytes(self.rx[:n])
        del self.rx[:n]
        return take


if wintypes is not None:  # Windows only: POSIX has no COMMCONFIG at all.
    class DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate", wintypes.DWORD),
            ("flags", wintypes.DWORD),
            ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD),
            ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE),
            ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE),
            ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]
else:  # pragma: no cover - POSIX only
    DCB = None
    COMMTIMEOUTS = None


def open_port(name: str):
    global OPEN_COUNT
    OPEN_COUNT += 1
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    kernel32.CreateFileW.restype = wintypes.HANDLE
    kernel32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(DCB)]
    kernel32.GetCommState.restype = wintypes.BOOL
    kernel32.BuildCommDCBW.argtypes = [wintypes.LPCWSTR, ctypes.POINTER(DCB)]
    kernel32.BuildCommDCBW.restype = wintypes.BOOL
    kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(DCB)]
    kernel32.SetCommState.restype = wintypes.BOOL
    kernel32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
    kernel32.SetCommTimeouts.restype = wintypes.BOOL
    path = name if name.startswith("\\\\.\\") else "\\\\.\\" + name
    handle = kernel32.CreateFileW(path, 0x80000000 | 0x40000000, 0, None, 3, 0x80, None)
    if handle == wintypes.HANDLE(-1).value or handle == ctypes.c_void_p(-1).value:
        raise OSError("CreateFileW failed for %s (%s)" % (name, ctypes.get_last_error()))
    dcb = DCB()
    dcb.DCBlength = ctypes.sizeof(DCB)
    if not kernel32.GetCommState(handle, ctypes.byref(dcb)):
        kernel32.CloseHandle(handle)
        raise OSError("GetCommState failed")
    # The baud rate below is meaningless for a USB CDC virtual COM port (the
    # firmware ignores it); it is kept so the same path still works for the
    # CH340/USART1 fallback link.
    if not kernel32.BuildCommDCBW("baud=115200 parity=N data=8 stop=1", ctypes.byref(dcb)):
        kernel32.CloseHandle(handle)
        raise OSError("BuildCommDCBW failed")
    if not kernel32.SetCommState(handle, ctypes.byref(dcb)):
        kernel32.CloseHandle(handle)
        raise OSError("SetCommState failed")
    timeouts = COMMTIMEOUTS(50, 0, 50, 0, 50)
    if not kernel32.SetCommTimeouts(handle, ctypes.byref(timeouts)):
        kernel32.CloseHandle(handle)
        raise OSError("SetCommTimeouts failed")
    return kernel32, handle


class Win32Port:
    def __init__(self, name):
        self.kernel32, self.handle = open_port(name)
        self.kernel32.ReadFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
        self.kernel32.ReadFile.restype = wintypes.BOOL
        self.kernel32.WriteFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
        self.kernel32.WriteFile.restype = wintypes.BOOL

    def write(self, data: bytes):
        done = wintypes.DWORD(0)
        buf = ctypes.create_string_buffer(data)
        if not self.kernel32.WriteFile(self.handle, buf, len(data), ctypes.byref(done), None):
            raise OSError("WriteFile failed")

    def read(self, n: int) -> bytes:
        buf = ctypes.create_string_buffer(n)
        done = wintypes.DWORD(0)
        if not self.kernel32.ReadFile(self.handle, buf, n, ctypes.byref(done), None):
            raise OSError("ReadFile failed")
        return buf.raw[:done.value]

    def close(self):
        self.kernel32.CloseHandle(self.handle)


class PosixPort:
    """POSIX serial port built on os.open + termios (standard library only).

    Raw 115200 8N1, no modem control, no flow control. termios and select are
    imported inside __init__ because termios does not exist on Windows, and a
    top-level import would make this module unusable there.

    Every failure surfaces as OSError: serial_bridge maps OSError onto
    ConfigError/ConnectionError, and that contract is load-bearing.
    """

    BAUDRATE = 115200
    BYTESIZE = 8
    PARITY = "N"
    STOPBITS = 1
    READ_TIMEOUT = 0.05   # seconds; mirrors the win32 COMMTIMEOUTS 50 ms total
    WRITE_TIMEOUT = 0.5

    def __init__(self, name):
        import select
        import termios
        global OPEN_COUNT
        OPEN_COUNT += 1
        self.name = name
        self.select = select
        self.termios = termios
        self.fd = None
        fd = os.open(name, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            self._set_raw(fd)
        except OSError:
            os.close(fd)
            raise
        self.fd = fd

    def _set_raw(self, fd):
        """Put the tty into raw 115200 8N1 mode (the cfmakeraw equivalent)."""
        termios = self.termios
        iflag, oflag, cflag, lflag, _ispeed, _ospeed, cc = termios.tcgetattr(fd)
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
                   | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG
                   | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        cflag &= ~getattr(termios, "CRTSCTS", 0)
        cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        speed = getattr(termios, "B%d" % self.BAUDRATE)
        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, speed, speed, cc])

    def write(self, data: bytes):
        view = memoryview(bytes(data))
        deadline = time.monotonic() + self.WRITE_TIMEOUT
        while len(view):
            try:
                written = os.write(self.fd, view)
            except InterruptedError:
                continue
            except BlockingIOError:
                written = 0
            if written <= 0:  # EAGAIN: wait for room in the tty buffer, retry
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise OSError("write timed out on %s" % self.name)
                self.select.select([], [self.fd], [], remaining)
                continue
            view = view[written:]

    def read(self, n: int) -> bytes:
        """Return up to n bytes; b"" when nothing arrived within READ_TIMEOUT."""
        try:
            ready, _write, _err = self.select.select([self.fd], [], [], self.READ_TIMEOUT)
        except InterruptedError:
            return b""
        if not ready:
            return b""
        try:
            return os.read(self.fd, n)
        except (BlockingIOError, InterruptedError):
            return b""

    def close(self):
        if self.fd is None:
            return
        fd, self.fd = self.fd, None
        os.close(fd)


def open_serial_port(name: str):
    """Open name with the port class of the running platform.

    Windows keeps the existing kernel32 path (Win32Port); everything else uses
    the os + termios path (PosixPort). Both raise OSError on failure.
    """
    if os.name == "nt":
        return Win32Port(name)
    return PosixPort(name)


def print_frames(frames, out):
    for frame in frames:
        if frame["type"] == TELEM:
            print(format_telemetry(unpack_telemetry(frame["payload"])), file=out)
        elif frame["type"] == PONG:
            print("pong", file=out)
        elif frame["type"] == ACK:
            print("ack type=0x%02X arg=%u" % (frame["payload"][0], frame["payload"][1]), file=out)
        elif frame["type"] == ERROR:
            print("error type=0x%02X code=%u" % (frame["payload"][0], frame["payload"][1]), file=out)
        else:
            print("frame type=0x%02X len=%u" % (frame["type"], len(frame["payload"])), file=out)


def run_reader(port, queries, seconds, out):
    parser = Parser()
    port.write(build_frame(PING))
    for query in queries:
        port.write(build_frame(QUERY, bytes([query])))
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        chunk = port.read(128)
        if not chunk:
            continue
        frames, errors = parser.feed(chunk)
        for err in errors:
            print("bad-frame code=%u" % err, file=out)
        print_frames(frames, out)


def selftest(out=sys.stdout) -> int:
    if crc8(b"123456789") != 0xF4:
        raise SystemExit("crc vector")
    if build_frame(PING) != bytes([0xA5, 0x00, 0x01, 0x8F]):
        raise SystemExit("ping vector")
    for typ in list(MOTION) + [RATE, STOP, TELEM, 0x20, 0x00]:
        try:
            build_frame(typ, b"\x01")
        except ValueError:
            pass
        else:
            raise SystemExit("motion or non-whitelist type was encoded")
    try:
        build_frame(QUERY, b"\x03")
    except ValueError:
        pass
    else:
        raise SystemExit("query 3 encoded")
    port = VirtualPort()
    text = []
    class Capture:
        def write(self, s):
            text.append(s)
        def flush(self):
            pass
    cap = Capture()
    before = OPEN_COUNT
    run_reader(port, [0, 1, 2], 0.05, cap)
    if OPEN_COUNT != before:
        raise SystemExit("selftest opened a port")
    if os.name == "nt":
        if ctypes.sizeof(DCB) != 28 or ctypes.sizeof(COMMTIMEOUTS) != 20:
            raise SystemExit("win32 struct size")
    else:
        # POSIX has no DCB: check the raw line settings PosixPort programs and
        # the three methods DeviceLink calls instead.
        settings = (PosixPort.BAUDRATE, PosixPort.BYTESIZE, PosixPort.PARITY, PosixPort.STOPBITS)
        if settings != (115200, 8, "N", 1):
            raise SystemExit("posix serial parameters changed: %r" % (settings,))
        for _method in ("read", "write", "close"):
            if not callable(getattr(PosixPort, _method, None)):
                raise SystemExit("posix port is missing %s()" % _method)
    if port.mcu.can != [bytes([0x1F, 0x6B]), bytes([0x24, 0x6B]), bytes([0x3A, 0x6B])]:
        raise SystemExit("can whitelist bytes changed: %r" % port.mcu.can)
    blob = "".join(text)
    if "pong" not in blob or "last_fn=0x1F" not in blob or "last_fn=0x24" not in blob or "last_fn=0x3A" not in blob:
        raise SystemExit("missing telemetry\n" + blob)
    raw = bytes([0x00, 0xFF, 0xA5, 0x00, 0x01, 0x00, 0xA5, 0x00, 0x01, 0x8F])
    recovered = VirtualMcu().feed(raw + build_frame(QUERY, b"\x00"))
    frames, errors = Parser().feed(recovered)
    if errors:
        raise SystemExit("unexpected parse error after recovery")
    types = [f["type"] for f in frames]
    if types[0] != ERROR or PONG not in types or ACK not in types:
        raise SystemExit("resync responses %s" % types)
    motion = VirtualMcu()
    reply = motion.feed(bytes([0xA5, 0x01, 0xF3, 0x01, crc8(bytes([0xA5, 0x01, 0xF3, 0x01]))]))
    if motion.can:
        raise SystemExit("motion reached CAN")
    parsed, _ = Parser().feed(reply)
    if not parsed or parsed[0]["type"] != ERROR or parsed[0]["payload"][1] != 5:
        raise SystemExit("motion not rejected")
    print("PC_SELFTEST pass", file=out)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Read-only GEArm telemetry client (USB CDC)")
    ap.add_argument("port", nargs="?",
                    help="serial port, for example COM10 on Windows or /dev/ttyACM0 "
                         "on Linux. Omit to auto-detect the board's USB CDC port.")
    ap.add_argument("--selftest", action="store_true", help="virtual transport closed loop, no hardware")
    ap.add_argument("--query", type=int, action="append", choices=[0, 1, 2], help="whitelist query 0 version, 1 voltage, 2 state")
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()
    port_name = args.port
    if not port_name:
        port_name = find_usb_port()
        if not port_name:
            ap.error("no serial port given and no USB CDC device (VID %04X/PID %04X) found; "
                     "plug the board's USB cable in or pass the port explicitly"
                     % (USB_CDC_VID, USB_CDC_PID))
        sys.stderr.write("auto-detected %s (VID %04X/PID %04X)\n"
                         % (port_name, USB_CDC_VID, USB_CDC_PID))
    port = open_serial_port(port_name)
    try:
        run_reader(port, args.query or [], args.seconds, sys.stdout)
    finally:
        port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())