"""Receive-only baud scan. USART1 (CH340) fallback-link troubleshooting only.

NOTE: this script is for the OLD CH340/USART1 link, where a baud mismatch really
did change what came out. The board's USB CDC virtual COM port ignores the baud
rate entirely, so on the USB link every rate shows the same (correct) stream and
this scan tells you nothing. For USB, use hw_probe.py or pc_service.py instead.

Default port: the board's USB CDC port (pc.find_usb_port) - pass --port to point
it at the CH340/USART1 port instead.
"""
import argparse
import ctypes
import time
from ctypes import wintypes

import pc_service as pc

RATES = [115200, 57600, 38400, 19200, 9600, 4800, 2400, 230400, 460800]


def open_at(baud, port):
    k = ctypes.WinDLL("kernel32", use_last_error=True)
    k.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                              ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    k.CreateFileW.restype = wintypes.HANDLE
    k.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(pc.DCB)]
    k.GetCommState.restype = wintypes.BOOL
    k.BuildCommDCBW.argtypes = [wintypes.LPCWSTR, ctypes.POINTER(pc.DCB)]
    k.BuildCommDCBW.restype = wintypes.BOOL
    k.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(pc.DCB)]
    k.SetCommState.restype = wintypes.BOOL
    k.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(pc.COMMTIMEOUTS)]
    k.SetCommTimeouts.restype = wintypes.BOOL
    k.ReadFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                           ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
    k.ReadFile.restype = wintypes.BOOL

    name = port if port.startswith("\\\\.\\") else "\\\\.\\" + port
    handle = k.CreateFileW(name, 0x80000000 | 0x40000000, 0, None, 3, 0x80, None)
    if handle in (wintypes.HANDLE(-1).value, ctypes.c_void_p(-1).value):
        raise OSError("open failed")
    dcb = pc.DCB()
    dcb.DCBlength = ctypes.sizeof(pc.DCB)
    k.GetCommState(handle, ctypes.byref(dcb))
    k.BuildCommDCBW("baud=%d parity=N data=8 stop=1" % baud, ctypes.byref(dcb))
    k.SetCommState(handle, ctypes.byref(dcb))
    k.SetCommTimeouts(handle, ctypes.byref(pc.COMMTIMEOUTS(50, 0, 50, 0, 50)))
    return k, handle


def main():
    ap = argparse.ArgumentParser(description="Receive-only baud scan (USART1/CH340 fallback only)")
    ap.add_argument("--port", default=None,
                    help="COM port to listen on. Omit to auto-detect the board's USB CDC port.")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()
    port = args.port
    if not port:
        port = pc.find_usb_port()
        if not port:
            print("找不到 USB CDC 设备（VID %04X/PID %04X）；插好板子的 USB 线，或用 --port 指定"
                  % (pc.USB_CDC_VID, pc.USB_CDC_PID))
            return 2
        print("自动识别到 %s（VID %04X/PID %04X）" % (port, pc.USB_CDC_VID, pc.USB_CDC_PID))
    print("只接收，不发送。每速率听 %.1f 秒。\n" % args.seconds)
    for baud in RATES:
        try:
            k, handle = open_at(baud, port)
        except OSError:
            print("  %-8d 打不开" % baud)
            continue
        try:
            deadline = time.monotonic() + args.seconds
            buf = b""
            while time.monotonic() < deadline:
                read = wintypes.DWORD(0)
                chunk = ctypes.create_string_buffer(256)
                k.ReadFile(handle, chunk, 256, ctypes.byref(read), None)
                if read.value:
                    buf += chunk.raw[:read.value]
            if not buf:
                print("  %-8d 0 字节" % baud)
                continue
            uniq = sorted(set(buf))
            sofs = [i for i, b in enumerate(buf) if b == pc.SOF]
            print("  %-8d %5d 字节  SOF@%s  唯一值: %s"
                  % (baud, len(buf), sofs[:4],
                     " ".join("%02X" % b for b in uniq[:14])))
        finally:
            k.CloseHandle(handle)


if __name__ == "__main__":
    main()
