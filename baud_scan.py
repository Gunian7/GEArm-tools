"""Baud-rate scan for the GEArm serial link. Read-only.

open_port() hardcodes 115200, so this builds the port by hand at each candidate
rate and sends exactly one PING per rate. A reply at any rate means the link is
alive and only the rate was wrong; no reply at any rate points at wiring,
flashing or the wrong port.
"""
from __future__ import annotations

import argparse
import ctypes
import sys
import time
from ctypes import wintypes

import pc_service as pc

RATES = [115200, 9600, 19200, 38400, 57600, 230400, 460800, 921600]


def open_at(name, baud):
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                                     ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    kernel32.CreateFileW.restype = wintypes.HANDLE
    kernel32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(pc.DCB)]
    kernel32.GetCommState.restype = wintypes.BOOL
    kernel32.BuildCommDCBW.argtypes = [wintypes.LPCWSTR, ctypes.POINTER(pc.DCB)]
    kernel32.BuildCommDCBW.restype = wintypes.BOOL
    kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(pc.DCB)]
    kernel32.SetCommState.restype = wintypes.BOOL
    kernel32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(pc.COMMTIMEOUTS)]
    kernel32.SetCommTimeouts.restype = wintypes.BOOL

    path = name if name.startswith("\\\\.\\") else "\\\\.\\" + name
    handle = kernel32.CreateFileW(path, 0x80000000 | 0x40000000, 0, None, 3, 0x80, None)
    if handle in (wintypes.HANDLE(-1).value, ctypes.c_void_p(-1).value):
        raise OSError("open failed")
    dcb = pc.DCB()
    dcb.DCBlength = ctypes.sizeof(pc.DCB)
    kernel32.GetCommState(handle, ctypes.byref(dcb))
    kernel32.BuildCommDCBW("baud=%d parity=N data=8 stop=1" % baud, ctypes.byref(dcb))
    kernel32.SetCommState(handle, ctypes.byref(dcb))
    timeouts = pc.COMMTIMEOUTS(50, 0, 50, 0, 50)
    kernel32.SetCommTimeouts(handle, ctypes.byref(timeouts))
    kernel32.ReadFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                                  ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
    kernel32.ReadFile.restype = wintypes.BOOL
    kernel32.WriteFile.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                                   ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p]
    kernel32.WriteFile.restype = wintypes.BOOL
    return kernel32, handle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None,
                        help="COM port, e.g. COM10. Omit to auto-detect the board's USB CDC port.")
    parser.add_argument("--wait", type=float, default=1.2)
    args = parser.parse_args()

    port_name = args.port
    if not port_name:
        port_name = pc.find_usb_port()
        if not port_name:
            print("找不到 USB CDC 设备（VID %04X/PID %04X）；插好板子的 USB 线，或用 --port 指定"
                  % (pc.USB_CDC_VID, pc.USB_CDC_PID))
            return 2
        print("自动识别到 %s（VID %04X/PID %04X）" % (port_name, pc.USB_CDC_VID, pc.USB_CDC_PID))
    print("在 %s 上逐个波特率试 PING（每次只发一帧，全部只读）" % port_name)
    print("注意：USB CDC 虚拟串口的波特率由固件忽略，各档结果应当一致；\n"
          "      本脚本主要用于旧的 CH340/USART1 链路排障。\n")
    hits = []
    for baud in RATES:
        try:
            kernel32, handle = open_at(port_name, baud)
        except OSError:
            print("  %-8d 打不开" % baud)
            continue
        try:
            done = wintypes.DWORD(0)
            frame = pc.build_frame(pc.PING)
            buf = ctypes.create_string_buffer(frame)
            kernel32.WriteFile(handle, buf, len(frame), ctypes.byref(done), None)

            parser = pc.Parser()
            deadline = time.monotonic() + args.wait
            noise = 0
            replies = []
            while time.monotonic() < deadline:
                read = wintypes.DWORD(0)
                chunk = ctypes.create_string_buffer(256)
                kernel32.ReadFile(handle, chunk, 256, ctypes.byref(read), None)
                data = chunk.raw[:read.value]
                if not data:
                    continue
                noise += len(data)
                got, _ = parser.feed(data)
                replies.extend(got)
            pongs = sum(1 for f in replies if f["type"] == pc.PONG)
            acks = sum(1 for f in replies if f["type"] == pc.ACK)
            status = "PONG!" if pongs else ("有数据(%dB,%d帧)但无PONG" % (noise, len(replies))
                                           if noise else "无回应")
            print("  %-8d %s" % (baud, status))
            if pongs:
                hits.append(baud)
        finally:
            kernel32.CloseHandle(handle)

    print()
    if hits:
        print("链路通的波特率：%s" % ", ".join(str(b) for b in hits))
    else:
        print("所有波特率都无 PONG。不是速率问题。")
        print("下一步该查的：固件是否烧了新版本 / PA9-PA10 是否真的接到这个口 / 接线与共地。")


if __name__ == "__main__":
    sys.exit(main())
