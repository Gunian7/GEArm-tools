"""Read-only hardware probe for the GEArm MC02 over its real serial port.

Deliberately staged so a silent link is distinguishable from a broken one:

  stage 1  passive listen only -- not one byte is written. If the device is
           already streaming telemetry we see it here without asking anything.
  stage 2  PING, then QUERY(0/1/2) -- the two frames in TX_WHITELIST. Both are
           read-only: PING asks "are you there", QUERY asks the firmware to poll
           one CAN function code and return the reply.

Nothing else is ever sent. Motion, enable, speed, position, homing and
parameter-write frames are refused by pc_service before a byte is produced.
"""
from __future__ import annotations

import argparse
import sys
import time

import pc_service as pc


def listen_only(port, seconds, out):
    """Read whatever arrives without writing anything."""
    parser = pc.Parser()
    deadline = time.monotonic() + seconds
    frames, noise = [], 0
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if not chunk:
            continue
        noise += len(chunk)
        got, errors = parser.feed(chunk)
        frames.extend(got)
        for _ in errors:
            print("  bad-frame ( framing error )", file=out)
    return frames, noise


def query_roundtrip(port, queries, seconds, out):
    """PING then each QUERY, collecting replies until the deadline."""
    parser = pc.Parser()
    port.write(pc.build_frame(pc.PING))
    for query in queries:
        port.write(pc.build_frame(pc.QUERY, bytes([query])))
    deadline = time.monotonic() + seconds
    replies = []
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if not chunk:
            continue
        got, errors = parser.feed(chunk)
        for _ in errors:
            print("  bad-frame ( framing error )", file=out)
        replies.extend(got)
    return replies


def describe(frames):
    lines = []
    for frame in frames:
        kind = frame["type"]
        if kind == pc.TELEM:
            lines.append("遥测 " + pc.format_telemetry(pc.unpack_telemetry(frame["payload"])))
        elif kind == pc.PONG:
            lines.append("PONG")
        elif kind == pc.ACK:
            payload = frame["payload"]
            lines.append("ACK cmd=0x%02X arg=%u" % (payload[0], payload[1]))
        elif kind == pc.ERROR:
            payload = frame["payload"]
            lines.append("ERROR cmd=0x%02X code=%u" % (payload[0], payload[1]))
        else:
            lines.append("frame type=0x%02X len=%u" % (kind, len(frame["payload"])))
    return lines


def main():
    parser = argparse.ArgumentParser(description="Read-only GEArm probe over USB CDC")
    parser.add_argument("--port", default=None,
                        help="COM port, e.g. COM10. Omit to auto-detect the board's USB CDC port.")
    parser.add_argument("--listen", type=float, default=3.0)
    parser.add_argument("--wait", type=float, default=4.0)
    parser.add_argument("--queries", default="0,1,2")
    args = parser.parse_args()

    queries = [int(q) for q in args.queries.split(",") if q.strip()]
    out = sys.stdout
    port_name = args.port
    if not port_name:
        port_name = pc.find_usb_port()
        if not port_name:
            print("找不到 USB CDC 设备（VID %04X/PID %04X）；插好板子的 USB 线，或用 --port 指定"
                  % (pc.USB_CDC_VID, pc.USB_CDC_PID))
            return 2
        print("自动识别到 %s（VID %04X/PID %04X）" % (port_name, pc.USB_CDC_VID, pc.USB_CDC_PID))
    print("端口 %s（USB CDC，波特率由固件忽略）" % port_name)
    print("将发送的帧：PING + QUERY(%s) —— 全部只读，无任何运动指令"
          % ",".join(str(q) for q in queries))
    print()

    try:
        port = pc.Win32Port(port_name)
    except OSError as exc:
        print("打不开 %s：%s" % (port_name, exc))
        print("（若提示被占用，说明别的程序占着这个口；bridge 或串口监视器要关掉）")
        return 2

    try:
        print("── 阶段 1：被动监听 %.1f 秒（不发送任何字节）" % args.listen)
        frames, noise = listen_only(port, args.listen, out)
        print("   收到 %d 字节，%d 个完整帧" % (noise, len(frames)))
        for line in describe(frames)[:12]:
            print("   " + line)
        if not frames and noise:
            print("   有字节但组不成帧 —— 可能是波特率不对，或设备在输出别的东西")
        print()

        print("── 阶段 2：PING + QUERY(%s)" % ",".join(str(q) for q in queries))
        replies = query_roundtrip(port, queries, args.wait, out)
        pongs = sum(1 for f in replies if f["type"] == pc.PONG)
        acks = sum(1 for f in replies if f["type"] == pc.ACK)
        errors = sum(1 for f in replies if f["type"] == pc.ERROR)
        telemetry = sum(1 for f in replies if f["type"] == pc.TELEM)
        print("   回包：PONG %d / ACK %d / ERROR %d / 遥测 %d"
              % (pongs, acks, errors, telemetry))
        for line in describe(replies)[:24]:
            print("   " + line)
        print()

        if pongs:
            print("结论：USART 链路通。固件有回应。")
        elif noise:
            print("结论：串口有数据但帧对不上 —— 先查波特率与接线，别急着发别的。")
        else:
            print("结论：没有任何回应。可能原因：固件未烧录新版本 / 引脚不对 / 接线问题 / "
                  "USART 未使能。不要再重试发送，先核对硬件。")
        return 0 if pongs else 1
    finally:
        port.close()


if __name__ == "__main__":
    sys.exit(main())
