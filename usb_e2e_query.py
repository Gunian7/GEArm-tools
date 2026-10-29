#!/usr/bin/env python3
"""Paced read-only query test over the board's USB CDC port.

pc_service sends PING + all QUERYs back to back, but the firmware only allows
one CAN command in flight at a time (the rest get SER_ERR_BUSY), so this paces
them: send one, wait for its reply, then send the next.

Read-only: PING / QUERY only. No motion, enable, speed, position, homing or
parameter writes.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pc_service as pc

QUERIES = [0, 1, 2]          # version, voltage, state
PER_QUERY_SECONDS = 1.5
TOTAL_SECONDS = 8.0


def main() -> int:
    port_name = pc.find_usb_port()
    if not port_name:
        print("找不到 USB CDC 设备（VID %04X/PID %04X）；插好板子的 USB 线"
              % (pc.USB_CDC_VID, pc.USB_CDC_PID))
        return 2
    print("port %s (auto-detected)" % port_name)
    port = pc.Win32Port(port_name)
    parser = pc.Parser()
    counts = {"tx_enqueued": 0, "tx_complete": 0, "rx_count": 0, "matched": 0,
              "timeouts": 0, "error": 0}
    raw_replies = {}
    failures = 0
    try:
        port.write(pc.build_frame(pc.PING))

        def pump(seconds):
            nonlocal failures
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                chunk = port.read(256)
                if not chunk:
                    continue
                frames, errors = parser.feed(chunk)
                for _err in errors:
                    failures += 1
                for frame in frames:
                    if frame["type"] == pc.TELEM:
                        fields = pc.unpack_telemetry(frame["payload"])
                        for key in counts:
                            counts[key] = max(counts[key], fields[key])
                        if fields["last_fn"] and fields["last_raw"]:
                            raw_replies[fields["last_fn"]] = fields["last_raw"][:fields["last_raw_len"]].hex()
                    elif frame["type"] == pc.PONG:
                        print("  ping -> pong")
                    elif frame["type"] == pc.ACK:
                        print("  query %u -> acked" % frame["payload"][1])
                    elif frame["type"] == pc.ERROR:
                        print("  error code=%u" % frame["payload"][1])
                        failures += 1

        pump(TOTAL_SECONDS - PER_QUERY_SECONDS * len(QUERIES))
        for query in QUERIES:
            print("== QUERY %u (%s) ==" % (query, ["version", "voltage", "state"][query]))
            before = dict(counts)
            port.write(pc.build_frame(pc.QUERY, bytes([query])))
            pump(PER_QUERY_SECONDS)
            gained = {k: counts[k] - before[k] for k in counts}
            print("   tx_enqueued +%d  tx_complete +%d  rx +%d  matched +%d  timeouts +%d"
                  % (gained["tx_enqueued"], gained["tx_complete"], gained["rx_count"],
                     gained["matched"], gained["timeouts"]))
            if gained["matched"] < 1:
                print("   !! 没有收到匹配回包")
                failures += 1
    finally:
        port.close()

    print()
    print("== 汇总 ==")
    for key, value in counts.items():
        print("  %-12s %u" % (key, value))
    print("== 原始回包（按功能码）==")
    for fn in sorted(raw_replies):
        print("  fn=0x%02X  %s" % (fn, raw_replies[fn]))
    ok = (counts["matched"] >= len(QUERIES) and counts["rx_count"] >= len(QUERIES)
          and counts["tx_complete"] >= len(QUERIES) and not failures)
    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
