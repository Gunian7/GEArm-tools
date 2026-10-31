# -*- coding: utf-8 -*-
"""Bridge HTTP/WS tests. No serial port, no hardware, no virtual_mc02.

The fake device is a LoopbackDeviceLink fed with scripted byte strings.
"""
import base64
import json
import socket
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))

import pc_service as pc

OPEN_BEFORE = pc.OPEN_COUNT


def _refuse_open(*args, **kwargs):
    raise AssertionError("real serial open_port was called: %r %r" % (args, kwargs))


pc.open_port = _refuse_open

import serial_bridge as sb  # noqa: E402


def frame(typ, payload=b""):
    body = bytes([pc.SOF, len(payload), typ]) + payload
    return body + bytes([pc.crc8(body)])


def telemetry(heartbeat, init_stage, tx_enqueued, tx_complete, rx_count, matched, timeouts, error, last_fn, raw):
    blob = bytearray(42)
    def put(off, value):
        blob[off:off + 4] = int(value).to_bytes(4, "little")
    put(0, heartbeat)
    put(4, init_stage)
    put(8, tx_enqueued)
    put(12, tx_complete)
    put(16, rx_count)
    put(20, matched)
    put(24, timeouts)
    put(28, error)
    blob[32] = last_fn
    blob[33] = len(raw)
    blob[34:34 + len(raw)] = raw
    return frame(pc.TELEM, bytes(blob))


class ScriptedDevice(sb.LoopbackDeviceLink):
    """Each host frame consumes the next scripted RX byte string."""

    def __init__(self, script):
        super().__init__()
        self.script = list(script)

    def send(self, data):
        super().send(data)
        if self.script:
            reply = self.script.pop(0)
            if reply:
                self.push(reply)


def http_json(method, url, obj=None, timeout=3):
    data = None if obj is None else json.dumps(obj).encode("utf-8")
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read()
            return resp.status, json.loads(raw.decode("utf-8")) if raw else None
    except urllib.error.HTTPError as exc:
        raw = exc.read()
        body = json.loads(raw.decode("utf-8")) if raw else None
        return exc.code, body


def http_text(url, timeout=3):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return resp.status, resp.read().decode("utf-8")


def wait_until(pred, timeout=2.5):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = pred()
        if last:
            return last
        time.sleep(0.02)
    return last


def ws_handshake(port):
    sock = socket.create_connection(("127.0.0.1", port), timeout=2)
    sock.settimeout(2)
    key = base64.b64encode(b"0123456789abcdef").decode("ascii")
    req = (
        "GET /ws HTTP/1.1\r\n"
        "Host: 127.0.0.1:%d\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n" % (port, key)
    )
    sock.sendall(req.encode("ascii"))
    data = b""
    while b"\r\n\r\n" not in data:
        chunk = sock.recv(2048)
        if not chunk:
            break
        data += chunk
    head, rest = data.split(b"\r\n\r\n", 1)
    status_line = head.split(b"\r\n", 1)[0]
    if b" 101 " not in status_line:
        raise AssertionError(head.decode("latin1", "replace"))
    return sock, rest


def ws_frames(sock, first, limit=8):
    buf = first
    found = []
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline and len(found) < limit:
        frames, buf = decode_ws(buf)
        if frames:
            found.extend(frames)
            if len(found) >= limit:
                break
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        buf += chunk
    return found


def decode_ws(buf):
    frames = []
    while True:
        if len(buf) < 2:
            break
        length = buf[1] & 0x7F
        masked = (buf[1] & 0x80) != 0
        offset = 2
        if length == 126:
            if len(buf) < 4:
                break
            length = int.from_bytes(buf[2:4], "big")
            offset = 4
        elif length == 127:
            if len(buf) < 10:
                break
            length = int.from_bytes(buf[2:10], "big")
            offset = 10
        mask_len = 4 if masked else 0
        if len(buf) < offset + mask_len + length:
            break
        if masked:
            mask = buf[offset:offset + 4]
            offset += 4
            data = bytes(buf[offset + i] ^ mask[i % 4] for i in range(length))
        else:
            data = bytes(buf[offset:offset + length])
        opcode = buf[0] & 0x0F
        buf = buf[offset + length:]
        frames.append((opcode, data))
    return frames, buf


class BridgeApiTest(unittest.TestCase):
    def setUp(self):
        self.bridge = None
        self.httpd = None
        self.assertEqual(pc.OPEN_COUNT, OPEN_BEFORE)

    def tearDown(self):
        if self.bridge is not None:
            self.bridge.stop()
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()
        self.assertEqual(pc.OPEN_COUNT, OPEN_BEFORE)
        self.assertEqual(set(pc.TX_WHITELIST), {pc.PING, pc.QUERY})

    def start(self, factory, **kwargs):
        kwargs.setdefault("retry_interval", 0.05)
        kwargs.setdefault("command_timeout", 1.2)
        self.bridge = sb.Bridge(factory, **kwargs)
        self.bridge.start()
        self.httpd = sb.serve(self.bridge, "127.0.0.1", 0)
        host, port = self.httpd.server_address
        self.assertEqual(host, "127.0.0.1")
        self.base = "http://127.0.0.1:%d" % port
        self.port = port

    def status(self):
        code, body = http_json("GET", self.base + "/api/status")
        self.assertEqual(code, 200)
        return body

    def post(self, obj, timeout=3):
        return http_json("POST", self.base + "/api/command", obj, timeout=timeout)

    def test_status_snapshot_from_scripted_telemetry(self):
        link = sb.LoopbackDeviceLink()
        self.start(lambda: link)
        raw = bytes([0x24, 0x00, 0x10, 0x6B])
        link.push(telemetry(11, 4, 8, 7, 6, 5, 3, 0, 0x24, raw))
        body = wait_until(lambda: (self.status().get("telemetry") or {}).get("heartbeat") == 11 and self.status())
        self.assertTrue(body)
        body = self.status()
        self.assertEqual(body["connection"], "connected")
        telem = body["telemetry"]
        self.assertEqual(telem["heartbeat"], 11)
        self.assertEqual(telem["init_stage"], 4)
        self.assertEqual(telem["tx_enqueued"], 8)
        self.assertEqual(telem["tx_complete"], 7)
        self.assertEqual(telem["rx_count"], 6)
        self.assertEqual(telem["matched"], 5)
        self.assertEqual(telem["timeouts"], 3)
        self.assertEqual(telem["error"], 0)
        self.assertEqual(telem["last_fn"], 0x24)
        self.assertEqual(telem["last_raw_len"], 4)
        self.assertEqual(telem["last_raw"], "2400106b")
        self.assertEqual(link.sent, [])

    def test_whitelist_commands_are_forwarded(self):
        script = [
            frame(pc.PONG),
            frame(pc.ACK, bytes([pc.QUERY, 0])) + telemetry(2, 4, 1, 1, 1, 1, 0, 0, 0x1F, bytes([0x1F, 0x01, 0x02, 0x6B])),
            frame(pc.ACK, bytes([pc.RATE, 10])),
            frame(pc.ACK, bytes([pc.STOP, 0])),
        ]
        link = ScriptedDevice(script)
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        ping = self.post({"cmd": "ping"})
        query = self.post({"cmd": "query", "arg": 0})
        rate = self.post({"cmd": "rate", "arg": 10})
        stop = self.post({"cmd": "stop"})
        self.assertEqual([ping[0], query[0], rate[0], stop[0]], [200, 200, 200, 200])
        self.assertEqual([item[1]["state"] for item in (ping, query, rate, stop)], ["acked", "acked", "acked", "acked"])
        self.assertEqual(link.sent, [
            bytes.fromhex("A500018F"),
            frame(pc.QUERY, bytes([0])),
            frame(pc.RATE, bytes([10])),
            frame(pc.STOP, b""),
        ])
        for sent in link.sent:
            self.assertIn(sent[2], (pc.PING, pc.QUERY, pc.RATE, pc.STOP))
        self.assertEqual(self.status()["telemetry"]["last_fn"], 0x1F)
        self.assertEqual(pc.build_frame(pc.PING), bytes.fromhex("A500018F"))

    def test_arm_text_command_is_sent_verbatim(self):
        """The `arm` command carries a whitelisted text line for GEArm.

        What must hold: the wire bytes are exactly "<text>" + bytes((0x0D, 0x0A)) with no
        marker byte and no binary framing.
        """
        link = ScriptedDevice([])
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        code, body = self.post({"cmd": "arm", "arg": "rel_rotate 0 10"})
        self.assertEqual(code, 200)
        self.assertEqual(link.sent, [b"rel_rotate 0 10" + bytes((0x0D, 0x0A))])

        code, body = self.post({"cmd": "arm", "arg": "estop"})
        self.assertEqual(code, 200)
        self.assertEqual(link.sent[-1], b"estop" + bytes((0x0D, 0x0A)))

    def test_arm_command_rejects_non_whitelisted_text(self):
        """Anything outside the firmware command table must never reach the wire."""
        link = ScriptedDevice([])
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        for bad in ("reboot", "rel_rotate 0 abc", "estop; rm -rf /", "", "estop\nhard_reset"):
            code, body = self.post({"cmd": "arm", "arg": bad})
            self.assertEqual(code, 400, "should reject %r" % bad)
        self.assertEqual(link.sent, [])

    def test_binary_commands_still_frame_normally_alongside_arm(self):
        """Adding the text path must not change the binary encoding."""
        script = [frame(pc.PONG), frame(pc.ACK, bytes([pc.STOP, 0]))]
        link = ScriptedDevice(script)
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        ping = self.post({"cmd": "ping"})
        arm = self.post({"cmd": "arm", "arg": "hard_reset"})
        stop = self.post({"cmd": "stop"})
        self.assertEqual([ping[0], arm[0], stop[0]], [200, 200, 200])
        self.assertEqual(link.sent, [
            bytes.fromhex("A500018F"),
            b"hard_reset" + bytes((0x0D, 0x0A)),
            frame(pc.STOP, b""),
        ])

    def test_estop_button_is_wired_on_the_console(self):
        """急停按钮必须在控制台上可见，且 estop.js 已被静态路由注册。

        控制台现在只有一个页面（viewer.html），其他页面已作为冗余移除。
        漏掉 estop.js = 按钮根本不执行，这是安全相关回归。
        """
        html = (ROOT / "web" / "viewer.html").read_text(encoding="utf8")
        self.assertIn("estop.js", html, "viewer.html 未引入 estop.js")
        # 路由表里必须能取到 estop.js，否则浏览器 404、按钮根本不执行
        self.assertIn("/estop.js", sb._STATIC)
        self.assertEqual(sb._STATIC["/estop.js"][1], "text/javascript; charset=utf-8")

    def test_root_path_serves_the_console(self):
        """根路径必须指向唯一的控制台，且已删页面不再被路由。"""
        self.assertEqual(sb._STATIC["/"][0], "viewer.html")
        for gone in ("/index.html", "/app.js", "/control.html", "/control.js",
                     "/joystick.html", "/joystick.js"):
            self.assertNotIn(gone, sb._STATIC, "%s 应已随冗余页面移除" % gone)

    def test_console_has_estop_and_mode_controls(self):
        """控制台必须带实机/干跑切换和侧栏急停入口。"""
        html = (ROOT / "web" / "viewer.html").read_text(encoding="utf8")
        for needle in ("mode-toggle", "estop-quick", "last-reply", "conn"):
            self.assertIn('id="%s"' % needle, html, "viewer.html 缺少 #%s" % needle)
        js = (ROOT / "web" / "viewer.js").read_text(encoding="utf8")
        self.assertIn("function setRealMode", js)
        self.assertIn("rel_rotate ", js, "控制台必须真实下发 rel_rotate")

    def test_estop_button_issues_arm_command(self):
        """点急停必须发出 arm/estop，且被桥接层接受后真正上线。"""
        link = ScriptedDevice([])
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        code, body = self.post({"cmd": "arm", "arg": "estop"})
        self.assertEqual(code, 200)
        self.assertEqual(link.sent, [b"estop" + bytes((0x0D, 0x0A))])

    def test_estop_clear_restores_motion(self):
        """解除急停走 estop_clear，同样上线。"""
        link = ScriptedDevice([])
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        code, body = self.post({"cmd": "arm", "arg": "estop_clear"})
        self.assertEqual(code, 200)
        self.assertEqual(link.sent, [b"estop_clear" + bytes((0x0D, 0x0A))])

    def test_device_echo_is_split_from_binary_frames(self):
        """固件的 "+OK <cmd>" 回显必须被识别为文本，不能喂给帧解析器。

        喂进去的后果：解析器把它当坏帧刷 warning，真实响应被淹掉，
        用户在 Web 上永远看不到"板子确认了"。
        """
        text, binary = sb._split_text_lines(b"+OK estop\r\n")
        self.assertEqual(text, ["+OK estop"])
        self.assertEqual(binary, b"")

        # 多条回显一次到达
        text, binary = sb._split_text_lines(b"+OK rel_rotate 0 5\r\n+OK estop\r\n")
        self.assertEqual(text, ["+OK rel_rotate 0 5", "+OK estop"])

        # 二进制帧不能被误判成文本（含非打印字节）
        frame = bytes((0xA5, 0x04, 0x01, 0x02, 0x03, 0x04))
        text, binary = sb._split_text_lines(frame)
        self.assertEqual(text, [])
        self.assertEqual(binary, frame)

        # 空数据
        self.assertEqual(sb._split_text_lines(b""), ([], b""))

    def test_device_echo_marks_command_acked(self):
        """发一条 arm 命令后，设备回显必须把该命令标成 acked 并带上回显内容。

        用 ScriptedDevice 的 script 机制让设备在收到命令的瞬间自动回灌回显 ——
        这正是真实固件的行为。不能先用 post() 再手动 push：post() 会阻塞到
        1.2s 命令超时，等它返回时命令已经是 timeout 态，回显再也关联不上。
        """
        link = ScriptedDevice([b"+OK estop\r\n"])
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))

        code, body = self.post({"cmd": "arm", "arg": "estop"})
        self.assertEqual(code, 200)

        cmds = [c for c in self.status()["commands"] if c["cmd"] == "arm"]
        self.assertTrue(cmds, "arm 命令应出现在命令列表")
        last = cmds[-1]
        self.assertEqual(last["device_reply"], "+OK estop")
        self.assertEqual(last["state"], "acked")
        self.assertIsNone(last["error"])

    def test_consecutive_commands_all_get_echo(self):
        """连续发多条命令，每一条都必须收到回显。

        这是回归测试：_on_device_line() 里曾经在持有 self.lock 的情况下
        调用 _pump_send()，而 threading.Lock 不可重入，直接死锁读取线程。
        症状很隐蔽——第一条命令能正常 acked，之后所有命令永远卡在 queued，
        Web 上看起来"能连上、第一条也成功了"，但再点任何按钮都没反应。
        """
        replies = [b"+OK estop\r\n", b"+OK estop_clear\r\n", b"+OK zero\r\n"]
        link = ScriptedDevice(replies)
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))

        for i, arg in enumerate(("estop", "estop_clear", "zero")):
            code, body = self.post({"cmd": "arm", "arg": arg})
            self.assertEqual(code, 200, "第 %d 条命令应被接受" % (i + 1))

        # 三条命令都必须 acked 且各自带上自己的回显
        self.assertTrue(wait_until(
            lambda: len([c for c in self.status()["commands"]
                         if c["cmd"] == "arm" and c.get("device_reply")]) == 3,
            timeout=10.0,
        ), "三条命令都应收到回显，实际: %s" % self.status()["commands"])

        cmds = [c for c in self.status()["commands"] if c["cmd"] == "arm"]
        self.assertEqual(len(cmds), 3)
        for c in cmds:
            self.assertEqual(c["state"], "acked")
        self.assertEqual([c["device_reply"] for c in cmds],
                         ["+OK estop", "+OK estop_clear", "+OK zero"])
        # 队列必须排空，不能有残留
        self.assertFalse([c for c in cmds if c["state"] == "queued"],
                         "不应有命令卡在 queued")

    def test_arm_command_without_echo_times_out(self):
        """没有回显时命令必须走超时，不能被文本路由误判成已确认。"""
        link = ScriptedDevice([])
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        code, body = self.post({"cmd": "arm", "arg": "estop"})
        self.assertEqual(code, 200)
        cmds = [c for c in self.status()["commands"] if c["cmd"] == "arm"]
        self.assertEqual(cmds[-1]["state"], "timeout")
        self.assertIsNone(cmds[-1]["device_reply"])

    def test_device_lines_surface_in_status(self):
        """回显行要进 status()，让 Web 刷新后也能看到。"""
        link = ScriptedDevice([])
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        link.push(b"+OK zero\r\n")
        self.assertTrue(wait_until(
            lambda: "+OK zero" in self.status().get("device_lines", []),
            timeout=5.0,
        ))

    def test_motion_and_unknown_commands_are_rejected_and_not_sent(self):
        link = sb.LoopbackDeviceLink()
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        names = [
            "enable", "speed", "velocity", "position", "pos", "home", "homing",
            "zero", "set_zero", "write", "write_param", "move", "motion", "jog",
            "sync", "estop", "motor_stop", "0xF3", "0xFE", "243", "0xF6",
        ]
        payloads = [{"cmd": name} for name in names]
        payloads.append({"cmd": "ping", "type": 0xF3})
        payloads.append({"cmd": "query", "arg": 3})
        payloads.append({"cmd": "query", "arg": 9})
        payloads.append({"cmd": "rate", "arg": 0})
        payloads.append({"cmd": "rate", "arg": 101})
        payloads.append({"cmd": "stop", "arg": 1})
        payloads.append({"cmd": "ping", "arg": 1})
        with self.assertLogs("serial_bridge", level="WARNING") as captured:
            results = [self.post(item) for item in payloads]
        self.assertTrue(link.sent == [])
        blob = b"".join(link.sent)
        for typ in pc.MOTION:
            self.assertNotIn(bytes([typ]), blob)
        for code, body in results:
            self.assertEqual(code, 400)
            self.assertFalse(body["ok"])
            self.assertEqual(body["error"], "rejected")
        log_text = "\n".join(captured.output).lower()
        self.assertIn("enable", log_text)
        self.assertIn("reject", log_text)
        listed = self.status()["rejections"]
        self.assertTrue(any(item["cmd"] == "enable" for item in listed))
        self.assertTrue(any(item["cmd"] == "0xF3" for item in listed))
        self.assertEqual(link.sent, [])

    def test_command_seq_correlation(self):
        link = sb.LoopbackDeviceLink()
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        code, ping = self.post({"cmd": "ping", "wait": False})
        code2, query = self.post({"cmd": "query", "arg": 1, "wait": False})
        self.assertEqual(code, 200)
        self.assertEqual(code2, 200)
        self.assertEqual(ping["seq"], 1)
        self.assertEqual(query["seq"], 2)
        self.assertNotEqual(ping["seq"], query["seq"])
        self.assertTrue(wait_until(lambda: len(link.sent) >= 1))
        self.assertEqual(link.sent[0], frame(pc.PING))
        self.assertEqual(len(link.sent), 1)
        link.push(frame(pc.PONG))
        self.assertTrue(wait_until(lambda: len(link.sent) >= 2))
        self.assertEqual(link.sent[1], frame(pc.QUERY, bytes([1])))
        link.push(frame(pc.ERROR, bytes([pc.QUERY, 4])))
        body = wait_until(lambda: (
            (lambda rows: len(rows) >= 2 and rows[0]["state"] == "acked" and rows[1]["state"] == "device_error" and rows)(self.status()["commands"])
        ))
        self.assertTrue(body)
        rows = self.status()["commands"]
        self.assertEqual(rows[0]["seq"], 1)
        self.assertEqual(rows[0]["cmd"], "ping")
        self.assertEqual(rows[0]["response"]["kind"], "pong")
        self.assertEqual(rows[1]["seq"], 2)
        self.assertEqual(rows[1]["cmd"], "query")
        self.assertEqual(rows[1]["arg"], 1)
        self.assertEqual(rows[1]["response"]["kind"], "error")
        self.assertEqual(rows[1]["response"]["type"], pc.QUERY)
        self.assertEqual(rows[1]["response"]["code"], 4)
        link.push(frame(pc.PONG))
        time.sleep(0.15)
        again = self.status()["commands"]
        self.assertEqual(again[1]["state"], "device_error")
        self.assertEqual(again[1]["response"]["kind"], "error")

    def test_spurious_pong_does_not_complete_query(self):
        link = sb.LoopbackDeviceLink()
        self.start(lambda: link, command_timeout=0.8)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        code, queued = self.post({"cmd": "query", "arg": 2, "wait": False})
        self.assertEqual(code, 200)
        self.assertEqual(queued["seq"], 1)
        self.assertTrue(wait_until(lambda: len(link.sent) == 1))
        link.push(frame(pc.PONG))
        time.sleep(0.15)
        row = self.status()["commands"][0]
        self.assertEqual(row["seq"], 1)
        self.assertNotEqual(row["state"], "acked")
        link.push(frame(pc.ACK, bytes([pc.QUERY, 2])))
        done = wait_until(lambda: self.status()["commands"][0]["state"] == "acked" and self.status())
        self.assertTrue(done)
        row = self.status()["commands"][0]
        self.assertEqual(row["response"]["kind"], "ack")
        self.assertEqual(row["response"]["type"], pc.QUERY)
        self.assertEqual(row["response"]["arg"], 2)

    def test_reconnect_status_and_queued_command_is_not_dropped(self):
        gate = threading.Event()
        created = []

        def factory():
            if not created:
                created.append("down")
                raise ConnectionError("virtual device down")
            if len(created) == 1:
                created.append("waiting")
                gate.wait(timeout=2)
                raise ConnectionError("still down")
            link = ScriptedDevice([frame(pc.PONG)])
            created.append(link)
            return link

        self.start(factory)
        seen = wait_until(lambda: self.status()["connection"] == "reconnecting" and self.status()["retry_count"] >= 1 and self.status())
        self.assertTrue(seen)
        self.assertTrue(seen["last_error"])
        code, queued = self.post({"cmd": "ping", "wait": False})
        self.assertEqual(code, 200)
        self.assertEqual(queued["state"], "queued")
        seq = queued["seq"]
        gate.set()
        done = wait_until(lambda: (
            self.status()["connection"] == "connected" and any(
                row["seq"] == seq and row["state"] == "acked" for row in self.status()["commands"]
            ) and self.status()
        ), timeout=3)
        self.assertTrue(done)
        self.assertGreaterEqual(done["retry_count"], 2)
        link = created[-1]
        self.assertIsInstance(link, ScriptedDevice)
        self.assertEqual(link.sent, [frame(pc.PING)])
        row = [item for item in self.status()["commands"] if item["seq"] == seq][0]
        self.assertEqual(row["response"]["kind"], "pong")

    def test_inflight_disconnect_is_explicit(self):
        first = sb.LoopbackDeviceLink()
        second = sb.LoopbackDeviceLink()
        links = []

        def factory():
            link = first if not links else second
            links.append(link)
            return link

        self.start(factory)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        code, queued = self.post({"cmd": "ping", "wait": False})
        self.assertEqual(queued["seq"], 1)
        self.assertTrue(wait_until(lambda: len(first.sent) == 1))
        first.fail_recv = True
        lost = wait_until(lambda: self.status()["commands"][0]["state"] == "transport_lost" and self.status())
        self.assertTrue(lost)
        self.assertEqual(lost["commands"][0]["seq"], 1)
        self.assertTrue(lost["commands"][0]["error"])
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected" and len(links) >= 2))
        self.assertTrue(second.sent == [])
        self.assertIn("transport_lost", self.status()["commands"][0]["state"])

    def test_single_device_connection(self):
        state = {"open": 0, "max": 0, "links": []}

        def factory():
            if state["open"] != 0:
                raise AssertionError("second device link opened before the first closed")
            link = sb.LoopbackDeviceLink()
            state["open"] += 1
            state["max"] = max(state["max"], state["open"])
            state["links"].append(link)
            original = link.close

            def close():
                if state["open"] > 0:
                    state["open"] -= 1
                original()

            link.close = close
            return link

        self.start(factory)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        self.assertEqual(len(state["links"]), 1)
        state["links"][0].fail_recv = True
        self.assertTrue(wait_until(lambda: len(state["links"]) >= 2 and self.status()["connection"] == "connected"))
        self.assertEqual(state["max"], 1)
        self.assertEqual(state["open"], 1)
        self.assertIsNone(self.status()["last_error"])

    def test_websocket_pushes_telemetry(self):
        link = sb.LoopbackDeviceLink()
        self.start(lambda: link)
        self.assertTrue(wait_until(lambda: self.status()["connection"] == "connected"))
        sock, rest = ws_handshake(self.port)
        try:
            link.push(telemetry(9, 3, 4, 4, 2, 2, 1, 0, 0x3A, bytes([0x3A, 0x01, 0x6B])))
            frames = ws_frames(sock, rest)
        finally:
            sock.close()
        texts = []
        for opcode, payload in frames:
            if opcode == 1:
                texts.append(json.loads(payload.decode("utf-8")))
        telem = [item for item in texts if item.get("kind") == "telemetry"]
        self.assertTrue(telem)
        snap = telem[-1]["telemetry"]
        self.assertEqual(snap["heartbeat"], 9)
        self.assertEqual(snap["init_stage"], 3)
        self.assertEqual(snap["tx_enqueued"], 4)
        self.assertEqual(snap["rx_count"], 2)
        self.assertEqual(snap["matched"], 2)
        self.assertEqual(snap["timeouts"], 1)
        self.assertEqual(snap["last_fn"], 0x3A)
        self.assertEqual(snap["last_raw"], "3a016b")

    def test_plan_runtime_defaults_to_usb_and_port_is_an_override(self):
        # --port is no longer refused: the USB CDC link is real now. Planning
        # itself must still never open a port.
        plan = sb.plan_runtime(["--port", "COM9"])
        self.assertEqual(plan["transport"], "usb")
        self.assertEqual(plan["serial"], "COM9")
        plan = sb.plan_runtime([])
        self.assertEqual(plan["transport"], "usb")
        self.assertIsNone(plan["serial"])
        self.assertEqual(plan["tcp"], ("127.0.0.1", 45111))
        self.assertEqual(plan["http_host"], "127.0.0.1")
        self.assertIsNone(plan["vid"])
        self.assertIsNone(plan["pid"])
        self.assertEqual(pc.OPEN_COUNT, OPEN_BEFORE)
        ids = sb.plan_runtime(["--vid", "0x1234", "--pid", "0x5678"])
        self.assertEqual((ids["vid"], ids["pid"]), (0x1234, 0x5678))
        with self.assertRaises(sb.ConfigError):
            sb.plan_runtime(["--vid", "nope"])
        with self.assertRaises(sb.ConfigError):
            sb.plan_runtime(["--tcp", "10.1.2.3:45111"])
        with self.assertRaises(ValueError):
            sb.serve(sb.Bridge(lambda: sb.LoopbackDeviceLink()), "0.0.0.0", 0)

    def test_usb_cdc_link_autodetects_and_translates_errors(self):
        opened = []

        class FakePort:
            def __init__(self, name):
                opened.append(name)
                self.name = name
                self.written = []
                self.closed = False

            def write(self, data):
                self.written.append(bytes(data))

            def read(self, n):
                return b"chunk"

            def close(self):
                self.closed = True

        real_find, real_port = pc.find_usb_port, pc.Win32Port
        try:
            pc.find_usb_port = lambda **kwargs: "COM10"
            pc.Win32Port = FakePort
            link = sb.UsbCdcDeviceLink()
            self.assertEqual(link.name, "COM10")
            link.send(b"abc")
            self.assertEqual(opened, ["COM10"])
            self.assertEqual(link.recv(), b"chunk")
            link.close()
            self.assertTrue(link._port.closed)

            explicit = sb.UsbCdcDeviceLink("COM7")
            self.assertEqual(explicit.name, "COM7")
            self.assertEqual(opened, ["COM10", "COM7"])

            pc.find_usb_port = lambda **kwargs: None
            with self.assertRaises(sb.ConfigError):
                sb.UsbCdcDeviceLink()

            class BadPort(FakePort):
                def read(self, n):
                    raise OSError("ReadFile failed")

            pc.Win32Port = BadPort
            bad = sb.UsbCdcDeviceLink("COM10")
            with self.assertRaises(ConnectionError):
                bad.recv()
        finally:
            pc.find_usb_port = real_find
            pc.Win32Port = real_port
        self.assertEqual(pc.OPEN_COUNT, OPEN_BEFORE)

    def test_tcp_loopback_device_without_serial(self):
        box = {}

        def accept_one(server):
            conn, _addr = server.accept()
            box["conn"] = conn
            data = b""
            conn.settimeout(2)
            while data != frame(pc.PING):
                chunk = conn.recv(64)
                if not chunk:
                    break
                data += chunk
            box["got"] = data
            conn.sendall(frame(pc.PONG))

        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        server.settimeout(3)
        device_port = server.getsockname()[1]
        thread = threading.Thread(target=accept_one, args=(server,))
        thread.start()
        try:
            self.start(lambda: sb.TcpDeviceLink("127.0.0.1", device_port, timeout=0.2))
            code, body = self.post({"cmd": "ping"}, timeout=3)
            self.assertEqual(code, 200)
            self.assertEqual(body["state"], "acked")
            self.assertEqual(body["response"]["kind"], "pong")
            thread.join(timeout=2)
            self.assertEqual(box["got"], bytes.fromhex("A500018F"))
        finally:
            thread.join(timeout=1)
            conn = box.get("conn")
            if conn:
                conn.close()
            server.close()

    def test_static_page_has_local_bridge_only(self):
        """控制台页面与资源必须全部本地化：无 CDN、无外链、只走桥接自身 API。"""
        link = sb.LoopbackDeviceLink()
        self.start(lambda: link)
        status, html = http_text(self.base + "/")
        self.assertEqual(status, 200)
        # 根路径现在就是唯一的控制台
        self.assertIn("viewer.js", html)
        self.assertIn("estop.js", html)
        _, js = http_text(self.base + "/viewer.js")
        _, estop = http_text(self.base + "/estop.js")
        _, css = http_text(self.base + "/style.css")
        for text in (html, js, estop, css):
            lowered = text.lower()
            self.assertNotIn("http://", lowered)
            self.assertNotIn("https://", lowered)
            self.assertNotIn("cdn", lowered)
        self.assertIn("style.css", html)
        # 指令必须走桥接自身的 /api/command，不能有第三个出口
        self.assertIn("/api/command", js)
        self.assertIn("/api/command", estop)
        self.assertIn("/api/status", js)
        self.assertNotIn("45111", js)
        self.assertNotIn("COM", js)
        # 控制台必须自带这些关键 UI 与能力（防回归：别又把干跑当成唯一模式）
        for needle in ("关节", "求解 IK", "急停", "rel_rotate", "setRealMode",
                       "mode-toggle", "estop-btn", "device_reply"):
            self.assertIn(needle, html + js)
        # 磁盘上的文件也必须干净（防有人改完服务没同步磁盘）
        disk_html = (ROOT / "web" / "viewer.html").read_text(encoding="utf-8")
        disk_js = (ROOT / "web" / "viewer.js").read_text(encoding="utf-8")
        disk_estop = (ROOT / "web" / "estop.js").read_text(encoding="utf-8")
        for text in (disk_html, disk_js, disk_estop):
            self.assertNotIn("http://", text.lower())
            self.assertNotIn("https://", text.lower())


if __name__ == "__main__":
    unittest.main(verbosity=2)
