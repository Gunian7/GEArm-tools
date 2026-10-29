# GEArm 设备桥接（USB CDC）

软件侧链路。桥接进程持有设备连接，网页只访问本机 HTTP/WebSocket。本目录都是新增文件，没有修改 serial-test 下已有源码。

## 启动

```
python.exe serial_bridge.py
python.exe serial_bridge.py --port COM10 --http-port 8765
python.exe serial_bridge.py --transport tcp --tcp 127.0.0.1:45111   # 软件替身，无硬件
```

- HTTP 只绑定 `127.0.0.1`。默认页面是 `http://127.0.0.1:8765/`。
- **`--transport usb`（默认）走板子的 USB CDC 虚拟串口**。板子枚举为 `VID 0483 / PID 5740`（usbser），Windows 装成虚拟 COM 口。
- **`--port` 省略时自动识别**该设备的 COM 号（读注册表 `Enum\USB\VID_0483&PID_5740` 的 `FriendlyName`，例如 `USB 串行设备 (COM10)`）。识别不到会以 `reconnecting` 重试，`last_error` 里说明原因。
- `--vid` / `--pid` 可覆盖默认 `0x0483` / `0x5740`，写成 `0x1234` 或十进制。
- `--transport tcp` 是**软件替身**模式：连 `127.0.0.1:45111`，完全不碰硬件，用于自动化测试。`--tcp` 只接受 `127.0.0.1` 或 `localhost`。
- USB CDC 虚拟串口的**波特率没有意义**，固件忽略；别用波特率扫描查 USB 链路。

测试：

```
C:\Users\ASUS\AppData\Local\Programs\Python\Python313\python.exe test_bridge_api.py
```

## API

`GET /api/status`

返回 `connection`、`retry_count`、`last_error`、`telemetry`、`commands`、`rejections`。

`connection` 取值：`connecting`、`connected`、`reconnecting`、`disconnected`、`stopped`。

`telemetry` 来自 `pc_service.unpack_telemetry`：`heartbeat`、`init_stage`、`tx_enqueued`、`tx_complete`、`rx_count`、`matched`、`timeouts`、`error`、`last_fn`、`last_raw_len`、`last_raw`。`last_raw` 是按有效长度截断后的小写十六进制。

`POST /api/command`

```json
{"cmd": "ping"}
{"cmd": "query", "arg": 0}
{"cmd": "rate", "arg": 10}
{"cmd": "stop", "wait": false}
```

- `ping`：无参数。
- `query`：`arg` 为 0 版本、1 电压、2 状态。
- `rate`：`arg` 为 1..100，单位 Hz。
- `stop`：无参数。这是遥测停止 `0x04`，不是电机急停 `0xFE`。
- 只允许字段 `cmd`、`arg`、`wait`。`wait` 缺省为 true，会等待设备响应或超时。

成功为 200，例如：

```json
{"ok": true, "seq": 1, "cmd": "ping", "arg": null, "state": "acked", "response": {"kind": "pong"}, "error": null}
```

`state`：`queued`、`sent`、`acked`、`device_error`、`timeout`、`transport_lost`、`cancelled`。

序号从 1 递增。同一时刻只向设备发出一条未完成命令。PONG 对应 ping；ACK 和 ERROR 的第一个载荷字节对应 query、rate、stop。响应写回该序号的 `response`。

白名单外、非法参数或多余字段返回 400：

```json
{"ok": false, "error": "rejected", "cmd": "enable"}
```

同时写日志 `rejected command`。这些请求不会变成字节，也不会发给设备。

`GET /`、`/app.js`、`/style.css` 是本地静态页，没有构建步骤。

`WS /ws` 推送 JSON，`kind` 为 `status`、`telemetry` 或 `command`。

断线后自动重连。`last_error` 和 `retry_count` 可见。还没发出的命令留在队列里，重连后发送。已经发出但没有响应的命令标成 `transport_lost`，序号仍在，不静默丢掉。

## 安全边界

- 只监听 `127.0.0.1`。传入 `0.0.0.0` 会在绑定前拒绝。
- 不烧录，不复位，不运行 JLink。唯一的外部动作是打开板子的 USB CDC COM 口。
- 命令只有 ping、query、rate、stop。运动、使能、速度、位置、回零、写参数在桥接层拒绝。
- 组帧和解析复用上级目录 `pc_service.py` 的 `Parser`、`unpack_telemetry`、`crc8`、`build_frame`，没有另写一份协议。
- `pc_service.TX_WHITELIST` 只有 PING 和 QUERY。桥接在锁内临时把 RATE 或 STOP 加入该集合后调用 `build_frame`，`finally` 恢复原集合。运动类型不会被加入。
- 页面只请求当前主机的 `/api` 和 `/ws`，没有 CDN，不直连设备。
- 同一时刻只有一个 `DeviceLink`。`UsbCdcDeviceLink` 只开板子的 USB CDC 口；`TcpDeviceLink` 只连接 `127.0.0.1`。测试注入 `LoopbackDeviceLink`。
- `plan_runtime()` 是纯规划，不会打开端口；端口只在桥接线程里由 `DeviceLink` 工厂打开。

## 未验证

- 真实设备实测（插回 USB 后跑 `hw_probe.py` 或 `python ../pc_service.py`）
- 浏览器实测由经理集成测试负责

自动化测试没有打开 COM 口，也没有跑浏览器。WebSocket 只用脚本客户端验证过。USB 设备未插时桥接会一直重试，这不代表设备可用。
