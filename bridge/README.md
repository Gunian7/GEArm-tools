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
{"cmd": "arm", "arg": "rel_rotate 0 5"}
{"cmd": "arm", "arg": "jstat"}
{"cmd": "arm", "arg": "estop"}
```

- **`arm`**：唯一会把字节发给设备的命令。`arg` 是一行文本，由 `pc_service.build_arm_command()`
  按 `ARM_CMDS` 白名单校验后才变成字节。当前白名单：
  `auto, estop, estop_clear, hard_reset, jstat, probe, qstat, rel_rotate,
  remote_disable, remote_enable, remote_event, soft_reset, zero`。
  **新增命令必须同时改固件 `robot_cmd.c` 的 `robot_uart1_cmd_table` 和 `pc_service.py` 的
  `ARM_CMDS`**，否则这里以 400 拒绝。
- 只允许字段 `cmd`、`arg`。`wait` 缺省为 true，会等待设备响应或超时。

成功为 200，例如：

```json
{"ok": true, "seq": 1, "cmd": "arm", "arg": "jstat", "state": "acked", "response": {"kind": "lines"}, "error": null}
```

`state`：`queued`、`sent`、`acked`、`device_error`、`timeout`、`transport_lost`、`cancelled`。

序号从 1 递增。同一时刻只向设备发出一条未完成命令。响应写回该序号的 `response`。

白名单外、非法参数或多余字段返回 400：

```json
{"ok": false, "error": "rejected", "cmd": "arm", "arg": "reboot"}
```

同时写日志 `rejected command`。这些请求不会变成字节，也不会发给设备。

其它只读端点（都不碰设备、不产生运动）：

| 端点 | 用途 |
|---|---|
| `GET /api/status` | 连接状态、遥测、命令与拒绝计数 |
| `GET /api/model` | URDF 链与各关节行程范围，供 viewer 画限位 |
| `POST /api/kinematics` | FK/IK，纯计算，无设备访问 |
| `GET /api/episodes` | 已录制的 episode 列表 |
| `POST /api/episode/<action>` | 录制 / 停止 / 回放，`action` 为 `start`/`stop`/`load` |
| `GET /api/episode/frames` | 某个 episode 的逐帧数据 |

`GET /`、`/viewer.html`、`/viewer.js`、`/estop.js`、`/style.css`、`/vendor/three.module.js`
及 `vendor/addons/` 下两个加载器是本地静态页，没有构建步骤，不连 CDN。

`WS /ws` 推送 JSON，`kind` 为 `status`、`telemetry` 或 `command`。

断线后自动重连。`last_error` 和 `retry_count` 可见。还没发出的命令留在队列里，重连后发送。已经发出但没有响应的命令标成 `transport_lost`，序号仍在，不静默丢掉。

## 安全边界

- 只监听 `127.0.0.1`。传入 `0.0.0.0` 会在绑定前拒绝。
- 不烧录，不复位，不运行 JLink。唯一的外部动作是打开板子的 USB CDC COM 口。
- 发往设备的命令只有 `arm` 一种，且 `arg` 必须通过 `pc_service.build_arm_command()`
  的 `ARM_CMDS` 白名单校验。运动、急停等真实动作由**固件侧**的软件限位、
  `robot_can_is_present()` 总线门禁和急停命令负责，桥接层不做二次判断。
- 组帧和解析复用上级目录 `pc_service.py` 的 `crc8`、`build_frame`、`Parser`，
  没有另写一份协议。`build_frame` 只接受 `TX_WHITELIST`（PING/QUERY），
  `arm` 走的是 `build_arm_command()` 这条独立路径，不经过 `TX_WHITELIST`。
- 页面只请求当前主机的 `/api` 和 `/ws`，没有 CDN，不直连设备。
- 同一时刻只有一个 `DeviceLink`。`UsbCdcDeviceLink` 只开板子的 USB CDC 口；
  `TcpDeviceLink` 只连接 `127.0.0.1`；测试注入 `LoopbackDeviceLink`。
- `plan_runtime()` 是纯规划，不会打开端口；端口只在桥接线程里由 `DeviceLink` 工厂打开。

## 验证状态

**已验证**
- 26 个 API 测试全过（`python -m unittest test_bridge_api`），无串口、无硬件依赖
- 真实链路已打通：Web 发 `rel_rotate` 能驱动电机，`g_dbg_can_tx_ok` 随命令增长
- 只读诊断 `jstat` / `qstat` / `probe` 可安全反复调用

**未验证 / 阻塞**
- USB CDC **只发不收**：设备枚举正常，但 PC→板子的 `WriteFile` 失败。
  已排除非线、非 bridge 逻辑、非缓冲区、非 GPIO 顺序，需示波器看 D+/D- 物理信号。
- 浏览器实测由经理集成测试负责
