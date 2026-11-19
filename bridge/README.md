# GEArm 设备桥接（USB CDC）

软件侧链路。桥接进程持有设备连接，网页只访问本机 HTTP/WebSocket。本目录是新增目录；Ubuntu 24.04 迁移另外改了上级目录的 `pc_service.py`（平台抽象）并新增了 `start_bridge.sh`。

## 启动

```bat
py -3 serial_bridge.py
py -3 serial_bridge.py --port COM10 --http-port 8765
py -3 serial_bridge.py --transport tcp --tcp 127.0.0.1:45111   # 软件替身，无硬件
```

Ubuntu 24.04（在仓库根目录执行 `./start_bridge.sh` 等价于下面第一条）：

```bash
python3 serial_bridge.py
python3 serial_bridge.py --port /dev/ttyACM0 --http-port 8765
python3 serial_bridge.py --port /dev/serial/by-id/<板子的稳定链接>
```

- HTTP 只绑定 `127.0.0.1`。默认页面是 `http://127.0.0.1:8765/`。
- **`--transport usb`（默认）走板子的 USB CDC 串口**。板子枚举为 `VID 0483 / PID 5740`：Windows 上 usbser 装成 `COM10`，Linux 上 cdc_acm 装成 `/dev/ttyACM0`。
- **`--port` 省略时自动识别**该设备的串口名：Windows 读注册表 `Enum\USB\VID_0483&PID_5740` 的 `FriendlyName`（例如 `USB 串行设备 (COM10)`）；Linux 先扫 `/dev/serial/by-id/`，再扫 `/sys/class/tty/ttyACM*/device` 的 `idVendor`/`idProduct`。两条路径都只查路径与属性，不打开端口。识别不到会以 `reconnecting` 重试，`last_error` 里说明原因。
- 选型与开关都在 `pc_service`：`open_serial_port()` 在 `os.name == "nt"` 时用 `Win32Port`（ctypes + kernel32，行为与迁移前完全一致），其余平台用 `PosixPort`（`os.open` + `termios`，raw 115200 8N1）。
- `--vid` / `--pid` 可覆盖默认 `0x0483` / `0x5740`，写成 `0x1234` 或十进制。
- `--transport tcp` 是**软件替身**模式：连 `127.0.0.1:45111`，完全不碰硬件，用于自动化测试。`--tcp` 只接受 `127.0.0.1` 或 `localhost`。
- USB CDC 虚拟串口的**波特率没有意义**，固件忽略；别用波特率扫描查 USB 链路。

测试：

```bat
py -3 test_bridge_api.py            # Windows
```

```bash
python3 test_bridge_api.py          # Ubuntu 24.04
```

共 31 个测试：26 个原 API 测试 + 5 个平台分支/设备发现测试。后者在 Windows 上也会跑
（用临时目录伪造 sysfs 树，不碰真实设备）。

## Ubuntu 24.04

- **无第三方依赖**：串口走标准库 `os.open` + `termios`，设备发现走 sysfs，
  不需要 `pyserial` / `pyudev`，仓库里也没有 `requirements.txt`。
- **权限**：`/dev/ttyACM0` 默认属于 `dialout` 组，不在组里会 `Permission denied`：

  ```bash
  sudo usermod -aG dialout "$USER"     # 重新登录（或 newgrp dialout）后生效
  ```

- **两种端口写法**：`--port /dev/ttyACM0` 用的是内核枚举编号，插拔别的串口设备后可能
  变化；`--port /dev/serial/by-id/<稳定链接>` 指向同一个设备且不会变，建议优先用。
  链接名由板子的 USB 描述符决定，形如 `usb-STMicroelectronics_...-if00`，**确切名字待实机确认**。
- **启动**：仓库根目录的 `./start_bridge.sh [参数]` 会先 `cd` 到脚本所在目录再
  `exec python3 bridge/serial_bridge.py "$@"`，从任何目录调用都可以。
- **待实机确认**：`PosixPort` 在真实 `/dev/ttyACM0` 上的 raw 115200 8N1 收发、
  短写/EAGAIN 的实际行为，以及 `/dev/serial/by-id` 链接名。本机是 Windows，
  只验证到平台选型与设备发现的纯路径分支。

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
- 31 个测试全过（`python -m unittest test_bridge_api`）：26 个原 API 测试，加 5 个
  `os.name` 选型 / POSIX 设备发现测试（在 Windows 上也会跑），无串口、无硬件依赖
- Windows 路径未被破坏：`py -3 pc_service.py --selftest` 输出 `PC_SELFTEST pass`
  （仍包含 DCB=28 / COMMTIMEOUTS=20 的尺寸断言）
- 真实链路已打通：Web 发 `rel_rotate` 能驱动电机，`g_dbg_can_tx_ok` 随命令增长
- 只读诊断 `jstat` / `qstat` / `probe` 可安全反复调用

**未验证 / 阻塞**
- USB CDC **只发不收**：设备枚举正常，但 PC→板子的写失败（Windows 上是 `WriteFile`）。
  已排除非线、非 bridge 逻辑、非缓冲区、非 GPIO 顺序，需示波器看 D+/D- 物理信号。
- **Ubuntu 24.04 上的真实收发未测**（待实机确认）：本机是 Windows，`PosixPort` 的
  termios 配置只在代码与 `--selftest` 常量断言层面验证过。
- 浏览器实测由经理集成测试负责
