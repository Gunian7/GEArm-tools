# serial-test：GEArm PC 侧链路

> 修订：2026-11-04 ｜ 定位从"独立测试固件（阶段0）"变更为**主控制链路的 PC 侧**
> 历史：早期诊断固件源码（`main.c` / `protocol.c` / `tests.c` / `virtual_mc02.c` /
> `kinematics_host.c` / `build*.py` 等）与一次性扫描联调脚本已于 2026-11-04 删除；
> 链路联调职责由生产固件 `GEArm-H723-Firmware` 接管。
> 原 README（描述那套 `0xA5+LEN+TYPE+CRC8` 诊断协议）已存入
> `F:/Output/Project_Launch/GEArm-doc-backup-20261104.tar.gz`。

## 组成

| 路径 | 职责 |
|---|---|
| `bridge/serial_bridge.py` | USB CDC 链路管理 + HTTP/WS 服务 + 命令白名单校验 |
| `bridge/web/` | Web UI：`viewer.html` / `viewer.js` / `style.css` / `estop.js` / `vendor/three.module.js` |
| `bridge/test_bridge_api.py` | 31 个测试（26 个原 API 测试 + 5 个平台分支/设备发现测试），无串口、无硬件依赖 |
| `start_bridge.sh` | Ubuntu/POSIX 启动脚本 |
| `pc_service.py` | 独立 PC 服务，端口 8765 |
| `arm-model/` | `arm.urdf` + STL meshes，`/arm/` 路由 + URDF 正运动学 |
| `episode_recorder.py` | 行为克隆录制，输出 LeRobot 兼容 JSONL |
| `urdf_kinematics.py` | PC 侧 FK/IK，`observation.state` 的 TCP 坐标由它算出 |
| `export_lerobot.py` | 把录制的 JSONL 导出为 LeRobot dataset |

## 启动

Windows（本机 Python 3 走 `py -3`，装进 PATH 的 `python` 也可）：

```bat
py -3 pc_service.py                             # 8765
py -3 bridge/serial_bridge.py                   # 自动识别 USB CDC 串口
py -3 bridge/serial_bridge.py --port COM10      # 手动指定
```

Ubuntu 24.04 / 任意 POSIX：

```bash
./start_bridge.sh                               # 自动识别 /dev/ttyACM0
./start_bridge.sh --port /dev/ttyACM0           # 手动指定
./start_bridge.sh --port /dev/serial/by-id/<板子的稳定链接>
python3 pc_service.py --selftest                # 自检，不开串口
```

板子经 USB 枚举为 `VID 0483 / PID 5740`（CDC ACM）：Windows 装成虚拟串口
（`COM10`），Linux 装成 `/dev/ttyACM0`。这是**主控制链路**；CH340/USART1 只是
备份，波特率对 USB CDC 无意义。省略端口时自动识别（`pc_service.find_usb_port()`）：
Windows 读注册表 `Enum\USB\VID_0483&PID_5740` 的 `FriendlyName`，Linux 先扫
`/dev/serial/by-id/`，再扫 `/sys/class/tty/ttyACM*/device` 的 `idVendor`/`idProduct`。
两条路径都只做路径/属性查询，**不会打开端口**。

## Ubuntu 24.04

- **无第三方依赖**：串口走标准库 `os.open` + `termios`，设备发现走 sysfs，
  不需要 `pyserial` / `pyudev`，仓库里也没有 `requirements.txt`。
- **权限**：`/dev/ttyACM0` 默认属于 `dialout` 组，不在组里会 `Permission denied`：

  ```bash
  sudo usermod -aG dialout "$USER"     # 重新登录（或 newgrp dialout）后生效
  ```

- **两种端口写法**：`/dev/ttyACM0` 的编号由内核枚举顺序决定，插拔别的串口设备后
  可能变化；`/dev/serial/by-id/` 下是指向同一个设备的稳定链接，建议优先用后者
  （链接名由板子的 USB 描述符决定，形如 `usb-STMicroelectronics_...-if00`，
  **确切名字待实机确认**）。
- **启动**：`./start_bridge.sh` 等价于 `exec python3 bridge/serial_bridge.py "$@"`。
- **自检**：`python3 pc_service.py --selftest` 应输出 `PC_SELFTEST pass`（不碰硬件）。
- **测试**：`cd bridge && python3 -m unittest test_bridge_api`。
- **待实机确认**：真实 `/dev/ttyACM0` 上的收发（见下方“已知问题”），以及
  `/dev/serial/by-id` 链接的确切名字。

## 测试

```bash
cd bridge && python -m unittest test_bridge_api
# 预期: Ran 31 tests ... OK
```

Windows 上用 `py -3` 代替 `python`。

## 命令白名单

`pc_service.py` 的 `ARM_CMDS` 集合。**新增命令必须同时改两处**，否则 PC 侧以 400 拒绝：

1. 固件 `GEArm-H723-Firmware/Core/Src/robot_cmd.c` 的 `robot_uart1_cmd_table`
2. `pc_service.py` 的 `ARM_CMDS`

当前白名单：
`auto, estop, estop_clear, hard_reset, jstat, probe, qstat, rel_rotate,
remote_disable, remote_enable, remote_event, soft_reset, zero`

## 数据采集（行为克隆）

`episode_recorder.py` 把操作过程录成 JSONL（一帧一行），schema 对齐 LeRobot；
`urdf_kinematics.py` 提供 PC 侧 FK/IK，`observation.state` 里的 3 个 TCP 坐标由它算出。
`export_lerobot.py` 把录制的 JSONL 转成 LeRobot dataset 格式（parquet + meta）。
产出在 `episodes/`（原始 JSONL）与 `dataset/`（导出结果）。

Web 界面的录制入口在 `bridge/web/viewer.js`，HTTP 端点是 `/api/episode/*`。
测试：`python -m unittest tests_episode_recorder tests_urdf_kinematics tests_export_lerobot`

`kinematics.py`（DH 表解算交叉校验，调用 `build/kinematics_host.exe`）已于 2026-11-04
随诊断固件一并删除；bridge 的 `_dh_kin()` 以 try/except 包裹，缺 exe 时返回 `None`
即"交叉校验不可用"，FK/IK 主路径不受影响。

## 已知问题

USB CDC **只发不收**：设备枚举正常（Windows 上出现 COM 口、Linux 上出现
`/dev/ttyACM0`，`g_cdc_rx_lines` 有计数），但 PC→板子的写失败（Windows 上是
`WriteFile` 返回错误）。已排除非线、非 bridge 逻辑、非缓冲区、非 GPIO 顺序，
需示波器看 D+/D- 物理信号才能继续。
