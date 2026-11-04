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
| `bridge/test_bridge_api.py` | 26 个 API 测试（无串口、无硬件依赖） |
| `pc_service.py` | 独立 PC 服务，端口 8765 |
| `arm-model/` | `arm.urdf` + STL meshes，`/arm/` 路由 + URDF 正运动学 |
| `episode_recorder.py` | 行为克隆录制，输出 LeRobot 兼容 JSONL |
| `urdf_kinematics.py` | PC 侧 FK/IK，`observation.state` 的 TCP 坐标由它算出 |
| `export_lerobot.py` | 把录制的 JSONL 导出为 LeRobot dataset |

## 启动

```bash
python pc_service.py              # 8765
python bridge/serial_bridge.py    # 自动识别 USB CDC 虚拟 COM 口
```

板子经 USB 枚举为 `VID 0483 / PID 5740`（CDC ACM），Windows 装成虚拟 COM 口，
这是**主控制链路**；CH340/USART1 只是备份，波特率对 USB CDC 无意义。
省略端口时自动识别（`pc_service.find_usb_port()` 读注册表
`Enum\USB\VID_0483&PID_5740` 的 `FriendlyName`）。

## 测试

```bash
cd bridge && python -m unittest test_bridge_api
# 预期: Ran 26 tests ... OK
```

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

USB CDC **只发不收**：设备枚举正常（COM 口出现、`g_cdc_rx_lines` 有计数），
但 PC→板子的 `WriteFile` 失败。已排除非线、非 bridge 逻辑、非缓冲区、非 GPIO 顺序，
需示波器看 D+/D- 物理信号才能继续。
