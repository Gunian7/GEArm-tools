# MC02 STM32H723VGT6 USB CDC 只读遥测（阶段0）

本目录是独立测试固件。`GEArm-H723-Firmware` 与 `communication-test` 只作只读引用，没有修改。实际执行了主机 C 测试、`pc_service.py --selftest`，以及 CubeIDE ARM GCC 13.3.1 编译/链接。主机假 transport 的成功不代表硬件成功。

**链路（2026-10-27 起）**：板子经 USB 枚举为 `VID 0483 / PID 5740`（CDC ACM），Windows 装成虚拟 COM 口，这是**主控制链路**。原来的 CH340/USART1 只是备份，波特率对 USB CDC 无意义。PC 侧省略端口时自动识别（`pc_service.find_usb_port()`，读注册表 `Enum\USB\VID_0483&PID_5740` 的 `FriendlyName`）。

## 帧格式

字节顺序：`0xA5` + `LEN` + `TYPE` + `PAYLOAD` + `CRC8`。

- `LEN` 只计 payload，范围 0..48。TYPE 单独占 1 字节，不计入 LEN。
- CRC-8：多项式 `0x07`，初值 `0x00`，不反射，结果不异或。覆盖从 SOF 到 payload 末字节，不含 CRC 自身。
- 固定校验：ASCII `123456789` 的 CRC 为 `0xF4`。PING 空载荷帧为 `A5 00 01 8F`。
- 非 `0xA5` 的失步字节直接丢弃。SOF 之后长度大于 48、或 CRC 不符，回错误帧并重新同步；若出错的那个字节本身是 `0xA5`，它作为下一帧帧头。合法 payload 内的 `0xA5` 不会提前切帧。

| TYPE | 方向 | 载荷 | 行为 |
|---|---|---|---|
| `0x01` PING | 主机到设备 | 空 | 回 `0x11` PONG，不碰 CAN |
| `0x02` QUERY | 主机到设备 | 1 字节：0 版本 / 1 电压 / 2 状态 | 只读 CAN 查询 |
| `0x03` TELEMETRY_RATE | 主机到设备 | 1 字节，1..100 | 遥测频率，周期毫秒为 `1000/Hz` 的整数除法 |
| `0x04` STOP | 主机到设备 | 空 | 停止遥测；若有在途只读查询则中止 TX 并回到监听。不是电机立即停止 |
| `0x10` TELEMETRY | 设备到主机 | 42 字节 | 周期遥测，上电默认 10 Hz |
| `0x12` ACK | 设备到主机 | `[命令, 参数]` | 白名单命令已接受 |
| `0x7F` ERROR | 设备到主机 | `[原 TYPE 或 0, 错误码]` | 拒绝，不执行 |

错误码：1 CRC，2 长度，3 未知功能码，4 参数越界，5 运动/使能/写参数类功能码，6 CAN 忙，7 已故障，8 查询超时，90 停止传输失败，91 入队失败。

遥测 payload 小端：

| 偏移 | 字段 |
|---:|---|
| 0 | heartbeat，主循环次数，不是毫秒 |
| 4 | init_stage：1 HAL 前，2 时钟，3 CAN 监听，4 USART 就绪 |
| 8 | CAN TX 入队 |
| 12 | CAN TX 完成 |
| 16 | CAN RX |
| 20 | 白名单匹配 |
| 24 | 超时 |
| 28 | error |
| 32 | 最近查询功能码 |
| 33 | 原始回包长度 |
| 34 | 原始回包 8 字节，有效长度见上一字节 |

## 白名单与拒绝

链路命令只接受上表四种。QUERY 参数只映射：

| 参数 | CAN 功能码 | 期望回包 DLC |
|---:|---:|---|
| 0 版本 | `0x1F` | 4 或 6 |
| 1 电压 | `0x24` | 4 |
| 2 状态 | `0x3A` | 3 |

CAN 侧与 communication-test 相同，不扩大：地址 1，扩展 ID `0x100`，经典数据帧，500 kbit/s，固定校验 `0x6B`，2 字节 `[功能码, 0x6B]`。硬件过滤只收 `0x100`，软件再核对 ID、扩展帧、非远程、非 FD、非 BRS、DLC 与校验。其它功能码没有发送函数。

下列 Emm V5 功能码作为链路 TYPE 时一律回错误码 5，CAN 发送计数不变：`0x0A` 当前位置清零，`0x0E` 解除堵转，`0x46` 改控制模式，`0x4C` 改回零参数，`0x93` 设零点，`0x9A` 触发回零，`0x9C` 中断回零，`0xF3` 使能，`0xF6` 速度，`0xFD` 位置，`0xFE` 立即停止，`0xFF` 同步运动。`0x20/0x21/0x33/0x35/0x36` 等非白名单读取同样拒绝，不转发到 CAN。

`pc_service.py` 比固件更窄：只组 PING 和 QUERY。RATE、STOP 和运动码在组帧前抛出，不会变成字节。

## 引脚、时钟、接收方式

复位入口沿用 communication-test 的 24 MHz HSE：PLL M3/N50/P2/Q4/R2，SYSCLK 200 MHz，AHB 100 MHz，APB 50 MHz，FDCAN 内核 100 MHz，10×(1+15+4) 得到 500 kbit/s。USART1 内核时钟显式选 D2 PCLK2（50 MHz），115200 8N1，16 倍过采样。这些是 HAL 参数检查，不是示波器测量。

GPIO 只写两处：PD0/PD1 AF9（FDCAN1），PA9/PA10 AF7（USART1_TX/RX）。不写 PB14/PB15，不写 PB10/PB11、PA13/PA14、PH0/PH1、PC13/PC14/PC15。

USART1 RX 用中断，不用轮询，也不用 DMA。115200 下约 87 µs 一个字节，主循环还要排空 CAN FIFO，HAL 调用可能超过一个字节时间。中断里只把字节放进 256 字节环；解析、拒绝命令和 CAN 发送都在主循环，避免在中断里重入 CAN。TX 只在主循环轮询发送。

无机器人、无 RTOS、无运动任务。`runtime.c` 只提供 `memset`/`memcpy` 和空的 C 构造入口。

## 构建

```
C:/Users/ASUS/AppData/Local/Programs/Python/Python313/python.exe build.py
C:/Users/ASUS/AppData/Local/Programs/Python/Python313/python.exe pc_service.py --selftest
```

`build.py` 先跑主机测试，再 ARM 编译链接，写 `result.json` 与 SHA256。自己的 C 使用 `-Wall -Wextra -Werror`，链接使用 `--fatal-warnings`。产物是 `build/mc02_serial_test.elf/.hex/.bin/.map`。不要给 `pc_service.py` 传入真实 COM 口，除非另一次经过授权的现场测试；`--selftest` 不调用 `CreateFile`。

## 已验证

- 第一轮桩实现主机测试：`HOST_TESTS pass=35 fail=68`。
- 协议实现后主机测试：`HOST_TESTS pass=104 fail=0`。`build.py` 再次运行结果相同，见 `result.json`。
- 覆盖：白名单接受、参数边界、CRC 错误、失步恢复、遥测构造、运动类功能码负例、CAN ID/帧类型/DLC 负例、超时与入队失败。
- ARM GCC 13.3.1 实际编译并链接，`nm` 可见 `link`、`USART1_IRQHandler`、`SysTick_Handler`、`FDCAN1_IT0_IRQHandler`、`FDCAN1_IT1_IRQHandler`。
- `pc_service.py --selftest` 打印 `PC_SELFTEST pass`。虚拟设备只记录 `1F/24/3A + 6B`，运动帧被拒绝。本次没有打开 COM 口。

## 未验证

真实启动、HSE 锁定、USB CDC 枚举与 COM 号分配、CAN 真实回包。另有 VOS/Flash 时序和 TX 取消时序未在硬件上测量。必须由管理者另行授权后才能做现场测试。本目录不提供烧录或 JLink 命令。
