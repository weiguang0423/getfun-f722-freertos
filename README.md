<!--
文件作用：项目顶层 README，面向新接触本仓库的开发者，用一页给出项目目标、硬件平台、分层架构、构建烧录、当前进度入口和关键约束。
覆盖范围：GETFUN F722 V3 FreeRTOS 飞控固件的整体介绍；详细的实时进度以 Docs/00 为准，本文只做摘要并指向各专题文档。
关联模块：APP 应用层（MSP 服务器、app_state、usb_cdc_transport、ImuTask/MspTask/RcTask、ICM42688P、校准、姿态、CRSF、参数存储）。
仍需实物验证项：本文中“当前进度”一节的状态标记与 Docs/00 同步；涉及真实硬件的结论（电机映射、VBAT 标定、首飞等）尚未完成，以 Docs/00 的关口为准。
-->

# GETFUN F722 V3 — FreeRTOS 飞控固件

![MCU](https://img.shields.io/badge/MCU-STM32F722RET6-CYAN)
![Core](https://img.shields.io/badge/Core-Cortex--M7%20%40216MHz-blue)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS%2010.2.1-orange)
![Language](https://img.shields.io/badge/Language-C11-informational)
![Status](https://img.shields.io/badge/status-WIP%20%E6%9C%AA%E9%A3%9E%E8%A1%8C-yellow)

> 一套基于 STM32F722 + FreeRTOS 的自研飞控固件。目标是在 Betaflight 硬件之上，
> 把板子**伪装成一个 Betaflight PID 控制器**，让 Betaflight Configurator / Betaflight App
> 通过 USB CDC 虚拟串口正常连接——而 PID、Mixer、ARM/Failsafe、电机安全链等核心
> 逻辑由本项目自主实现并完全掌握。

> **当前状态（2026-08-06）**：主线位于 `S3 IMU、姿态与飞行输入`，小目标 `S3.8 /
> v0.9.0-rc-failsafe-app-baseline`，**软件已完成，待真实硬件与 App 验收**。
> 最新冻结基线：[`v0.8.0-crsf-rc-baseline`](#-已冻结版本历史)。
> 本固件**尚未飞行**，电机闭环、DShot、ARM 状态机均未实现，请勿接桨。

---

## 目录

- [项目目标](#项目目标)
- [硬件平台](#硬件平台)
- [软件架构](#软件架构)
- [APP 应用层模块](#app-应用层模块)
- [目录结构](#目录结构)
- [构建](#构建)
- [烧录与调试](#烧录与调试)
- [开发路线与当前进度](#开发路线与当前进度)
- [关键设计约束](#关键设计约束)
- [添加新代码](#添加新代码)
- [第三方组件与许可证](#第三方组件与许可证)
- [文档索引](#文档索引)

---

## 项目目标

1. **App 兼容**：以最小代价让 Betaflight Configurator / Betaflight App 通过 USB CDC
   连上板子，能读到板子身份、姿态、电池、RC 通道等状态——复用 Betaflight 成熟的地面端
   UI，而不必自己写配置工具。
2. **核心自主**：任务调度、状态发布、参数系统、PID、Mixer、ARM/Failsafe、电机安全链
   全部由 `APP/` 自研，不把飞控安全行为交给第三方库。
3. **可验收、可冻结**：每个里程碑都必须经过 Debug/Release 构建 + 静态/运行时检查 +
   真实硬件验收，才能冻结为版本基线。

详细路线见 [Docs/00_项目开发路线与统一进度.md](Docs/00_项目开发路线与统一进度.md)。

---

## 硬件平台

| 项目 | 规格 |
|---|---|
| MCU | STM32F722RETx，Cortex-M7 @ 216 MHz，FPv5-SP-D16 |
| 存储器 | 512 KB Flash / 256 KB RAM；Flash 前 256 KB 为程序区，Sector 6/7 各 128 KB 保留为参数 A/B 槽 |
| RTOS | FreeRTOS Kernel V10.2.1（heap_4，15 KB 堆）+ CMSIS-RTOS V2 |
| 时钟 | HSE 8 MHz → PLL → 216 MHz，OverDrive 开启 |
| USB | USB Device CDC（虚拟串口） |
| IMU | ICM42688P，SPI1 + DMA2，1 kHz / ±2000 dps / ±16 g |
| RC 接收 | CRSF / ELRS，UART2 @ 420000 baud，DMA 循环接收 |
| 诊断串口 | UART4，1 Hz 摘要输出 |
| 电机输出 | 8 路 GPIO 安全输出（DShot 尚未实现） |

### 关键引脚（定义见 [Core/Inc/main.h](Core/Inc/main.h)）

| 功能 | 引脚 |
|---|---|
| 电机 1～8 | PA15 / PA10 / PA9 / PA8 / PC9 / PC8 / PB11 / PB10 |
| ICM42688P SPI1 | SCK=PA5，MISO=PA6，MOSI=PA7，CS=PA4 |
| CRSF UART2 | PA2 (TX) / PA3 (RX) |
| 诊断 UART4 | 见 [Core/Inc/usart.h](Core/Inc/usart.h) |

> 引脚/电气事实以 [Docs/01_*.md](Docs/)（硬件基线文档）为准；如需修改，先在
> [GETFUN_F722_FreeRTOS.ioc](GETFUN_F722_FreeRTOS.ioc) 中改，再用 CubeMX 重新生成。

---

## 软件架构

固件严格分成 **CubeMX 生成层** 与 **APP 自研应用层** 两部分，二者边界清晰：

| 层 | 目录 | 说明 |
|---|---|---|
| **应用（APP，自研）** | `APP/` | MSP 协议服务器、应用状态、USB CDC 传输、IMU/RC/姿态算法、RTOS 任务、参数存储 |
| CubeMX 应用 | `Core/Src/` | `main.c`（入口）、`freertos.c`（任务）、`gpio.c`、`spi.c`、`usart.c` |
| USB 设备 | `USB_DEVICE/App/` | CDC 虚拟串口，其接收 ISR 转发给 `usb_cdc_transport` |
| FreeRTOS | `Middlewares/Third_Party/FreeRTOS/` | 内核 + CMSIS-RTOS V2 封装 |
| USB 库 | `Middlewares/ST/STM32_USB_Device_Library/` | USB 设备核心与 CDC 类 |
| HAL 驱动 | `Drivers/STM32F7xx_HAL_Driver/` | STM32 HAL/LL 驱动 |
| CMSIS | `Drivers/CMSIS/` | Cortex-M 核心头与启动文件 |
| CubeMX 构建 | `cmake/stm32cubemx/` | CubeMX 生成的源/头/宏（**不含 APP/**） |

> **`APP/` 下的源文件登记在根 [CMakeLists.txt](CMakeLists.txt) 的 `add_executable`
> 列表里，不在 `cmake/stm32cubemx/CMakeLists.txt` 中**——所以 CubeMX 重新生成不会
> 动到它。新增 APP 源文件需手动加到根 `CMakeLists.txt`。

### 数据流（自底向上）

```
SPI1 + PA4 CS + DMA2 Stream 0/3
  └─ imu_bus -> icm42688p -> ImuTask (1 kHz DRDY门控, idle+4, 静态 512 words)
       ├─ DMA完成ISR捕获DWT CYCCNT -> platform_time -> 真实dt
       └─ SI + CW90 -> gyro_calibration -> accel_calibration
            ├─ parameter_store (Sector 6/7 A/B)
            ├─ 校准后未滤波 -> MSP_RAW_IMU
            └─ imu_filter PT1 -> filtered_* -> app_state_publish_imu()
                                                    │
                                                    ├─ attitude_estimator (Mahony) -> 欧拉角/四元数
                                                    │
UART2 + DMA1 Stream5 循环接收 (IDLE/HT/TC)
  └─ crsf_uart -> crsf 解析 (CRC-8/DVB-S2) -> rc_input (AETR映射) -> RcTask
       └─ app_state.rc (300ms Failsafe / 100ms+5帧恢复)
                                                    │
USB CDC 接收回调 (usbd_cdc_if.c, ISR 上下文)
  └─ usb_cdc_transport_receive_from_isr()        环形缓冲 + 任务通知
       │
MspTask (APP/Src/rtos/app_task.c, idle+2, 静态 768 words 栈)
  ├─ usb_cdc_transport_read()                      从环缓冲取字节
  ├─ msp_parser_process_byte()                     逐字节状态机解析 (MSP V1/V2)
  ├─ msp_server_process()                          按 command 派发，读 app_state 快照
  └─ msp_transport_build_response() + usb_cdc_transport_write()
```

`app_state_snapshot_t` 是全局运行态快照，多任务安全靠 **PRIMASK 关中断 + DMB** 的短临界区，
发布者必须短小，临界区内不得调任何可能阻塞的 API。

---

## APP 应用层模块

| 模块 | 路径 | 职责 |
|---|---|---|
| MSP 协议层 | [APP/Src/protocol/msp_transport.c](APP/Src/protocol/msp_transport.c) | 纯协议，无 RTOS 依赖。逐字节状态机解析 `$M<` V1 / `$X<` V2（V2 用 CRC8-DVB-S2），构造回包帧 |
| MSP 命令派发 | [APP/Src/protocol/msp_server.c](APP/Src/protocol/msp_server.c) | `MSP_FC_VARIANT` 返回 `"BTFL"`、`MSP_BOARD_INFO` 报 `GF72`；状态命令 + 校准 + Receiver 页面依赖 + GETFUN MSP2 `0x4000` 系列 |
| IMU 总线 | [APP/Src/bsp/imu_bus.c](APP/Src/bsp/imu_bus.c) | SPI1 适配，阻塞初始化 + 14 字节 DMA 采样 |
| ICM42688P 驱动 | [APP/Src/drivers/icm42688p.c](APP/Src/drivers/icm42688p.c) | 寄存器级驱动，固定 1 kHz / ±2000 dps / ±16 g |
| 陀螺校准 | [APP/Src/algorithms/gyro_calibration.c](APP/Src/algorithms/gyro_calibration.c) | 上电静态零偏 Welford 状态机，250 预热 + 2000 样本，不持久化 |
| 加速度校准 | [APP/Src/algorithms/accel_calibration.c](APP/Src/algorithms/accel_calibration.c) | 水平单面校准，偏置经 Flash 验证后才应用 |
| IMU 低通 | [APP/Src/algorithms/imu_filter.c](APP/Src/algorithms/imu_filter.c) | 三轴 PT1，按真实 dt 更新（Gyro 100 Hz / Accel 30 Hz） |
| 姿态解算 | [APP/Src/algorithms/attitude_estimator.c](APP/Src/algorithms/attitude_estimator.c) | 六轴 Mahony 四元数，FRD/NED 约定，驱动 App 三维模型 |
| CRSF 解析 | [APP/Src/protocol/crsf.c](APP/Src/protocol/crsf.c) | 长度 2～62 + CRC-8/DVB-S2 `0xD5`，解码 16 路 11-bit 通道与 Link Statistics |
| RC 输入 | [APP/Src/algorithms/rc_input.c](APP/Src/algorithms/rc_input.c) | AETR→App 逻辑顺序映射 + Failsafe 超时/恢复状态机 |
| CRSF UART | [APP/Src/bsp/crsf_uart.c](APP/Src/bsp/crsf_uart.c) | UART2 420000 baud DMA 循环接收 |
| 参数存储 | [APP/Src/storage/parameter_store.c](APP/Src/storage/parameter_store.c) | Sector 6/7 双槽，48 字节 v1 记录 + magic/version/CRC32/commit，序号选择 |
| 平台时基 | [APP/Src/platform/platform_time.c](APP/Src/platform/platform_time.c) | DWT 微秒时基，ISR 只读 CYCCNT，ImuTask 单写者做 32 位回绕扩展 |
| 平台诊断 | [APP/Src/platform/platform_diag.c](APP/Src/platform/platform_diag.c) | UART4 1 Hz 摘要 + 致命故障/参数保存前强制 Motor 1～8 低电平 |
| USB CDC 传输 | [APP/Src/bsp/usb_cdc_transport.c](APP/Src/bsp/usb_cdc_transport.c) | RX 1024 环缓冲 + 任务通知；TX 320 字节轮询 |
| 应用状态 | [APP/Src/app_state.c](APP/Src/app_state.c) | 全局快照 `app_state_snapshot_t`，关中断临界区 |
| RTOS 任务 | [APP/Src/rtos/](APP/Src/rtos/) | `app_task.c`（静态创建任务）、`imu_task.c`、`msp` 任务、`rc_task.c` |

---

## 目录结构

```
GETFUN_F722_FreeRTOS/
├── APP/                      # 自研应用层（用户区，不受 CubeMX 重生成影响）
│   ├── Inc/                  #   头文件（按 algorithms/bsp/drivers/protocol/rtos/storage/platform 分层）
│   └── Src/                  #   实现
├── Core/                     # CubeMX 生成：main / freertos / gpio / spi / usart / dma
├── Drivers/                  # HAL + CMSIS（CubeMX 生成）
├── Middlewares/              # FreeRTOS + ST USB Device Library（CubeMX 生成）
├── USB_DEVICE/               # USB Device CDC（CubeMX 生成）
├── cmake/
│   ├── gcc-arm-none-eabi.cmake   # 工具链文件
│   └── stm32cubemx/          # CubeMX 生成的 CMake 配置（不含 APP/）
├── Docs/                     # 设计/路线/调试/验收文档（00 是唯一实时进度源）
├── Tools/                    # 辅助脚本（如 verify_rc_failsafe.mjs）
├── CMakeLists.txt            # 根构建：登记 APP/ 源文件
├── CMakePresets.json         # Debug / Release 预设
├── GETFUN_F722_FreeRTOS.ioc  # STM32CubeMX 工程配置
├── STM32F722XX_FLASH.ld      # 链接脚本（程序区 256 KB + Sector 6/7 参数槽）
└── startup_stm32f722xx.s     # 启动文件
```

---

## 构建

### 依赖

- `arm-none-eabi-gcc`（或 `starm-clang`）工具链
- `Ninja`
- `CMake >= 3.22`

### 命令

```bash
# 配置 + 构建（Debug）
cmake --preset Debug
cmake --build build/Debug

# Release 构建
cmake --preset Release
cmake --build build/Release
```

构建产物位于 `build/Debug/`（或 `build/Release/`）：

- `GETFUN_F722_FreeRTOS.elf` — ELF，用于调试/烧录
- `GETFUN_F722_FreeRTOS.hex` — Intel HEX
- `GETFUN_F722_FreeRTOS.bin` — 原始二进制
- `GETFUN_F722_FreeRTOS.map` — 链接映射

链接时会输出内存占用（`--print-memory-usage`）。本项目无单元测试 / lint 配置；
验证靠**编译通过 + 烧录后在 Betaflight Configurator 中观察 MSP 交互**。

---

## 烧录与调试

- **SWD 调试**：`.elf` 可用 OpenOCD + J-Link / ST-Link 加载调试，或在 IDE 中加载。
- **ROM DFU 恢复**：可通过 BOOT 引脚进入 STM32 ROM DFU 恢复烧录（已验收）；
  软件 DFU 跳转（App/CLI 命令）尚未实现。
- **验证 MSP 连通**（最快路径）：
  1. 烧录 `.hex` / `.bin`
  2. USB 连板子，打开 **Betaflight Configurator**
  3. 选择对应虚拟串口 → **Connect**
  4. 能读到状态 / 姿态 / 电池 / Receiver 即说明整条 `USB → 解析 → 状态 → 回包` 链路通
- **运行时观察**：UART4 1 Hz 输出 IMU 时间/dt、低通、校准、参数槽、偏置摘要；
  `usb_cdc_transport_rx_dropped()` 可查 RX 环缓冲溢出丢包计数。

---

## 开发路线与当前进度

> 本节为摘要，**唯一实时进度源是 [Docs/00](Docs/00_项目开发路线与统一进度.md)**；
> 若本文与 `00` 不一致，以 `00` 为准。

```text
S1 硬件基线            🟠 主体完成，并行实测项按依赖补齐
S2 最小FreeRTOS平台    🟠 v0.1.0 已冻结；App/CLI 软件 DFU 未实现
S3 IMU、姿态与飞行输入 🟠 当前阶段（S3.8 软件完成，待实物/App 验收）
S4 控制与电机          ⬜ 已规划（FlightTask / PID / Mixer / DShot / ARM）
S5 基础飞行            ⬜ 已规划
S6 功能完善            ⬜ 已规划（OSD / Blackbox / 气压计 / CLI）
S7 后续扩展            ⏸ 条件式（GPS / 双向 DShot / 伴随计算）
```

S3 细分里程碑：S3.1～S3.7 已全部冻结（IMU 轮询 → SPI DMA → 陀螺校准 → 加速度校准 →
低通+精确 dt → Mahony 姿态 → CRSF RC），当前推进 **S3.8 RC Failsafe 与 App Receiver 页面**。

### 🏷 已冻结版本历史

| 版本标签 | 提交 | 日期 | 冻结内容 |
|---|---|---|---|
| `v0.1.0-platform-baseline` | `15cc25a` | 2026-07-27 | 构建、时钟、UART、FreeRTOS、USB/MSP、ROM DFU 恢复、Motor 安全 |
| `v0.2.0-imu-polling-baseline` | `4452a28` | 2026-07-27 | ICM42688P 1 kHz 轮询、SI 单位、CW90、MSP 原始数据 |
| `v0.3.0-imu-dma-drdy-baseline` | `67e38ea` | 2026-07-27 | SPI1 DMA、DRDY 寄存器门控、超时恢复、诊断 |
| `v0.4.0-gyro-calibration-baseline` | `758d424` | 2026-07-30 | 陀螺静态零偏、运动拒绝、解锁抑制、恢复重校准 |
| `v0.5.0-accel-calibration-params-baseline` | `f2ebed3` | 2026-07-30 | 加速度水平校准、Flash 参数 A/B 槽、损坏/掉电恢复 |
| `v0.6.0-imu-filter-timing-baseline` | `c9aba14` | 2026-07-30 | DWT 微秒时基、真实 dt、Gyro/Accel PT1、异常恢复 |
| `v0.7.0-mahony-attitude-baseline` | `32140de` | 2026-08-04 | Mahony 六轴姿态、坐标/比力语义、MSP_ATTITUDE、App 三维模型 |
| `v0.8.0-crsf-rc-baseline` | `5bce889` | 2026-08-05 | UART2 循环 DMA、CRSF CRC、16 通道/Link Statistics、错误恢复 |

---

## 关键设计约束

- ImuTask、MspTask、RcTask 均用 `xTaskCreateStatic` 静态分配，需保持
  `configSUPPORT_STATIC_ALLOCATION=1`。
- **ImuTask 是 SPI1/ICM42688P 唯一所有者**，不得从 MSP、诊断或 ISR 直接访问传感器。
- 陀螺/加速度校准必须在 **SI 换算 + CW90 之后**执行；新加速度偏置必须 **Flash 验证成功后**
  才应用。IMU/校准/参数无效时对应 `arming_inhibit_flags` 必须置位。
- **DWT 周期扩展只能由 ImuTask 更新**；DMA ISR 只捕获原始 CYCCNT。`dt` 不在
  500～2000 µs、首样本或滤波未 READY 时，`APP_ARMING_INHIBIT_IMU_TIMING_INVALID`
  必须置位，后续 Mahony 不得积分。
- `MSP_RAW_IMU` 用校准后**未滤波**数据；`filtered_*` 专供姿态链，两条支路语义不能互换。
- Flash 参数只能由 ImuTask 在安全状态写入；未来 ARM 后必须拒绝 Sector 6/7 擦写。
- `usb_cdc_transport_receive_from_isr()` 跑在 **USB ISR 上下文，不能阻塞**，只写环缓冲并发通知。
- 不在生成段（`Core/`、`USB_DEVICE/`、CubeMX 驱动）手写外设句柄、MSP、IRQ 或业务逻辑；
  生成文件里的项目代码只能写在成对的 `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` 之间。

---

## 添加新代码

1. **APP 应用层新源文件** → 加到根 [CMakeLists.txt](CMakeLists.txt) 的 `add_executable`
   列表，include 路径加到 `target_include_directories` 的 `APP/Inc` 处。
2. **CubeMX 管理的源文件** → 加到 [cmake/stm32cubemx/CMakeLists.txt](cmake/stm32cubemx/CMakeLists.txt)。
3. **改外设/引脚/时钟** → 编辑 [GETFUN_F722_FreeRTOS.ioc](GETFUN_F722_FreeRTOS.ioc) →
   CubeMX Regenerate Code；重新生成前先做 Git 检查点，生成后核对 `git diff`。
4. **新建文件**必须写文件头说明（`.c/.h` 用注释说明作用/核心函数/数据流/约束；
   `.md` 用 HTML 注释说明用途/覆盖范围/关联模块/待验证项），并与实现同步。
5. **新增/升级外部组件**前，必须在 [Docs/00](Docs/00_项目开发路线与统一进度.md) 登记上游来源、
   精确版本/提交、获取日期、许可证、本地修改和验收里程碑；**禁止构建时联网拉取未固定的最新版**。

完整规则见 [CLAUDE.md](CLAUDE.md)。

---

## 第三方组件与许可证

本项目复用官方基础设施，飞控核心自研：

| 组件 | 版本 | 来源策略 | 许可证 |
|---|---|---|---|
| CMSIS Core (M) | 5.1 | 官方直接集成 | `Drivers/CMSIS/LICENSE.txt` |
| STM32F7 CMSIS Device | 1.2.10 | ST 官方 | `Drivers/CMSIS/Device/ST/STM32F7xx/LICENSE.txt` |
| STM32F7 HAL | 1.3.3 | ST 官方 | `Drivers/STM32F7xx_HAL_Driver/LICENSE.txt` |
| FreeRTOS Kernel | 10.2.1 | 官方版本随工程固定 | 各源码文件许可证头 |
| CMSIS-RTOS V2 适配层 | 随 CubeMX 快照 | 官方适配层 | 适配层源码许可证头 |
| ST USB Device Library | 随 CubeMX 快照 | ST 官方 | `Middlewares/ST/STM32_USB_Device_Library/LICENSE.txt` |

`APP/` 自研模块（MSP、状态、参数、校准、姿态、CRSF、任务/安全架构等）的许可证尚未声明。
新增社区组件统一放入 `ThirdParty/<组件名>/` 并保留原始许可证。

---

## 文档索引

所有设计文档位于 [Docs/](Docs/)：

- [00_项目开发路线与统一进度.md](Docs/00_项目开发路线与统一进度.md) — **唯一实时进度源**
- `01_*` 硬件基线、引脚资源、构建烧录、恢复证据
- `02_*` FreeRTOS 架构、驱动边界、任务、参数系统设计
- `03_*` 姿态、控制、Mixer、DShot、ARM/Failsafe、App 兼容设计
- `04_*` 调试方法与按日期追加的开发/测试记录
- 各版本开发计划与验收文档（编号递增）

工程内开发指引见 [CLAUDE.md](CLAUDE.md)。
