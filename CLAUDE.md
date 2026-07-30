# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F722RETx (Cortex-M7) 固件项目，基于 STM32CubeMX 生成，使用 FreeRTOS + CMSIS-RTOS V2，支持 USB CDC 虚拟串口。

- **MCU**: STM32F722RETx, Cortex-M7 @ 216MHz, FPv5-SP-D16, 512KB Flash / 256KB RAM
- **RTOS**: FreeRTOS Kernel V10.2.1 (heap_4, 15KB 堆)
- **HAL**: STM32F7xx HAL Drivers, CMSIS
- **USB**: USB Device CDC (Virtual COM Port)
- **外设**: GPIO (8 路电机安全输出), SPI1 + ICM42688P, UART4, TIM6 (时间基准)
- **系统时钟**: HSE 8MHz → PLL → 216MHz, OverDrive 开启

## 构建

依赖: `arm-none-eabi-gcc` (或 `starm-clang`), `Ninja`, `CMake >= 3.22`

```bash
# 配置 (Debug)
cmake --preset Debug

# 构建
cmake --build build/Debug

# Release 构建
cmake --preset Release && cmake --build build/Release
```

构建产物为 `build/Debug/GETFUN_F722_FreeRTOS.elf` 及 `.hex`/`.bin`。链接器会输出内存使用情况（`--print-memory-usage`），`.map` 也在构建目录下。

无单元测试/lint 配置；验证靠编译通过 + 烧录后在 Betaflight Configurator 中观察 MSP 交互。

## 架构分层

| 层 | 目录 | 说明 |
|---|---|---|
| **应用 (APP)** | `APP/` | 自研应用层：MSP 协议服务器、应用状态、USB CDC 传输、RTOS 任务（见下） |
| **CubeMX 应用** | `Core/Src/` | `main.c` (入口), `freertos.c` (任务), `gpio.c`, `spi.c`, `usart.c` |
| **USB 设备** | `USB_DEVICE/App/` | CDC 虚拟串口 (`usbd_cdc_if.c`)，其 CDC 接收回调转发给 `usb_cdc_transport` |
| **FreeRTOS** | `Middlewares/Third_Party/FreeRTOS/` | 内核 + CMSIS-RTOS V2 封装 |
| **USB 库** | `Middlewares/ST/STM32_USB_Device_Library/` | USB 设备核心和 CDC 类 |
| **HAL 驱动** | `Drivers/STM32F7xx_HAL_Driver/` | STM32 HAL/LL 驱动层 |
| **CMSIS** | `Drivers/CMSIS/` | Cortex-M 核心头文件和启动文件 |
| **CubeMX** | `cmake/stm32cubemx/` | CMake 构建配置（CubeMX 生成的源/头/宏，**不含 APP/**） |

## APP 应用层架构（自研，核心）

`APP/` 是在 CubeMX 产物之上加的应用层，目标是把板子伪装成 Betaflight PID 控制器，让 Betaflight Configurator 通过 USB CDC 虚拟串口连接。**`APP/` 下的源文件登记在根 [CMakeLists.txt](CMakeLists.txt) 的 `add_executable` 列表里，不在 `cmake/stm32cubemx/CMakeLists.txt` 中**——所以 CubeMX 重新生成不会动到它，新增 APP 源文件需手动加到根 `CMakeLists.txt`。

数据流（自底向上）：

```
SPI1 + PA4 CS + DMA2 Stream 0/3
  └─ imu_bus -> icm42688p -> ImuTask (1 kHz DRDY门控, idle+4, 静态 512 words)
       └─ SI + CW90 -> gyro_calibration -> accel_calibration
            └─ parameter_store (Sector 6/7 A/B) -> app_state_publish_imu()
                                │
USB CDC 接收回调 (usbd_cdc_if.c, ISR 上下文)
  └─ usb_cdc_transport_receive_from_isr()        环形缓冲 + 任务通知
       │
MspTask (APP/Src/rtos/app_task.c, idle+2, 静态分配 768 words 栈)
  ├─ usb_cdc_transport_read()                      从环缓冲取字节
  ├─ msp_parser_process_byte()                     逐字节状态机解析 (MSP V1/V2)
  ├─ msp_server_process()                          按 command 派发，读 app_state 快照
  └─ msp_transport_build_response() + usb_cdc_transport_write()
```

模块职责：

- [APP/Inc/protocol/msp_transport.h](APP/Inc/protocol/msp_transport.h) / [msp_transport.c](APP/Src/protocol/msp_transport.c) — 纯协议层，无 RTOS 依赖。`msp_parser_t` 是逐字节状态机（`$M<` V1 / `$X<` V2，V2 用 CRC8-DVB-S2）；`msp_transport_build_response()` 构造回包帧。`MSP_MAX_PAYLOAD_SIZE=256`。
- [APP/Src/protocol/msp_server.c](APP/Src/protocol/msp_server.c) — MSP 命令派发。`MSP_FC_VARIANT` 返回 `"BTFL"`（Configurator 只对 BTFL 开正常 UI），但 `MSP_BOARD_INFO` 仍报 `GF72`/`GETFUN_F722` 保持板子身份。除状态命令外，实现标准 `MSP_ACC_CALIBRATION`（205）向ImuTask排队请求，以及GETFUN MSP2 `0x4000`参数/校准诊断。
- [APP/Src/bsp/imu_bus.c](APP/Src/bsp/imu_bus.c) / [APP/Src/drivers/icm42688p.c](APP/Src/drivers/icm42688p.c) — SPI1 总线适配和 ICM42688P 寄存器驱动。初始化与 DRDY 状态读取使用阻塞事务，14 字节样本使用 DMA2 Stream 0/3、Channel 3 和两个 32 字节对齐静态槽位；CS PA4、Mode 0、1.6875 MHz，器件固定为 1 kHz、±2000 dps、±16 g。
- [APP/Src/algorithms/gyro_calibration.c](APP/Src/algorithms/gyro_calibration.c) — 不依赖 HAL/RTOS 的上电陀螺静态零偏状态机。输入为 SI 换算和 CW90 后的机体系数据；连续 250 样本预热、2000 样本 Welford 均值/方差，运动或异常时重置窗口，READY 后冻结并扣除三轴零偏。每次上电或 IMU 完整恢复都重新校准，不持久化。
- [APP/Src/algorithms/accel_calibration.c](APP/Src/algorithms/accel_calibration.c) — 不依赖HAL/RTOS的水平单面加速度校准。使用250预热、2000样本Welford窗口以及运动、水平、方差和偏置门限；候选值持久化成功前不切换。
- [APP/Src/storage/parameter_store.c](APP/Src/storage/parameter_store.c) — STM32F722 Sector 6/7双槽参数存储。48字节v1记录包含magic/version/length/sequence/flags/CRC32/commit；只擦除非活动槽且commit最后写入，支持空白默认、损坏恢复和序号选择。
- [APP/Src/rtos/imu_task.c](APP/Src/rtos/imu_task.c) — ImuTask 是 IMU/参数保存单写者，静态栈512 words、优先级idle+4；负责初始化/重试、1 kHz门控DMA、SI/CW90、陀螺/加速度校准、参数提交和app_state发布。当前板级资料未定义IMU INT外部引脚，不得把PC4/PINIO1猜作DRDY EXTI。
- [APP/Inc/app_state.h](APP/Inc/app_state.h) / [APP/Src/app_state.c](APP/Src/app_state.c) — 全局运行态快照 `app_state_snapshot_t`（含IMU、陀螺/加速度校准、参数槽状态、解锁抑制、姿态/电池/运行信息）。多任务安全靠 **PRIMASK关中断 + DMB** 做短临界区。
- [APP/Src/bsp/usb_cdc_transport.c](APP/Src/bsp/usb_cdc_transport.c) — USB CDC 上的一层收发适配。RX 是 1024 字节环形缓冲，ISR 写入后 `vTaskNotifyGiveFromISR` 唤醒绑定任务；`usb_cdc_transport_bind_current_task()` 必须在 MspTask 启动时调用一次。TX 320 字节缓冲，`tx_idle` 标志 + 轮询等待 `CDC_Transmit_FS` 完成（`usb_cdc_transport_transmit_complete_from_isr()` 在发送完成时置位）。
- [APP/Src/platform/platform_diag.c](APP/Src/platform/platform_diag.c) — 平台基线诊断与安全停机。UART4 1 Hz输出IMU、两种校准、参数槽和偏置摘要；致命故障或参数保存前强制Motor 1～8低电平。
- [APP/Src/rtos/app_task.c](APP/Src/rtos/app_task.c) — `app_tasks_init()` 由 [freertos.c](Core/Src/freertos.c) 调用，初始化 app_state/transport，并**静态创建** ImuTask 和 MspTask。

关键约束：
- ImuTask、MspTask 均用 `xTaskCreateStatic`，需在 `FreeRTOSConfig.h` 保持 `configSUPPORT_STATIC_ALLOCATION=1`。
- ImuTask 是 SPI1/ICM42688P 唯一所有者，不得从 MSP、诊断或 ISR 直接访问传感器。
- 陀螺和加速度校准必须在SI换算和CW90之后执行；新加速度偏置必须持久化验证成功后才应用。IMU、校准或参数无效时对应 `arming_inhibit_flags` 必须保持置位。
- 参数Flash只能由ImuTask在安全状态写入。未来真实ARM状态机接入后，Armed时必须拒绝Sector 6/7擦写。
- `usb_cdc_transport_receive_from_isr()` 跑在 USB ISR 上下文，**不能阻塞**；它只写环缓冲并发通知。
- `app_state` 临界区是关中断而非互斥量，发布者要短小、不得在临界区内调任何可能阻塞的 API。

## 关键文件

- [Docs/00_项目开发路线与统一进度.md](Docs/00_项目开发路线与统一进度.md) — 项目唯一实时进度源：阶段坐标、细分里程碑、依赖、验收门槛、版本基线和下一步
- [GETFUN_F722_FreeRTOS.ioc](GETFUN_F722_FreeRTOS.ioc) — STM32CubeMX 项目配置，双击用 CubeMX 编辑引脚/时钟/外设
- [Core/Src/main.c](Core/Src/main.c) — 程序入口: MPU → Cache → HAL → 时钟 → GPIO/DMA/SPI1/UART 初始化 → FreeRTOS 调度启动
- [Core/Src/freertos.c](Core/Src/freertos.c) — FreeRTOS 任务创建: InitTask(栈 2KB, osPriorityIdle) + APP 静态任务；InitTask 初始化 USB 并每秒输出维护诊断
- [Core/Inc/FreeRTOSConfig.h](Core/Inc/FreeRTOSConfig.h) — FreeRTOS 配置（优先级 56 级, 抢占使能, 静态/动态分配, 栈溢出检测, 软件定时器）
- [USB_DEVICE/App/usbd_cdc_if.c](USB_DEVICE/App/usbd_cdc_if.c) — CDC 收发实现, `CDC_Transmit_FS()` 发送, `CDC_Receive_FS()` 接收回调
- [STM32F722XX_FLASH.ld](STM32F722XX_FLASH.ld) — 链接脚本：程序FLASH `0x08000000`前256 KB；Sector 6/7各128 KB保留为参数槽A/B；RAM 256 KB
- [Core/Inc/stm32f7xx_hal_conf.h](Core/Inc/stm32f7xx_hal_conf.h) — HAL 模块开关: 使能了 GPIO/DMA/RCC/FLASH/PWR/CORTEX/TIM/SPI/UART/PCD/EXTI

## 引脚定义

在 [Core/Inc/main.h](Core/Inc/main.h) 中定义:
- 8 路电机: MOTOR1-8 (PA15, PA10, PA9, PA8, PC9, PC8, PB11, PB10)
- ICM42688P: SPI1 SCK/MISO/MOSI = PA5/PA6/PA7，CS = PA4
- USART: UART4 (具体引脚见 usart.h)

## GPIO 引脚见 [Core/Inc/gpio.h](Core/Inc/gpio.h) 和 [Core/Src/gpio.c](Core/Src/gpio.c)

## 添加新代码

### 项目进度维护规则

- `Docs/00_项目开发路线与统一进度.md` 是唯一的实时进度源。开始开发前先确认其中的当前坐标、前置依赖和本里程碑退出条件。
- 里程碑进入开发、软件交付完成、实物验收通过或目标范围调整时，必须在同一次变更中更新 `00` 的状态、证据和下一步。
- 创建阶段性标签前，在冻结提交中登记验收日期和目标标签，并把下一个可执行目标切换为当前目标；提交后创建标签，实际提交哈希可在下一次文档变更中回填，不得为回填哈希移动已冻结标签。
- 总纲和调试记录不得重新建立并行的“当前待办”清单；总纲描述长期路线，调试记录保存按日期的测试证据。

### 官方与开源组件复用规则

- 新模块编码前必须先调研芯片/器件官方驱动、权威协议规范和成熟开源实现，并在 `00` 或当前版本开发计划中明确选择“官方直接集成、官方裁剪适配、开源固定版本引入、参考复现、完全自主实现”之一。
- 默认优先级为官方方案、成熟开源方案、参考复现、自主实现；若候选存在动态内存、不可控阻塞、过重依赖、不明确许可证或不安全故障行为，则不得直接引入。
- CMSIS、HAL、FreeRTOS和ST USB等官方基础设施继续使用；参数事务、任务/状态架构、PID、Mixer、ARM/Failsafe和电机安全链由项目自主实现并掌握。
- 新增或升级外部组件前，必须在 `00` 登记上游来源、精确版本/提交、获取日期、许可证、本地修改、适配接口和验收里程碑；禁止构建时联网获取未固定的最新版。
- CubeMX/ST官方组件保留在生成目录；新增社区组件放入 `ThirdParty/<组件名>/` 并保留许可证。`APP/` 通过项目适配接口使用第三方模块，不得在业务层散布第三方内部类型和全局状态。
- 第三方代码仍须通过本项目的Debug/Release构建、资源占用、最坏执行时间、故障注入和实物验收，上游测试结果不能替代本项目验收。

### 新文件头部说明规则

每次新建文件时，必须在文件最前面写清楚文件作用和核心功能：

- 新建 `.c` / `.h`：使用文件头注释说明文件作用、核心函数或类型、主要数据流和关键约束。
- 新建 `.md`：在标题前使用 HTML 注释说明文档用途、覆盖范围、关联核心函数或模块，以及哪些结论仍需实物验证。
- 文件头说明必须与实际实现同步更新，不能保留空泛模板或已经失效的函数描述。

1. 如需 CubeMX 重新生成：编辑 `.ioc` 文件 → 在 CubeMX 中 Regenerate Code
2. **CubeMX 管理的源文件**（`Core/`、`USB_DEVICE/`、`Drivers/`、`Middlewares/`）添加到 [cmake/stm32cubemx/CMakeLists.txt](cmake/stm32cubemx/CMakeLists.txt) 的 `MX_Application_Src` / `STM32_Drivers_Src` 等列表
3. **APP 应用层的新源文件**添加到根 [CMakeLists.txt](CMakeLists.txt) 的 `add_executable` 列表，include 路径加到 `target_include_directories` 的 `APP/Inc` 处
4. `Core/`、`USB_DEVICE/`、`cmake/stm32cubemx/` 和 CubeMX 管理的驱动文件由 `.ioc` 单向生成；不得在生成段手写外设句柄、MSP、IRQ、初始化调用或业务逻辑。
5. CubeMX 生成文件中的项目代码只能放在成对且名称匹配的 `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` 之间；生成文件的详细文件说明必须写在现有 `USER CODE BEGIN Header` 块内，不能删除这两个标记。
6. GPIO、SPI、DMA、NVIC 等参数先在 `.ioc`/CubeMX 中固化，再由 CubeMX生成；不要手工维护一份与 `.ioc` 并行的生成代码。
7. `APP/` 完全属于用户区，驱动策略、DMA缓冲、状态机、任务和协议实现都应放在这里；生成层只提供HAL句柄和中断入口。
8. 每次重新生成前创建Git检查点；生成后检查`git diff`并完成Debug/Release构建。整改生成边界时应连续生成两次，第二次不得再出现功能性源码变化。
9. 如果添加 CMake 配置，在根 [CMakeLists.txt](CMakeLists.txt) 的用户区添加；不要手工维护`cmake/stm32cubemx/CMakeLists.txt`中CubeMX能够生成的条目。

## 调试

- `.elf` 可用 OpenOCD + JLink/ST-Link 调试，或直接加载到 IDE
- 验证 MSP 连通的最快方式：烧录后连 USB，打开 Betaflight Configurator，选对应虚拟串口 → Connect，能读到状态/姿态/电池即说明整条 USB→解析→状态→回包链路通
- `usb_cdc_transport_rx_dropped()` 可查 RX 环缓冲溢出丢包计数
