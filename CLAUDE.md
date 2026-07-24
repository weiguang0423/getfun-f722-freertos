# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F722RETx (Cortex-M7) 固件项目，基于 STM32CubeMX 生成，使用 FreeRTOS + CMSIS-RTOS V2，支持 USB CDC 虚拟串口。

- **MCU**: STM32F722RETx, Cortex-M7 @ 216MHz, FPv5-SP-D16, 512KB Flash / 256KB RAM
- **RTOS**: FreeRTOS Kernel V10.2.1 (heap_4, 15KB 堆)
- **HAL**: STM32F7xx HAL Drivers, CMSIS
- **USB**: USB Device CDC (Virtual COM Port)
- **外设**: GPIO (8 路电机输出), UART4, TIM6 (时间基准)
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
| **CubeMX 应用** | `Core/Src/` | `main.c` (入口), `freertos.c` (任务), `gpio.c`, `usart.c` |
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
USB CDC 接收回调 (usbd_cdc_if.c, ISR 上下文)
  └─ usb_cdc_transport_receive_from_isr()        环形缓冲 + 任务通知
       │
MspTask (APP/Src/rtos/app_task.c, osPriorityNormal+2, 静态分配 768 words 栈)
  ├─ usb_cdc_transport_read()                      从环缓冲取字节
  ├─ msp_parser_process_byte()                     逐字节状态机解析 (MSP V1/V2)
  ├─ msp_server_process()                          按 command 派发，读 app_state 快照
  └─ msp_transport_build_response() + usb_cdc_transport_write()
```

模块职责：

- [APP/Inc/protocol/msp_transport.h](APP/Inc/protocol/msp_transport.h) / [msp_transport.c](APP/Src/protocol/msp_transport.c) — 纯协议层，无 RTOS 依赖。`msp_parser_t` 是逐字节状态机（`$M<` V1 / `$X<` V2，V2 用 CRC8-DVB-S2）；`msp_transport_build_response()` 构造回包帧。`MSP_MAX_PAYLOAD_SIZE=256`。
- [APP/Src/protocol/msp_server.c](APP/Src/protocol/msp_server.c) — MSP 命令派发。`MSP_FC_VARIANT` 返回 `"BTFL"`（Configurator 只对 BTFL 开正常 UI），但 `MSP_BOARD_INFO` 仍报 `GF72`/`GETFUN_F722` 保持板子身份。实现了 STATUS/STATUS_EX/RAW_IMU/ATTITUDE/ANALOG/BATTERY_STATE/UID/SET_RTC/SET_ARMING_DISABLED 等命令，全部数据来自 `app_state` 快照。
- [APP/Inc/app_state.h](APP/Inc/app_state.h) / [APP/Src/app_state.c](APP/Src/app_state.c) — 全局运行态快照 `app_state_snapshot_t`（IMU/姿态/电池/uptime/CPU 负载/故障标志/宿主 RTC）。多任务安全靠 **PRIMASK 关中断 + DMB** 做临界区（不是 FreeRTOS mutex），`app_state_get_snapshot()` 整体拷贝后把 `uptime_ms` 替换为 `HAL_GetTick()`。发布者（传感器/电池任务）调 `app_state_publish_*`，MspTask 只读快照。
- [APP/Src/bsp/usb_cdc_transport.c](APP/Src/bsp/usb_cdc_transport.c) — USB CDC 上的一层收发适配。RX 是 1024 字节环形缓冲，ISR 写入后 `vTaskNotifyGiveFromISR` 唤醒绑定任务；`usb_cdc_transport_bind_current_task()` 必须在 MspTask 启动时调用一次。TX 320 字节缓冲，`tx_idle` 标志 + 轮询等待 `CDC_Transmit_FS` 完成（`usb_cdc_transport_transmit_complete_from_isr()` 在发送完成时置位）。
- [APP/Src/platform/platform_diag.c](APP/Src/platform/platform_diag.c) — 平台基线诊断与安全停机。UART4 输出构建时间、运行时时钟、RTOS 启动状态和 1 Hz 心跳；HardFault、FreeRTOS assert、栈溢出或内存分配失败时强制 Motor 1～8 为低电平，并把故障码保留在可通过 SWD 读取的全局变量中。
- [APP/Src/rtos/app_task.c](APP/Src/rtos/app_task.c) — `app_tasks_init()` 由 [freertos.c](Core/Src/freertos.c) 的 `MX_FREERTOS_Init()` 调用，**静态创建** MspTask 并初始化 app_state/transport。CubeMX 的 InitTask 初始化 USB，输出 RTOS 启动信息和 1 Hz 平台心跳。

关键约束：
- MspTask 用 `xTaskCreateStatic`（栈和控制块静态分配）——新增任务若也走静态，需在 `FreeRTOSConfig.h` 保持 `configSUPPORT_STATIC_ALLOCATION=1`。
- `usb_cdc_transport_receive_from_isr()` 跑在 USB ISR 上下文，**不能阻塞**；它只写环缓冲并发通知。
- `app_state` 临界区是关中断而非互斥量，发布者要短小、不得在临界区内调任何可能阻塞的 API。

## 关键文件

- [GETFUN_F722_FreeRTOS.ioc](GETFUN_F722_FreeRTOS.ioc) — STM32CubeMX 项目配置，双击用 CubeMX 编辑引脚/时钟/外设
- [Core/Src/main.c](Core/Src/main.c) — 程序入口: MPU → Cache → HAL → 时钟 → GPIO/UART 初始化 → FreeRTOS 调度启动
- [Core/Src/freertos.c](Core/Src/freertos.c) — FreeRTOS 任务创建: MX_FREERTOS_Init() 创建 InitTask(栈 2KB, osPriorityNormal) → USB 初始化 → 循环 delay(1)
- [Core/Inc/FreeRTOSConfig.h](Core/Inc/FreeRTOSConfig.h) — FreeRTOS 配置（优先级 56 级, 抢占使能, 静态/动态分配, 栈溢出检测, 软件定时器）
- [USB_DEVICE/App/usbd_cdc_if.c](USB_DEVICE/App/usbd_cdc_if.c) — CDC 收发实现, `CDC_Transmit_FS()` 发送, `CDC_Receive_FS()` 接收回调
- [STM32F722XX_FLASH.ld](STM32F722XX_FLASH.ld) — 链接脚本 (FLASH 0x8000000 512K, RAM 0x20000000 256K)
- [Core/Inc/stm32f7xx_hal_conf.h](Core/Inc/stm32f7xx_hal_conf.h) — HAL 模块开关: 使能了 GPIO/DMA/RCC/FLASH/PWR/CORTEX/TIM/UART/PCD/EXTI

## 引脚定义

在 [Core/Inc/main.h](Core/Inc/main.h) 中定义:
- 8 路电机: MOTOR1-8 (PA15, PA9, PA8, PA8, PC9, PC8, PB11, PB10)
- USART: UART4 (具体引脚见 usart.h)

## GPIO 引脚见 [Core/Inc/gpio.h](Core/Inc/gpio.h) 和 [Core/Src/gpio.c](Core/Src/gpio.c)

## 添加新代码

### 新文件头部说明规则

每次新建文件时，必须在文件最前面写清楚文件作用和核心功能：

- 新建 `.c` / `.h`：使用文件头注释说明文件作用、核心函数或类型、主要数据流和关键约束。
- 新建 `.md`：在标题前使用 HTML 注释说明文档用途、覆盖范围、关联核心函数或模块，以及哪些结论仍需实物验证。
- 文件头说明必须与实际实现同步更新，不能保留空泛模板或已经失效的函数描述。

1. 如需 CubeMX 重新生成：编辑 `.ioc` 文件 → 在 CubeMX 中 Regenerate Code
2. **CubeMX 管理的源文件**（`Core/`、`USB_DEVICE/`、`Drivers/`、`Middlewares/`）添加到 [cmake/stm32cubemx/CMakeLists.txt](cmake/stm32cubemx/CMakeLists.txt) 的 `MX_Application_Src` / `STM32_Drivers_Src` 等列表
3. **APP 应用层的新源文件**添加到根 [CMakeLists.txt](CMakeLists.txt) 的 `add_executable` 列表，include 路径加到 `target_include_directories` 的 `APP/Inc` 处
4. CubeMX 生成区的用户代码放在 `/* USER CODE BEGIN */` / `/* USER CODE END */` 之间，重生成时才不会被覆盖；APP/ 完全是用户区，不受此约束
5. 如果添加 CMake 配置，在根 [CMakeLists.txt](CMakeLists.txt) 的用户区添加

## 调试

- `.elf` 可用 OpenOCD + JLink/ST-Link 调试，或直接加载到 IDE
- 验证 MSP 连通的最快方式：烧录后连 USB，打开 Betaflight Configurator，选对应虚拟串口 → Connect，能读到状态/姿态/电池即说明整条 USB→解析→状态→回包链路通
- `usb_cdc_transport_rx_dropped()` 可查 RX 环缓冲溢出丢包计数
