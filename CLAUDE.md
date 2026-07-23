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

构建产物为 `build/Debug/GETFUN_F722_FreeRTOS.elf` 及 `.hex`/`.bin`。

## 架构分层

| 层 | 目录 | 说明 |
|---|---|---|
| **应用** | `Core/Src/` | `main.c` (入口), `freertos.c` (任务), `gpio.c`, `usart.c` |
| **USB 设备** | `USB_DEVICE/App/` | CDC 虚拟串口 (`usbd_cdc_if.c`) |
| **FreeRTOS** | `Middlewares/Third_Party/FreeRTOS/` | 内核 + CMSIS-RTOS V2 封装 |
| **USB 库** | `Middlewares/ST/STM32_USB_Device_Library/` | USB 设备核心和 CDC 类 |
| **HAL 驱动** | `Drivers/STM32F7xx_HAL_Driver/` | STM32 HAL/LL 驱动层 |
| **CMSIS** | `Drivers/CMSIS/` | Cortex-M 核心头文件和启动文件 |
| **CubeMX** | `cmake/stm32cubemx/` | CMake 构建配置（源文件/头文件/宏列表） |

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

1. 如需 CubeMX 重新生成：编辑 `.ioc` 文件 → 在 CubeMX 中 Regenerate Code
2. 新源文件需添加到 `cmake/stm32cubemx/CMakeLists.txt` 的 `MX_Application_Src` 列表
3. 用户代码放在 `/* USER CODE BEGIN */` / `/* USER CODE END */` 之间，CubeMX 重生成时不会覆盖
4. 如果添加 CMake 配置，在根 [CMakeLists.txt](CMakeLists.txt) 的用户区添加

## 调试

- `.elf` 可用 OpenOCD + JLink/ST-Link 调试，或直接加载到 IDE
- 链接器会输出内存使用情况 (`--print-memory-usage`)
- `.map` 文件生成在构建目录下
