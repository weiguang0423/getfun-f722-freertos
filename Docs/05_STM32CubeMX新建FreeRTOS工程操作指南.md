# GETFUN F722 V3 STM32CubeMX 新建 FreeRTOS 工程操作指南

> 文档版本：V1.0  
> 编写日期：2026-07-22  
> 目标硬件：GETFUN F722 V3  
> MCU：STM32F722RET6  
> 建议工具：STM32CubeMX 6.18.x、STM32CubeF7、Arm GNU Toolchain、CMake、Ninja  
> 本文目标：完成一个可以重复生成、构建、烧录，能够运行 FreeRTOS 并通过 UART4 输出心跳日志的最小独立工程。

---

## 1. 使用范围与最终结果

本文只负责建立独立固件的第一个可运行基线，不在第一次 CubeMX 配置中接入 IMU、CRSF、ADC、DShot、OSD 或 Blackbox。

完成本文后应得到：

- 独立工程 `getfun-f722-freertos`，不依赖 Betaflight 调度器。
- STM32F722RET6 从内部 Flash `0x08000000` 启动。
- 系统主频为 216 MHz。
- SWD 调试口保留。
- UART4 使用 PA0/PA1，能够输出启动日志。
- USB FS 被配置为 CDC/VCP，且不占用 PA9 的 VBUS 检测。
- FreeRTOS 使用 SysTick，HAL 使用 TIM6 作为 1 ms 时间基准。
- 一个最小静态任务可以持续运行。
- 未解锁和复位期间，Motor 1～8 引脚不产生有效电机脉冲。
- CMake 可以生成 ELF、HEX、BIN 和 MAP 文件。

本文的硬件事实来源为：

- `01_GETFUN_F722_V3_硬件基线_构建烧录与恢复.md`
- `02_FreeRTOS架构与硬件驱动.md`
- `src/config/configs/GETFUNF722V3/config.h`

如果本文与硬件基线文档冲突，以硬件基线文档为准，并同步修订本文。

---

## 2. 新建前的安全准备

### 2.1 必须先完成

- [ ] 拆除全部螺旋桨。
- [ ] 保存原厂固件和当前可运行 Betaflight HEX。
- [ ] 确认可以通过 BOOT 按键或 BOOT 焊盘进入 STM32 ROM DFU。
- [ ] 记录当前 Betaflight 固件版本及 HEX 的 SHA-256。
- [ ] 准备可以观察 UART4 TX/PA0 的 USB 转串口模块或逻辑分析仪。
- [ ] 如果使用 USB 转串口模块，只连接 GND、飞控 TX 和可选 RX，不要同时用两个电源给飞控供电。

### 2.2 工程位置

建议将新工程建立为 Betaflight 的同级目录：

```text
E:\
├─ betaflight\
└─ getfun-f722-freertos\
```

不要在 `E:\betaflight\src` 中创建新工程，也不要在 Betaflight 仓库内部再初始化一个嵌套 Git 仓库。

推荐工程名：

```text
GETFUN_F722_FreeRTOS
```

推荐目录名：

```text
getfun-f722-freertos
```

---

## 3. 安装与记录工具版本

安装以下工具：

1. STM32CubeMX。
2. STM32CubeF7 Firmware Package。
3. Arm GNU Toolchain；已验证参考版本为 `13.3.Rel1`。
4. CMake。
5. Ninja。
6. STM32CubeProgrammer。
7. 可选：ST-LINK 和 OpenOCD。

在工程 `README.md` 中记录实际使用版本，不要只写“最新版”：

```text
STM32CubeMX: x.y.z
STM32CubeF7: x.y.z
FreeRTOS kernel: x.y.z
arm-none-eabi-gcc: x.y.z
CMake: x.y.z
Ninja: x.y.z
```

STM32CubeMX 当前版本可以直接生成 CMake 工程；如果所用旧版本的 Toolchain/IDE 列表没有 `CMake`，不要改选多个 IDE 长期并行维护，应升级 CubeMX，或者暂时生成 `Makefile` 工程用于核对源文件后再建立项目 CMake。

---

## 4. 第一次创建 CubeMX 工程

### 4.1 选择 MCU

1. 打开 STM32CubeMX。
2. 选择 **New Project**。
3. 进入 **MCU/MPU Selector**。
4. 在 Part Number 中输入：

   ```text
   STM32F722RET6
   ```

5. 核对以下信息：

   | 项目 | 必须匹配 |
   |---|---|
   | MCU family | STM32F7 |
   | Part number | STM32F722RET6 |
   | Package | LQFP64 |
   | Flash | 512 KiB |
   | Core | Arm Cortex-M7 |

6. 单击 **Start Project**。

不要选择名称相近但 Flash、封装不同的 STM32F722Z、V、I 型号。

### 4.2 保存 `.ioc`

立即保存项目：

```text
E:\getfun-f722-freertos\GETFUN_F722_FreeRTOS.ioc
```

`.ioc` 是必须提交到 Git 的工程输入文件。以后每次重新生成代码前，先提交当前可运行状态。

---

## 5. System Core 配置

### 5.1 SYS

进入：

```text
Pinout & Configuration
  → System Core
  → SYS
```

设置：

| 选项 | 值 |
|---|---|
| Debug | Serial Wire |
| Timebase Source | TIM6 |

说明：

- `Serial Wire` 保留 PA13/SWDIO 和 PA14/SWCLK。
- 不选择完整 JTAG，以释放 PA15 给 Motor 1。
- FreeRTOS 使用 SysTick，因此 HAL 的 1 ms tick 改用 TIM6。
- TIM6 当前只用于 HAL tick，后续不得再分配给 DShot 或其他周期任务。

### 5.2 RCC

进入：

```text
System Core → RCC
```

设置：

| 选项 | 值 |
|---|---|
| High Speed Clock (HSE) | Crystal/Ceramic Resonator |
| Low Speed Clock (LSE) | Disable |

注意：当前 GETFUNF722V3 Betaflight 配置没有显式声明 `SYSTEM_HSE_MHZ`，已验证固件使用 STM32F7 默认 8 MHz HSE 配置运行。本文因此暂按 **8 MHz 晶振**配置，但它仍应通过晶振丝印、示波器/MCO 或其他硬件证据补充确认为冻结事实。

如果实测晶振不是 8 MHz，禁止照抄下一节 PLL 值，必须重新计算整个时钟树。

### 5.3 Cortex Interface Settings

不同 CubeMX 版本可能把这一页称为 `Cortex Interface Settings`，并把 Flash 加速和 Cortex-M7 CPU Cache 分开显示。按照界面逐项设置：

| CubeMX 界面项目 | 首次点亮设置 | 说明 |
|---|---|---|
| Flash Interface | AXI Interface | STM32F722 的 Flash 接口类型，通常为只读项，保持不变 |
| ART ACCELERATOR | Enabled | 启用 Flash 侧 ART 加速器 |
| Instruction Prefetch | Enabled | 启用 Flash 指令预取 |
| CPU ICache | Enabled | 启用 Cortex-M7 一级指令缓存 |
| CPU DCache | Disabled | 首次点亮阶段暂不启用数据缓存 |

也就是说，若当前页面初始状态与下图相同，需要把 `ART ACCELERATOR`、`Instruction Prefetch` 和 `CPU ICache` 从 `Disabled` 改为 `Enabled`，只保留 `CPU DCache = Disabled`。

ART/Prefetch 是 Flash 侧加速，CPU ICache/DCache 是 Cortex-M7 内核侧缓存，它们不是同一个开关。I-Cache 不涉及 DMA 数据一致性；D-Cache 要等 DMA 缓冲区布局及 clean/invalidate 策略确定后再启用。

如果当前 CubeMX 版本没有独立 CPU Cache 配置项，可在 `main.c` 的 `USER CODE BEGIN 1` 中调用：

```c
SCB_EnableICache();
```

此阶段不要调用 `SCB_EnableDCache()`。

---

## 6. 216 MHz 时钟树

进入 **Clock Configuration**。

### 6.1 HSE 与主 PLL

> CubeMX 联动提示：如果尚未启用需要 48 MHz 时钟的外设，`PLLQ` 可能显示为灰色且无法修改。这是正常现象。先完成本文 7.2 节的 `USB_OTG_FS → Device Only` 和 `USB_DEVICE → CDC` 配置，再返回 Clock Configuration；48 MHz 时钟域产生需求后，PLLQ 通常会自动变为可编辑。也可以先完成其他时钟参数，启用 USB 后再回来设置 PLLQ。

输入或核对：

| 参数 | 值 |
|---|---:|
| HSE input frequency | 8 MHz |
| PLL source | HSE |
| PLLM | 8 |
| PLLN | 432 |
| PLLP | 2 |
| PLLQ | 9 |
| System Clock Mux | PLLCLK |

计算关系：

```text
PLL input = 8 MHz / 8 = 1 MHz
PLL VCO   = 1 MHz × 432 = 432 MHz
SYSCLK    = 432 MHz / 2 = 216 MHz
PLLQ      = 432 MHz / 9 = 48 MHz
```

启用 USB 后，如果 PLLQ 仍是灰色：

1. 在 Clock Configuration 页面向右查看 `48MHz clocks Mux` 或 `CLK48` 时钟域。
2. 将 48 MHz 时钟源选择为主 PLL 的 `PLLQ` 输出，而不是 PLLSAI 或其他来源。
3. CubeMX 建立该连接后，回到主 PLL 区域，把 `PLLQ` 分频设置为 `/9`。
4. 核对 48 MHz 域最终显示 `48 MHz`，且页面没有红色冲突。

如果第一阶段暂时不启用 USB，PLLQ 可以暂时保持灰色；这不影响 PLLP 输出的 216 MHz CPU 主频，但启用 USB CDC 前必须完成 48 MHz 时钟配置。

### 6.2 总线分频

设置：

| 时钟 | 值 |
|---|---:|
| SYSCLK | 216 MHz |
| AHB Prescaler | /1 |
| HCLK | 216 MHz |
| APB1 Prescaler | /4 |
| PCLK1 | 54 MHz |
| APB2 Prescaler | /2 |
| PCLK2 | 108 MHz |
| 48 MHz clock | 48 MHz |

CubeMX 应自动启用电压调节 Scale 1、OverDrive 和正确的 Flash latency。生成后仍需检查 `SystemClock_Config()` 中是否包含 OverDrive 调用。

第一次工程暂不以 CubeMX 显示的 Timer clock 作为 DShot 设计依据。DShot 接入时，要重新确认 TIMPRE、APB 分频、Timer 输入时钟和 DMA 映射。

### 6.3 时钟配置验收

Clock Configuration 页面不得存在红色冲突，且必须显示：

```text
SYSCLK = 216 MHz
HCLK   = 216 MHz
PCLK1  = 54 MHz
PCLK2  = 108 MHz
USB/48 MHz domain = 48 MHz
```

![[05_STM32CubeMX新建FreeRTOS工程操作指南-1784699685291.webp]]
---

## 7. 第一阶段引脚配置

第一次生成只启用 UART4、USB FS、SWD、TIM6 和安全电机 GPIO。

### 7.1 UART4 调试口

在 Connectivity 中启用：

```text
UART4 → Asynchronous
```

核对引脚：

| 信号 | 引脚 |
|---|---|
| UART4_TX | PA0 |
| UART4_RX | PA1 |

参数：

| 参数 | 值 |
|---|---|
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Hardware Flow Control | None |
| Oversampling | 16 |

第一阶段使用阻塞发送即可，不启用 UART4 DMA，不启用 UART4 中断。

### 7.2 USB FS CDC/VCP

启用：

```text
Connectivity → USB_OTG_FS → Device Only
Middleware → USB_DEVICE → Communication Device Class (Virtual Port Com)
```

核对：

| 信号 | 引脚 |
|---|---|
| USB_DM | PA11 |
| USB_DP | PA12 |

必须将 **VBUS sensing** 设置为 Disable。STM32 USB VBUS 感知通常会使用 PA9，而本板 PA9 已冻结为 Motor 3，不能让 USB 配置占用 PA9。

USB 第一阶段只验证枚举和 VCP，不立即加入 MSP。

### 7.3 电机引脚安全状态

第一次工程不要把电机引脚配置成 Timer Alternate Function。先配置为普通 GPIO Output，并设置默认低电平：

| 电机 | 引脚 | 初始模式 | 初始电平 |
|---|---|---|---|
| Motor 1 | PA15 | GPIO_Output | Low |
| Motor 2 | PA10 | GPIO_Output | Low |
| Motor 3 | PA9 | GPIO_Output | Low |
| Motor 4 | PA8 | GPIO_Output | Low |
| Motor 5 | PC9 | GPIO_Output | Low |
| Motor 6 | PC8 | GPIO_Output | Low |
| Motor 7 | PB11 | GPIO_Output | Low |
| Motor 8 | PB10 | GPIO_Output | Low |

GPIO 参数建议：

```text
Output Push Pull
No pull-up and no pull-down
Low output level
Low speed
```

为引脚设置 User Label：`MOTOR1` 至 `MOTOR8`。

生成代码后确认 `MX_GPIO_Init()` 在配置 GPIO 输出模式之前，先用 `HAL_GPIO_WritePin(..., GPIO_PIN_RESET)` 写入低电平。

第一版仅使用 Motor 1～4，但 Motor 5～8 也保持无脉冲安全状态。

### 7.4 第一阶段暂不配置的引脚

以下引脚只记录，不在第一次工程中启用对应外设：

| 功能 | 引脚/总线 | 首次工程处理 |
|---|---|---|
| ICM42688P | SPI1 PA5/PA6/PA7，CS PA4 | 暂不启用 |
| CRSF | UART2 PA2/PA3 | 暂不启用 |
| VBAT/Current | ADC3 PC0/PC1 | 暂不启用 |
| DPS310 | I2C1 PB8/PB9 | 暂不启用 |
| MAX7456 | SPI2 PB13/PB14/PB15，CS PB12 | 暂不启用 |
| W25Q128FV | SPI3 PB3/PB4/PB5，CS PC13 | 暂不启用 |
| LED Strip | PB1/TIM3_CH4 | 暂不启用 |
| Buzzer | PB0，反相行为待实测 | 保持复位状态 |
| PINIO1/PINIO2 | PC4/PB2 | 保持复位状态 |
| LED0/LED1 | PC15/PC14 | 保持复位状态 |

不要为了让 Pinout 页面“看起来完整”一次开启全部外设。每增加一个外设，都应形成单独可回退的提交。

---

## 8. FreeRTOS 配置

### 8.1 启用 FreeRTOS

进入：

```text
Middleware and Software Packs → FREERTOS
```

建议选择：

```text
Interface: CMSIS_V2
```

CubeMX 负责生成启动包装代码；后续 `rtos/` 和飞控模块内部可以直接使用 FreeRTOS 原生 API。不要再运行 Betaflight scheduler。

### 8.2 基础内核参数

建议初始值：

| 参数 | 建议值 |
|---|---:|
| `configTICK_RATE_HZ` | 1000 |
| `configMAX_PRIORITIES` | 8 |
| `configMINIMAL_STACK_SIZE` | 128 words 或 CubeMX 默认值以上 |
| `configUSE_PREEMPTION` | 1 |
| `configUSE_TIME_SLICING` | 1 |
| `configSUPPORT_STATIC_ALLOCATION` | 1 |
| `configSUPPORT_DYNAMIC_ALLOCATION` | 1，启动阶段暂留 |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 |
| `configUSE_MALLOC_FAILED_HOOK` | 1 |
| `configUSE_IDLE_HOOK` | 0 |
| `configUSE_TICK_HOOK` | 0 |

初始阶段保留动态分配，是为了兼容 CubeMX/CMSIS-RTOS 自动生成对象；业务任务、队列和缓冲区仍优先使用静态创建。等所有对象都转为静态后，再把动态分配关闭。

### 8.3 默认任务

将默认任务改名为：

```text
InitTask
```

建议：

| 项目 | 值 |
|---|---|
| Priority | Normal |
| Stack | 至少 512 words；以生成界面单位为准 |
| Allocation | Static（如果界面提供） |

第一次只创建这一个任务，不要提前建立 ImuTask、FlightTask、RcTask、MspTask 和 BlackboxTask。

### 8.4 FreeRTOS 中断优先级

STM32F722 实现 4 位 NVIC 优先级，数值越小优先级越高。建议：

```text
configLIBRARY_LOWEST_INTERRUPT_PRIORITY = 15
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
```

规则：

- 会调用 `...FromISR()` 的中断，其抢占优先级数值必须大于或等于 5。
- 0～4 保留给完全不调用 FreeRTOS API 的高紧急中断。
- USB、UART DMA、IMU Data Ready 等接入时逐项登记优先级。
- 不要凭“High/Very High”文字判断，必须记录最终数值。

---

## 9. Project Manager 与代码生成

进入 **Project Manager**。

### 9.1 Project 设置

| 项目 | 建议值 |
|---|---|
| Project Name | `GETFUN_F722_FreeRTOS` |
| Project Location | `E:\getfun-f722-freertos` 的父级或按界面规则选择 |
| Application Structure | Advanced（如可选） |
| Toolchain / IDE | CMake |
| Default Compiler/Linker | GCC |
| Firmware Package | 已安装并记录版本的 STM32CubeF7 |

确认生成结果不会出现重复目录，例如：

```text
E:\getfun-f722-freertos\GETFUN_F722_FreeRTOS\GETFUN_F722_FreeRTOS
```

生成前从路径预览中核对一次最终位置。

### 9.2 Code Generator 设置

建议：

- 勾选 `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`。
- 勾选保留用户代码区域。
- 选择复制工程实际需要的库文件，不依赖某台电脑上的绝对 Cube Repository 路径。
- 不把整个未使用的 STM32CubeF7 包复制进仓库。
- 保留 `.ioc` 和生成所用固件包版本信息。

CubeMX 再生成时，只保证 `USER CODE BEGIN/END` 区域内的手写代码不被覆盖。正式业务代码应放在独立的 `app/`、`bsp/`、`drivers/`、`protocol/`、`flight/` 和 `rtos/` 文件中。

### 9.3 生成代码

单击 **Generate Code**。

生成后先不要烧录，依次检查：

- [ ] `.ioc` 文件存在。
- [ ] CMake 文件存在。
- [ ] `startup_stm32f722xx.s` 存在。
- [ ] STM32F722RE 链接脚本存在。
- [ ] `system_stm32f7xx.c` 存在。
- [ ] FreeRTOS 源码及 `FreeRTOSConfig.h` 存在。
- [ ] USB Device CDC 文件存在。
- [ ] TIM6 被 HAL timebase 使用。
- [ ] SysTick 被 FreeRTOS 使用。

---

## 10. 生成后必须进行的源码审查

### 10.1 链接脚本

确认 Flash 起始地址和长度：

```ld
FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
```

第一次工程不要预留参数 Flash 扇区。等参数存储方案明确后，再单独修改链接脚本并记录 STM32F7 扇区边界。

确认 RAM 区域来自 STM32F722RE 的实际内存布局，不要复制其他 F7 型号的链接脚本。

### 10.2 系统时钟

检查 `SystemClock_Config()`：

- HSE 被启用。
- PLLM=8、PLLN=432、PLLP=2、PLLQ=9。
- OverDrive 被启用。
- AHB=/1、APB1=/4、APB2=/2。
- HAL 时钟配置成功后 `SystemCoreClock` 为 216000000。

所有 HAL 返回值失败时都必须进入可诊断的 `Error_Handler()`，不能静默继续运行。

### 10.3 GPIO 安全顺序

检查 `MX_GPIO_Init()`：

1. 先使能 GPIO 时钟。
2. 先将 Motor 1～8 写为 Low。
3. 再设置为 GPIO Output。

任何复位、错误处理或调度器启动失败路径，都不得把电机引脚切换为 Timer 输出。

### 10.4 编译参数

检查生成的 GCC 参数是否匹配 STM32F722：

- CPU 为 Cortex-M7。
- Thumb 指令集。
- FPU 和 float ABI 在所有目标及库中一致。
- Debug 带调试符号和较低优化。
- Release 开启优化但保留 MAP 文件。
- 使用 `-ffunction-sections`、`-fdata-sections` 和链接垃圾回收。

不要在不同静态库之间混用 hard-float 与 soft-float ABI。

---

## 11. 加入最小启动日志与心跳

### 11.1 启动日志

在 CubeMX 用户代码区或独立 `app/app_init.c` 中，通过 UART4 输出：

```text
GETFUN F722 FreeRTOS
MCU: STM32F722RET6
SYSCLK: 216000000
RTOS: starting
```

初始验证可使用阻塞发送：

```c
static void debug_write(const char *text)
{
    HAL_UART_Transmit(&huart4,
                      (const uint8_t *)text,
                      (uint16_t)strlen(text),
                      100U);
}
```

需要包含 `<string.h>`。该函数只用于启动和低频调试，不能在未来的 1 kHz 飞行任务或中断中使用。

### 11.2 InitTask 心跳

在 `InitTask` 中输出低频心跳：

```c
void StartInitTask(void *argument)
{
    uint32_t counter = 0U;

    debug_write("RTOS: started\r\n");

    for (;;) {
        char line[48];
        int length = snprintf(line,
                              sizeof(line),
                              "heartbeat %lu\r\n",
                              (unsigned long)counter++);

        if (length > 0) {
            HAL_UART_Transmit(&huart4,
                              (const uint8_t *)line,
                              (uint16_t)length,
                              100U);
        }

        osDelay(1000U);
    }
}
```

需要包含 `<stdio.h>`。如果不希望初期引入 `snprintf` 的代码体积，也可以固定输出 `heartbeat\r\n`。

所有手写内容必须位于 CubeMX 用户代码区域，或者放在不会被 CubeMX 重写的独立文件中。

---

## 12. 构建工程

### 12.1 检查工具

在 PowerShell 或开发终端执行：

```powershell
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

### 12.2 配置和编译

CubeMX 不同版本生成的 CMake toolchain 文件路径可能不同。先查找生成的 `gcc-arm-none-eabi.cmake`，再执行类似命令：

```powershell
cmake -S . -B build/debug -G Ninja `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

cmake --build build/debug
```

如果 CubeMX 已生成 `CMakePresets.json`，优先使用生成并审查过的 preset：

```powershell
cmake --preset debug
cmake --build --preset debug
```

构建必须得到：

```text
*.elf
*.hex
*.bin
*.map
```

如果生成项目没有自动生成 HEX/BIN，在 CMake 中使用 `arm-none-eabi-objcopy` 添加 post-build 命令，不要手工每次转换。

### 12.3 构建结果检查

执行：

```powershell
arm-none-eabi-size path\to\firmware.elf
```

确认：

- Flash 使用量明显小于 512 KiB。
- RAM 使用量没有越界。
- 链接 MAP 中没有意外引入大量未使用组件。
- 无未解决符号和 ABI 不匹配警告。

---

## 13. 第一次烧录与验证

### 13.1 推荐烧录顺序

优先级：

1. ST-LINK/SWD，便于查看 HardFault 和寄存器。
2. STM32CubeProgrammer 通过 ROM DFU。

第一次不要连接电池和电机动力电源，只使用 USB 或受限实验电源给逻辑部分供电，并保持无桨。

### 13.2 DFU 烧录

如果使用 ROM DFU：

1. 让飞控进入 BOOT/DFU。
2. 在 STM32CubeProgrammer 中连接 USB DFU。
3. 核对设备为 STM32F7，Flash 为 512 KiB。
4. 烧录生成的 HEX，地址应从 `0x08000000` 开始。
5. 执行 Verify。
6. 退出 DFU 并重新上电。

不要在未核对目标和文件地址时执行全片擦除。

### 13.3 首次运行验收

- [ ] UART4 为 115200 8N1。
- [ ] 上电后打印 MCU 和 SYSCLK。
- [ ] `HAL_RCC_GetSysClockFreq()` 返回 216000000。
- [ ] `HAL_RCC_GetHCLKFreq()` 返回 216000000。
- [ ] `HAL_RCC_GetPCLK1Freq()` 返回 54000000。
- [ ] `HAL_RCC_GetPCLK2Freq()` 返回 108000000。
- [ ] `RTOS: started` 只出现一次。
- [ ] 心跳每约 1 秒输出一次，并持续至少 30 分钟。
- [ ] USB 能枚举为 CDC/VCP。
- [ ] PA9 没有被 USB VBUS sensing 占用。
- [ ] Motor 1～8 引脚没有周期脉冲。
- [ ] 复位和断开 USB 后仍可再次进入 ROM DFU。

通过后创建 Git 标签，例如：

```text
v0.1.0-platform-baseline
```

---

## 14. 推荐 Git 初始化与提交顺序

在独立工程目录初始化 Git：

```powershell
git init
git branch -M main
```

`.gitignore` 至少忽略：

```gitignore
build/
cmake-build-*/
.vscode/
.idea/
*.launch
*.log
*.hex
*.bin
*.elf
*.map
```

如果计划把经过验证的发布 HEX 一并归档，应放入专门的 `releases/` 目录，并调整忽略规则，不要把普通构建目录提交到 Git。

推荐提交顺序：

```text
docs: add project design and hardware baseline
build: add cubemx cmake project for stm32f722ret6
board: configure 216 mhz clock and safe motor gpio
bsp: add uart4 startup diagnostics
rtos: add minimal init task and heartbeat
usb: enable usb cdc enumeration
```

每个提交都应能够独立构建；不要把后续 IMU、CRSF 和 DShot 代码塞进最小工程提交。

---

## 15. 后续外设接入顺序

最小工程通过后，按以下顺序重新打开 `.ioc`，每次只接入一组外设：

### 阶段 A：SPI1 与 ICM42688P

```text
SPI1_SCK  PA5
SPI1_MISO PA6
SPI1_MOSI PA7
GYRO_CS   PA4
```

先轮询读取 WHO_AM_I，再设计 DMA 和 Data Ready。当前 v0.3.0 已完成：

```text
SPI1_RX  DMA2 Stream 0 / Channel 3 / High / Normal
SPI1_TX  DMA2 Stream 3 / Channel 3 / High / Normal
IRQ      抢占优先级 5
```

原始板级配置没有定义 IMU INT 外部引脚，因此 Data Ready 当前通过
`INT_STATUS.DATA_RDY_INT` 门控，不配置 GPIO EXTI，也不得占用 PC4/PINIO1。

v0.4.0在APP层增加陀螺静态零偏校准，不修改CubeMX外设配置，并已通过实物验收。
校准软件和验收见
`11_v0.4.0_陀螺静态零偏校准开发计划.md`与
`12_v0.4.0_陀螺静态零偏校准软件交付与实物验收.md`。下一算法里程碑是
`v0.5.0-accel-calibration-params-baseline`；阶段B仍表示下一个需要CubeMX接入的
外设，但应在IMU校准、滤波和姿态基础链完成后实施。

### 阶段 B：UART2 与 CRSF

```text
UART2_TX PA2
UART2_RX PA3
Baud     420000
```

先中断/轮询验证帧，再加入循环 DMA。不要在中断中完整解析 CRSF。

### 阶段 C：ADC3

```text
PC0 VBAT
PC1 Current
PC2 RSSI ADC
PC3 External ADC
```

先输出原始值，实物标定后再冻结比例。

### 阶段 D：DShot300

```text
Motor 1 PA15 TIM2_CH1
Motor 2 PA10 TIM1_CH3
Motor 3 PA9  TIM1_CH2
Motor 4 PA8  TIM1_CH1
```

DShot 接入前必须重新设计 Timer/DMA；不能把 Betaflight 的 DMA option 直接当成 CubeMX DMA Stream 配置。始终先进行无桨逻辑分析仪测试。

### 阶段 E：附加设备

```text
SPI2 + MAX7456
SPI3 + W25Q128FV
I2C1 + DPS310
LED、Buzzer、PINIO
```

这些功能不阻塞第一版飞行。

---

## 16. 常见问题排查

### 16.1 上电没有日志

依次检查：

1. UART4 TX 是否确实为 PA0。
2. 串口工具是否为 115200 8N1。
3. USB 转串口与飞控是否共地。
4. `MX_UART4_Init()` 是否在首次发送前执行。
5. 程序是否进入 `Error_Handler()`。
6. 启动地址是否为 `0x08000000`。
7. BOOT 状态是否导致 MCU 再次进入 ROM DFU。

### 16.2 卡在时钟初始化

优先怀疑：

- 实际 HSE 不是 8 MHz。
- HSE 没有起振。
- PLL 参数与输入频率不匹配。
- OverDrive 没有成功启用。
- 电源电压或供电不稳定。

可先用 HSI 建立低频诊断固件，不能为了“先跑起来”而把错误 HSE 值继续写成硬件事实。

### 16.3 FreeRTOS 启动后 HardFault

检查：

- 任务栈是否过小。
- FPU/float ABI 是否一致。
- 中断优先级是否违反 FreeRTOS 规则。
- HAL tick 是否仍错误占用 SysTick。
- 静态任务内存是否具有完整生命周期。
- 是否在调度器启动前使用了只能在调度器运行后调用的 API。

### 16.4 USB 不枚举

检查：

- 48 MHz 时钟是否准确。
- PA11/PA12 是否被其他功能占用。
- USB Device Class 是否选择 CDC。
- VBUS sensing 是否已关闭。
- USB 中断优先级是否满足 FreeRTOS 约束。
- USB 描述符的 VID/PID、字符串和端点配置是否有效。

### 16.5 CubeMX 再生成后手写代码丢失

原因通常是手写代码位于 `USER CODE BEGIN/END` 之外。恢复后应把业务代码移入独立源文件，只在生成文件的用户区保留调用入口。

---

## 17. 最小工程完成定义

只有全部满足以下条件，才开始 ICM42688P 驱动：

- [ ] 工程目录与 Betaflight 仓库独立。
- [ ] `.ioc`、CMake 和源码已提交 Git。
- [ ] Debug 与 Release 均可从干净目录构建。
- [ ] ELF、HEX、BIN、MAP 均能生成。
- [ ] 主频和各总线频率经过运行时读取确认。
- [ ] UART4 连续运行 30 分钟无异常。
- [ ] FreeRTOS tick 和 HAL tick 工作正常。
- [ ] USB CDC 可以重复连接和断开。
- [ ] Motor 1～8 在复位、启动、错误状态下均无有效脉冲。
- [ ] ROM DFU 恢复路径仍然可用。
- [ ] 首次基线固件已记录 SHA-256。
- [ ] 尚未实测的配置仍明确标注为待确认。

完成后保存以下资料：

```text
CubeMX .ioc
CubeMX 和 STM32CubeF7 版本
Git commit
ELF/HEX SHA-256
UART 启动日志
时钟运行时读数
USB 枚举截图或设备信息
Motor 引脚无脉冲的逻辑分析仪记录
```

这些资料组成后续驱动开发的 `v0.1.0-platform-baseline`。

---

## 18. 官方工具参考

- STM32CubeMX 用户手册 UM1718：`https://www.st.com/resource/en/user_manual/um1718-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf`
- STM32CubeMX 在线文档：`https://dev.st.com/stm32cube-docs/stm32cubemx/`
- STM32CubeMX 产品页：`https://www.st.com/en/development-tools/stm32cubemx.html`

CubeMX 的菜单名称可能随版本变化，但本文冻结的 MCU、引脚、安全约束和运行验收要求不应随界面变化。
