# GETFUN F722 V3 硬件基线、构建烧录与恢复

> 文档版本：V1.0  
> 冻结日期：2026-07-22  
> 适用硬件：GETFUN F722 V3  
> 状态说明：本文件用于冻结已经确认的板级事实；未实测内容必须明确标注，不得把配置值直接写成实测结论。

---

## 1. 冻结规则

本文中的信息分为三类：

- **已确认**：已有固件输出、文件校验或实机识别结果支持。
- **已配置**：已经写入板级配置，但对应外部功能尚未完成实物测试。
- **待确认**：当前没有足够证据，后续测试后再更新。

已经冻结的引脚、器件、总线或校验值不能仅凭推测修改。确需修改时，应同时记录：

1. 修改前的值。
2. 修改后的值。
3. 判断依据。
4. 测试结果。
5. 对应固件或源码版本。

---

## 2. 飞控身份

| 项目 | 冻结值 | 状态 |
|---|---|---|
| 硬件名称 | GETFUN F722 V3 | 已确认 |
| Manufacturer ID | `GFUN` | 已确认 |
| Board Name | `GETFUNF722V3` | 已确认 |
| MCU | STM32F722RET6 | 已确认 |
| Betaflight MCU Target | `STM32F7X2` | 已确认 |
| 内部 Flash | 512 KiB | 已确认 |
| 运行主频 | 216 MHz | 已确认 |
| USB | STM32 USB VCP / ROM DFU | 已确认 |

原厂参考基线：

```text
Betaflight 4.5.1
commit: 77d01ba3b
MSP API: 1.46
manufacturer_id: GFUN
board_name: GETFUNF722V3
```

当前已烧录并完成设备识别的自编译基线：

```text
Betaflight 2026.6.0-alpha
firmware commit: 4146538b5
MSP API: 1.48
config rev reported by firmware: 7b1f01a
build time: 2026-07-21 18:07:13
```

2026-07-22 检查时，Betaflight 主工作区 HEAD 为：

```text
bb80690ffc2036a653ee5cb427ce58c3df19ccc2
```

该工作区提交不等同于已烧录固件提交。重新构建后必须重新记录版本和 HEX 校验值。

---

## 3. 板载器件

| 功能 | 器件 | 总线与片选 | 启动识别结果 | 状态 |
|---|---|---|---|---|
| 陀螺仪/加速度计 | ICM42688P | SPI1，CS PA4 | `GYRO: ICM42688P`、`ACC: ICM42688P` | 已确认 |
| 气压计 | DPS310 | I2C1，PB8/PB9 | `BARO: DPS310`，I2C 0 errors | 已确认 |
| 模拟 OSD | MAX7456 | SPI2，CS PB12 | `MAX7456 (30 x 13)` | 已确认芯片通信，待视频画面验证 |
| Blackbox Flash | W25Q128FV | SPI3，CS PC13 | JEDEC `0x00ef4018`，16MB | 已确认 |
| 接收机 | CR8 ELRS | UART2 / CRSF | 板级串口已配置 | 待通道、LQ 和 Failsafe 实测 |

ICM42688P 板级方向配置：

```c
#define GYRO_1_ALIGN CW90_DEG
```

原始 GETFUNF722V3 目标没有定义 `GYRO_1_EXTI_PIN`，现有资料也没有冻结
ICM42688P INT1/INT2 到 STM32 的连接。PC4 已定义为 `PINIO1_PIN`，不得把其他
F722 板卡的连接套用到本板。自研固件 v0.3.0 因此使用
`INT_STATUS.DATA_RDY_INT` 状态门控；只有取得原理图、PCB 连通性或实测证据后，
才能增加 GPIO DRDY/EXTI。

该配置来自原厂基线并已写入目标文件；实际姿态方向仍需通过机体动作验证。

MAX7456 必须同时启用模拟 OSD 构建：

```c
#define USE_OSD_SD
#define USE_MAX7456
#define MAX7456_SPI_INSTANCE SPI2
#define MAX7456_SPI_CS_PIN PB12
```

正确启动结果：

```text
osd_displayport_device = AUTO
OSD: MAX7456 (30 x 13)
B12: OSD_CS
```

---

## 4. SPI、I2C 与 UART

### 4.1 SPI

| 总线 | SCK | MISO/SDI | MOSI/SDO | 设备 |
|---|---|---|---|---|
| SPI1 | PA5 | PA6 | PA7 | ICM42688P，CS PA4 |
| SPI2 | PB13 | PB14 | PB15 | MAX7456，CS PB12 |
| SPI3 | PB3 | PB4 | PB5 | W25Q128FV，CS PC13 |

### 4.2 I2C

| 总线 | SCL | SDA | 设备 |
|---|---|---|---|
| I2C1 | PB8 | PB9 | DPS310；预留磁力计扩展 |

### 4.3 UART

| 串口 | TX | RX | 当前用途/说明 |
|---|---|---|---|
| UART1 | PB6 | PB7 | 可配置 |
| UART2 | PA2 | PA3 | 默认 CRSF 接收机 |
| UART3 | PC10 | PC11 | 可配置 |
| UART4 | PA0 | PA1 | 自研固件调试串口候选 |
| UART5 | PC12 | PD2 | 可配置 |
| UART6 | PC6 | PC7 | 可配置 |

CRSF 接线约定：

```text
CR8 TX  → 飞控 RX2 / PA3
CR8 RX  ← 飞控 TX2 / PA2
CR8 5V  → 飞控 5V
CR8 GND → 飞控 GND
```

---

## 5. 电机、Timer 与 DMA

### 5.1 电机与 LED Strip

| 资源 | 引脚 | Timer 映射 | 状态 |
|---|---|---|---|
| Motor 1 | PA15 | TIM2_CH1 / AF1 | DShot600 GPIO bitbang，文档26无桨验收通过 |
| Motor 2 | PA10 | TIM1_CH3 / AF1 | DShot600 GPIO bitbang，文档26无桨验收通过 |
| Motor 3 | PA9 | TIM1_CH2 / AF1 | DShot600 GPIO bitbang，文档26无桨验收通过 |
| Motor 4 | PA8 | TIM1_CH1 / AF1 | DShot600 GPIO bitbang，文档26无桨验收通过 |
| Motor 5 | PC9 | TIM8_CH4 / AF3 | 已配置，第一版不使用 |
| Motor 6 | PC8 | TIM8_CH3 / AF3 | 已配置，第一版不使用 |
| Motor 7 | PB11 | TIM2_CH4 / AF1 | 已配置，第一版不使用 |
| Motor 8 | PB10 | TIM2_CH3 / AF1 | 已配置，第一版不使用 |
| LED Strip | PB1 | TIM3_CH4 / AF2 | 已配置，待实测 |

默认 QUADX 只激活 Motor 1～4，因此运行时 Motor 5～8 显示 `FREE` 属于正常情况。

### 5.2 已归档 DMA 选项

```text
dma ADC 3 0
dma pin A15 0
dma pin A10 0
dma pin A09 0
dma pin A08 0
dma pin C09 0
dma pin C08 0
dma pin B11 0
dma pin B10 0
dma pin B01 0
```

S4.2当前候选直接采用Betaflight在STM32F7上的默认DShot bitbang路径：TIM8_CH1作
1.8 MHz DMA pacer，DMA2 Stream2/Channel7写GPIOA BSRR，一次传输同步驱动M1～M4。
它没有占用CRSF的DMA1 Stream5、ADC3的DMA2 Stream1或SPI1的DMA2 Stream0/3。
软件、时序和四路同步性已按文档26通过逻辑分析仪及无桨联合验收。

自研固件 v0.3.0 为 SPI1 样本读取分配 `DMA2 Stream 0 / Channel 3`（RX）和
`DMA2 Stream 3 / Channel 3`（TX）。这是当前独立工程的软件资源选择，不代表
Betaflight 的归档 DMA option，也不得据此推导 Motor/DShot DMA 映射。

自研固件 v0.8.0 为USART2 CRSF接收分配 `DMA1 Stream 5 / Channel 4`（RX），使用
128字节循环DMA和UART IDLE事件；PA2/PA3继续保持既有CR8双线接法。通道、LQ、
断链与恢复仍必须按文档20完成真实硬件验收，软件构建不能关闭S1.5关口。

---

## 6. ADC、GPIO 与默认值

| 功能 | 引脚/外设 | 冻结状态 |
|---|---|---|
| VBAT | PC0 / ADC3 IN10 | 自研软件已接入，待仪表标定 |
| Current | PC1 / ADC3 IN11 | 自研软件已接入，待仪表标定 |
| RSSI ADC | PC2 / ADC3 IN12 | 自研软件保留原始值，默认未使用 |
| External ADC | PC3 / ADC3 IN13 | 自研软件保留原始值 |
| ADC DMA | 归档option 0；自研DMA2 Stream1/Channel2 | 已接入，待实物验证 |
| Buzzer | PB0 | 已配置为反相，待实测 |
| PINIO1 | PC4 | 已配置，待实测 |
| PINIO2 | PB2 | 已配置，待实测 |
| LED0 | PC15 | 已配置，待实测 |
| LED1 | PC14 | 已配置，待实测 |
| LED Strip | PB1 | 已配置，待实测 |
| SWDIO | PA13 | 保留为 SWD |
| SWCLK | PA14 | 保留为 SWD |

原配置默认值：

```text
Voltage meter source: ADC
Current meter source: ADC
Voltage meter scale: 110
Current meter scale: 100
Blackbox device: Flash
Serial RX UART: UART2
PINIO1 config: 129
PINIO2 config: 129
PINIO1 box: 40
PINIO2 box: 41
```

电压表比例 110 和电流表比例 100 只是原配置基准，不能替代实物标定。
自研固件因SPI1 RX已占DMA2 Stream0，使用STM32F722允许的ADC3备用映射
DMA2 Stream1/Channel2；50 Hz单次四通道DMA契约与标定步骤见文档23/24。

---

## 7. 自定义 Betaflight 目标

板级配置位置：

```text
E:\betaflight\src\config\configs\GETFUNF722V3\config.h
```

当前配置文件 SHA-256：

```text
930B8A9DEE99B99F21200E45CAD73BCD1CC63B0F5EBC2FE19896B65E7D0CC1FE
```

`src/config` 属于独立 Git 子模块。GETFUNF722V3 不在官方公共 config 列表中，因此后续必须采用以下一种方式保存：

- 提交到个人 config Fork，并更新主仓库子模块指针。
- 在个人项目仓库中单独保存 `config.h` 和复制说明。

不能只提交 Betaflight 主仓库而遗漏该自定义目标。

---

## 8. 已验证构建环境

构建环境：

```text
Windows + WSL Ubuntu
Arm GNU Toolchain 13.3.Rel1
```

安装项目本地工具链：

```bash
make arm_sdk_install
```

工具链本体位置：

```text
tools/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi
```

`tools` 是编译器本体，继续编译时不能删除；`downloads` 是下载缓存，可以清理。工具链不提交到 Git，其他电脑首次构建时重新执行安装命令。

已验证构建命令：

```bash
cd /mnt/e/betaflight
make clean
make GETFUNF722V3
```

构建输出：

```text
obj/betaflight_2026.6.0-alpha_STM32F7X2_GETFUNF722V3.hex
obj/main/betaflight_STM32F7X2_GETFUNF722V3.elf
obj/main/betaflight_STM32F7X2_GETFUNF722V3.map
```

当前已验证 HEX SHA-256：

```text
CF23BEC96D0E0E4C0731248DCD5E0ECB74BDB40B3546F0DA127898569F1B9871
```

该 HEX 对应 2026-07-21 18:07:13 构建的 `4146538b5` 固件。源码更新或重新构建后必须重新计算校验值。

只执行 `make GETFUNF722V3` 时，其他 MCU 平台不会进入最终固件，也不会增加 HEX 大小。不建议为了个人项目删除 `src/main`、`src/platform/common`、`src/platform/STM32` 或 `lib/main` 中的共享代码。

---

## 9. 烧录与恢复基线

已实机验证：

- STM32 ROM DFU 可以进入。
- 设备可识别 512 KiB 内部 Flash。
- 可以执行 512 KiB 全片擦除。
- 可以烧录自编译 HEX。
- 烧录后 USB VCP 和 Betaflight App 可以重新连接。

原厂恢复文件：

```text
文件：F722 V3 BF crsf协议固件4.5.1.hex
地址范围：0x08000000～0x08080000
覆盖大小：524288 bytes
Intel HEX 错误记录：0
SHA-256：77A7E468DB8E49910D43AB62FABA033A8926340DFDBA05E62DD64E2BE08F3030
```

原厂配置参考：

```text
F722 V3 配置文件.txt
```

ROM Bootloader 位于 STM32 System Memory，不会被用户 Flash 全片擦除。用户 Flash 全片擦除会删除飞控固件和配置，但仍可通过 BOOT 进入 ROM DFU。

原厂 HEX 已通过格式和地址完整性校验，但实际回写原厂固件并再次回到自编译基线的完整演练仍为待确认事项。

旧版 4.5.1 的完整配置不能直接导入当前 MSP 1.48 或未来 FreeRTOS 固件。跨版本只迁移能够明确对应的串口、模式、接收机、OSD、PID 和标定参数。

---

## 10. 已确认启动状态

当前自编译固件的正确启动摘要：

```text
MCU: STM32F722xx CLK=216MHz
GYRO: ICM42688P enabled
ACC: ICM42688P
BARO: DPS310
OSD: MAX7456 (30 x 13)
FLASH: JEDEC ID=0x00ef4018 16M
I2C errors: 0
```

正确 OSD 资源：

```text
B12: OSD_CS
DMA1 Stream 3: SPI_SDI 2
DMA1 Stream 4: SPI_SDO 2
```

---

## 11. 尚未冻结的实测项目

ICM42688P 的实际机体姿态方向已在 `v0.2.0`、`v0.3.0` 和 `v0.4.0` 的真实硬件
验收中连续通过，不再列为待确认项。

以下内容不得提前写成“已经正常”：

1. CR8 ELRS 的通道、LQ、遥测和 Failsafe。
2. Motor 1～4 的实体位置、编号、DShot 输出和旋转方向。
3. PC0 VBAT 和 PC1 Current 的实际标定值。
4. MAX7456 的模拟摄像头和图传画面。
5. PB0、PC14、PC15、PC4、PB2 和 PB1 的实际电气行为。
6. ST-LINK/SWD 的长期调试与 Flash 读回。
7. Option Bytes 归档。
8. 原厂 4.5.1 固件实际回写演练。
9. 当前 MSP 1.48 运行配置的完整 `diff all` 归档。

每完成一项，应把测试结果更新到本文件，并将状态从“待确认”改为“已确认”。
