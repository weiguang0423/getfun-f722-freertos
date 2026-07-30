# GETFUN F722 V3 FreeRTOS 架构与硬件驱动

> 文档版本：V1.0  
> 文档状态：开发设计稿  
> 目标：建立可以启动、调试、连接 Betaflight App，并逐步驱动板载器件的独立 FreeRTOS 工程  
> 硬件事实来源：[[01_GETFUN_F722_V3_硬件基线_构建烧录与恢复]]

---

## 1. 本文范围

本文说明自研固件的工程结构、启动流程、FreeRTOS 任务和底层驱动组织方式。

本文不重复维护具体引脚和 Timer/DMA 冻结值。实现中使用的板级资源必须以 `01_GETFUN_F722_V3_硬件基线_构建烧录与恢复.md` 为准。

第一阶段只追求：

- 工程可以重复构建和烧录。
- STM32F722 正常启动。
- FreeRTOS 稳定运行。
- UART 日志可用。
- USB VCP 和基础 MSP 可用。
- IMU、CRSF 和 DShot 能按开发顺序接入。
- 代码结构足够清晰，便于个人继续修改。

不为了“架构完整”提前建立暂时用不到的抽象层。

---

## 2. 建议工程结构

```text
GETFUN_F722_FreeRTOS/
├─ GETFUN_F722_FreeRTOS.ioc          # CubeMX 配置的唯一事实来源
│
├─ Core/                             # CubeMX 生成；不放业务模块
│  ├─ Inc/
│  └─ Src/
│     ├─ main.c                      # 时钟、外设初始化顺序、调度器启动
│     ├─ freertos.c                  # 仅保留 RTOS 生成入口和 USER CODE 调用点
│     ├─ gpio.c / usart.c / ...      # CubeMX 外设初始化与 HAL 句柄
│     └─ stm32f7xx_it.c              # 中断入口
│
├─ USB_DEVICE/                       # CubeMX 生成的 USB CDC 栈
│  ├─ App/
│  └─ Target/
│
├─ Drivers/                          # CubeMX/HAL/CMSIS；不放自研驱动
├─ Middlewares/                      # CubeMX/FreeRTOS/USB 中间件
│
├─ App/                              # 手写业务代码；CubeMX 不生成到这里
│  ├─ Inc/
│  │  ├─ app_init.h
│  │  ├─ app_state.h
│  │  ├─ app_diagnostics.h
│  │  ├─ board/
│  │  ├─ bsp/
│  │  ├─ drivers/
│  │  ├─ protocol/
│  │  ├─ flight/
│  │  ├─ rtos/
│  │  └─ storage/
│  │
│  └─ Src/
│     ├─ app_init.c
│     ├─ app_state.c
│     ├─ app_diagnostics.c
│     ├─ board/
│     │  ├─ board_getfun_f722_v3.c
│     │  └─ board_getfun_f722_v3.h
│     ├─ bsp/
│     │  ├─ timebase.c
│     │  ├─ usb_cdc_transport.c
│     │  ├─ spi_bus.c
│     │  └─ motor_output.c
│     ├─ drivers/
│     │  ├─ icm42688p.c
│     │  ├─ dps310.c
│     │  ├─ crsf_receiver.c
│     │  └─ w25q128.c
│     ├─ protocol/
│     │  ├─ msp_transport.c
│     │  ├─ msp_server.c
│     │  └─ cli.c
│     ├─ flight/
│     │  ├─ imu.c
│     │  ├─ attitude.c
│     │  ├─ control.c
│     │  ├─ mixer.c
│     │  └─ safety.c
│     ├─ rtos/
│     │  ├─ app_tasks.c
│     │  └─ task_monitor.c
│     └─ storage/
│        ├─ parameters.c
│        └─ blackbox.c
│
├─ cmake/                            # 已有 CubeMX 导出的 CMake 配置
├─ CMakeLists.txt
└─ README.md
```

个人项目不要求一次建立全部文件。建议随着模块真正开始实现时再创建对应目录和文件。

---

## 3. 底层技术选择

### 3.1 CMSIS、HAL 与 LL

建议采用混合方式：

- CMSIS：启动、内核寄存器、NVIC、SysTick、FPU。
- STM32 HAL：USB、早期外设跑通和不敏感的低频功能。
- STM32 LL 或寄存器：SPI/DMA、Timer、DShot 和时间敏感路径。

不要求为了“纯寄存器”重写所有内容，也不建议所有实时路径都依赖阻塞式 HAL 调用。

### 3.2 FreeRTOS

FreeRTOS 是唯一调度器，不同时运行 Betaflight scheduler。

建议：

- 使用静态任务和静态队列。
- 调度器启动后尽量不动态申请内存。
- 使用任务通知处理高频单一事件。
- 使用队列传递低频命令。
- 不在中断中执行协议解析、姿态估计、PID 或日志格式化。

### 3.3 构建系统

独立工程建议使用 CMake + Ninja + `arm-none-eabi-gcc`。

最少构建类型：

- Debug：断言和调试信息完整。
- Release：开启优化，用于台架和飞行。

第一版不需要复杂的自动发布或 CI。

---

## 4. 启动流程

建议启动顺序：

```text
Reset_Handler
    ↓
初始化栈、.data、.bss
    ↓
保持电机 GPIO 为安全状态
    ↓
检查是否请求进入 ROM DFU
    ↓
SystemInit / FPU / Cache
    ↓
时钟配置
    ↓
基础 GPIO 与 UART 日志
    ↓
时间基准
    ↓
外设与设备初始化
    ↓
创建 FreeRTOS 对象和任务
    ↓
启动调度器
```

启动阶段的优先原则是：即使时钟、IMU 或参数初始化失败，也不能让电机引脚输出有效脉冲。

### 4.1 时钟

目标主频为 216 MHz。需要正确配置：

- HSE/PLL
- AHB/APB 分频
- Timer 时钟
- USB 48 MHz 时钟
- Flash wait states

开发初期可以先使用已知稳定的时钟配置，不必立即加入复杂的时钟回退策略。

### 4.2 FPU 与 Cache

STM32F722 使用 Cortex-M7 FPU。编译器、启动代码和链接库必须使用一致的浮点 ABI。

启用 D-Cache 后，DMA 缓冲区要注意缓存一致性。简单做法是：

- 将关键 DMA 缓冲区放入适合的内存区域；或
- DMA 前后按需要执行 Cache clean/invalidate。

在没有验证 Cache 与 DMA 前，不要同时优化多个外设。

### 4.3 链接脚本

第一版应明确：

- 向量表位置
- 程序 Flash 区域
- 参数存储区域
- RAM、栈和堆
- DMA 缓冲区

Betaflight 基线从 `0x08000000` 启动，但独立工程的参数扇区必须单独规划，不能直接假定与 Betaflight 完全一致。

---

## 5. FreeRTOS 任务

第一版建议只使用五个主要任务。

| 任务 | 优先级关系 | 触发方式 | 职责 |
|---|---|---|---|
| `ImuTask` | 最高或与 Flight 接近 | 数据就绪/定时通知 | SPI 读取、校准、滤波、发布 IMU 数据 |
| `FlightTask` | 高 | 1 kHz 通知 | 姿态估计、控制器、Mixer、安全输出请求 |
| `RcTask` | 中高 | UART/DMA 事件 | CRSF 解包、通道和失联状态 |
| `MspTask` | 中 | USB RX 事件 | MSP/CLI、App 参数读写 |
| `BlackboxTask` | 低 | 队列/周期 | Flash 日志写入 |

低频维护功能可以暂时放入 `MspTask` 的空闲处理或增加一个低优先级 `MaintenanceTask`，不必为 LED、蜂鸣器和参数分别创建任务。

### 5.1 ImuTask

第一版可以采用：

```text
IMU Data Ready / 定时器
        ↓
通知 ImuTask
        ↓
SPI 读取一帧
        ↓
单位换算、校准、滤波
        ↓
更新 IMU 快照和时间戳
        ↓
通知 FlightTask
```

如果第一版尚未接入 Data Ready，引入固定周期读取也可以，但要记录实际采样间隔。

当前 `v0.4.0` 软件仍由 ImuTask 以 1 tick 的 `vTaskDelayUntil()` 周期轮询
ICM42688P 的 `INT_STATUS.DATA_RDY_INT`，但 14 字节样本事务已改为 SPI1 RX/TX
DMA。DMA 完成 ISR 只恢复 CS、保存结果并用任务通知唤醒 ImuTask；解析、SI换算、
`CW90`、陀螺静态零偏校准和发布均留在任务上下文。校准要求连续250个预热样本和
2000个Welford统计样本，只有窗口标准差通过后才进入READY并扣除机体系零偏。
任务静态栈512 words、优先级 `tskIDLE_PRIORITY+4`；InitTask保持 `osPriorityIdle`，
避免1 Hz UART诊断抢占采样。

原始板级目标没有定义 ICM42688P INT1/INT2 的 STM32 引脚，PC4 已属于 PINIO1，
因此当前版本仍不启用 GPIO DRDY/EXTI。DMA/DRDY 设计见
`09_v0.3.0_IMU_DMA与DRDY开发计划.md`，软件交付和实物验收见
`10_v0.3.0_IMU_DMA与DRDY软件交付与实物验收.md`；陀螺校准设计和验收见
`11_v0.4.0_陀螺静态零偏校准开发计划.md`与
`12_v0.4.0_陀螺静态零偏校准软件交付与实物验收.md`。

### 5.2 FlightTask

目标周期 1 kHz。每次执行：

1. 取得最新 IMU 快照。
2. 检查数据是否过期。
3. 更新姿态估计。
4. 读取 RC 快照和飞行模式。
5. 计算 Rate/Angle 控制。
6. 执行 Quad-X Mixer。
7. 经过安全检查后提交 DShot 值。

### 5.3 RcTask

RcTask 负责 UART 接收缓冲、CRSF 帧校验和状态更新。FlightTask 不直接读取 UART 缓冲区。

### 5.4 MspTask

MspTask 负责 Betaflight App 和 CLI。App 查询数据时读取系统快照，修改参数时通过参数接口提交，不直接改正在运行的驱动寄存器。

### 5.5 BlackboxTask

BlackboxTask 只处理已经排队的日志记录。Flash 擦除或长时间写操作不能阻塞 FlightTask。

---

## 6. 中断与任务通信

中断中只做必要工作：

- 读取和清除中断标志。
- 记录时间戳。
- 切换 DMA 缓冲区。
- 更新少量计数器。
- 使用 `...FromISR` API 通知任务。

禁止在中断中：

- 完整解析 MSP 或 CRSF。
- 进行浮点姿态计算。
- 执行 PID。
- 格式化日志字符串。
- 等待 SPI/UART 完成。

共享数据优先采用简单快照：

```c
typedef struct {
    uint32_t timestamp_us;
    uint32_t sequence;
    bool valid;
    float gyro[3];
    float accel[3];
} ImuSample;
```

生产者更新完整快照后再更新序号；消费者确认读取前后序号一致，避免读到半更新数据。

---

## 7. BSP 接口建议

保持接口简单，先满足本板使用。

```c
void board_init(void);
uint32_t time_us(void);
void delay_us(uint32_t us);

bool spi_transfer(uint8_t bus, uint8_t cs, const uint8_t *tx, uint8_t *rx, size_t len);
bool uart_write(uint8_t port, const uint8_t *data, size_t len);
size_t uart_read(uint8_t port, uint8_t *data, size_t max_len);

uint16_t adc_read_vbat_raw(void);
uint16_t adc_read_current_raw(void);

void motor_output_enable(bool enable);
void motor_write_dshot(const uint16_t values[4]);
```

如果后续只有这一块板，不需要为每个接口设计复杂的运行时设备注册系统。

---

## 8. 驱动实现顺序

### 8.1 第一批：系统可运行

1. GPIO 安全状态
2. UART4 日志
3. 微秒时间基准
4. FreeRTOS
5. USB VCP
6. ROM DFU 跳转

完成结果：App 可以连接，CLI 可以显示版本和任务状态。

### 8.2 第二批：基础飞行输入

1. SPI1
2. ICM42688P
3. UART2 DMA
4. CRSF
5. ADC3

完成结果：App 可以显示姿态原始数据、RC 通道和电池信息。

### 8.3 第三批：电机

1. Timer/DMA 或选定 DShot 方案
2. DShot300 编码
3. Motor 1～4
4. 输出开关和超时归零

完成结果：App Motors 页面可以完成无桨测试。

### 8.4 第四批：附加功能

1. SPI2 / MAX7456
2. SPI3 / W25Q128
3. I2C1 / DPS310
4. LED、蜂鸣器和 PINIO

---

## 9. 设备驱动最低要求

### 9.1 ICM42688P

需要实现：

- [x] 复位
- [x] WHO_AM_I
- [x] Gyro/Accel 量程
- [x] ODR
- [ ] 软件滤波
- [x] 轮询连续读取
- [x] DRDY状态门控
- [x] SPI1 DMA双缓冲读取
- [ ] GPIO DRDY/EXTI（硬件引脚未冻结）
- [x] RTOS tick 时间戳
- [x] 错误计数
- [x] 简单重试

驱动输出使用统一物理单位：角速度建议 rad/s，加速度建议 m/s²；如果为方便对照使用 deg/s 和 g，必须在数据结构名称或注释中明确。

### 9.2 CRSF

需要实现：

- 420000 baud
- 帧长度和 CRC
- RC Channels Packed
- Link Statistics
- 通道更新时间
- 失联判定

### 9.3 DShot

需要实现：

- 16 位 DShot 帧
- 校验位
- DShot300 时序
- 四路更新
- 停止值
- 输出超时保护

具体控制算法见 `03_飞行控制与Betaflight_App兼容.md`。

### 9.4 ADC

先输出原始 ADC 和简单换算值。比例校准后再冻结最终电压和电流参数。

### 9.5 MAX7456、W25Q128、DPS310

这些不阻塞第一版飞行。基础飞行完成后，再根据 Betaflight 已验证的总线和片选逐个迁移。

---

## 10. 参数保存

第一版只需要简单参数系统：

```c
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    FlightConfig flight;
    RcConfig rc;
    MotorConfig motor;
    MspConfig msp;
    uint32_t crc;
} PersistentConfig;
```

要求：

- 有 magic、版本、长度和 CRC。
- 参数损坏时加载默认值。
- Armed 时不写内部 Flash。
- `save` 后再统一写入。
- 参数结构变化时提升版本号。

不需要第一版就实现 Betaflight 完整 PG 参数系统。

---

## 11. 日志与故障信息

UART 日志至少包含：

- 固件版本
- 启动原因
- 时钟结果
- 设备初始化结果
- FreeRTOS 任务启动
- IMU/CRSF 错误
- 解锁禁止原因
- Failsafe 原因
- HardFault 基本寄存器

高频任务只增加计数器或写入固定大小事件，不直接打印大量字符串。

---

## 12. 本专题完成标准

- [ ] 独立工程可以构建 ELF、HEX 和 MAP。
- [ ] 固件可以通过 DFU 或 SWD 写入。
- [ ] UART4 可以输出启动日志。
- [ ] FreeRTOS 五个基础任务可以运行。
- [ ] USB VCP 可以与 Betaflight App 连接。
- [ ] 可以通过 App 或 CLI 重启进入 ROM DFU。
- [ ] ICM42688P 可以持续输出有效数据。
- [ ] CRSF 可以输出通道和链路状态。
- [ ] ADC 可以读取电压和电流原始值。
- [ ] DShot300 可以在无桨条件下控制 Motor 1～4。
- [ ] 未解锁、复位和错误状态下电机保持停止。
