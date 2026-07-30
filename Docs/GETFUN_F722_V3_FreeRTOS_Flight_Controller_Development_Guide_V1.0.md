# GETFUN F722 V3 FreeRTOS 飞控项目总纲与开发路线

> 文档版本：V1.0  
> 目标硬件：GETFUN F722 V3  
> MCU：STM32F722RET6  
> 飞行器类型：四旋翼 Quad-X  
> 项目性质：个人学习与开发项目  
> 当前阶段：独立FreeRTOS固件阶段3——IMU校准、姿态和接收机<br>
> 基本安全要求：所有新固件和电机测试默认拆除螺旋桨

---

## 1. 项目目标

本项目的目标是在 GETFUN F722 V3 飞控板上，开发一套基于 FreeRTOS、能够脱离 Betaflight 独立运行的四旋翼飞控固件。

最终固件需要实现：

- FreeRTOS 任务调度
- ICM42688P 陀螺仪和加速度计驱动
- 姿态估计
- CRSF 遥控接收
- Rate 和 Angle 飞行模式
- PID 控制
- Quad-X Mixer
- DShot 电机输出
- ARM、DISARM 和基本 Failsafe
- 电池电压与电流检测
- 参数保存
- OSD 和 Blackbox
- USB 通信和固件恢复
- 与 Betaflight App 尽可能完整的调试兼容

项目完成的主要判断标准是：

1. 无人机可以正常解锁、起飞、悬停、操纵和降落。
2. Rate 和 Angle 模式工作正常。
3. 接收机失联、IMU 异常和通信断开时，电机能够进入安全状态。
4. Betaflight App 可以完成主要配置、观察和调试工作。
5. 固件刷写失败后，可以通过 BOOT/DFU 恢复。

Betaflight 在本项目中作为硬件和行为参考，不作为最终固件的运行框架。

---

## 2. 第一版功能范围

### 2.1 第一版必须实现

第一可飞版本包含：

- STM32F722 启动和 216 MHz 时钟
- FreeRTOS
- UART 调试日志
- USB VCP
- MSP 通信
- ICM42688P
- CRSF
- ADC 电压与电流采样
- 传感器校准
- Mahony 姿态估计
- Rate 模式
- Angle 模式
- PID
- RC Rates 和 Expo
- Quad-X Mixer
- DShot300
- Motor 1～4
- ARM、DISARM
- RC Failsafe
- IMU 故障保护
- 基本参数保存
- Betaflight App 主要调试页面

### 2.2 基础飞行完成后加入

- MAX7456 模拟 OSD
- W25Q128 Blackbox
- DPS310 气压计
- LED
- 蜂鸣器
- PINIO
- 更完整的 CLI
- Betaflight App 的 OSD 和 Blackbox 页面
- 更丰富的运行日志

### 2.3 暂时不做

- GPS
- 磁力计
- 定高
- 定点
- 自动返航
- 自动航线
- 自主起降
- Motor 5～8
- 双向 DShot
- RPM Filter
- ESC 遥测
- 多飞控板支持
- 其他 MCU 支持
- 自定义地面站
- Linux 伴随计算
- 视觉避障

这些功能在基础飞行稳定后再决定是否加入。

---

## 3. Betaflight 的作用

Betaflight 不属于最终运行架构，但在开发过程中承担四项作用。

### 3.1 硬件基线

用 Betaflight 确认：

- MCU 和时钟
- 传感器型号
- SPI、I2C 和 UART
- 电机引脚
- Timer 和 DMA
- ADC
- OSD
- Blackbox Flash
- USB 和 DFU

### 3.2 行为对照

自研固件的数据可以与 Betaflight 对比：

- IMU 原始值
- 姿态方向
- RC 通道
- 电压和电流
- 电机编号
- Failsafe
- 设备状态

### 3.3 代码参考

可以参考 Betaflight 中的：

- STM32F7 初始化
- ICM42688P 驱动
- CRSF
- MSP
- DShot
- MAX7456
- Flash
- Timer/DMA 配置

迁移代码时需要理解其依赖，不能直接把整个模块复制到 FreeRTOS 工程后假定能够正常工作。

### 3.4 上位机兼容参考

自研固件通过 MSP 与 Betaflight App 连接，使 Betaflight App 继续作为主要配置和调试工具。

---

## 4. 基本软件架构

自研工程采用相对简单的模块结构：

```text
GETFUN F722 V3
       │
       ▼
启动、时钟、GPIO、DMA、USB
       │
       ▼
     FreeRTOS
       │
       ├── ImuTask
       ├── RcTask
       ├── FlightTask
       ├── MspTask
       └── BlackboxTask
```

飞行数据链：

```text
ICM42688P
    │
    ▼
校准与滤波
    │
    ▼
姿态估计
    │
    ▼
Rate / Angle 控制
    │
    ▼
Quad-X Mixer
    │
    ▼
解锁与安全检查
    │
    ▼
DShot300
    │
    ▼
四个 ESC 和电机
```

遥控数据链：

```text
CR8 ELRS
    │
    ▼
UART2 / CRSF
    │
    ▼
通道、模式和 ARM 开关
    │
    ▼
Rate / Angle 设定值
```

调试链：

```text
Betaflight App
    │
    ▼
USB VCP / MSP
    │
    ├── 查看姿态
    ├── 查看接收机
    ├── 调整 PID
    ├── 配置模式
    ├── 测试电机
    ├── 配置 OSD
    └── 查看 Blackbox
```

---

## 5. FreeRTOS 任务规划

第一版不建立过多任务。

| 任务 | 建议运行方式 | 主要功能 |
|---|---|---|
| `ImuTask` | 数据就绪或固定高频 | 读取 ICM42688P、校准、滤波 |
| `FlightTask` | 1 kHz | 姿态估计、PID、Mixer、电机值 |
| `RcTask` | UART 接收事件 | CRSF 解包、通道、Failsafe |
| `MspTask` | USB 接收事件 | Betaflight App 通信 |
| `BlackboxTask` | 低优先级 | 写入 W25Q128 |

LED、蜂鸣器、参数保存等低频功能可以先放在低优先级维护逻辑中，不必为每个功能单独创建任务。

中断只负责：

- 接收或发送少量数据
- 更新时间戳
- 切换 DMA 缓冲区
- 通知任务运行

姿态估计、PID、MSP 解析和日志写入放在任务中完成。

---

## 6. Betaflight App 兼容目标

Betaflight App 是本项目的主要上位机，不另外开发地面站。

建议固定兼容环境：

```text
Betaflight App：2025.12.2
MSP API：1.48
MSP 参考源码：Betaflight commit 4146538b5
```

需要优先支持以下页面。

### Setup

- 固件和板卡信息
- 传感器状态
- 三维姿态
- 电池信息
- 解锁状态
- 加速度计校准
- 重启和进入 DFU

### Ports

- UART1～UART6
- MSP
- Serial RX
- 其他必要串口功能

### Configuration

- Quad-X
- CRSF
- 电机协议
- 传感器
- 电压和电流表
- 系统功能开关

### PID Tuning

- Rate PID
- Angle 参数
- Rates
- Expo
- 基本滤波参数

### Receiver

- CRSF 通道
- RSSI/LQ
- 通道映射
- Failsafe 状态

### Modes

- ARM
- ANGLE
- BEEPER
- PREARM
- 其他实际使用的模式

### Motors

- Motor 1～4 实时值
- 单电机测试
- 四电机联动测试
- DShot 配置

### OSD

实现 MAX7456 后支持：

- OSD 元素位置
- PAL/NTSC
- 告警项
- 字体

### Blackbox

实现 W25Q128 日志后支持：

- Flash 状态
- Flash 擦除
- 日志设置
- 日志读取

### CLI

至少支持：

```text
version
status
tasks
resource
timer
dma
serial
feature
get
set
save
defaults
diff
dump
reboot
dfu
```

没有实现的 Betaflight 功能不需要为了页面完整而编写无意义代码。

---

## 7. 基本安全要求

项目是个人项目，不建立复杂安全体系，但保留必要保护。

### 7.1 解锁条件

至少满足：

- IMU 已正常初始化
- IMU 数据持续更新
- CRSF 接收正常
- 未处于 Failsafe
- 油门位于低位
- ARM 开关状态有效
- 没有严重系统错误
- 当前不处于 App 电机测试

### 7.2 必须停止电机的情况

- DISARM
- RC 失联
- IMU 长时间无数据
- FlightTask 停止运行
- MSP 电机测试超时
- 固件重启
- 严重错误

### 7.3 电机测试

- 必须拆除螺旋桨
- Armed 时不接受 App 电机测试
- App 断开后立即停止
- 重启后不恢复上一次测试状态

---

## 8. 开发阶段

### 阶段 1：Betaflight 硬件基线

目标：

- 确认所有硬件资源
- 确认构建和烧录
- 保留原厂恢复能力

当前已经基本完成。

剩余工作：

- CRSF 实测
- 姿态方向实测
- 电机实测
- ADC 标定
- LED、蜂鸣器和 PINIO 实测
- 原厂固件实际回写
- SWD 调试

### 阶段 2：最小 FreeRTOS 工程

实现：

- 启动文件
- 链接脚本
- 时钟
- GPIO
- UART 日志
- FreeRTOS
- USB
- MSP 基础连接
- DFU 跳转

完成标志：

- 固件稳定启动
- App 能连接
- 可以查看版本和状态
- 可以重新进入 DFU

当前状态：平台、USB/MSP、ICM42688P轮询和DMA基线均已冻结。

### 阶段 3：IMU、姿态和接收机

实现：

- ICM42688P
- 加速度计校准
- Mahony
- CRSF
- RC Failsafe
- App Setup 和 Receiver 页面

完成标志：

- App 三维模型方向正确
- 遥控通道显示正常
- 接收机断线能够识别

### 阶段 4：控制和电机

实现：

- Rate 模式
- Angle 模式
- PID
- Rates
- Quad-X Mixer
- DShot300
- ARM/DISARM
- App PID、Modes、Motors 页面

完成标志：

- 无桨电机编号正确
- 电机方向正确
- 手动转动机体时修正方向正确
- 失联后电机停止

### 阶段 5：基础飞行

实施顺序：

1. Rate 模式低空测试
2. PID 调整
3. Angle 模式测试
4. 电压和 Failsafe 测试
5. 多次起飞、悬停和降落

完成标志：

- 可以重复正常起飞
- 可以稳定悬停
- 操纵方向正确
- 可以安全降落
- 无异常电机输出

### 阶段 6：功能完善

逐步加入：

- MAX7456 OSD
- W25Q128 Blackbox
- DPS310
- LED
- 蜂鸣器
- PINIO
- 更完整的 App 页面和 CLI

### 阶段 7：后续扩展

基础飞行稳定后再考虑：

- 双向 DShot
- RPM Filter
- GPS
- 高度控制
- Linux
- 视觉避障

---

## 9. 当前进度

已完成：

- GETFUNF722V3 Betaflight 自定义配置
- Betaflight 源码构建
- 自编译 HEX 烧录
- USB/MSP 连接
- STM32 ROM DFU
- ICM42688P 识别
- DPS310 识别
- W25Q128 识别
- MAX7456 识别
- Motor 1～8 资源映射
- Timer 和 DMA 归档
- UART 和 ADC 映射
- 原厂 4.5.1 HEX 完整性校验
- 独立FreeRTOS平台基线 `v0.1.0-platform-baseline`
- ICM42688P 1 kHz轮询基线 `v0.2.0-imu-polling-baseline`
- ICM42688P DMA/DRDY门控基线 `v0.3.0-imu-dma-drdy-baseline`
- 陀螺静态零偏校准基线 `v0.4.0-gyro-calibration-baseline`

当前下一步：

1. `v0.5.0-accel-calibration-params-baseline`：冻结参数Flash分区和记录格式。
2. 实现加速度水平单面校准、运动/非水平拒绝和校准状态。
3. 实现带magic、版本、长度和CRC的参数保存、加载及损坏回退。
4. 验证重启后校准保持、无效参数安全拒绝和Betaflight水平姿态基线。
5. v0.5.0冻结后实现软件低通、精确 `dt` 和Mahony姿态解算。

---

## 10. 专题文档

总纲只负责项目目标和路线，具体内容拆分到：

```text
01_GETFUN_F722_V3_硬件基线_构建烧录与恢复.md
02_FreeRTOS架构与硬件驱动.md
03_飞行控制与Betaflight_App兼容.md
04_调试步骤_飞行测试与开发记录.md
11_v0.4.0_陀螺静态零偏校准开发计划.md
12_v0.4.0_陀螺静态零偏校准软件交付与实物验收.md
```

各专题职责：

- `01`：冻结已经确认的引脚、外设、构建、固件校验值、烧录和恢复信息；未经新证据不得随意修改。
- `02`：FreeRTOS、任务、BSP 和设备驱动。
- `03`：姿态、PID、Mixer、DShot、MSP 和 App 页面。
- `04`：实际调试步骤、测试结果和问题记录。
- `11`：v0.4.0陀螺静态零偏校准的固定算法、接口和安全边界。
- `12`：v0.4.0软件交付、构建结果和真实硬件验收表。

---

## 11. 最终完成定义

满足以下条件时，可以认为基础项目完成：

- 自研 FreeRTOS 固件不依赖 Betaflight 运行。
- Betaflight App 可以完成主要配置和调试。
- ICM42688P、CRSF、ADC、DShot 工作正常。
- Rate 和 Angle 模式正常。
- 四个电机编号和方向正确。
- ARM、DISARM 和 Failsafe 正常。
- 无人机能够多次正常起飞、悬停、操纵和降落。
- OSD 和 Blackbox 可以正常使用。
- 固件可以正常升级，也能在刷写失败后恢复。
- 代码结构清晰到足以继续加入后续功能。
