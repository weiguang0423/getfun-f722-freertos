# GETFUN F722 V3 飞行控制与 Betaflight App 兼容

> 文档版本：V1.0  
> 文档状态：开发设计稿  
> 目标：实现能够正常飞行的 Rate/Angle 控制链，并使用 Betaflight App 完成主要配置和调试  
> 硬件事实来源：[[01_GETFUN_F722_V3_硬件基线_构建烧录与恢复]]  
> RTOS 与驱动来源：[[02_FreeRTOS架构与硬件驱动]]

---

## 1. 本文范围

本文负责从 IMU 数据到电机输出的完整飞行链，并规定自研固件怎样映射到 Betaflight App。

第一版目标：

- 姿态方向正确。
- Rate 模式可飞。
- Angle 模式可飞。
- CRSF 控制正常。
- Quad-X Mixer 正确。
- DShot300 四电机输出正常。
- ARM、DISARM 和基本 Failsafe 正常。
- Betaflight App 可以完成主要调试。

---

## 2. 坐标和单位

第一版必须先统一坐标约定，不能在驱动、姿态和 Mixer 中分别修正方向。

建议机体系：

```text
X：机头方向
Y：机体右侧
Z：机体向下
Roll：绕 X 轴
Pitch：绕 Y 轴
Yaw：绕 Z 轴
```

ICM42688P 的板级方向基线为 `CW90_DEG`，但最终轴变换必须通过实物动作确认。

建议内部单位：

| 数据 | 内部单位 |
|---|---|
| 角速度 | rad/s 或 deg/s，项目内固定一种 |
| 加速度 | m/s² 或 g，项目内固定一种 |
| 角度 | degree，便于 App 显示 |
| 时间 | µs |
| PID dt | second |
| 电机逻辑值 | 0.0～1.0 |

不要在同一控制函数内混用 deg/s 和 rad/s。

---

## 3. IMU 数据处理

数据链：

```text
ICM42688P 原始值
        ↓
轴向变换
        ↓
零偏和比例校准
        ↓
低通滤波
        ↓
姿态估计和控制器
```

### 3.1 Gyro 校准

上电后保持机体静止，采集一段角速度样本并计算三轴零偏。

校准失败条件可以保持简单：

- 采样数量不足。
- 采样期间运动过大。
- 数据不更新。
- 数值明显越界。

校准未完成时禁止解锁。

### 3.2 Accelerometer 校准

第一版至少支持水平单面校准，用于 Angle 模式。后续可以增加六面校准。

### 3.3 滤波

第一版不复制 Betaflight 的完整滤波链。建议先使用：

- Gyro 一阶或二阶低通。
- Accelerometer 低通。
- 可通过 App 调整基本截止频率。

飞行稳定后再决定是否增加动态滤波、Notch 或 RPM Filter。

---

## 4. 姿态估计

第一版采用 Mahony 或等价轻量四元数算法。

输入：

- 三轴角速度
- 三轴加速度
- 实际 `dt`

输出：

- 四元数
- Roll
- Pitch
- Yaw

无磁力计时：

- Roll 和 Pitch 由加速度长期修正。
- Yaw 主要依赖陀螺仪积分，允许缓慢漂移。
- Rate 模式不应依赖绝对 Yaw。

### 4.1 姿态调试顺序

1. App 显示原始 Gyro/Accel。
2. 分别沿三个机体轴转动，确认轴和符号。
3. 静止时确认 Roll/Pitch 接近零。
4. 向右倾斜、向前俯仰，确认三维模型方向。
5. 快速动作后回到水平，确认姿态能够回归。

姿态方向没有确认前，禁止进入真实电机闭环测试。

---

## 5. 遥控输入和设定值

CRSF 通道转换为标准范围：

```text
Roll/Pitch/Yaw：-1.0 ～ +1.0
Throttle：0.0 ～ 1.0
AUX：按开关范围判定
```

处理顺序：

```text
CRSF 原始通道
      ↓
通道映射与端点
      ↓
Deadband
      ↓
Rates / Expo
      ↓
角速度或角度设定值
```

第一版需要支持 Betaflight App 中的：

- Channel Map
- 通道中点
- 通道最小/最大值
- RC Deadband
- Rates
- Expo
- ARM 和 ANGLE AUX 范围

---

## 6. Rate 模式

Rate 模式中，摇杆直接生成目标角速度：

```text
Roll stick  → Roll rate setpoint
Pitch stick → Pitch rate setpoint
Yaw stick   → Yaw rate setpoint
```

控制误差：

```text
error = desired_rate - measured_rate
```

第一版 PID：

```text
P = Kp × error
I = I + Ki × error × dt
D = Kd × derivative
output = P + I + D
```

需要具备：

- I 项限幅。
- 油门低位或撤锁时重置积分。
- 输出限幅。
- `dt` 异常时跳过或限制更新。
- NaN/Inf 时立即撤销本周期控制输出。

D 项可以先对测量值求导，降低设定值跳变带来的冲击。

---

## 7. Angle 模式

Angle 模式采用角度外环 + Rate 内环：

```text
摇杆
  ↓
目标 Roll/Pitch 角度
  ↓
角度误差 × Angle P
  ↓
目标 Roll/Pitch 角速度
  ↓
Rate PID
```

Yaw 第一版仍使用 Rate 控制。

第一版只需提供：

- 最大倾角。
- Angle P。
- Rate PID。

不需要一开始实现 Horizon、定高或位置控制。

---

## 8. Quad-X Mixer

Mixer 输入：

- Throttle
- Roll correction
- Pitch correction
- Yaw correction

Mixer 输出：

- Motor 1～4 的归一化逻辑值

物理电机位置和旋转方向尚未实测冻结，因此代码中的最终符号矩阵必须在 `01` 的电机实测完成后确定。

验证方法：

- 增加 Roll correction，左右两侧电机变化方向应相反。
- 增加 Pitch correction，前后电机变化方向应相反。
- 增加 Yaw correction，CW 与 CCW 电机变化方向应相反。
- 输出超范围时统一缩放或限幅，不能发生整数回绕。

Betaflight App 中显示的 Motor 1～4 顺序必须与实际焊盘和 Mixer 一致。

---

## 9. DShot300 输出

DShot 帧为 16 位：

```text
11 位 throttle/command
1 位 telemetry
4 位 checksum
```

第一版需要实现：

- 停止值。
- 有效油门范围映射。
- 校验值。
- 四路同步或足够接近的更新。
- 1 kHz 控制周期提交。
- 撤锁和超时归零。

DShot 开发顺序：

1. 在逻辑分析仪观察单路帧。
2. 确认 0/1 脉宽和帧周期。
3. 无桨测试单个电机。
4. 无桨测试 Motor 1～4。
5. 确认编号和旋转方向。
6. 最后接入 Mixer 输出。

---

## 10. ARM、DISARM 与 Failsafe

个人项目使用简单状态即可：

```text
BOOT
  ↓
DISARMED
  ↓ ARM 条件满足
ARMED
  ↓ DISARM 或故障
DISARMED
```

### 10.1 ARM 条件

- IMU 初始化成功。
- Gyro 校准完成。
- IMU 数据持续更新。
- CRSF 正常。
- 未处于 Failsafe。
- 油门低位。
- ARM 开关有效。
- 不处于 App 电机测试。

### 10.2 立即停止电机

- DISARM 开关。
- CRSF 失联。
- IMU 数据超时。
- FlightTask 停止更新。
- 系统严重错误。
- App 电机测试超时或断开。

故障恢复后不自动 ARM，必须重新操作 ARM 开关。

---

## 11. 参数系统

第一版参数分组：

```text
SystemConfig
SensorConfig
RcConfig
RateConfig
PidConfig
MixerConfig
MotorConfig
FailsafeConfig
OsdConfig
BlackboxConfig
```

Betaflight App 写入参数时，MSP 层转换到这些内部结构。

参数修改分两种：

- 可即时生效：PID、Rates、Expo、部分 OSD 设置。
- 保存并重启生效：串口、传感器总线、控制周期、电机协议等。

`save` 时统一写入 Flash。参数损坏时加载默认值。

---

## 12. Betaflight App 兼容基线

固定调试环境：

```text
Betaflight App：2025.12.2
MSP API：1.48
参考 Betaflight commit：4146538b5
```

该 App 版本应保留安装包。未来升级 App 时，如果页面无法使用，优先对照这个固定版本。

自研固件内部不必使用 Betaflight 数据结构，但 MSP 返回长度、字段顺序和数值范围必须符合对应 API。

---

## 13. MSP 软件结构

```text
USB RX
  ↓
MSP 字节流解析
  ↓
命令分发
  ↓
状态读取 / 参数修改
  ↓
MSP 应答
```

建议接口：

```c
void msp_process_bytes(const uint8_t *data, size_t len);
bool msp_handle_command(uint16_t cmd, const uint8_t *payload, size_t len);

void msp_fill_status(...);
void msp_fill_raw_imu(...);
void msp_fill_attitude(...);
void msp_fill_rc(...);
void msp_fill_motor(...);
```

收到暂未实现的命令时返回 MSP error，不要断开连接或返回错误长度。

---

## 14. App 页面实现顺序

### 14.1 连接与 Setup

优先实现：

```text
MSP_API_VERSION
MSP_FC_VARIANT
MSP_FC_VERSION
MSP_BOARD_INFO
MSP_BUILD_INFO
MSP_STATUS / MSP_STATUS_EX
MSP_RAW_IMU
MSP_ATTITUDE
MSP_ANALOG
```

目标：App 稳定连接，Setup 页面和三维模型工作。

### 14.2 Receiver

实现：

- RC 通道
- RX 配置
- Channel Map
- Link 状态
- Failsafe 状态

目标：Receiver 页面可完成 CRSF 调试。

### 14.3 Modes

实现 AUX Range 的读取和写入，至少支持：

- ARM
- ANGLE
- BEEPER
- PREARM（如果实际启用）

### 14.4 Configuration 与 Ports

实现本项目实际使用的：

- UART1～UART6
- MSP
- Serial RX
- Mixer
- CRSF
- DShot
- ADC meter
- 传感器和功能开关

### 14.5 PID Tuning

实现：

- Roll/Pitch/Yaw Rate PID
- Angle P
- Rates
- Expo
- 基本滤波参数

Betaflight 参数与内部算法不完全一致时，在 MSP 适配层做明确换算。

### 14.6 Motors

实现：

- Motor 1～4 当前值。
- 电机测试值写入。
- DShot 配置。

Armed 时拒绝 App 电机测试；App 命令超时或断开后立即归零。

### 14.7 OSD 与 Blackbox

基础飞行完成后实现：

- MAX7456 OSD 配置和布局。
- W25Q128 状态、擦除、日志配置和读取。

---

## 15. CLI

CLI 与 MSP 共用参数和状态接口。第一版至少支持：

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

CLI 输出不必逐字符复制 Betaflight，但常用命令和含义尽量保持一致。

---

## 16. 调试顺序

1. App 可以稳定连接并显示版本。
2. Setup 页面显示原始 IMU。
3. 三维姿态方向正确。
4. Receiver 页面显示 CRSF 通道。
5. Modes 页面可以配置 ARM 和 ANGLE。
6. PID 页面可以读写并保存参数。
7. Motors 页面显示逻辑输出。
8. 拆桨后完成真实电机测试。
9. 接入 Mixer 和 ARM。
10. 完成 Rate 模式台架验证。
11. 完成 Angle 模式台架验证。
12. 进入首飞测试。

---

## 17. 本专题完成标准

- [ ] 原始 IMU 和姿态能够在 App 中显示。
- [ ] 姿态方向通过实物动作验证。
- [ ] CRSF 通道和 Failsafe 能在 App 中观察。
- [ ] ARM 和 ANGLE 模式可以在 App 中配置。
- [ ] PID、Rates 和 Expo 可以读写和保存。
- [ ] Rate 模式控制方向正确。
- [ ] Angle 模式控制方向正确。
- [ ] Quad-X Mixer 与 Motor 1～4 顺序正确。
- [ ] DShot300 四路无桨测试通过。
- [ ] App Motors 页面断开后电机自动停止。
- [ ] 无人机可以正常起飞、悬停、操纵和降落。
- [ ] 基础飞行后 OSD 和 Blackbox 可以在 App 中配置。
