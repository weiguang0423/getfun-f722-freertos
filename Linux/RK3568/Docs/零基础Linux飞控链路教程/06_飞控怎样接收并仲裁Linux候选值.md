---
title: 飞控怎样从第一个字节开始接收并仲裁Linux候选值
---

# 飞控怎样从第一个字节开始接收并仲裁 Linux 候选值

> **文档进度：`Linux候选帧 → [本篇：USART接收、验证快照、物理/虚拟仲裁] → 拆桨端到端验收`**
>
> **真实数据流位置：`PC7电平 → USART6字节 → 44字节验证 → rc_virtual_candidate_t → RcTask → 有效RC快照`**
>
> 本篇最重要的一句话：**收到 CRC 正确的 Linux 帧，只能产生一个候选；只有仲裁器满足全部安全条件时，候选 Roll/Pitch/Yaw 才能进入已有控制链。**

## 先看整条飞控内部路线和文件边界

上一篇停在 RK3568 TX 引脚。现在每个字节进入 F722 后，实际经过：

```text
RK3568 3.3V TX
  ↓
PC7 / USART6_RX，115200 8N1
  ↓ 每收到1字节触发HAL回调
linux_rc_monitor_uart_rx_byte(byte)
  ↓ 找GR帧头，收满44字节
CRC/格式/范围/时间/序号/会话验证
  ↓
rc_virtual_candidate_t（仅候选快照）
  ↓ RcTask约每20ms取一次
物理CRSF快照 + 飞行门禁 + AUX3授权 + Linux候选
  ↓ rc_source_arbiter_update
有效 mapped_channel_us
  ↓
已有 rc_setpoint → PID → Mixer → ARM/Failsafe → DShot
```

本篇使用的真实文件：

| 文件 | 单一职责 |
|---|---|
| `Core/Src/usart.c` | 初始化 USART6 和 PC6/PC7；逐字节接收中断 |
| `APP/Src/bsp/linux_rc_monitor.c/.h` | 同步、验帧并公布原子候选快照 |
| `APP/Src/algorithms/rc_source_arbiter.c/.h` | 纯 C 的物理/虚拟源选择和二次限速 |
| `APP/Src/rtos/rc_task.c` | 同时收集物理 CRSF 与 Linux 候选，发布唯一有效 RC |
| `APP/Src/platform/platform_diag.c` | 从 UART4 输出接收、仲裁和端到端诊断 |

这些工作树源文件受外部透明加密层包装，直接文本读取可能看到 `f6effsoftecrypt`。本篇依据 Git 中同路径的逻辑源码 `HEAD:APP/...` 走读，没有修改这些源文件。

历史文档记录 S7.7 在 2026-08-22 被确认验收通过。本轮没有烧录、没有连接物理 RC、没有看 UART4 实时日志，因此不把历史结果说成本轮实测。

## USART6 先把电平变成字节

> **当前位置：`RK3568已经在导线上发送 → [STM32硬件串口接收1字节] → 字节同步器`**
>
> 本节只讨论接收，不讨论帧是否可信。

`Core/Src/usart.c` 冻结：

```text
USART6 波特率：115200
数据位：8
停止位：1
校验：无
硬件流控：无
PC6：USART6_TX
PC7：USART6_RX
中断优先级：5
```

`UART_HandleTypeDef huart6` 是 HAL 保存 USART6 配置和运行状态的对象。HAL 是 ST 提供的硬件操作库。你现在只需要把 `huart6` 理解为“USART6 的软件档案”，里面记录波特率、收发状态和硬件实例。

`usart6_rx_byte` 是一个全局 8 位缓冲，只容纳这一次收到的字节。这里的“8 位”就是 1 字节，可表示 `0～255`。这块缓冲故意只有 1 字节，因为当前设计让每个字节一到达就立刻交给协议接收器，而不是在中断里处理一大批数据。

初始化末尾：

```c
if (HAL_UART_Receive_IT(&huart6, &usart6_rx_byte, 1U) != HAL_OK) {
    Error_Handler();
}
```

- `HAL_UART_Receive_IT` 来自 STM32 HAL；`IT` 表示 interrupt，中断方式。
- `&huart6` 指向要使用的串口对象。
- `&usart6_rx_byte` 是接收目标地址。
- `1U` 表示本次只接一个字节；`U` 表示无符号整数常量。
- 返回非 `HAL_OK` 表示连第一次接收都没挂好，初始化进入 `Error_Handler()`。这个函数是工程已有的严重初始化错误出口；本篇不展开它的内部实现。

收到 1 字节后，HAL 会调用接收完成回调。此时我们还不能写回调，因为回调要调用本篇自己的“字节接收函数”，而这个函数尚未定义。先把被调用能力完整建立，再回来接 HAL 回调。

为什么中断里不做仲裁、PID 或日志格式化？115200 8N1 的一个字节在线路上实际占 1 个起始位、8 个数据位和 1 个停止位，共约 10 bit。`10 ÷ 115200 ≈ 86.8 us`，所以字节会很快连续到来。中断只适合推进收字节状态并更新一个小快照；耗时的仲裁和日志由任务处理。

## 先把接收器需要保存的数据定义清楚

> **当前位置：`USART6能收到1字节 → [定义协议常量、候选和长期状态] → 完整验帧`**
>
> 本节还不调用任何项目函数，只回答“收到的字节放在哪里、为什么要长期保存”。

### 这些整数名字是什么意思

接下来的真实 C 源码会使用：

- `uint8_t`：无符号 8 位整数，正好保存 1 字节；
- `uint16_t`：无符号 16 位整数，保存 CRC 或两个字节拼成的数；
- `uint32_t`：无符号 32 位整数，保存计数、序号和毫秒时间；
- `uint64_t`：无符号 64 位整数，保存 Linux 微秒时间；
- `int16_t`：有符号 16 位整数，可保存 `-300～+300` 这样的轴偏移；
- `bool`：布尔值，只表达 `true` 或 `false`。

`uint` 开头可理解为 unsigned integer，无符号整数；`int` 是可带正负号的整数。后面的数字说明它占多少位。

### 协议常量为什么不能散落在代码里

真实源文件用宏给固定规则命名：

```c
#define LINUX_RC_FRAME_SIZE 44U
#define LINUX_RC_CRC_OFFSET 42U
#define LINUX_RC_FLAG_VALID 0x01U
#define LINUX_RC_GESTURE_MAX 4U
#define LINUX_RC_CHANNEL_LIMIT 300
#define LINUX_RC_THROTTLE_LIMIT 250
#define LINUX_RC_AUX_LIMIT 1000
#define LINUX_RC_LINK_TIMEOUT_MS 150U
#define LINUX_RC_SOURCE_TIMEOUT_US 250000ULL
```

- `FRAME_SIZE=44`：一帧总共 44 字节；
- `CRC_OFFSET=42`：前 42 字节参与 CRC，最后两个字节保存 CRC；
- `FLAG_VALID=0x01`：flags 的最低位表示候选是否有效；
- `GESTURE_MAX=4`：协议允许的最大手势编号；
- 三个 `LIMIT`：姿态轴、油门和 AUX 字段的绝对边界；
- `LINK_TIMEOUT_MS=150`：STM32 侧链路新鲜窗口；
- `SOURCE_TIMEOUT_US=250000`：Linux 从产生动作到发送帧最多允许 250000 us，也就是 250 ms。

宏不会在运行时占一份可修改变量。编译器会在使用处代入数值。这样写的目的不是炫技，而是让 `44`、`42`、`150` 各自表达明确含义。

### 候选、诊断和接收器状态

协议通过后要交给仲裁器的对象叫候选：

```c
typedef struct {
    bool valid;
    uint32_t received_ms;
    uint32_t source_sequence;
    uint32_t heartbeat;
    uint32_t session_generation;
    int16_t channel[5];
} rc_virtual_candidate_t;
```

- `valid`：这一帧是否声明了可用动作；
- `received_ms`：STM32 本地收到并通过验证的时刻；
- `source_sequence`：Linux 侧动作序号；
- `heartbeat`：Linux 每帧都前进的存活计数；
- `session_generation`：Linux 被判断为重启了多少次；
- `channel[5]`：依次保存 Roll、Pitch、Yaw、Throttle、AUX 的协议偏移值。

诊断结构保存“已经收到多少、拒绝了多少、最后通过的字段是什么”。它只用于观察和定位，不直接决定飞行：

```c
typedef struct {
    uint32_t received_bytes;
    uint32_t complete_frames;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t sequence_errors;
    uint32_t timestamp_errors;
    uint32_t session_reset_count;
    uint32_t uart_errors;
    uint32_t last_heartbeat;
    uint32_t last_source_sequence;
    int16_t last_channels[5];
    uint8_t last_gesture_id;
    uint8_t last_confidence_percent;
    bool last_frame_valid;
} linux_rc_monitor_diagnostics_t;
```

最后把正在拼的帧、候选和诊断放进同一个长期状态：

```c
typedef struct {
    linux_rc_monitor_diagnostics_t diagnostics;
    rc_virtual_candidate_t candidate;
    uint8_t frame[LINUX_RC_FRAME_SIZE];
    uint8_t frame_length;
    bool progression_initialized;
} linux_rc_monitor_state_t;

static linux_rc_monitor_state_t monitor;
```

- `frame` 是正在拼装的 44 字节；
- `frame_length` 是已经保存的字节数，也表示同步器当前进度；
- `progression_initialized` 表示是否已经有上一组序号可供比较；
- `static monitor` 从开机一直活到关机，而且只允许当前 `.c` 文件访问。

初始化要在启用 USART6 接收前完成。`memset(..., 0, ...)` 把状态的每个字节清零：

```c
void linux_rc_monitor_init(void)
{
    memset(&monitor, 0, sizeof(monitor));
}
```

此时 `frame_length=0`，表示等待第一个帧头字节；所有计数为 0；候选也是无效状态。

## 完整帧验证必须先于同步器中的第一次调用

> **当前位置：`长期状态已定义 → [从小端字节和CRC长出完整验帧] → 字节同步器调用它`**
>
> 同步器收满 44 字节后需要调用验帧函数。按照认知依赖顺序，本节先定义验帧依赖的全部能力，再允许同步器调用。

### 代码构思：验帧函数依赖哪些小能力

输入是 `monitor.frame` 中的 44 个外部字节。输出不是一个返回值，而是对长期状态的两类改变：失败时增加对应错误计数并保持候选无效；成功时更新诊断和候选。

```text
2/4/8字节小端读取
  ↓
CRC计算、范围检查、手势与通道语义检查
  ↓
完整validate_complete_frame
  ├─ 失败：错误计数+return
  └─ 成功：更新candidate
  ↓
同步器收满44字节后第一次调用
```

这里的“小端”表示低有效字节放在低地址。例如两个字节 `34 12` 合起来是十六进制 `0x1234`，不是 `0x3412`。

### 先定义小端读取函数

`get_u16_le()`、`get_u32_le()`、`get_u64_le()` 的名字分别表示“从小端字节读取 16/32/64 位无符号数”。参数 `data` 指向第一个字节；返回值是拼好的整数；它们不修改任何状态。

```c
static uint16_t get_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t get_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static uint64_t get_u64_le(const uint8_t *data)
{
    return (uint64_t)get_u32_le(data) |
           ((uint64_t)get_u32_le(data + 4U) << 32U);
}
```

`<< 8U` 表示把数值左移 8 位，相当于把第二个字节放到更高的字节位置。`|` 是按位或，用来合并互不重叠的各段。`data + 4U` 表示从第五个字节继续读。具体输入 `data={0x34,0x12}` 时，第一项是 `0x0034`，第二项是 `0x1200`，合并得到 `0x1234`。

### 再定义序号前进判断和 CRC

`sequence_is_newer(value, previous)` 判断当前 32 位序号是否比上一个新。转换为 `int32_t` 后检查差值大于 0，可以在差值不跨越半个 32 位范围的前提下自然处理回绕：

```c
static bool sequence_is_newer(uint32_t value, uint32_t previous)
{
    return (int32_t)(value - previous) > 0;
}
```

例如 `previous=100`、`value=101`，差值为 1，返回 true；两者都为 101，差值为 0，返回 false。

`crc16_ccitt_false(data, length)` 接收待保护字节地址和字节数，返回 16 位 CRC。初值是 `0xFFFF`，多项式是 `0x1021`，不修改接收器状态：

```c
static uint16_t crc16_ccitt_false(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t index;

    for (index = 0U; index < length; ++index) {
        uint32_t bit;

        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                    ? (uint16_t)((crc << 1U) ^ 0x1021U)
                    : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}
```

外层循环处理每个字节，内层循环处理这个字节的 8 个 bit。`& 0x8000U` 检查 CRC 当前最高位；三元运算符 `条件 ? A : B` 表示条件成立取 A，否则取 B。第 05 篇已经用 `G`、`R` 两个具体字节走读过同一算法；飞控必须使用完全相同的算法，才能得到相同结果。

### 范围正确仍不够，还要检查手势语义

`channels_in_range(frame)` 只回答每个数值是否落在协议硬边界内。它先用已经定义的 `get_u16_le()` 读取偏移 32～41 的五个 16 位字段，再转换为 `int16_t`：

```c
static bool channels_in_range(const uint8_t *frame)
{
    const int16_t roll = (int16_t)get_u16_le(&frame[32]);
    const int16_t pitch = (int16_t)get_u16_le(&frame[34]);
    const int16_t yaw = (int16_t)get_u16_le(&frame[36]);
    const int16_t throttle = (int16_t)get_u16_le(&frame[38]);
    const int16_t aux = (int16_t)get_u16_le(&frame[40]);

    return (roll >= -LINUX_RC_CHANNEL_LIMIT) &&
           (roll <= LINUX_RC_CHANNEL_LIMIT) &&
           (pitch >= -LINUX_RC_CHANNEL_LIMIT) &&
           (pitch <= LINUX_RC_CHANNEL_LIMIT) &&
           (yaw >= -LINUX_RC_CHANNEL_LIMIT) &&
           (yaw <= LINUX_RC_CHANNEL_LIMIT) &&
           (throttle >= 0) && (throttle <= LINUX_RC_THROTTLE_LIMIT) &&
           (aux >= 0) && (aux <= LINUX_RC_AUX_LIMIT);
}
```

`&&` 表示“并且”。五组条件必须全部成立才返回 true。

`channels_match_gesture(frame)` 再检查“这个通道方向是否真能由声明的手势产生”。当前协议中，Yaw、Throttle、AUX 必须为 0；中立手势必须三轴为 0；手掌向上/向下只允许 Pitch 对应方向；左移动作只允许 Roll 为负：

```c
static bool channels_match_gesture(const uint8_t *frame)
{
    const int16_t roll = (int16_t)get_u16_le(&frame[32]);
    const int16_t pitch = (int16_t)get_u16_le(&frame[34]);
    const int16_t yaw = (int16_t)get_u16_le(&frame[36]);
    const int16_t throttle = (int16_t)get_u16_le(&frame[38]);
    const int16_t aux = (int16_t)get_u16_le(&frame[40]);

    if ((yaw != 0) || (throttle != 0) || (aux != 0)) {
        return false;
    }
    if (frame[5] == 0U) {
        return (roll == 0) && (pitch == 0) && (frame[6] == 0U);
    }
    if (frame[5] == 1U) {
        return (roll == 0) &&
               (pitch >= -LINUX_RC_CHANNEL_LIMIT) && (pitch <= 0);
    }
    if (frame[5] == 3U) {
        return (roll == 0) &&
               (pitch >= 0) && (pitch <= LINUX_RC_CHANNEL_LIMIT);
    }
    if (frame[5] == 4U) {
        return (roll >= -LINUX_RC_CHANNEL_LIMIT) && (roll <= 0) &&
               (pitch == 0);
    }
    return false;
}
```

例如声明手势 1，却给出 `roll=+100`，虽然数值没超出 ±300，语义仍不匹配，函数返回 false。这样可以拦住“格式合法但含义矛盾”的帧。

### 现在才能完整定义验帧函数

`validate_complete_frame()` 没有参数，因为它固定读取 `monitor.frame`；也没有返回值，因为成功/失败结果写入 `monitor.diagnostics` 和 `monitor.candidate`。它只由下面尚未出现的字节同步器在收满 44 字节时调用。

完整实现还使用两个外部基础接口：

- `HAL_GetTick()` 来自 STM32 HAL，不接收参数，返回 MCU 从启动到现在经过的毫秒数；这里用它给通过验证的候选记录本地接收时刻；
- `memcpy(destination, source, byte_count)` 来自 C 标准库，把指定字节数从源地址复制到目标地址；这里把 5 个已验证通道整体复制进候选。它只复制字节，不替我们判断内容是否安全，所以必须放在所有验证通过之后。

```c
static void validate_complete_frame(void)
{
    linux_rc_monitor_diagnostics_t *const diagnostics = &monitor.diagnostics;
    const uint8_t *const frame = monitor.frame;
    const bool frame_valid = (frame[4] & LINUX_RC_FLAG_VALID) != 0U;
    const uint16_t expected_crc = get_u16_le(&frame[LINUX_RC_CRC_OFFSET]);
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t source_sequence = get_u32_le(&frame[8]);
    const uint32_t heartbeat = get_u32_le(&frame[12]);
    const uint64_t source_timestamp_us = get_u64_le(&frame[16]);
    const uint64_t send_timestamp_us = get_u64_le(&frame[24]);
    bool new_session = false;

    diagnostics->complete_frames++;
    monitor.candidate.valid = false;

    if (expected_crc != crc16_ccitt_false(frame, LINUX_RC_CRC_OFFSET)) {
        diagnostics->crc_errors++;
        return;
    }
    if ((frame[2] != 1U) || (frame[3] != LINUX_RC_FRAME_SIZE) ||
        ((frame[4] & ~LINUX_RC_FLAG_VALID) != 0U) ||
        (frame[5] > LINUX_RC_GESTURE_MAX) || (frame[6] > 100U) ||
        (frame[7] != 0U) || !channels_in_range(frame)) {
        diagnostics->format_errors++;
        return;
    }
    if (frame_valid &&
        ((frame[5] == 2U) ||
         ((frame[5] != 0U) && (frame[6] < 75U)) ||
         !channels_match_gesture(frame))) {
        diagnostics->format_errors++;
        return;
    }
    if (!frame_valid &&
        ((get_u16_le(&frame[32]) != 0U) ||
         (get_u16_le(&frame[34]) != 0U) ||
         (get_u16_le(&frame[36]) != 0U) ||
         (get_u16_le(&frame[38]) != 0U) ||
         (get_u16_le(&frame[40]) != 0U))) {
        diagnostics->format_errors++;
        return;
    }
    if ((send_timestamp_us < source_timestamp_us) ||
        (frame_valid &&
         ((source_timestamp_us == 0U) ||
          ((send_timestamp_us - source_timestamp_us) >
           LINUX_RC_SOURCE_TIMEOUT_US)))) {
        diagnostics->timestamp_errors++;
        return;
    }

    if (monitor.progression_initialized &&
        !sequence_is_newer(heartbeat, diagnostics->last_heartbeat)) {
        if ((uint32_t)(now_ms - monitor.candidate.received_ms) <
            LINUX_RC_LINK_TIMEOUT_MS) {
            diagnostics->sequence_errors++;
            return;
        }
        new_session = true;
    }
    if (frame_valid && monitor.progression_initialized && !new_session &&
        !sequence_is_newer(source_sequence,
                           diagnostics->last_source_sequence)) {
        diagnostics->sequence_errors++;
        return;
    }

    diagnostics->valid_frames++;
    diagnostics->last_frame_valid = frame_valid;
    diagnostics->last_gesture_id = frame[5];
    diagnostics->last_confidence_percent = frame[6];
    diagnostics->last_source_sequence = source_sequence;
    diagnostics->last_heartbeat = heartbeat;
    diagnostics->last_channels[0] = (int16_t)get_u16_le(&frame[32]);
    diagnostics->last_channels[1] = (int16_t)get_u16_le(&frame[34]);
    diagnostics->last_channels[2] = (int16_t)get_u16_le(&frame[36]);
    diagnostics->last_channels[3] = (int16_t)get_u16_le(&frame[38]);
    diagnostics->last_channels[4] = (int16_t)get_u16_le(&frame[40]);
    if (new_session) {
        diagnostics->session_reset_count++;
    }
    monitor.progression_initialized = true;
    monitor.candidate.valid = frame_valid;
    monitor.candidate.received_ms = now_ms;
    monitor.candidate.source_sequence = source_sequence;
    monitor.candidate.heartbeat = heartbeat;
    monitor.candidate.session_generation =
        diagnostics->session_reset_count;
    memcpy(monitor.candidate.channel, diagnostics->last_channels,
           sizeof(monitor.candidate.channel));
}
```

`const` 表示这次使用中不应通过该名字修改对象。`diagnostics->crc_errors` 中的 `->` 表示“通过结构体指针访问字段”。`return` 立即离开函数，因此后面的检查和成功更新都不会执行。

### 用一帧具体输入走读验帧

假设帧具有这些字段：版本 1、长度 44、valid 位为 1、gesture=1、confidence=90、source sequence=31、heartbeat=80、源时间 1000000 us、发送时间 1020000 us、Roll=0、Pitch=-300、其余三通道为 0，而且末尾 CRC 与前 42 字节计算结果相同。

1. `complete_frames` 加 1，并先把旧候选 `valid` 清为 false；
2. CRC 相等，继续；
3. 版本、长度、标志、手势、置信度、保留位和数值范围都通过；
4. 手势 1 配合负 Pitch，语义通过；
5. `1020000-1000000=20000 us`，小于 250000 us；
6. 若上一 heartbeat=79、sequence=30，两个序号都前进；
7. `valid_frames` 加 1，候选写入 `valid=true`、本地接收 Tick、序号 31、心跳 80 和 `[0,-300,0,0,0]`。

若只把最后 CRC 改错，第 1 步仍会先使候选失效，随后 `crc_errors` 加 1 并返回。旧动作不会因为“上一帧曾经正确”而继续被当作当前有效候选。

### 会话重启怎样识别

正常心跳必须前进。若心跳倒退或重复，并且上一候选距现在还不到 150 ms，按序号错误拒绝。若已经超过 150 ms，接收器把它视为 Linux 可能重启：允许建立新会话，同时 `session_reset_count++`。

允许新会话进入候选不等于继续旧控制。候选带着新的 `session_generation`，活动中的仲裁器发现代数变化会退出，之后仍需操作者重新做 AUX3 低→高授权。

## 字节同步器怎样从任意位置重新找到 `G R`

> **当前位置：`已收到单个byte → [在连续流里寻找帧起点] → 收满44字节`**

串口没有“消息边界”。飞控可能在 Linux 已经发送一半时才上电，也可能因噪声丢一个字节。因此接收端不能每 44 字节盲切；它先找魔数 `G`、`R`。

现在同步器所依赖的 `monitor` 和 `validate_complete_frame()` 都已经完整定义。同步器输入一个 `byte`，长期改变 `frame`、`frame_length` 和接收计数；它没有返回值。状态含义是：0 等 G，1 等 R，2～43 收主体。

代码构思：

```text
每来1字节
  ├─ 状态0：不是G就丢弃；是G就保存
  ├─ 状态1：是R就进入主体；又是G就保留新起点；其他值清零
  └─ 状态2～43：继续保存
        ↓ 收满44
      调用已经定义的完整验帧
        ↓
      无论成功失败都回到状态0
```

完整定义：

```c
void linux_rc_monitor_uart_rx_byte(uint8_t byte)
{
    monitor.diagnostics.received_bytes++;

    if (monitor.frame_length == 0U) {
        if (byte == 'G') {
            monitor.frame[monitor.frame_length++] = byte;
        }
        return;
    }

    if (monitor.frame_length == 1U) {
        if (byte == 'R') {
            monitor.frame[monitor.frame_length++] = byte;
        } else {
            monitor.frame_length = byte == 'G' ? 1U : 0U;
            if (monitor.frame_length != 0U) monitor.frame[0] = byte;
        }
        return;
    }

    monitor.frame[monitor.frame_length++] = byte;
    if (monitor.frame_length == LINUX_RC_FRAME_SIZE) {
        validate_complete_frame();
        monitor.frame_length = 0U;
    }
}
```

`frame_length++` 先用旧值做数组下标，再加 1。第二个字节若不是 R，但它自己又是 G，状态保留为 1；这样字节流 `G G R ...` 会把第二个 G 当新帧头，不错过重叠起点。

具体输入走读：

```text
00 7E 47 47 52 01 2C ...
```

00、7E 在长度 0 时忽略；第一个 47(G) 令长度 1；第二个 47 不是 R，但可作为新 G，长度仍 1；52(R) 令长度 2；之后持续收，长度到 44 才调用已经定义的完整验证。验证结束无论成功失败都回到等待 G。

串口硬件错误还需要一个统一出口。它增加错误计数、使候选失效并丢弃未完成帧：

```c
void linux_rc_monitor_uart_error(void)
{
    monitor.diagnostics.uart_errors++;
    monitor.candidate.valid = false;
    monitor.frame_length = 0U;
}
```

## 现在才让 HAL 回调第一次调用项目接收器

> **当前位置：`验帧和同步器均已定义 → [HAL每字节回调接线] → 原子候选快照`**

现在被调函数已经完整建立，才看真正的接收完成回调：

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        linux_rc_monitor_uart_rx_byte(usart6_rx_byte);
        if (HAL_UART_Receive_IT(&huart6, &usart6_rx_byte, 1U) != HAL_OK) {
            linux_rc_monitor_uart_error();
        }
    }
}
```

`RxCplt` 是 receive complete，接收完成。参数 `huart` 由 HAL 传入；`huart->Instance == USART6` 用来确认这次完成事件属于 USART6，而不是工程中的其他串口。

顺序必须是“交出当前字节 → 立刻重新挂下一字节”。忘记第二次 `HAL_UART_Receive_IT`，系统只会收到第一个字节。若重挂失败，调用已经定义的错误出口，防止半帧或旧候选继续使用。

具体走读：导线上 `0x47` 到达，HAL 放入 `usart6_rx_byte`；回调把它交给同步器，`frame_length` 从 0 变 1；然后重新挂 1 字节接收。约 86.8 us 后 `0x52` 到达，同一个回调再次执行，长度变 2。收满 44 字节时，同步器内部才进入前面已经完整定义的验帧函数。

## 候选快照为什么要“原子地”复制

> **当前位置：`ISR能更新candidate → [RcTask安全取得同一时刻快照] → 物理/虚拟仲裁`**

候选结构包含多个 32 位字段和 5 个通道。中断可能在任务复制到一半时更新它，造成“新心跳 + 旧通道”的撕裂快照。

`linux_rc_monitor_get_candidate()` 的输入是调用者提供的目标地址，输出是一份完整副本；不修改监视器。定义：

代码中的 `NULL` 表示“没有指向任何有效对象的空指针”。若调用者没有提供目标地址，函数直接返回。`PRIMASK` 是 Cortex-M 的中断屏蔽状态位；`__disable_irq()` 暂时屏蔽普通中断，`__enable_irq()` 恢复允许普通中断。`*candidate` 表示访问这个指针指向的整个候选对象。

```c
void linux_rc_monitor_get_candidate(rc_virtual_candidate_t *candidate)
{
    uint32_t primask;
    if (candidate == NULL) return;

    primask = __get_PRIMASK();
    __disable_irq();
    *candidate = monitor.candidate;
    if (primask == 0U) {
        __enable_irq();
    }
}
```

这里各动作的因果关系是：

- `PRIMASK` 为 1 表示原本已经屏蔽普通中断，0 表示原本允许。
- `__disable_irq()` 暂时禁止普通中断。
- `*candidate = monitor.candidate` 复制整个结构。
- 结束时只在原来允许中断时重新开启；不能无条件开启，否则会破坏调用者原有临界区。
- **临界区**就是这段必须保持不可被同一数据写者打断的短代码。

它不是长期锁，也不是把整次仲裁放进中断关闭区。这里只复制一个小结构，然后立刻恢复，避免增加中断延迟。

具体走读：任务读到旧 `PRIMASK=0`，关中断，复制候选的 valid/时间/序号/心跳/代数/5通道，再开中断。USART6 新字节只能在复制后处理，所以副本全部来自同一版本。

## `rc_virtual_candidate_t` 仍然不是有效 RC

> **当前位置：`得到一致候选快照 → [定义仲裁输入与状态] → 第一次仲裁调用`**

前面已经完整定义过候选类型。现在再次强调它的权限边界：它只说明协议验证结果。它没有 ARM 状态，没有物理遥控器，没有 AUX3 授权，也没有最终输出数组，所以它绝不能直接写进 `app_state.rc.mapped_channel_us`。

## 仲裁器先定义所有输入、状态和输出

> **当前位置：`协议候选已得到 → [物理源和虚拟源安全选择] → RcTask接线`**

### 两种源和所有退出原因

```c
typedef enum {
    RC_SOURCE_PHYSICAL = 0,
    RC_SOURCE_VIRTUAL
} rc_source_t;

typedef enum {
    RC_SOURCE_EXIT_NONE = 0,
    RC_SOURCE_EXIT_AUTH_REVOKED,
    RC_SOURCE_EXIT_PHYSICAL_TAKEOVER,
    RC_SOURCE_EXIT_VIRTUAL_INVALID,
    RC_SOURCE_EXIT_VIRTUAL_TIMEOUT,
    RC_SOURCE_EXIT_VIRTUAL_RESTART,
    RC_SOURCE_EXIT_FLIGHT_INHIBIT,
    RC_SOURCE_EXIT_DISARMED,
    RC_SOURCE_EXIT_PHYSICAL_INVALID,
    RC_SOURCE_EXIT_CONFIGURATION_INVALID
} rc_source_exit_reason_t;
```

退出原因数字 1～9 依次是：授权撤销、物理摇杆接管、虚拟候选无效、虚拟超时、Linux 会话重启、飞行门禁、已 DISARM、物理 RC 无效、AUX3 配置冲突。

### 仲裁器长期保存什么

```c
typedef struct {
    rc_source_t active_source;
    rc_source_exit_reason_t last_exit_reason;
    bool authorization_seen_low;
    bool authorization_active;
    uint32_t activation_count;
    uint32_t exit_count;
    uint32_t last_transition_ms;
    uint32_t last_update_ms;
    uint32_t active_session_generation;
    uint16_t virtual_channel_us[3];
    uint16_t slew_remainder[3];
} rc_source_arbiter_t;
```

- `authorization_seen_low` 记录操作者是否先让 AUX3 处于低位。没有它，上电时 AUX3 已高可能自动接入。
- `active_session_generation` 绑定进入时的 Linux 会话。
- `virtual_channel_us[3]` 只保存 Roll/Pitch/Yaw 的飞控侧限速结果。
- `slew_remainder` 保存小于 1 us 的累计余数，避免 20 ms 周期下整数除法长期丢失速度。

初始化函数接收长期状态地址 `state` 和当前本地时间 `now_ms`。它不产生返回值；成功后状态被清零、活动源被强制设为物理源，并把本次时间保存为更新和切换起点。`NULL` 的含义前面已经解释；`memset` 是 C 标准库的按字节填充函数，这里用 0 填满整个状态结构。

```c
void rc_source_arbiter_init(rc_source_arbiter_t *state, uint32_t now_ms)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
    state->active_source = RC_SOURCE_PHYSICAL;
    state->last_update_ms = now_ms;
    state->last_transition_ms = now_ms;
}
```

### `rc_source_arbiter_update()` 的输入输出合同

目标函数每约 20 ms 运行一次。它接收：

- `state`：长期仲裁状态；
- `physical[RC_INPUT_CHANNEL_COUNT]`：当前物理 CRSF AETR 微秒值；
- `physical_valid`：物理 RC 是否通过既有 failsafe；
- `aircraft_armed`：飞控是否已 ARM；
- `arming_inhibit_flags`：任何非 0 位都表示存在飞行安全门禁；
- `authorization_channel_available`：AUX3 未被 ARM/ANGLE 占用；
- `candidate`：刚才取得的 Linux 候选；
- `now_ms`：STM32 本地毫秒 Tick；
- `output`：本次唯一有效的 RC 通道数组。

函数不返回成功/失败数字，而是更新 `state` 并写满 `output`。在第一次定义它之前，还要建立它依赖的几个小函数。

### 先给仲裁规则中的固定数值命名

头文件和源文件共同给规则命名：

```c
#define RC_SOURCE_AUTH_CHANNEL 6U
#define RC_SOURCE_AUTH_MIN_US 1700U
#define RC_SOURCE_TAKEOVER_AXIS_DELTA_US 150U
#define RC_SOURCE_VIRTUAL_TIMEOUT_MS 150U
#define RC_SOURCE_AXIS_MID_US 1500U
#define RC_SOURCE_AXIS_RATE_PER_S 600U
#define RC_SOURCE_MAX_SLEW_DT_MS 100U
```

- `RC_INPUT_CHANNEL_COUNT=16` 来自工程已有的 `rc_input.h`，表示物理 RC 快照共有 16 个通道；仲裁器必须写满同样长度的输出；
- 物理数组下标 6 是 AUX3 授权通道；数组从 0 开始计数，所以第 7 个元素的下标是 6；
- AUX3 达到 1700 us 才算授权高位；
- 任一物理姿态轴离中心超过 150 us 就请求人工接管；
- Linux 候选本地年龄必须严格小于 150 ms；
- 姿态中心是 1500 us；
- 飞控侧姿态变化速度最多 600 us/s；
- 单次限速计算最多承认 100 ms，避免任务长时间卡顿后一步跳太远。

### 从最小的时间和距离计算开始

`elapsed_ms(now_ms, then_ms)` 返回两个 32 位 Tick 的差。无符号减法能在间隔没有长到跨越整个 32 位范围时处理 Tick 回绕：

```c
static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}
```

输入 `now_ms=1320`、`then_ms=1300`，返回 20 ms。

`absolute_delta(value, center)` 返回两个通道微秒值之间不带正负号的距离：

```c
static uint16_t absolute_delta(uint16_t value, uint16_t center)
{
    return value >= center ? (uint16_t)(value - center)
                           : (uint16_t)(center - value);
}
```

输入 1700 和 1500 时返回 200；输入 1400 和 1500 时返回 100。

### 定义授权、接管和候选新鲜度

`authorization_is_active(physical)` 只读取物理通道数组；AUX3 达到 1700 us 返回 true：

```c
static bool authorization_is_active(
    const uint16_t physical[RC_INPUT_CHANNEL_COUNT])
{
    return physical[RC_SOURCE_AUTH_CHANNEL] >= RC_SOURCE_AUTH_MIN_US;
}
```

`takeover_is_requested(physical)` 用已经定义的 `absolute_delta()` 检查前三个物理姿态轴。任一轴严格超过 150 us 就返回 true：

```c
static bool takeover_is_requested(
    const uint16_t physical[RC_INPUT_CHANNEL_COUNT])
{
    return (absolute_delta(physical[0], RC_SOURCE_AXIS_MID_US) >
            RC_SOURCE_TAKEOVER_AXIS_DELTA_US) ||
           (absolute_delta(physical[1], RC_SOURCE_AXIS_MID_US) >
            RC_SOURCE_TAKEOVER_AXIS_DELTA_US) ||
           (absolute_delta(physical[2], RC_SOURCE_AXIS_MID_US) >
            RC_SOURCE_TAKEOVER_AXIS_DELTA_US);
}
```

`||` 表示“或者”。Throttle 不在这里，因为虚拟源活动时 Throttle 本来就一直由物理遥控器控制。

`candidate_is_fresh(candidate, now_ms)` 同时检查指针存在、候选有效和本地年龄严格小于 150 ms：

```c
static bool candidate_is_fresh(const rc_virtual_candidate_t *candidate,
                               uint32_t now_ms)
{
    return (candidate != NULL) && candidate->valid &&
           (elapsed_ms(now_ms, candidate->received_ms) <
            RC_SOURCE_VIRTUAL_TIMEOUT_MS);
}
```

`candidate==NULL` 表示调用者没有提供候选地址。C 的 `&&` 从左向右短路：若指针为空，后面不会继续读取 `candidate->valid`，因此不会访问无效地址。

### 把 Linux 偏移变成 RC 目标，并限制每周期变化

Linux 候选保存的是以 1500 us 为中心的偏移。`virtual_target_us(-300)` 返回 `1200 us`：

```c
static uint16_t virtual_target_us(int16_t value)
{
    const int32_t target = (int32_t)RC_SOURCE_AXIS_MID_US + value;
    return (uint16_t)target;
}
```

`slew_toward()` 的意思是“从 current 朝 target 靠近，但本周期最多走允许的距离”。`rate_per_s` 的单位是 us/s，`dt_ms` 是本周期毫秒数，`remainder` 保存整数除法丢下的千分余数：

```c
static uint16_t slew_toward(uint16_t current, uint16_t target,
                            uint32_t rate_per_s, uint32_t dt_ms,
                            uint16_t *remainder)
{
    uint32_t maximum_step;
    uint32_t scaled_step;

    if (dt_ms > RC_SOURCE_MAX_SLEW_DT_MS) {
        dt_ms = RC_SOURCE_MAX_SLEW_DT_MS;
    }
    if (current == target) {
        *remainder = 0U;
        return current;
    }
    scaled_step = rate_per_s * dt_ms + *remainder;
    maximum_step = scaled_step / 1000U;
    *remainder = (uint16_t)(scaled_step % 1000U);
    if (maximum_step == 0U) {
        return current;
    }
    if (current < target) {
        const uint32_t delta = (uint32_t)target - current;
        return delta <= maximum_step ? target
                                     : (uint16_t)(current + maximum_step);
    }
    if (current > target) {
        const uint32_t delta = (uint32_t)current - target;
        return delta <= maximum_step ? target
                                     : (uint16_t)(current - maximum_step);
    }
    return current;
}
```

具体输入：`current=1500`、`target=1200`、速度 600 us/s、`dt_ms=20`、余数 0。`scaled_step=600×20=12000`，除以 1000 得本周期最大 12 us，所以返回 1488，而不是直接跳到 1200。

### 所有退出共同经过一个函数

`exit_virtual(state, reason, now_ms)` 把活动源切回物理、记录原因和时刻、增加退出计数，并清除“已经见过授权低位”的证据：

```c
static void exit_virtual(rc_source_arbiter_t *state,
                         rc_source_exit_reason_t reason,
                         uint32_t now_ms)
{
    state->active_source = RC_SOURCE_PHYSICAL;
    state->last_exit_reason = reason;
    state->authorization_seen_low = false;
    state->last_transition_ms = now_ms;
    ++state->exit_count;
}
```

清成 false 的直接结果是：Linux 恢复流量、CRC 变好或进程重启后，只要 AUX3 一直保持高位，就不能自动重新进入；操作者必须重新低→高。

### 先看完整流程，再看完整定义

```text
参数地址无效 → 立即返回
  ↓
先把全部物理通道复制到output，建立安全默认值
  ↓
计算授权、人工接管、候选新鲜度
  ↓
物理有效且AUX3低 → 记住“见过低位”
  ↓
当前是虚拟源？
  ├─ 是：按固定优先级检查9种退出原因
  └─ 否：全部进入条件同时满足才进入
  ↓
计算距上次更新的dt_ms
  ↓
仍不是虚拟源 → 保留完整物理output并返回
  ↓
只对Roll/Pitch/Yaw做目标换算和限速覆盖
  ↓
Throttle和全部AUX保持开头复制的物理值
```

现在所有被调辅助函数都已经定义，才能给出 `rc_source_arbiter_update()` 的完整实现：

```c
void rc_source_arbiter_update(
    rc_source_arbiter_t *state,
    const uint16_t physical[RC_INPUT_CHANNEL_COUNT],
    bool physical_valid,
    bool aircraft_armed,
    uint32_t arming_inhibit_flags,
    bool authorization_channel_available,
    const rc_virtual_candidate_t *candidate,
    uint32_t now_ms,
    uint16_t output[RC_INPUT_CHANNEL_COUNT])
{
    bool authorized;
    bool takeover;
    bool candidate_fresh;
    uint32_t dt_ms;
    uint32_t channel;

    if ((state == NULL) || (physical == NULL) || (output == NULL)) {
        return;
    }

    /* 先建立安全默认值。没有进入虚拟源时，完整输出就是物理RC。 */
    memcpy(output, physical, sizeof(uint16_t) * RC_INPUT_CHANNEL_COUNT);
    authorized = authorization_is_active(physical);
    takeover = takeover_is_requested(physical);
    candidate_fresh = candidate_is_fresh(candidate, now_ms);
    state->authorization_active = authorized;

    if (physical_valid && !authorized) {
        state->authorization_seen_low = true;
    }

    if (state->active_source == RC_SOURCE_VIRTUAL) {
        if (!physical_valid) {
            exit_virtual(state, RC_SOURCE_EXIT_PHYSICAL_INVALID, now_ms);
        } else if (!authorization_channel_available) {
            exit_virtual(state, RC_SOURCE_EXIT_CONFIGURATION_INVALID, now_ms);
        } else if (!aircraft_armed) {
            exit_virtual(state, RC_SOURCE_EXIT_DISARMED, now_ms);
        } else if (arming_inhibit_flags != 0U) {
            exit_virtual(state, RC_SOURCE_EXIT_FLIGHT_INHIBIT, now_ms);
        } else if (!authorized) {
            exit_virtual(state, RC_SOURCE_EXIT_AUTH_REVOKED, now_ms);
        } else if (takeover) {
            exit_virtual(state, RC_SOURCE_EXIT_PHYSICAL_TAKEOVER, now_ms);
        } else if ((candidate == NULL) || !candidate->valid) {
            exit_virtual(state, RC_SOURCE_EXIT_VIRTUAL_INVALID, now_ms);
        } else if (candidate->session_generation !=
                   state->active_session_generation) {
            exit_virtual(state, RC_SOURCE_EXIT_VIRTUAL_RESTART, now_ms);
        } else if (!candidate_fresh) {
            exit_virtual(state, RC_SOURCE_EXIT_VIRTUAL_TIMEOUT, now_ms);
        }
    } else if (physical_valid && authorization_channel_available &&
               aircraft_armed && (arming_inhibit_flags == 0U) && authorized &&
               state->authorization_seen_low && !takeover && candidate_fresh) {
        state->active_source = RC_SOURCE_VIRTUAL;
        state->authorization_seen_low = false;
        state->last_exit_reason = RC_SOURCE_EXIT_NONE;
        state->last_transition_ms = now_ms;
        state->active_session_generation = candidate->session_generation;
        ++state->activation_count;
        memcpy(state->virtual_channel_us, physical,
               sizeof(state->virtual_channel_us));
        memset(state->slew_remainder, 0, sizeof(state->slew_remainder));
    }

    dt_ms = elapsed_ms(now_ms, state->last_update_ms);
    state->last_update_ms = now_ms;
    if (state->active_source != RC_SOURCE_VIRTUAL) {
        return;
    }

    /* 只覆盖前三个姿态轴；油门和全部AUX保留开头复制的物理值。 */
    for (channel = 0U; channel < 3U; ++channel) {
        const uint16_t target = virtual_target_us(candidate->channel[channel]);
        state->virtual_channel_us[channel] =
            slew_toward(state->virtual_channel_us[channel], target,
                        RC_SOURCE_AXIS_RATE_PER_S, dt_ms,
                        &state->slew_remainder[channel]);
        output[channel] = state->virtual_channel_us[channel];
    }
}
```

### 进入和退出条件怎样映射到完整代码

进入虚拟源必须同时满足：当前为物理源、物理 RC 有效、AUX3 可专用于视觉授权、已 ARM、无 inhibit、AUX3 当前高、此前见过低位、物理 RPY 居中、候选有效且年龄小于 150 ms。

注意两个边界：候选年龄是严格“小于 150 ms”，等于 150 已不新鲜；物理接管是严格“大于 150 us”，恰好偏 150 不触发。

活动时 `if/else if` 按以下真实优先级退出，同一周期只记录最先命中的原因：

1. 物理 RC 无效；
2. AUX3 配置冲突；
3. 已 DISARM；
4. 存在飞行 inhibit；
5. AUX3 拉低；
6. 物理 R/P/Y 任一离中超过 150 us；
7. 候选为空或无效；
8. Linux 会话代数改变；
9. 候选年龄达到或超过 150 ms。

函数开头先把完整物理数组复制到 `output`。只有最终仍处于虚拟源时，循环才覆盖下标 0、1、2，也就是 Roll、Pitch、Yaw。通道 3 的 Throttle 和所有 AUX 从未被 Linux 覆盖。这不是一句约定，而是由“先复制全部物理值 + 只覆盖前三轴”的代码结构保证。

## 用具体 Tick 走完一次进入、动作、接管和重授权

> **当前位置：`进入/退出规则都已定义 → [具体时间线] → RcTask第一次调用`**

假设物理通道：RPY=1500，Throttle=1000，AUX1 ARM、AUX3 授权；Linux 候选 Roll=0、Pitch=-300、Yaw=0。

| STM32时间 | 输入和状态 | 仲裁结果 |
|---:|---|---|
| 1000 ms | 启动；AUX3=1000；物理有效；未ARM | 物理源；`authorization_seen_low=true` |
| 1100 ms | Linux 健康中立刚在1090 ms收到 | 候选年龄10 ms，但未ARM，仍物理源 |
| 1200 ms | AUX1 ARM；AUX3仍低 | 已ARM但未授权，仍物理源 |
| 1300 ms | AUX3拉高到2000；RPY居中；候选在1290收到 | 全部条件满足，进入虚拟源；三轴从1500开始 |
| 1320 ms | Pitch候选=-300 | 飞控限速最大12 us，输出Pitch约1488；Throttle仍1000 |
| 1340 ms | 同一候选链继续 | Pitch约1476；AUX全部仍来自物理 |
| 1500 ms | 物理Roll拨到1700，偏中心200 us | 超过150，当前周期退出物理源；输出直接是当前物理快照，reason=2 |
| 1520 ms | 摇杆回中，AUX3仍高，Linux仍健康 | 不重入，因为 `authorization_seen_low=false` |
| 1600 ms | AUX3拉低 | 物理源；重新看到低位，`authorization_seen_low=true` |
| 1700 ms | AUX3再拉高且其余条件正常 | 可再次进入，activation_count加1 |

候选接收于 2000 ms，若之后断线：在 2139 ms 仍小于 150 ms，候选可算新鲜；在 2150 ms 年龄正好 150，严格条件失败，最迟下一个 RcTask 周期退出。由于 RcTask 约 20 ms 一次，外部观测会有一个调度周期量级的误差。

## RcTask 怎样把两条来源接到唯一输出

> **当前位置：`仲裁器已经能独立工作 → [RcTask每20ms组织输入和发布结果] → UART4观察`**

`RcTask` 先通过 UART2/DMA/CRSF 解析得到物理通道，执行既有 AETR 映射和物理 RC failsafe，保存到 `physical_mapped_channel_us`。这条路径早于 Linux 集成存在，仍负责 ARM、Throttle 和全部 AUX。

循环中相关顺序：

```text
服务UART2 CRSF并解析物理通道
  ↓
更新物理RC failsafe
  ↓
读取小型飞行上下文：armed、inhibit、AUX3配置是否可用
  ↓
原子复制Linux candidate
  ↓
rc_source_arbiter_update(...)
  ↓
把仲裁状态/序号/心跳/代数写入rc诊断字段
  ↓
app_state_publish_rc(&rc)
  ↓
最多等待20ms通知/超时，再循环
```

现在所有类型、参数和规则都已定义，才看第一次实际调用：

```c
rc_source_arbiter_update(
    &arbiter,
    rc.physical_mapped_channel_us,
    rc.channels_valid,
    source_context.flight_armed,
    source_context.arming_inhibit_flags,
    source_context.authorization_channel_available,
    &candidate,
    HAL_GetTick(),
    rc.mapped_channel_us);
```

`rc.mapped_channel_us` 是唯一向后续 `rc_setpoint` 发布的有效通道数组。Linux 监视器无权直接写它。

源码还用 `_Static_assert` 确认 FreeRTOS Tick 为 1 kHz、RcTask 静态栈至少 1536 字节。历史上曾因复制完整系统快照把 RcTask 固定栈帧推到约 1800 B，接近 2048 B 任务栈；后来改为只读取约 16 B 的 `app_rc_source_context_t`，历史 Debug/Release 静态帧降到约 664/728 B。这是源码/构建记录，实际最坏栈仍需运行时高水位验证。

## UART4 日志怎样从前向后缩小故障范围

> **当前位置：`有效RC已经发布 → [观察接收、仲裁和后级证据] → 端到端验收`**

UART4 也是 115200 8N1，但它是诊断输出，不是 Linux 候选输入。主要三类行：

### `linux_rc`：字节和帧有没有通过

```text
linux_rc rx=... frame=... valid=... crc=... fmt=... seqerr=... timeerr=...
restart=... uart=... last=[v... g... c... seq=... hb=... ch=...]
```

- `rx` 收到的总字节；正常发 44 字节一帧时持续增加。
- `frame` 收满的帧；`valid` 是通过监视器全部结构检查的帧数，不等于字段 `valid=1`。
- `crc/fmt/seqerr/timeerr` 对应四类拒绝。
- `restart` 是新会话计数；`uart` 是接收重挂错误。
- `last=[v...]` 中 `v` 是最后通过验证帧自身的 valid 位。

### `rc source`：为什么还没采用，或为什么退出

```text
rc source=0(physical) auth=0 reauth=1 virtual=1 age_ms=20
seq=318 hb=901 gen=0 enter=0 exit=0 reason=0(none)
aux3_us=1000 armed=1 inhibit=0x00000000 cfg=1
```

- `source=0/1` 是最终物理/虚拟选择。
- `auth` 是 AUX3 当前是否高；`reauth=1` 表示已经见过低位，可以等下一次高位。
- `virtual=1` 只表示候选有效且本地年龄<150 ms。
- `cfg=1` 表示 AUX3 没被 ARM/ANGLE 配置占用。
- `reason` 使用前述 0～9 退出原因。

若 `linux_rc valid` 不涨，先查协议/串口；若它涨、`virtual=1`，但 source 仍 0，才查 armed、inhibit、cfg、reauth、AUX3 和 RPY 居中。不要倒着从 PID 查起。

### `e2e`：被选择后有没有进入既有控制链

```text
e2e rc_source=1 rc_valid=1 rc_seq=318 rc_hb=901
setpoint_valid=1 pid_valid=1 mixer_valid=1 flight_ready=1
armed=1 failsafe=0 dshot_ready=1 dshot_busy=0 motors=[...]
```

- `rc_source=1` 才表示仲裁器采用虚拟 RPY。
- `rc_seq/hb` 可与 Linux JSONL 和线上帧对齐。
- `setpoint_valid/pid_valid/mixer_valid` 逐层说明后级是否接受。
- `armed/failsafe/dshot_ready` 是既有飞行安全状态。
- `motors` 是最终电机命令证据；拆桨台架观察不等于允许装桨飞行。

## 验证闭环

> **当前位置：`从字节到后级日志都可观察 → [逐层验证] → 下一篇故障注入`**

| 层级 | 正常 | 失败表现 | 第一检查点 | 证明边界 |
|---|---|---|---|---|
| USART6 | `rx` 持续增长 | 完全不涨或uart错误涨 | PC7、共地、3.3V、115200 8N1、中断重挂 | 字节进入MCU |
| 帧同步 | `frame` 约每44字节增长 | rx涨但frame不涨 | 是否有 `47 52`、是否丢字节 | 找到完整边界 |
| 完整验证 | `valid` 涨，错误计数不涨 | crc/fmt/seqerr/timeerr涨 | 只查对应字段和抓包 | 协议候选通过 |
| 候选新鲜 | `virtual=1 age_ms<150` | age增长或valid位0 | Linux进程、TX线、FIST/源时间 | 当前候选资格 |
| 进入授权 | source从0到1，enter加1 | 始终0 | armed/inhibit/cfg/reauth/AUX3/RPY | 仲裁条件满足 |
| 通道所有权 | 只替换RPY；Throttle/AUX跟物理 | 油门/AUX被覆盖 | `output`初始物理复制和0..2循环 | Linux权限边界 |
| 退出锁定 | 当前周期回物理，保持高不重入 | 旧虚拟值残留或自动重入 | reason、seen_low、session gen | 失效关闭和人工重授权 |
| 后级 | setpoint/PID/mixer证据连续 | 某层valid=0 | 从第一处断点对应模块查 | 采用后链路；不等于飞行放行 |

所有硬件操作必须拆桨。上电前 AUX1/2/3 低、油门最低、RPY 回中，飞控固定在绝缘台面；UART/逻辑分析仪共地且只接已确认 3.3 V 引脚。任何默认进入虚拟源、无物理 RC 仍活动、油门/AUX被覆盖或退出后旧值残留，都应立即断电并判失败。

## 迁移练习：只改变物理接管阈值

在纯 C 仲裁测试环境中，把 `RC_SOURCE_TAKEOVER_AXIS_DELTA_US` 从 150 改为 120，其他条件不变。构造物理 Roll=1621 us：新阈值下应接管，旧阈值下不会；Roll=1620 恰好等于新阈值，因源码使用 `>`，仍不接管。这个练习让你学会检查边界是否包含等号。不要直接把实验值刷进飞行固件；修改安全阈值必须重新做完整拆桨验收。

## 文字复盘和最终结构图

你现在能从第一个电气字节讲到唯一有效 RC：USART6 每字节中断只推进同步器并重新挂接收；监视器找到 `GR` 后收满 44 字节，先令旧候选失效，再按 CRC、格式、语义、时间、序号和会话验证。任务通过短暂 PRIMASK 临界区取得一致快照。

候选不等于控制。RcTask 同时维护物理 CRSF 和 Linux 候选，仲裁器默认复制物理通道；只有物理有效、已 ARM、无 inhibit、AUX3 配置合法且先低后高、RPY 居中、候选新鲜时才进入。进入后只覆盖 RPY，Throttle 和所有 AUX 始终来自物理遥控器。任何退出都立即回物理并要求再次低高授权。

```text
RK3568 TX
  ↓ 3.3V、共地
PC7 / USART6_RX 115200 8N1
  ↓ 1字节HAL中断
G→R→累计44字节
  ↓ CRC/格式/语义/时间/序号/会话
rc_virtual_candidate_t
  ↓ PRIMASK短临界区原子复制
RcTask（约20ms）
  ├─ UART2 CRSF物理快照
  ├─ ARM / inhibit / AUX3配置
  └─ Linux候选
  ↓ rc_source_arbiter_update
  ├─ 默认完整物理通道
  ├─ 全条件满足才覆盖RPY
  ├─ Throttle/AUX永久物理所有权
  └─ 任一退出→物理源+低高重授权
mapped_channel_us
  ↓
下一篇：拆桨条件下把Linux JSONL、UART4接收/仲裁/e2e和故障注入连成证据闭环
```
