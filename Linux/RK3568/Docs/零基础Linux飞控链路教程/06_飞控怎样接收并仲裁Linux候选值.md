---
title: 飞控怎样接收并仲裁Linux候选值
---

# 飞控怎样接收并仲裁 Linux 候选值

> **当前学习位置：** `RK3568 已能发出经过限制的候选帧` → **飞控验证、人工授权并选择输入源** → `拆桨端到端验收`
>
> 本篇对应 S7.7，解释 Linux 数据进入 STM32 后为什么不会直接成为控制指令。你不需要先学会 FreeRTOS 或 C 语言；本篇会用现有源码说明每一层的职责和实际安全条件。

## Linux 串口字节到飞控控制链，中间经历了什么

Linux 发出的 44 字节帧没有直达 PID。飞控内部的真实顺序是：

```text
RK3568 UART TX
  -> STM32 USART6 RX（PC7）中断，每次取 1 个字节
  -> linux_rc_monitor：找 GR 帧头，累计 44 字节并验证
  -> rc_virtual_candidate_t：仅保存“已验证候选”快照
  -> RcTask：同时维护物理 CRSF 遥控器快照
  -> rc_source_arbiter：按 ARM/授权/接管/超时决定选哪一源
  -> app_state.rc.mapped_channel_us
  -> 已有 rc_setpoint / PID / Quad-X Mixer / ARM-Failsafe / DShot
```

这里最关键的词是“候选”。`linux_rc_monitor` 只负责检查 Linux 的字节合法；它不能自行进入飞控输入，更不能绕过 ARM 或电机门禁。真正允许把候选值放入 `mapped_channel_us` 的唯一位置是 `RcTask` 中调用的 `rc_source_arbiter_update()`。

相关真实文件如下：

| 文件 | 只需要先理解的职责 |
|---|---|
| [`Core/Src/usart.c`](../../../../Core/Src/usart.c) | 初始化 `USART6` 115200 8N1；收到每个字节时交给监视器 |
| [`APP/Src/bsp/linux_rc_monitor.c`](../../../../APP/Src/bsp/linux_rc_monitor.c) | 拼帧、CRC/范围/时间/序号校验，产出候选快照 |
| [`APP/Src/rtos/rc_task.c`](../../../../APP/Src/rtos/rc_task.c) | 读取物理 CRSF 和 Linux 候选，发布最终 RC 快照 |
| [`APP/Src/algorithms/rc_source_arbiter.c`](../../../../APP/Src/algorithms/rc_source_arbiter.c) | 按安全规则选择物理源或虚拟源 |

## USART6 为什么只做“接收和验证”

F722 的 `USART6` 在源码中配置为 `115200`、8 数据位、无校验、1 停止位，且：

```text
PC6 -> USART6_TX
PC7 -> USART6_RX
```

当 `PC7` 收到一个字节，中断回调只做两件小事：把字节交给 `linux_rc_monitor_uart_rx_byte()`，然后重新挂起下一字节接收。它不在中断里做飞行控制运算。这样中断保持很短，复杂判断留给正常任务上下文执行。

`linux_rc_monitor` 首先找固定帧头 `G`、`R`，再收满 44 字节。完整帧还要通过：CRC、版本、长度、`valid` 位、手势 ID、置信度、5 个通道范围、手势与通道匹配关系、源/发送时间顺序、心跳和源序号的前进关系。任何一项失败，当前 `candidate.valid` 变为 false。

所以即使线上噪声刚好组成了 44 个字节，它也必须同时满足很多协议条件才会变成一个候选。这是“输入边界验证”，不是相信外部 Linux 的默认数据。

## RcTask 为什么同时保留两份遥控器快照

> **当前学习位置：** `Linux 帧已经被验证` → **与物理 CRSF 遥控器比较并选择来源** → `交给既有控制链`

`RcTask` 每 20 ms 左右运行一次。它保留两份不同含义的数据：

| 快照 | 来源 | 用途 |
|---|---|---|
| `physical_mapped_channel_us` | UART2 上的 ELRS/CRSF 物理遥控器 | ARM、AUX、油门、人工接管和默认控制 |
| Linux `candidate` | USART6 帧监视器 | 只有在条件齐全时替换 Roll/Pitch/Yaw 的候选 |

**注意：** UART2 仍只给 CRSF 接收机使用。项目没有把 RK3568 和 CRSF 的两个发送端并接到一个 UART。Linux 使用独立 USART6，这样每条线的电气和协议责任都能分开检查。

仲裁器开始时总是选择物理源。进入虚拟源要同时满足下面全部条件：

```text
物理 RC 有效
且 已经 ARM
且 飞控没有任何 arming inhibit
且 AUX3 是可用且未与 ARM/ANGLE 配置冲突的授权通道
且 操作者先让 AUX3 处于低位，再做一次低 -> 高的授权动作
且 物理 Roll/Pitch/Yaw 都在中间附近，没有接管动作
且 Linux 候选 valid 且距接收不超过 150 ms
```

这份“先低后高”规则非常重要。它避免系统在上电时因为 AUX3 恰好已经高位、或 Linux 重连后流量恢复，就自动接管控制。

当虚拟源已被选中时，飞控只替换 Roll、Pitch、Yaw 三轴。`Throttle` 和全部 AUX 始终继续采用物理遥控器的实时值。Linux 的候选 `-300..+300` 会被飞控映射到以 `1500 us` 为中点的 RC 微秒值，并在飞控侧再次以 `600/s` 限速靠近目标。

## 哪些情况会立即回到物理遥控器

任一退出条件出现，仲裁器在当前 `RcTask` 更新中返回物理源，并清除“已低位授权”记忆。Linux 流量恢复后，必须再做 AUX3 低到高的完整动作才能重新进入。

| 退出原因 | 日常动作或故障 | 为什么要退出 |
|---|---|---|
| 授权撤销 | AUX3 拉低 | 操作者明确取消视觉控制 |
| 人工接管 | 物理 Roll/Pitch/Yaw 任一偏离中心超过 150 us | 摇杆永远能夺回姿态控制 |
| 候选无效 | 握拳、CRC/格式/时间错误 | Linux 已明确或被判定为不可信 |
| 超时 | 150 ms 内没有新鲜候选 | 串口断线、Linux 卡住或发送程序被杀死 |
| Linux 重启 | 心跳序列重新开始，接收会话代数改变 | 不允许重启后自动恢复旧动作 |
| 飞行门禁 | DISARM、inhibit、物理 RC 失效、授权通道配置冲突 | 继续交给原有 ARM/Failsafe 安全链 |

物理油门变化是一个刻意的例外：它**不**触发人工接管。飞控在虚拟源期间仍实时使用物理油门，因此操作者可以改变基础油门，Linux 手势只能叠加有限的姿态差动。这样油门所有权从未让给 Linux。

## 用 UART4 日志观察飞控做出的决定

项目让 UART4 保持为诊断输出。它提供 `linux_rc`、`rc source` 和 `e2e` 等行，用来回答“字节收到了吗”“候选有效吗”“仲裁器真的采用了吗”。

一条端到端摘要形如：

```text
e2e rc_source=1 rc_valid=1 rc_seq=318 rc_hb=901 setpoint_valid=1 pid_valid=1 mixer_valid=1 flight_ready=1 armed=1 failsafe=0 dshot_ready=1 dshot_busy=0 motors=[...]
```

最先看三个字段：

- `rc_source=0` 表示物理遥控器；`rc_source=1` 才表示飞控已经选择 Linux 候选。
- `rc_seq` 是飞控实际采用的 Linux 源序号，后续可和 Linux JSONL 的摄像头序号对齐。
- `armed`、`failsafe`、`setpoint_valid`、`pid_valid`、`mixer_valid` 等展示后面的控制链是否完整；只出现 `rc_source=1` 不足以证明整链正确。

如果你在“预期进入虚拟源”时仍看到 `rc_source=0`，不要先改代码。依次检查：Linux 候选 `valid` 与年龄、是否真的已 ARM、AUX3 是否先低后高、RPY 摇杆是否回中、飞行 inhibit 是否为 0、AUX3 是否被错误配置成 ARM/ANGLE。

## 本篇只应在拆桨条件下验证

在已经验证线路后，按照 [Docs/42_S7.7_飞控RC源仲裁人工授权与Failsafe开发验收.md](../../../../Docs/42_S7.7_飞控RC源仲裁人工授权与Failsafe开发验收.md) 的顺序做检查：

1. 上电、AUX1/2/3 都低，确认默认 `rc_source=0`。
2. Linux 持续发送健康中立帧；只 ARM，不做授权，确认仍是物理源。
3. RPY 回中、油门在安全低值时，执行 AUX3 低到高授权，确认 `rc_source=1`。
4. 依次验证张掌、单指、V 字一次性脉冲和握拳释放。
5. 分别验证 AUX3 拉低、三轴物理摇杆接管、杀 Linux 进程、拔 TX、Linux 重启和非法帧。

全程保持拆桨。即使日志显示 DShot 就绪，也不能把该软件/拆桨验收结论解释为允许飞行。

## 本篇完成判据

你应能用一句话说清：**Linux 帧通过 CRC 不等于飞控会采用它；只有仲裁器在物理 RC、ARM、AUX3 低高授权、无 inhibit、无接管且候选新鲜时，才只替换 RPY。**

下一篇：[07_拆桨端到端验收与故障定位](07_拆桨端到端验收与故障定位.md)。
