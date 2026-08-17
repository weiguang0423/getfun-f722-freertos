<!--
文件作用：S7.6 虚拟 RC 映射、固定帧、串口发送和验收入口。
覆盖范围：S7.5 ACTIVE 快照到有界候选通道；不含 STM32 接收、RC 源仲裁、ARM 或电机。
关联模块：s7_6_virtual_rc.cpp、s7_6_serial.cpp、复用的 S7.3～S7.5 实时链。
仍需实物验证：RK3568 实际 UART、电平与接线，串口回环/逻辑分析仪及断线超时。
-->

# S7.6 手势到虚拟 RC 映射与发送链路

S7.6 复用已验收的 S7.5 `ACTIVE` 快照。Linux 只产生候选 RC 帧，不修改 STM32 固件，也不拥有
ARM、总授权、Failsafe、PID、Mixer 或 DShot。实际串口设备必须在核对 RK3568 引脚、电平和
飞控空闲 UART 后通过 `S7_6_RC_OUTPUT` 指定；源码不把两个发送端并接到 UART2/CRSF。

## 冻结映射与范围

线上的整数 `1000` 表示归一化 `1.0`。首版映射有意不抬升 Throttle 和 AUX：

| ACTIVE 手势 | valid | Roll | Pitch | Yaw | Throttle | AUX | 含义 |
|---|---:|---:|---:|---:|---:|---:|---|
| `OPEN_PALM` | 1 | 0 | 0 | 0 | 0 | 0 | 有效中立 |
| `FIST` | 0 | 0 | 0 | 0 | 0 | 0 | 显式释放 |
| `POINT` | 1 | 0 | -300 | 0 | 0 | 0 | 小幅 Pitch 负向候选 |
| `V_SIGN` | 1 | 0 | 0 | +300 | 0 | 0 | 小幅 Yaw 正向候选 |

Roll/Pitch/Yaw 合法范围为 `[-300,+300]`，最大变化率 `600/s`；Throttle 合法范围
`[0,250]`、最大变化率 `400/s`；AUX 合法范围 `[0,1000]`、最大变化率 `1000/s`。当前映射的
Throttle/AUX 目标始终为释放值 0，ARM 和总授权 AUX 永远不由 Linux 生成。任何无效输入都绕过
斜坡，立即把五个通道恢复为 0。

只有状态为 `ACTIVE`、观测和活动手势一致、置信度不低于 0.75、单调时钟未回退且源结果年龄
不超过 250 ms 时才可置 `valid=1`。启动、无手、未知、低置信度、握拳、切换、deadline miss、
采集/推理失败或进程重启均发送无效释放帧；串口写失败直接终止，不自动重连或恢复旧动作。

## 线协议

固定 `115200 8N1`、小端、44 字节一帧，实时链目标 20 Hz。帧尾使用 CRC16-CCITT-FALSE：

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 2 | 魔数 `GR` |
| 2 | 1 | 版本 1 |
| 3 | 1 | 帧长 44 |
| 4 | 1 | bit0=`valid`，其他位必须为 0 |
| 5 | 1 | S7.5 手势 ID |
| 6 | 1 | 置信度百分数 0～100 |
| 7 | 1 | 保留 0 |
| 8 | 4 | 摄像头源序号 |
| 12 | 4 | 每次发送递增的心跳 |
| 16 | 8 | 源单调时间戳，微秒 |
| 24 | 8 | 发送单调时间戳，微秒 |
| 32 | 10 | Roll/Pitch/Yaw/Throttle/AUX，五个 `int16` |
| 42 | 2 | 前 42 字节 CRC16 |

接收侧契约已在纯逻辑 `decode()` 中冻结：CRC、版本、范围、150 ms 链路新鲜度、250 ms 源
新鲜度、递增心跳和递增有效源序号任一失败即拒绝。真正的飞控接收和人工授权只在 S7.7 实现。

## 构建与运行

```sh
# x86 主机纯逻辑回归
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  Linux/RK3568/s7_5/s7_5_gesture.cpp \
  Linux/RK3568/s7_6/s7_6_virtual_rc.cpp \
  Linux/RK3568/s7_6/s7_6_virtual_rc_test.cpp -o /tmp/s7_6_virtual_rc_test
/tmp/s7_6_virtual_rc_test

# WSL/Linux 文件回环（同时走实际 SerialLink::send 写出路径）
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  Linux/RK3568/s7_6/s7_6_virtual_rc.cpp \
  Linux/RK3568/s7_6/s7_6_serial.cpp \
  Linux/RK3568/s7_6/s7_6_serial_test.cpp -o /tmp/s7_6_serial_test
/tmp/s7_6_serial_test

# 冻结 SDK 中交叉构建 ARM64 程序
sh Linux/RK3568/s7_6/build_board.sh

# 板端先自检；/dev/null 只验证实时软件链，不算通信验收
./s7_6_live --self-test
S7_6_RC_OUTPUT=/dev/null ./s7_6_live --camera \
  hand_detector_fp16.rknn hand_landmarks_detector_fp16.rknn \
  /dev/video0 800 600 20 180 200 4776 /userdata/s7_6_acceptance/annot 5 \
  /dev/dri/card0 >/userdata/s7_6_acceptance/results.jsonl \
  2>/userdata/s7_6_acceptance/runtime.log

# 核对 RK3568 物理资源后才把 /dev/null 换成已验证的 /dev/tty*。
```

当前主机纯逻辑自检和文件回环均通过，ARM64 交叉构建产物为 112992 字节，SHA-256
`61cfff8d59ed8ce28f42836fdfc875cef54ae01f3ca01bcda38135adac9933fb`。x86 WSL 不能执行
ARM64 文件；板端自检、实际端口、回环/逻辑分析仪、杀进程后 150 ms 失效和断线恢复仍是实物关口。
