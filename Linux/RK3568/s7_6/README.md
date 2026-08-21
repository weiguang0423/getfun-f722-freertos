<!--
文件作用：S7.6 虚拟 RC 映射、固定帧、串口发送和验收入口。
覆盖范围：S7.5 ACTIVE 快照到有界候选通道，以及 STM32 USART6 被动接收监视；不含 RC 源仲裁、ARM 或电机。
关联模块：s7_6_virtual_rc.cpp、s7_6_serial.cpp、复用的 S7.3～S7.5 实时链。
仍需实物验证：RK3568 实际 UART、电平与接线，串口回环/逻辑分析仪及断线超时。
-->

# S7.6 手势到虚拟 RC 映射与发送链路

S7.6 复用已验收的 S7.5 `ACTIVE` 快照。Linux 只产生候选 RC 帧，不修改 STM32 固件，也不拥有
ARM、总授权、Failsafe、PID、Mixer 或 DShot。实际串口设备必须在核对 RK3568 引脚、电平和
飞控空闲 UART 已切换为 USART6（PC6=TX、PC7=RX，115200 8N1）；Linux 侧通过 `S7_6_RC_OUTPUT`
指定实际设备。源码不把两个发送端并接到 UART2/CRSF。

## 冻结映射与范围

线上的整数 `1000` 表示归一化 `1.0`。首版映射有意不抬升 Throttle 和 AUX：

| ACTIVE 手势边沿 | valid | Roll | Pitch | Yaw | Throttle | AUX | 单次命令 |
|---|---:|---:|---:|---:|---:|---:|---|
| `OPEN_PALM` | 1 | 0 | -300 | 0 | 0 | 0 | 向前（低头）的短 Pitch 脉冲 |
| `FIST` | 0 | 0 | 0 | 0 | 0 | 0 | 显式释放 |
| `POINT` | 1 | 0 | +300 | 0 | 0 | 0 | 向后（抬头）的短 Pitch 脉冲 |
| `V_SIGN` | 1 | -300 | 0 | 0 | 0 | 0 | 向左的短 Roll 脉冲 |

Roll/Pitch/Yaw 合法范围为 `[-300,+300]`，最大变化率 `600/s`；Throttle 合法范围
`[0,250]`、最大变化率 `400/s`；AUX 合法范围 `[0,1000]`、最大变化率 `1000/s`。当前映射的
Throttle/AUX 目标始终为释放值 0，ARM 和总授权 AUX 永远不由 Linux 生成。每个手势只在
首次进入 `ACTIVE` 时触发一次 1 s 目标脉冲，保持同一手势不会重复触发；目标保持期结束后
按同一变化率自动回中。没有光流、定位或速度闭环时，这只是“短时姿态命令”，不保证精确位移。

摄像头和推理管线健康时，`NO_HAND/UNKNOWN/RELEASED/CANDIDATE` 都持续发送 `valid=1` 的中立帧；
尚未结束的 1 s 命令继续执行，结束后回中，因此 AUX3 授权不会被正常识别空档打断。中立帧使用
`gesture_id=UNKNOWN`、`confidence=0`、全通道 0。握拳、deadline miss、采集/推理失败、源时间过期或
进程重启才发送无效释放帧。串口写失败直接终止，不自动重连或恢复旧动作。

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
新鲜度、递增心跳和递增有效源序号任一失败即拒绝。飞控端现已在 USART6 增加被动接收监视，
并从 UART4 输出字节/帧/CRC/格式统计和最近帧字段。该监视不发布 RC 输入；真正的源仲裁和人工授权仍在 S7.7 实现。

## 实物接线

正点原子硬件参考手册第 26 页在 RK3568 扩展排针 JP11 上给出 UART9 发送脚：10 号脚是 3.3 V
`UART9_TX_M1`，21 号脚是地。接飞控时用这一路连到飞控 USART6 的接收脚 PC7。板端设备名通常
是 `/dev/ttyS9`，运行前仍须用 `ls -l /dev/ttyS*` 核对；飞控 UART2 继续专供 CRSF，UART4
继续输出调试日志。

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

# 板端先自检；DRM 直显前停止 Weston。/dev/null 只验证实时软件链，不算通信验收。
cd /userdata
./s7_6_live --self-test
killall weston 2>/dev/null || true

# 不开摄像头，直接向实际串口发送 20 个测试帧；终端逐帧打印完整 44 字节
./s7_6_live --uart-test /dev/ttyS9 20 cycle

# 连续发送固定单指帧，供 S7.7 授权、接管和超时测试
./s7_6_live --uart-test /dev/ttyS9 200 point

mkdir -p /userdata/live_annot
nohup env \
  LD_LIBRARY_PATH=/userdata/s7_3_acceptance/s7_3_deploy_20260813/lib:/usr/lib \
  S7_6_RC_OUTPUT=/dev/null \
  ./s7_6_live --camera \
  /userdata/s7_3_acceptance/s7_3_deploy_20260813/models/fp16/hand_detector_fp16.rknn \
  /userdata/s7_3_acceptance/s7_3_deploy_20260813/models/fp16/hand_landmarks_detector_fp16.rknn \
  /dev/video0 800 600 20 0 200 4776 /userdata/live_annot 5 /dev/dri/card0 \
  >/userdata/live_display.jsonl 2>/userdata/live_display.log </dev/null &

# 确认 RK3568 实际 UART 设备后，把上面 /dev/null 替换为对应 /dev/tty*，才会同时发送到飞控 USART6。

# 核对 RK3568 物理资源后才把 /dev/null 换成已验证的 /dev/tty*。
```

每次 `--uart-test` 启动时，第一帧固定为无效释放标记，随后才发送所选固定手势。这样 Linux
进程重启后，飞控可以安全重置序号门禁，但仍必须由操作者重新拨动授权开关。

当前事件式映射已通过主机纯逻辑自检。旧版 ARM64 产物和 SHA 对应持续量映射，已失效；
必须重新交叉构建并在板端复核单次脉冲、1 s 切换宽限、握拳释放、杀进程后 150 ms 失效和断线恢复。
