<!--
文件作用：S7.5 板端手势分类、时序状态机、构建与验收入口。
覆盖范围：四类单手静态手势、置信度、五态防抖、JSONL/DRM 叠加和离线统计。
关联模块：s7_5_gesture.cpp、复用的 s7_3_board_infer.cpp、s7_4_display.cpp、evaluate_results.py。
验收状态：开发者于2026-08-17明确确认通过；本文件保留复现方法，不重复推测验收数据。
-->

# S7.5 手势集合与时序状态机

S7.5 复用 S7.4 已验收的摄像头、FP16 两级推理和 DRM 显示，只增加纯 C++ 几何分类与安全
状态机。当前不生成虚拟 RC、不连接 STM32，也不改变 S7.4 已冻结程序。

## 冻结手势与门槛

| ID | 名称 | 几何定义 |
|---:|---|---|
| 1 | `OPEN_PALM` | 拇指及四指伸展 |
| 2 | `FIST` | 拇指及四指弯曲 |
| 3 | `POINT` | 仅食指伸展 |
| 4 | `V_SIGN` | 食指、中指伸展且分开，环指、小指弯曲 |

分类置信度同时受关键点几何得分和 `presence_score` 限制；最低分类门槛 0.58、第一/第二候选
最小差值 0.08。状态机只有同一手势置信度不低于 0.75、连续 5 帧且跨度至少 150 ms 才进入
`ACTIVE`。相邻有效输入间隔超过 250 ms 立即释放，释放冷却 300 ms。

状态固定为 `NO_HAND`、`UNKNOWN`、`CANDIDATE`、`ACTIVE`、`RELEASED`。只有 `ACTIVE` 时
`active_gesture_id/name` 非零/非 `UNKNOWN`；无手、未知、低置信度、推理/采集失败、超时、
deadline miss 或活动手势切换都会在当前结果清空活动手势。

## 构建与运行

```sh
# PC/WSL 纯逻辑检查（不需要 RKNN/OpenCV）
g++ -std=c++17 -O2 -Wall -Wextra \
  Linux/RK3568/s7_5/s7_5_gesture.cpp \
  Linux/RK3568/s7_5/s7_5_gesture_test.cpp -o /tmp/s7_5_gesture_test
/tmp/s7_5_gesture_test

# 使用 S7.3/S7.4 冻结环境交叉构建
sh Linux/RK3568/s7_5/build_board.sh

# 板端先自检，再运行 20 fps 实时屏显
./s7_5_live --self-test
./s7_5_live --camera hand_detector_fp16.rknn hand_landmarks_detector_fp16.rknn \
  /dev/video0 800 600 20 180 200 4776 /userdata/s7_5_acceptance/annot 5 \
  /dev/dri/card0 >/userdata/s7_5_acceptance/results.jsonl \
  2>/userdata/s7_5_acceptance/runtime.log
```

屏显第一行是逐帧观测手势与置信度，第二行是状态、活动手势和候选连续帧数。绿色表示
`ACTIVE`，橙色表示其他安全非活动状态。

## 混淆、延迟和误触发统计

按 `labels.example.csv` 格式记录每段真实手势的驱动序号范围，然后执行：

```sh
python3 Linux/RK3568/s7_5/evaluate_results.py labels.csv results.jsonl >summary.json
```

摘要包含完整 `expected x active` 混淆矩阵、每个标注区间的首次进入延迟，以及所有无标签/
`UNKNOWN` 区间内的误触发序号。验收方法要求四类手势各重复至少 10 次，并单独覆盖无手、半握、
遮挡、画面边缘和快速切换。开发者已于2026-08-17确认 S7.5 验收通过；本阶段不再复查原始数据。
