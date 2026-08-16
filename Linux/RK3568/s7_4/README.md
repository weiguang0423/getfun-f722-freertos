# S7.4 摄像头实时手部跟踪

本目录只把 S7.1 的 IMX335 采集与 S7.3 选定的 FP16 两级推理串成实时链。它不判断手势，
不生成虚拟 RC，也不修改飞控。推理仍复用已完成离线验收的
`../s7_3/s7_3_board_infer.cpp`，实时模式只增加 Linux 原生 V4L2 MMAP 采集。

## 构建与运行

```sh
sh Linux/RK3568/s7_4/build_board.sh
./Linux/RK3568/s7_4/s7_4_live --self-test
./Linux/RK3568/s7_4/s7_4_live --camera \
  hand_detector_fp16.rknn hand_landmarks_detector_fp16.rknn \
  /dev/video0 800 600 30 60 200 -1 >results.jsonl 2>runtime.log
```

末尾五个可选参数依次是运行秒数、最大端到端延迟毫秒数、`vertical_blanking`（`-1`表示不改
传感器值）、标注帧目录 `ANNOTATE_DIR` 和采样间隔 `EVERY_N`（默认5，即每5个处理帧存一张
标注图）。省略运行秒数时持续运行到信号退出。程序退出时恢复启动前的`vertical_blanking`。
程序排空已就绪的 V4L2 缓冲，只处理最新帧。每条结果记录驱动帧序号、采集/完成时间戳、
累计丢帧、端到端延迟、有效性、置信度和各推理阶段耗时。坏帧、采集超时、推理失败或
超过延迟门槛统一输出 `valid:false`，不会沿用旧结果。

## 可视化验收：真人手部关键点确认

程序只输出逐帧 JSONL，不保存画面，仅靠数值无法确认关键点是否贴合真实手部。验收时把
`ANNOTATE_DIR` 指到输出目录，程序把检测框、21个关键点骨架和置信度文本叠加画到帧上存为
JPEG，取回 PC 后肉眼核对：

```sh
# 板端：20 fps 实时推理，每5帧存一张标注图（EVERY_N 调小可增加密度）
./s7_4_live --camera hand_detector_fp16.rknn hand_landmarks_detector_fp16.rknn \
  /dev/video0 800 600 20 90 200 4776 /userdata/s7_4_acceptance/annot 5 \
  >/userdata/s7_4_acceptance/results.jsonl 2>/userdata/s7_4_acceptance/runtime.log

# 取回 PC（IP/账号按实际板端调整）
scp -r user@169.254.151.86:/userdata/s7_4_acceptance/annot .

# 可选：不跑推理也能确认摄像头画面本身（零代码）
v4l2-ctl -d /dev/video0 --set-fmt-video=width=800,height=600,pixelformat=NV12 \
  --stream-mmap --stream-count=1 --stream-to=/tmp/frame.raw
# PC 侧转 JPEG：ffmpeg -f rawvideo -pix_fmt nv12 -s 800x600 -i frame.raw frame.jpg

# 可选：标注帧合成视频（序号有缺口时 ffmpeg 会在缺口处停止；Windows 上也可用 VLC 直接播放文件夹）
ffmpeg -framerate 10 -pattern_type glob -i 'annot_*.jpg' annot.mp4
```

标注图左上角叠加 `seq`、`valid`、`presence` 和 `handed`，绿框为检测框，黄线为手骨架，
红点为关键点。只保存检测器输出非空的帧；`no_hand` 帧不进目录，无手时长与误检用
JSONL 统计。

真人验收场景（对应文档39关口2）：

| 场景 | 操作 | 判据 |
|---|---|---|
| 真人单手 | 五指张开伸入画面，缓慢移动和旋转 | 多数帧 `valid:true`、presence 高，标注图骨架贴合五指并跟随移动 |
| 无手 | 手完全移出画面 | 连续 `no_hand`，无残留 `valid:true` |
| 遮挡 | 手指部分遮挡、半握拳 | presence 下降或 `valid:false`，骨架不凭空拼出完整手 |
| 边缘 | 手在画面四边进出 | 框随手出现/消失，无卡死或错误裁剪 |
| 拔摄像头 | 运行中拔出 | 输出 `capture_failure` 失效记录后进程终止 |

## 5 分钟实物关口

```sh
sh Linux/RK3568/s7_4/run_stability.sh \
  ./Linux/RK3568/s7_4/s7_4_live \
  hand_detector_fp16.rknn hand_landmarks_detector_fp16.rknn evidence/s7_4
```

脚本保存摄像头配置、逐帧 JSONL、运行日志、退出码，以及每 10 秒一次的 RSS、文件描述符、
温度和 NPU 负载。本机只能完成源码检查和交叉构建；实时帧率、曝光、安装方向、延迟不增长
及5分钟稳定性必须在 ATK-DLRK3568 + IMX335 实物上验收后才能关闭 S7.4。

## 当前能判断什么

S7.4没有手势分类器。`valid`、`presence_score`、边框和21个关键点只能判断“是否跟踪到一只手”以及
几何位置是否合理，不能说明“这是哪个手势”。本次无手稳定性运行仍出现2条`valid:true`，因此原始
单帧结果不能直接触发动作。S7.5必须新增`gesture_id/name`、分类置信度、连续帧状态和叠加显示，用户
看到手势名称、置信度与关键点同时正确后，才能判断手势识别正确。

## 板端屏幕实时显示（2026-08-14 已实物验收通过）

开发板 DSI 屏（720×1280@60Hz）可以把带标注的处理帧实时显示到屏上。实现分两级，均不依赖
libdrm/gstreamer/Qt，只使用 SDK 配套的 DRM uapi 头（`/home/user123/s7_4_drm_headers/drm`，
由 `build_board.sh` 的 `DRMHDR` 指定）：

1. **屏显冒烟** `s7_4_drm_smoke`：验证屏是否被识别、分辨率/刷新率与 KMS 点亮链路。
   板端执行 `./s7_4_drm_smoke /dev/dri/card0 10`，屏上应出现随时间变化的渐变色块，
   输出 `smoke: ok, display link usable` 且退出码 0。
2. **显示后端** `s7_4_display.cpp/hpp`：`display_open` 枚举已连接 connector 首选模式，
   `display_show` 把 BGR 帧按纵横比 letterbox 缩放到屏分辨率（黑边）、转 XRGB8888 并用
   双缓冲 page flip 显示；失败只告警，不影响推理与 JSONL。

**使用前提**：板端 Weston 桌面合成器占用 DRM master，屏显前必须先释放：
`killall weston`（板子重启后 Weston 会恢复，屏显场景需要再次停止）。

**带屏显的实时推理**（最后一个参数是显示设备）：

```sh
./s7_4_live --camera hand_detector_fp16.rknn hand_landmarks_detector_fp16.rknn \
  /dev/video0 800 600 20 180 200 4776 /userdata/annot2 5 /dev/dri/card0
```

屏上实时显示摄像头画面（居中、上下黑边）与检测框、21点骨架、presence/handed 文本；
手移出画面后框消失。验收证据见 `evidence/live_20260814/`。
