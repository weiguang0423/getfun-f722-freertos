# S7.3 RKNN 转换与板端离线验收

本目录只覆盖 S7.3：把 S7.2 冻结的两级 TFLite 模型转换为 RK3568 模型，并完成板端离线误差、性能与稳定性验收。不包含摄像头实时链、手势状态机或飞控接入。

转换环境固定为 SDK 自带 RKNN-Toolkit2 1.3.0、Python 3.8 和 `target_platform=rk3568`。输入保持 RGB，转换层执行 `value / 255`，与 S7.2 PC 参考链一致。

```sh
python Tools/s7_3/convert_models.py \
  --detector hand_detector.tflite \
  --landmark hand_landmarks_detector.tflite \
  --output s7_3_models
```

`conversion_manifest.json`、两个转换日志和模型 SHA-256 是转换交付证据。模型原件与测试集仍保留在外部验证环境，不复制进飞控仓库。

## 当前交付物

- 最终 FP16：`/home/user123/s7_3_models_final_fp16`。
- 最终 INT8：`/home/user123/s7_3_models_final_int8`。
- 固定同字节语料：`C:\Users\user\AppData\Local\Temp\s7_3_fixed_testset`，2859 帧。
- PC 参考结果：`C:\Users\user\AppData\Local\Temp\s7_3_pc_fixed`。
- ARM64 程序：`Tools/s7_3/s7_3_board_infer`。
- 主机侧证据与冻结验收范围：`Tools/s7_3/evidence/host_preparation.json`。
- 一次性部署包：`/home/user123/s7_3_deploy_20260813.tar.gz`，包含程序、FP16/INT8模型、
  私有OpenCV库、2859帧固定语料和稳定性脚本；SHA-256为
  `bfe5d236914b3c5ed7543d78fa4516360851792729a804e5ecf7742493ce31b4`。

`build_opencv.sh` 从SDK已缓存的OpenCV 4.5.5源码离线构建最小运行库；`build_board.sh` 使用SDK
交叉编译器和`/home/user123/s7_3_opencv/install`中仅含
`core/imgproc/imgcodecs` 的 OpenCV 4.5.5。板端部署时把程序旁建 `lib/`，复制这三个模块及其
`.405` 符号链接；板上已有 `librknnrt.so`，不得替换板端 Runtime。

完整固定集运行：

```sh
./s7_3_board_infer hand_detector_fp16.rknn hand_landmarks_detector_fp16.rknn fixed_testset > fp16.jsonl
python Tools/s7_3/compare_results.py --pc s7_3_pc_fixed/results.jsonl --board fp16.jsonl --output fp16_compare.json
```

INT8 使用同一命令替换模型。`board_stability.sh` 在同一进程内循环固定集 1800 秒，每 10 秒记录
RSS、文件描述符、温度和可用时的 NPU 负载。S7.3 只有在 FP16/INT8 比较、板端性能与 30 分钟
稳定性证据齐全后才能关闭；当前 RK3568 未连接，主机准备完成不等于实物验收完成。
