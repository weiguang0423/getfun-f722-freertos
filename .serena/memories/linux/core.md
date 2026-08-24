# RK3568 Linux core

- Git originals live under `Linux/RK3568/`; stages are cumulative, intentionally preserving acceptance boundaries.
- S7.1: RKNN/NPU smoke (`s7_1_rknn_smoke.c`). S7.2: PC LiteRT reference. S7.3: TFLite->RKNN conversion plus offline ARM64 inference. S7.4: V4L2 real-time capture + reused FP16 inference + optional raw DRM/KMS display. S7.5: dependency-light C++ geometric gesture classifier and temporal state machine. S7.6: bounded mapping, fixed wire frame, termios serial output.
- The shared executable entry remains `Linux/RK3568/s7_3/s7_3_board_infer.cpp:main`; build flags `S7_5_GESTURE` and `S7_6_VIRTUAL_RC` progressively add gesture and virtual-RC behavior.
- Main modes: `--self-test`; offline media inference; `--camera`; S7.6 also `--uart-test`.
- Camera loop uses V4L2 MMAP NV12, drains ready buffers and processes the newest frame, converts NV12->BGR, runs two RKNN models, emits per-frame JSONL to stdout and diagnostics to stderr.
- Optional DRM/KMS backend uses kernel uapi ioctls directly (no libdrm/Qt/GStreamer), XRGB8888 double buffering/page flips; Weston must release DRM master for direct display.
- Gesture state machine states: NO_HAND/UNKNOWN/CANDIDATE/ACTIVE/RELEASED. ACTIVE needs stable confidence/time/frame evidence; gaps, invalid input and changes clear active output.
- S7.6 mapper emits one bounded 1 s attitude pulse on a new ACTIVE edge; healthy recognition gaps emit valid neutral frames. FIST, stale source, failures or restart emit invalid release. Linux never creates ARM authority.
- `SerialLink` opens an explicit device with POSIX `open`, raw 115200 8N1 termios, nonblocking writes + `poll`, and fails closed on write/drain errors.