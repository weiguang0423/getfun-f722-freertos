#!/bin/sh
# 文件作用：用 S7.4 已验收的同一 SDK/OpenCV/RKNN 环境交叉构建 S7.5 实时手势程序。
# 核心功能：复用实时采集、两级推理和 DRM 显示，只增加纯 C++ 手势模块与编译开关。
# 关键约束：不联网、不生成 RC；SDK、模型和 DRM 头版本继续沿用 S7.3/S7.4 冻结值。
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
SDK=${SDK:-/home/user123/rk3568_sdk}
HOST="$SDK/buildroot/output/rockchip_rk3568/host"
RKNN="$SDK/external/rknpu2/runtime/RK356X/Linux/librknn_api"
OPENCV=${OPENCV:-/home/user123/s7_3_opencv/install}
DRMHDR=${DRMHDR:-/home/user123/s7_4_drm_headers}

"$HOST/bin/aarch64-buildroot-linux-gnu-g++" -std=c++17 -O2 -Wall -Wextra -D__user= \
  -DS7_5_GESTURE \
  "$SCRIPT_DIR/../s7_3/s7_3_board_infer.cpp" \
  "$SCRIPT_DIR/../s7_4/s7_4_display.cpp" \
  "$SCRIPT_DIR/s7_5_gesture.cpp" \
  -I"$OPENCV/include/opencv4" -I"$RKNN/include" -I"$DRMHDR" \
  -I"$SCRIPT_DIR/../s7_4" -I"$SCRIPT_DIR" \
  -L"$OPENCV/lib" -L"$RKNN/aarch64" \
  -Wl,-rpath,'$ORIGIN/lib' -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lrknnrt \
  -o "$SCRIPT_DIR/s7_5_live"

file "$SCRIPT_DIR/s7_5_live"
