#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
SDK=${SDK:-/home/user123/rk3568_sdk}
HOST="$SDK/buildroot/output/rockchip_rk3568/host"
RKNN="$SDK/external/rknpu2/runtime/RK356X/Linux/librknn_api"
OPENCV=${OPENCV:-/home/user123/s7_3_opencv/install}
# DRM uapi 头使用与板端内核配套的 SDK 拷贝（/home/user123/s7_4_drm_headers/drm），
# 由 precheck/构建前手动建立；不与 glibc 头冲突。
DRMHDR=${DRMHDR:-/home/user123/s7_4_drm_headers}

"$HOST/bin/aarch64-buildroot-linux-gnu-g++" -std=c++17 -O2 -Wall -Wextra -D__user= \
  "$SCRIPT_DIR/../s7_3/s7_3_board_infer.cpp" "$SCRIPT_DIR/s7_4_display.cpp" \
  -I"$OPENCV/include/opencv4" -I"$RKNN/include" -I"$DRMHDR" -I"$SCRIPT_DIR" \
  -L"$OPENCV/lib" -L"$RKNN/aarch64" \
  -Wl,-rpath,'$ORIGIN/lib' -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lrknnrt \
  -o "$SCRIPT_DIR/s7_4_live"

# 屏显链路冒烟程序：不依赖 libdrm/OpenCV，只使用 SDK 内核 uapi 头
"$HOST/bin/aarch64-buildroot-linux-gnu-g++" -std=c++17 -O2 -Wall -Wextra -D__user= \
  "$SCRIPT_DIR/s7_4_drm_smoke.cpp" \
  -I"$DRMHDR" \
  -o "$SCRIPT_DIR/s7_4_drm_smoke"

file "$SCRIPT_DIR/s7_4_live" "$SCRIPT_DIR/s7_4_drm_smoke"
