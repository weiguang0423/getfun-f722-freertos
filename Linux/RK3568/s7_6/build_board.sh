#!/bin/sh
# 复用 S7.5 实时链，增加 S7.6 映射、CRC 帧与可选串口输出。
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
SDK=${SDK:-"$HOME/rk3568_sdk"}
HOST="$SDK/buildroot/output/rockchip_rk3568/host"
RKNN="$SDK/external/rknpu2/runtime/RK356X/Linux/librknn_api"
OPENCV=${OPENCV:-"$HOME/s7_3_opencv/install"}
DRMHDR=${DRMHDR:-"$HOME/s7_4_drm_headers"}

# 板端 Buildroot 为 glibc 2.35，必须用 gcc-10 编出 GLIBC_2.17 兼容产物；
# SDK 自带 buildroot host 符号链接指向 gcc-13（需要 GLIBC_2.36+），板上无法运行。
RK3568_CC=${RK3568_CC:-aarch64-linux-gnu-g++-10}
if ! command -v "$RK3568_CC" >/dev/null 2>&1; then
  echo "找不到 ARM64 交叉编译器：$RK3568_CC（可用 RK3568_CC 覆盖）" >&2
  exit 1
fi

"$RK3568_CC" -std=c++17 -O2 -Wall -Wextra -D__user= \
  -DS7_5_GESTURE -DS7_6_VIRTUAL_RC \
  "$SCRIPT_DIR/../s7_3/s7_3_board_infer.cpp" \
  "$SCRIPT_DIR/../s7_4/s7_4_display.cpp" \
  "$SCRIPT_DIR/../s7_5/s7_5_gesture.cpp" \
  "$SCRIPT_DIR/s7_6_virtual_rc.cpp" \
  "$SCRIPT_DIR/s7_6_serial.cpp" \
  -I"$OPENCV/include/opencv4" -I"$RKNN/include" -I"$DRMHDR" \
  -I"$SCRIPT_DIR/../s7_4" -I"$SCRIPT_DIR/../s7_5" -I"$SCRIPT_DIR" \
  -L"$OPENCV/lib" -L"$RKNN/aarch64" \
  -static-libstdc++ -static-libgcc -Wl,--allow-shlib-undefined -Wl,-rpath,'$ORIGIN/lib' -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lrknnrt \
  -o "$SCRIPT_DIR/s7_6_live"

file "$SCRIPT_DIR/s7_6_live"
