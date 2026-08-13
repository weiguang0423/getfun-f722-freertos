#!/bin/sh
set -eu

SDK=${SDK:-/home/user123/rk3568_sdk}
HOST="$SDK/buildroot/output/rockchip_rk3568/host"
RKNN="$SDK/external/rknpu2/runtime/RK356X/Linux/librknn_api"
OPENCV=${OPENCV:-/home/user123/s7_3_opencv/install}

"$HOST/bin/aarch64-buildroot-linux-gnu-g++" -std=c++17 -O2 -Wall -Wextra \
  Tools/s7_3/s7_3_board_infer.cpp \
  -I"$OPENCV/include/opencv4" -I"$RKNN/include" \
  -L"$OPENCV/lib" -L"$RKNN/aarch64" \
  -Wl,-rpath,'$ORIGIN/lib' -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lrknnrt \
  -o Tools/s7_3/s7_3_board_infer

file Tools/s7_3/s7_3_board_infer
