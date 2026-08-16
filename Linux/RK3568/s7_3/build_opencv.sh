#!/bin/sh
set -eu

SDK=${SDK:-/home/user123/rk3568_sdk}
SOURCE="$SDK/buildroot/output/rockchip_rk3568/build/opencv4-4.5.5"
HOST="$SDK/buildroot/output/rockchip_rk3568/host"
BUILD=${BUILD:-/home/user123/s7_3_opencv}

mkdir -p "$BUILD"
cd "$BUILD"
"$HOST/bin/cmake" "$SOURCE" \
  -DCMAKE_TOOLCHAIN_FILE="$HOST/share/buildroot/toolchainfile.cmake" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$BUILD/install" \
  -DBUILD_LIST=core,imgproc,imgcodecs -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF \
  -DBUILD_JPEG=ON -DWITH_JPEG=ON -DWITH_PNG=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF \
  -DWITH_OPENJPEG=OFF -DWITH_PROTOBUF=OFF -DWITH_IPP=OFF -DWITH_OPENCL=OFF -DWITH_ITT=OFF
make -j4
make install
