#!/bin/sh
# 文件作用：S7.4 屏显方案前置条件核验（WSL 主机侧）。检查 SDK 里是否具备 DRM/KMS 直显
# 所需的构建条件：内核 uapi 头、libdrm（可选）、modetest（可选）、工具链和 OpenCV。
# 用法：在 WSL 中执行  sh Linux/RK3568/s7_4/precheck_wsl.sh
# 输出：逐项存在性；任何 [MISSING] 项需在开始屏显开发前处理。
set -u
SDK=${SDK:-/home/user123/rk3568_sdk}
HOST="$SDK/buildroot/output/rockchip_rk3568/host"
STAGING="$SDK/buildroot/output/rockchip_rk3568/staging"
TARGET="$SDK/buildroot/output/rockchip_rk3568/target"

echo "== 1. SDK 根目录 =="
[ -d "$SDK" ] && echo "OK   $SDK" || echo "MISSING $SDK"

echo "== 2. 内核 uapi DRM 头（DRM/KMS 直显必需）=="
for h in "$SDK/kernel/include/uapi/drm/drm.h" "$SDK/kernel/include/uapi/drm/drm_mode.h"; do
  [ -f "$h" ] && echo "OK   $h" || echo "MISSING $h"
done
grep -E "^(VERSION|PATCHLEVEL|SUBLEVEL) =" "$SDK/kernel/Makefile" 2>/dev/null || echo "MISSING kernel Makefile version"

echo "== 3. libdrm 库（可选；缺失则用内核 uapi 头裸 ioctl）=="
for f in "$STAGING/usr/include/libdrm/drm.h" "$STAGING/usr/lib/libdrm.so" "$TARGET/usr/lib/libdrm.so.2"; do
  [ -e "$f" ] && echo "OK   $f" || echo "absent $f (可选)"
done

echo "== 4. modetest 工具（可选，板端调试用）=="
for f in "$HOST/bin/modetest" "$HOST/usr/bin/modetest" "$STAGING/usr/bin/modetest" "$TARGET/usr/bin/modetest"; do
  [ -e "$f" ] && echo "OK   $f" || echo "absent $f (可选)"
done

echo "== 5. 交叉工具链 =="
[ -x "$HOST/bin/aarch64-buildroot-linux-gnu-g++" ] && echo "OK   toolchain g++" || echo "MISSING toolchain g++"

echo "== 6. OpenCV 4.5.5 构建产物（S7.3 已用）=="
for f in /home/user123/s7_3_opencv/install/include/opencv4/opencv2/imgproc.hpp          /home/user123/s7_3_opencv/install/lib/libopencv_core.so; do
  [ -e "$f" ] && echo "OK   $f" || echo "MISSING $f"
done

echo "== 7. 板端部署包（模型/二进制所在）=="
for f in "$SDK/../s7_3_deploy_20260813.tar.gz"; do
  [ -e "$f" ] && echo "OK   $f" || echo "absent $f (部署包可留在板端，主机不必有)"
done
echo "核验完成"
