#!/bin/sh
# 文件作用：S7.4 屏显方案前置条件核验（RK3568 板端）。确认显示屏是否被内核识别、
# 分辨率/接口、fbdev 是否可用，以及当前显示相关进程。任何 [MISSING]/异常输出
# 都应先解决再进入屏显开发。
# 用法：板端 root shell 执行  sh precheck_board.sh
set -u
echo "== 1. DRM 设备节点 =="
ls -l /dev/dri/ 2>&1 || echo "MISSING /dev/dri（内核显示被禁用或未配置）"

echo "== 2. 各 card 状态与可用模式 =="
for d in /sys/class/drm/card*/; do
  [ -d "$d" ] || continue
  echo "--- $d"
  echo "status: $(cat "$d/status" 2>&1)"
  echo "modes:  $(tr '\n' ' ' < "$d/modes" 2>&1)"
done

echo "== 3. fbdev 模拟（可选备选路线）=="
ls -l /dev/fb* 2>&1 || echo "absent /dev/fb*（无 fbdev 模拟，不影响 DRM 直显）"
[ -r /sys/class/graphics/fb0/name ] && echo "fb0 name: $(cat /sys/class/graphics/fb0/name 2>&1)"

echo "== 4. 内核 cmdline =="
cat /proc/cmdline; echo

echo "== 5. 显示相关内核日志（drm/dsi/panel/hdmi/vop）=="
dmesg 2>/dev/null | grep -iE "drm|dsi|panel|hdmi|vop" | tail -30 || echo "dmesg 不可用"

echo "== 6. 已运行的显示/合成进程 =="
ps aux 2>/dev/null | grep -iE "weston|Xorg|wayland|kms|qt" | grep -v grep || echo "无（干净环境，DRM 直显可直接用）"

echo "== 7. 屏的物理信息（需人工确认）=="
echo "屏型号/接口(MIPI-DSI/HDMI/RGB)、分辨率、是否已通电连接，请人工记录"
echo "核验完成"
