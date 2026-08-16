#!/bin/sh
set -eu

if [ "$#" -lt 4 ] || [ "$#" -gt 10 ]; then
  echo "usage: run_stability.sh APP DETECTOR LANDMARK OUTPUT_DIR [DEVICE] [WIDTH] [HEIGHT] [FPS] [MAX_LATENCY_MS] [VBLANK]" >&2
  exit 2
fi

APP=$1
DETECTOR=$2
LANDMARK=$3
OUTPUT=$4
DEVICE=${5:-/dev/video0}
WIDTH=${6:-800}
HEIGHT=${7:-600}
FPS=${8:-20}
MAX_LATENCY_MS=${9:-200}
VBLANK=${10:-4776}
mkdir -p "$OUTPUT"

v4l2-ctl -d "$DEVICE" --all >"$OUTPUT/camera.txt" 2>&1 || true
"$APP" --camera "$DETECTOR" "$LANDMARK" "$DEVICE" "$WIDTH" "$HEIGHT" "$FPS" \
  300 "$MAX_LATENCY_MS" "$VBLANK" >"$OUTPUT/results.jsonl" 2>"$OUTPUT/runtime.log" &
PID=$!
trap 'kill "$PID" 2>/dev/null || true' EXIT INT TERM
echo 'epoch_s,rss_kb,fd_count,temp_mC,npu_load' >"$OUTPUT/resources.csv"
while kill -0 "$PID" 2>/dev/null; do
  RSS=$(awk '/VmRSS:/ {print $2}' "/proc/$PID/status")
  FDS=$(ls -1 "/proc/$PID/fd" | wc -l)
  TEMP=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo unavailable)
  NPU=unavailable
  for PATHNAME in /sys/class/devfreq/*npu*/load /sys/class/devfreq/fde40000.npu/load; do
    [ -r "$PATHNAME" ] && NPU=$(cat "$PATHNAME") && break
  done
  echo "$(date +%s),${RSS:-0},$FDS,$TEMP,$NPU" >>"$OUTPUT/resources.csv"
  sleep 10
done
set +e
wait "$PID"
STATUS=$?
set -e
trap - EXIT INT TERM
echo "$STATUS" >"$OUTPUT/exit_code.txt"
exit "$STATUS"
