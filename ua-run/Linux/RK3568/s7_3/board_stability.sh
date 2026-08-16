#!/bin/sh
set -eu

if [ "$#" -ne 5 ]; then
  echo "usage: board_stability.sh APP DETECTOR LANDMARK TESTSET OUTPUT_DIR" >&2
  exit 2
fi

APP=$1
DETECTOR=$2
LANDMARK=$3
TESTSET=$4
OUTPUT=$5
mkdir -p "$OUTPUT"

"$APP" "$DETECTOR" "$LANDMARK" "$TESTSET" 1800 >"$OUTPUT/results.jsonl" 2>"$OUTPUT/runtime.log" &
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
wait "$PID"
trap - EXIT INT TERM
