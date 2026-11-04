#!/bin/sh
# Read-only version/model/process/performance check. Never starts or arms motors.
set -u
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
APP=$(CDPATH= cd -- "$HERE/.." && pwd)
printf '%s\n' 'Unified perception read-only diagnostic'
if [ -x "$APP/perception-dashboard.aarch64" ]; then "$APP/perception-dashboard.aarch64" --version;fi
printf '%s\n' 'Model and matching label fingerprints (industrial custom 10-class YOLOv8s INT8)'
for model in "$APP/model/industrial-yolov8s-int8-rk3588.rknn" "$APP/model/industrial_labels_list.txt"; do
 [ ! -f "$model" ] || sha256sum "$model"
done
printf '%s\n' 'Running perception programs (multiple active copies compete for camera/NPU)'
for process in /proc/[0-9]*; do
 [ -r "$process/comm" ] || continue
 name=$(cat "$process/comm" 2>/dev/null) || continue
 case "$name" in
  perception-dash*|robot-phone-ser*|rknn_yolov8_dem*|isp-probe)
   executable=$(readlink "$process/exe" 2>/dev/null || printf '%s' unavailable)
   printf '  pid=%s name=%s executable=%s\n' "${process##*/}" "$name" "$executable" ;;
 esac
done
if [ -f "$HERE/diagnose-performance.sh" ]; then sh "$HERE/diagnose-performance.sh";fi
printf '%s\n' 'For a comparison: quit the current window, then start the same launcher with --no-phone.' \
 'Compare PERF result_fps and STAGE npu_worker means after warm-up, then restart without --no-phone.'
