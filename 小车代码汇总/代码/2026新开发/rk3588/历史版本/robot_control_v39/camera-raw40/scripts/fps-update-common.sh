#!/bin/sh
# Shared by the receiver-only install and rollback tools; POSIX/BusyBox sh.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ISP=${ISP_DIR:-/opt/fpga-camera-isp-1080}
fail() { printf '%s\n' "$*" >&2; exit 2; }
check_board() {
 case $(uname -m) in aarch64|arm64) ;; *) fail 'Run this installer on the ARM64 RK3588 board.' ;; esac
 [ "$(id -u)" = 0 ] || fail 'Run as root.'
 [ -f "$ISP/isp-probe" ] && [ -f "$ISP/lib/librkaiq.so" ] && [ -f "$ISP/run_isp_camera.sh" ] || fail "Existing isolated camera runtime not found: $ISP (set ISP_DIR if needed)."
 ISP=$(CDPATH= cd -- "$ISP" && pwd -P)
 [ ! -L "$ISP/isp-probe" ] || fail 'isp-probe is a symlink; refusing to replace a shared binary.'
}
check_consumers() {
 pids=
 for directory in /proc/[0-9]*; do
  [ -r "$directory/comm" ] || continue
  executable=$(readlink "$directory/exe" 2>/dev/null || true)
  executable=${executable% (deleted)}
  name=${executable##*/}
  case "$name" in
   perception-dashboard|perception-dashboard.aarch64|rknn_yolov8_demo)
    fail 'Close the YOLO/IMU window first (Ctrl+C in its terminal), then rerun.' ;;
   isp-probe)
    [ "$executable" = "$ISP/isp-probe" ] || fail "Another ISP runtime is active: $executable. Close that session first."
    pids="$pids ${directory##*/}" ;;
  esac
 done
}
stop_receiver() {
 for pid in $pids; do kill -TERM "$pid" 2>/dev/null || true; done
 for pid in $pids; do
  i=0
  while kill -0 "$pid" 2>/dev/null; do
   state=$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null || true)
   [ "$state" != Z ] || break
   i=$((i+1))
   [ "$i" -le 15 ] || fail "ISP $pid has not stopped. No binary was replaced; retry after closing the camera session."
   sleep 1
  done
 done
 # Let the existing launcher's wait/EXIT cleanup release its own lock.
 if [ -n "$pids" ]; then
  i=0
  while [ -f /tmp/fpga-isp.lock/pid ]; do
   launcher=$(cat /tmp/fpga-isp.lock/pid 2>/dev/null || true)
   case "$launcher" in ''|*[!0-9]*) break ;; esac
   kill -0 "$launcher" 2>/dev/null || break
   i=$((i+1))
   [ "$i" -le 10 ] || fail 'ISP launcher has not exited. Close its terminal session and rerun; no binary was replaced.'
   sleep 1
  done
 fi
}
