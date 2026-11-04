#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ISP=${ISP_DIR:-/opt/fpga-camera-isp-1080}
DASH=${DASHBOARD_DIR:-/root/robot-perception}
fail() { printf '%s\n' "$*" >&2; exit 2; }
check_board() {
 case $(uname -m) in aarch64|arm64) ;; *) fail 'Run on the ARM64 RK3588 board.' ;; esac
 [ "$(id -u)" = 0 ] || fail 'Run as root.'
 [ -d "$ISP" ] && [ -d "$DASH" ] || fail 'Install the current full RK application package first.'
 ISP=$(CDPATH= cd -- "$ISP" && pwd -P)
 DASH=$(CDPATH= cd -- "$DASH" && pwd -P)
 for path in "$ISP/isp-probe" "$ISP/run_isp_camera.sh" "$ISP/lib/librkaiq.so" "$DASH/run-perception.sh"; do
  [ -f "$path" ] && [ ! -L "$path" ] || fail "Missing or symlinked runtime file: $path"
 done
 [ ! -L "$ISP/camera-target-fps" ] || fail 'Refusing a symlinked camera FPS configuration.'
}
check_consumers() {
 pids=
 for directory in /proc/[0-9]*; do
  [ -r "$directory/comm" ] || continue
  executable=$(readlink "$directory/exe" 2>/dev/null || true)
  executable=${executable% (deleted)}
  name=${executable##*/}
  case "$name" in
   perception-dashboard|perception-dashboard.aarch64|rknn_yolov8_demo|robot-navigation.aarch64)
    fail 'Close the YOLO/IMU and navigation programs before updating.' ;;
   isp-probe)
    [ "$executable" = "$ISP/isp-probe" ] || fail "A different ISP runtime is active: $executable"
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
   i=$((i+1)); [ "$i" -le 15 ] || fail "ISP $pid did not exit; no runtime file was replaced."
   sleep 1
  done
 done
 if [ -n "$pids" ]; then
  i=0
  while [ -f /tmp/fpga-isp.lock/pid ]; do
   launcher=$(cat /tmp/fpga-isp.lock/pid 2>/dev/null || true)
   case "$launcher" in ''|*[!0-9]*) break ;; esac
   kill -0 "$launcher" 2>/dev/null || break
   i=$((i+1)); [ "$i" -le 10 ] || fail 'ISP launcher still active; no runtime file was replaced.'
   sleep 1
  done
 fi
}
sha() { value=$(sha256sum "$1"); printf '%s\n' "${value%% *}"; }
verify_backup() {
 [ -d "$backup" ] && [ ! -L "$backup" ] || fail 'Backup not found or symlinked.'
 backup=$(CDPATH= cd -- "$backup" && pwd -P)
 case "$backup" in "$ISP"/camera-highfps-backup.*) ;; *) fail 'Backup must belong to this ISP runtime.' ;; esac
 [ -f "$backup/SHA256SUMS" ] || fail 'Backup checksum file is missing.'
 (cd "$backup" && sha256sum -c SHA256SUMS) || fail 'Backup checksum verification failed.'
 [ "$(cat "$backup/isp_path")" = "$ISP" ] && [ "$(cat "$backup/dash_path")" = "$DASH" ] || fail 'Backup targets do not match this runtime.'
 case $(cat "$backup/config_state") in
  present) [ -f "$backup/camera-target-fps" ] && [ ! -L "$backup/camera-target-fps" ] || fail 'Backup FPS configuration is missing.' ;;
  absent) [ ! -e "$backup/camera-target-fps" ] || fail 'Inconsistent backup FPS configuration.' ;;
  *) fail 'Unknown backup FPS configuration state.' ;;
 esac
}
restore_backup() {
 # Stages have already been verified before commit/rollback; use atomic renames.
 restore_probe=$(mktemp "$ISP/.highfps-restore-probe.XXXXXX") || return 1
 restore_isp=$(mktemp "$ISP/.highfps-restore-launch.XXXXXX") || return 1
 restore_app=$(mktemp "$DASH/.highfps-restore-launch.XXXXXX") || return 1
 restore_config=$(mktemp "$ISP/.highfps-restore-config.XXXXXX") || return 1
 cp -p "$backup/isp-probe" "$restore_probe" &&
 cp -p "$backup/run_isp_camera.sh" "$restore_isp" &&
 cp -p "$backup/run-perception.sh" "$restore_app" || return 1
 if [ "$(cat "$backup/config_state")" = present ]; then
  cp -p "$backup/camera-target-fps" "$restore_config" || return 1
 fi
 mv -f "$restore_probe" "$ISP/isp-probe" &&
 mv -f "$restore_isp" "$ISP/run_isp_camera.sh" &&
 mv -f "$restore_app" "$DASH/run-perception.sh" || return 1
 if [ "$(cat "$backup/config_state")" = present ]; then
  mv -f "$restore_config" "$ISP/camera-target-fps" || return 1
 else rm -f "$ISP/camera-target-fps" "$restore_config" || return 1; fi
 [ "$(sha "$ISP/isp-probe")" = "$(sha "$backup/isp-probe")" ] &&
 [ "$(sha "$ISP/run_isp_camera.sh")" = "$(sha "$backup/run_isp_camera.sh")" ] &&
 [ "$(sha "$DASH/run-perception.sh")" = "$(sha "$backup/run-perception.sh")" ] || return 1
 if [ "$(cat "$backup/config_state")" = present ]; then
  [ "$(sha "$ISP/camera-target-fps")" = "$(sha "$backup/camera-target-fps")" ] || return 1
 else [ ! -e "$ISP/camera-target-fps" ] || return 1; fi
}
