#!/bin/sh
# Shared by this local dashboard/model/receiver update; POSIX/BusyBox sh.
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
BASE=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd -P)
DASH=${DASHBOARD_DIR:-/root/perception-dashboard}
ISP=${ISP_DIR:-/opt/fpga-camera-isp-1080}
ROOT_YOLO=${ROOT_YOLO_SCRIPT:-/root/run-yolo.sh}
FILES='dashboard launcher model labels receiver rootentry'
V11_SHA=96489da6be36fd7700ad0df127c6e91c22b251b17f0f68877b2dc778f5b522b7
BASE_LAUNCHER_SHA=8045c13a0ce4a443398acce07b1c86df627384a77dc7f0520d31a61f641b3abb
# Previously delivered preview-v8 differs only in argument scanning for NPU workers.
V8_LAUNCHER_SHA=5c9c5e86a999a82326d4da2fe410184e1e61204ac956bb449e8e26e7e31ed355
fail() { printf '%s\n' "$*" >&2; exit 2; }
hash_file() { sha256sum "$1" | awk '{print $1}'; }
target_path() {
 case "$1" in
  dashboard) printf '%s\n' "$DASH/perception-dashboard.aarch64" ;;
  launcher) printf '%s\n' "$DASH/run-perception.sh" ;;
  model) printf '%s\n' "$DASH/model/yolo11s-int8-rk3588.rknn" ;;
  labels) printf '%s\n' "$DASH/model/coco_80_labels_list.txt" ;;
  receiver) printf '%s\n' "$ISP/isp-probe" ;;
  rootentry) printf '%s\n' "$ROOT_YOLO" ;;
  *) fail 'Unknown internal update entry.' ;;
 esac
}
payload_path() {
 case "$1" in
  dashboard) printf '%s\n' "$BASE/payload/perception-dashboard.aarch64" ;;
  launcher) printf '%s\n' "$BASE/payload/run-perception.sh" ;;
  model) printf '%s\n' "$BASE/models/yolo11s-int8-rk3588.rknn" ;;
  labels) printf '%s\n' "$BASE/models/coco_80_labels_list.txt" ;;
  receiver) printf '%s\n' "$BASE/payload/isp-probe" ;;
  rootentry) printf '%s\n' "$BASE/payload/run-yolo.sh" ;;
 esac
}
check_board() {
 case $(uname -m) in aarch64|arm64) ;; *) fail 'Run this installer on the ARM64 RK3588 board.' ;; esac
 [ "$(id -u)" = 0 ] || fail 'Run as root.'
 DASH=$(CDPATH= cd -- "$DASH" && pwd -P) || fail 'Existing dashboard directory not found.'
 ISP=$(CDPATH= cd -- "$ISP" && pwd -P) || fail 'Existing isolated ISP directory not found.'
 case "$ROOT_YOLO" in /*) ;; *) fail 'ROOT_YOLO_SCRIPT must be an absolute path.' ;; esac
 root_parent=$(CDPATH= cd -- "$(dirname -- "$ROOT_YOLO")" && pwd -P) || fail 'Root launcher parent directory not found.'
 ROOT_YOLO=$root_parent/$(basename -- "$ROOT_YOLO")
 case "$ROOT_YOLO" in "$DASH"|"$DASH"/*|"$ISP"|"$ISP"/*) fail 'ROOT_YOLO_SCRIPT must be outside the dashboard and ISP directories.' ;; esac
 [ -f "$DASH/perception-dashboard.aarch64" ] && [ -f "$DASH/run-perception.sh" ] && [ -f "$DASH/run-dashboard.sh" ] || fail "Working dashboard installation not found: $DASH"
 [ -f "$ISP/isp-probe" ] && [ -f "$ISP/lib/librkaiq.so" ] && [ -f "$ISP/run_isp_camera.sh" ] || fail "Working isolated camera runtime not found: $ISP"
 [ ! -L "$DASH/model" ] || fail 'The model directory is a symlink; refusing to change shared files.'
 [ ! -e "$DASH/model" ] || [ -d "$DASH/model" ] || fail 'The model directory is not a directory.'
 for key in $FILES; do
  path=$(target_path "$key")
  [ ! -L "$path" ] || fail "Refusing symbolic-link target: $path"
  [ ! -e "$path" ] || [ -f "$path" ] || fail "Update target is not a regular file: $path"
 done
}
check_payload() {
 (cd "$BASE" && sha256sum -c SHA256SUMS) || fail 'Package checksum verification failed.'
 for key in $FILES; do
  path=$(payload_path "$key")
  [ -s "$path" ] && [ ! -L "$path" ] || fail "Missing/invalid packaged file: $path"
 done
 [ "$(hash_file "$BASE/models/yolo11s-int8-rk3588.rknn")" = "$V11_SHA" ] || fail 'The packaged model is not the tested YOLO11s INT8 model.'
 current_launcher=$(hash_file "$DASH/run-perception.sh")
 new_launcher=$(hash_file "$BASE/payload/run-perception.sh")
 case "$current_launcher" in "$BASE_LAUNCHER_SHA"|"$V8_LAUNCHER_SHA"|"$new_launcher") ;; *) fail 'The existing run-perception.sh has unknown custom changes. It was preserved. Send this file for a targeted merge.' ;; esac
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
   [ "$i" -le 15 ] || fail "ISP $pid has not stopped. Close that camera session and retry."
   sleep 1
  done
 done
 if [ -n "$pids" ]; then
  i=0
  while [ -f /tmp/fpga-isp.lock/pid ]; do
   launcher=$(cat /tmp/fpga-isp.lock/pid 2>/dev/null || true)
   case "$launcher" in ''|*[!0-9]*) break ;; esac
   kill -0 "$launcher" 2>/dev/null || break
   i=$((i+1))
   [ "$i" -le 10 ] || fail 'ISP launcher has not exited. Close its terminal session and retry.'
   sleep 1
  done
 fi
}
save_backup() {
 BACKUP=$(mktemp -d "$DASH/v11-30-backup.XXXXXX")
 chmod 700 "$BACKUP"
 mkdir "$BACKUP/files"
 printf '%s\n' "$DASH" > "$BACKUP/dashboard_path"
 printf '%s\n' "$ISP" > "$BACKUP/isp_path"
 printf '%s\n' "$ROOT_YOLO" > "$BACKUP/root_entry_path"
 if [ -d "$DASH/model" ]; then printf 'present\n'; else printf 'missing\n'; fi > "$BACKUP/model_directory"
 : > "$BACKUP/MISSING"
 for key in $FILES; do
  path=$(target_path "$key")
  if [ -f "$path" ]; then cp -p "$path" "$BACKUP/files/$key"
  else printf '%s\n' "$key" >> "$BACKUP/MISSING"; fi
 done
 (cd "$BACKUP" && sha256sum dashboard_path isp_path root_entry_path model_directory MISSING files/* > SHA256SUMS)
}
validate_backup() {
 RESTORE=$(CDPATH= cd -- "$1" && pwd -P) || fail 'Backup directory not found.'
 case "$RESTORE" in "$DASH"/v11-30-backup.*) ;; *) fail 'Backup must belong to this dashboard installation.' ;; esac
 [ "$(cat "$RESTORE/dashboard_path")" = "$DASH" ] && [ "$(cat "$RESTORE/isp_path")" = "$ISP" ] && [ "$(cat "$RESTORE/root_entry_path")" = "$ROOT_YOLO" ] || fail 'Backup targets do not match DASHBOARD_DIR, ISP_DIR and ROOT_YOLO_SCRIPT.'
 (cd "$RESTORE" && sha256sum -c SHA256SUMS) || fail 'Backup checksum verification failed.'
 for key in $FILES; do
  if [ ! -f "$RESTORE/files/$key" ]; then grep -qx "$key" "$RESTORE/MISSING" || fail "Backup entry missing: $key"; fi
 done
}
restore_files() {
 source_backup=$1
 restore_rc=0
 if [ "$(cat "$source_backup/model_directory")" = present ] || [ -f "$source_backup/files/model" ] || [ -f "$source_backup/files/labels" ]; then
  mkdir -p "$DASH/model" || return 1
 fi
 for restore_key in $FILES; do
  restore_target=$(target_path "$restore_key")
  if [ -f "$source_backup/files/$restore_key" ]; then
   restore_stage=$(mktemp "${restore_target%/*}/.v11-restore.XXXXXX") || { restore_rc=1; continue; }
   if cp -p "$source_backup/files/$restore_key" "$restore_stage" && mv -f "$restore_stage" "$restore_target"; then :
   else rm -f "$restore_stage"; restore_rc=1; fi
  else rm -f "$restore_target" || restore_rc=1; fi
 done
 if [ "$(cat "$source_backup/model_directory")" = missing ]; then rmdir "$DASH/model" 2>/dev/null || true; fi
 return "$restore_rc"
}
begin_transaction() {
 STAGE=$(mktemp -d "$DASH/.v11-30-stage.XXXXXX")
 ISP_STAGE=$(mktemp "$ISP/.isp-probe-v11-30.XXXXXX")
 ROOT_STAGE=$(mktemp "${ROOT_YOLO%/*}/.run-yolo-v11-30.XXXXXX")
 committed=0
 changed=0
 trap cleanup EXIT
 trap 'exit 1' HUP INT TERM
}
cleanup() {
 rc=$?
 trap - EXIT HUP INT TERM
 if [ "$committed" = 0 ] && [ "$changed" = 1 ]; then
  if restore_files "$BACKUP"; then printf '%s\n' 'Update interrupted; all previous files were restored.' >&2
  else printf 'Automatic restore failed. Keep backup %s and do not restart yet.\n' "$BACKUP" >&2; fi
  [ "$rc" != 0 ] || rc=1
 fi
 rm -rf "$STAGE"
 rm -f "$ISP_STAGE"
 rm -f "$ROOT_STAGE"
 exit "$rc"
}
quote_shell() {
 printf "'"
 printf '%s' "$1" | sed "s/'/'\\\\''/g"
 printf "'"
}
make_root_wrapper() {
 {
  printf '#!/bin/sh\n# Installed dashboard entry; all command-line arguments are retained.\nset -eu\nDEFAULT_DASH='
  quote_shell "$DASH"
  printf '\nDASHBOARD_DIR=${DASHBOARD_DIR:-$DEFAULT_DASH}\nexec sh "$DASHBOARD_DIR/run-dashboard.sh" "$@"\n'
 } > "$1"
 chmod 755 "$1"
}
