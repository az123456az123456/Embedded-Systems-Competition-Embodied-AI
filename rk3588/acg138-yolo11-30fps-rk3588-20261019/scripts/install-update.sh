#!/bin/sh
# Update the installed window/model and RAW receiver; keep all root entry points.
set -eu
. "$(dirname -- "$0")/update-common.sh"
[ "$#" = 0 ] || fail "Usage: $0"
check_board
check_payload
check_consumers
save_backup
begin_transaction
for key in $FILES; do
 source_path=$(payload_path "$key")
 case "$key" in receiver) stage_path=$ISP_STAGE ;; rootentry) make_root_wrapper "$ROOT_STAGE"; continue ;; *) stage_path=$STAGE/$key ;; esac
 cp "$source_path" "$stage_path"
 case "$key" in model|labels) chmod 644 "$stage_path" ;; *) chmod 755 "$stage_path" ;; esac
 cmp -s "$source_path" "$stage_path" || fail 'Staged payload does not match the package.'
done
stop_receiver
check_consumers
[ -z "$pids" ] || fail 'The ISP restarted during the update. Close other camera sessions and retry; files were not replaced.'
changed=1
mkdir -p "$DASH/model"
for key in $FILES; do
 case "$key" in receiver) stage_path=$ISP_STAGE ;; rootentry) stage_path=$ROOT_STAGE ;; *) stage_path=$STAGE/$key ;; esac
 mv -f "$stage_path" "$(target_path "$key")"
done
committed=1
printf 'YOLO11s INT8 + 30 FPS update installed. Backup: %s\n' "$BACKUP"
printf 'Default model: %s/model/yolo11s-int8-rk3588.rknn\n' "$DASH"
printf '%s\n' 'Program the matching 30 FPS FPGA image before restarting camera capture.'
printf 'Start: cd "%s" && ./run-dashboard.sh -0\n' "$DASH"
printf '%s\n' 'Use -90 for the rotated fullscreen window. Model/path overrides still work.'
printf 'The root shortcut %s now forwards all arguments to the existing run-dashboard.sh. Its previous contents are in the backup.\n' "$ROOT_YOLO"
