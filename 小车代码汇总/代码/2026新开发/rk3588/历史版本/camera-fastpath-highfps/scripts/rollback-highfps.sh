#!/bin/sh
set -eu
. "$(dirname -- "$0")/highfps-common.sh"
[ "$#" = 1 ] || fail "Usage: $0 /opt/fpga-camera-isp-1080/camera-highfps-backup.XXXXXX"
check_board
backup=$1
verify_backup
check_consumers
stop_receiver
restore_backup || fail "Unable to restore all originals; keep programs closed. Backup: $backup"
printf 'Original receiver, launcher settings and FPS metadata restored from: %s\n' "$backup"
printf '%s\n' 'Restore the matching original FPGA image before restarting the dashboard.'
