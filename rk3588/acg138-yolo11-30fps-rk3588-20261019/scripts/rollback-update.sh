#!/bin/sh
set -eu
. "$(dirname -- "$0")/update-common.sh"
[ "$#" = 1 ] || fail "Usage: $0 /root/perception-dashboard/v11-30-backup.XXXXXX"
check_board
validate_backup "$1"
check_consumers
# Keep the current files as a recovery point if an explicit rollback fails.
save_backup
begin_transaction
stop_receiver
check_consumers
[ -z "$pids" ] || fail 'The ISP restarted during rollback. Close other camera sessions and retry; files were not replaced.'
changed=1
mkdir -p "$DASH/model"
restore_files "$RESTORE" || fail 'Rollback failed.'
committed=1
printf 'Previous dashboard/model/receiver files restored from: %s\n' "$RESTORE"
printf 'The displaced update was saved at: %s\n' "$BACKUP"
printf '%s\n' 'Also program the FPGA image matching the restored receiver (normally the previous 20 FPS image), then restart the dashboard.'
