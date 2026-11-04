#!/bin/sh
set -eu
. "$(dirname -- "$0")/fps-update-common.sh"
[ "$#" = 1 ] || fail "Usage: $0 /opt/fpga-camera-isp-1080/fps20-backup.XXXXXX"
check_board
backup=$(CDPATH= cd -- "$1" && pwd -P) || fail 'Backup directory not found.'
case "$backup" in "$ISP"/fps20-backup.*) ;; *) fail 'Backup must belong to this isolated ISP runtime.' ;; esac
[ -f "$backup/target_path" ] && [ "$(cat "$backup/target_path")" = "$ISP" ] || fail 'Backup target does not match ISP_DIR.'
(cd "$backup" && sha256sum -c SHA256SUMS) || fail 'Backup checksum verification failed.'
check_consumers
stage=$(mktemp "$ISP/.isp-probe-rollback.XXXXXX")
trap 'rm -f "$stage"' EXIT
cp -p "$backup/isp-probe" "$stage"
cmp -s "$backup/isp-probe" "$stage" || fail 'Staged rollback does not match the backup.'
stop_receiver
mv -f "$stage" "$ISP/isp-probe"
trap - EXIT
printf '%s\n' 'Original receiver restored. Program the original 15 FPS FPGA image, then restart the dashboard.'
