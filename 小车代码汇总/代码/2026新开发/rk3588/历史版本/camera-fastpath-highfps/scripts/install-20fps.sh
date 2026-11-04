#!/bin/sh
# Receiver-only update: no model, desktop, launcher, library or kernel changes.
set -eu
. "$(dirname -- "$0")/fps-update-common.sh"
[ "$#" = 0 ] || fail "Usage: $0"
check_board
(cd "$BASE" && sha256sum -c SHA256SUMS) || fail 'Package checksum verification failed.'
[ -s "$BASE/isp-probe" ] || fail 'Missing packaged isp-probe.'
check_consumers
backup=$(mktemp -d "$ISP/fps20-backup.XXXXXX")
chmod 700 "$backup"
cp -p "$ISP/isp-probe" "$backup/isp-probe"
printf '%s\n' "$ISP" > "$backup/target_path"
(cd "$backup" && sha256sum isp-probe > SHA256SUMS)
stage=$(mktemp "$ISP/.isp-probe-20fps.XXXXXX")
trap 'rm -f "$stage"' EXIT
cp "$BASE/isp-probe" "$stage"
chmod 755 "$stage"
cmp -s "$BASE/isp-probe" "$stage" || fail 'Staged binary does not match the package.'
stop_receiver
mv -f "$stage" "$ISP/isp-probe"
trap - EXIT
printf '20 FPS receiver installed. Backup: %s\n' "$backup"
printf '%s\n' 'Program the matching 20 FPS FPGA image, then start your existing dashboard.'
printf '%s\n' 'Verify actual FASTPATH rx_fps/output_fps; advertised V4L2 rate alone is not a measurement.'
