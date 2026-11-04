#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
. "$HERE/common.sh"
CHECK=0
case "${1:-}" in '') ;; --check-only) CHECK=1 ;; *) die 'Usage: sh install.sh [--check-only]' ;; esac
[ "$#" -le 1 ] || die 'Usage: sh install.sh [--check-only]'
setup_paths
verify_package
assert_stopped
read_current
assert_launchers
assert_pair
if new_set; then echo 'The complete v32-live-sonar-20261028 dashboard/navigation pair is already installed; files/config/backups unchanged.'; exit 0; fi
old_set
if [ "$CHECK" = 1 ]; then echo 'Complete baseline and package checks passed. No installed files changed.'; exit 0; fi
snapshot
take_lock
make_stages
cp -p "$DASH" "$STAGE_DASH"; cat "$DASH_SOURCE" > "$STAGE_DASH"
cp -p "$PAIR" "$STAGE_PAIR"; printf '%s\n' "$NEW_DASH" > "$STAGE_PAIR"
DEST_DASH=$NEW_DASH; DEST_PAIR=$NEW_PAIR; DEST_NAV=$NEW_NAV
cp -p "$NAV" "$STAGE_NAV"; cat "$NAV_SOURCE" > "$STAGE_NAV"
[ "$(hash "$STAGE_DASH")" = "$DEST_DASH" ] && [ "$(hash "$STAGE_PAIR")" = "$DEST_PAIR" ] || die 'Staged pair checksum failed.'
[ "$(hash "$STAGE_NAV")" = "$DEST_NAV" ] || die 'Staged navigation checksum failed.'
assert_stopped
assert_current
BACKUP=$(mktemp -d "$BASE/robot-phone-v32-live-sonar-20261028-backup.XXXXXX") || die 'Cannot create backup.'
copy_current "$BACKUP"
verify_copy "$BACKUP" "$EXPECT_DASH" "$EXPECT_PAIR" "$EXPECT_NAV" || die 'Backup checksum failed.'
{
    printf 'format=robot-phone-v32-live-sonar-20261028\nmode=%s\ndashboard_target=%s\nnavigation_target=%s\npair_target=%s\n' "$MODE" "$DASH" "$NAV" "$PAIR"
    printf 'old_dashboard=%s\nold_navigation=%s\nold_pair=%s\nnew_dashboard=%s\nnew_navigation=%s\nnew_pair=%s\n' "$EXPECT_DASH" "$EXPECT_NAV" "$EXPECT_PAIR" "$DEST_DASH" "$DEST_NAV" "$DEST_PAIR"
    printf 'dashboard_launcher_sha256=%s\nnavigation_launcher_sha256=%s\nconfig_sha256_at_install=%s\nconfig_policy=never_replaced_allow_later_edits_on_rollback\ncreated_utc=%s\n' "$EXPECT_ENTRY" "$EXPECT_ORCH" "$EXPECT_CONFIG" "$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
} > "$BACKUP/receipt.txt"
chmod 600 "$BACKUP/receipt.txt"
assert_stopped
assert_current
RECOVERY=$BACKUP; RECOVER_DASH=$EXPECT_DASH; RECOVER_PAIR=$EXPECT_PAIR; RECOVER_NAV=$EXPECT_NAV
IN_TX=1
replace_stages
IN_TX=0
printf 'Installed complete v32-live-sonar-20261028 pair. Config/tokens/launchers/models untouched.\nBackup: %s\n' "$BACKUP"
printf 'Rollback: sh %s/rollback.sh %s\n' "$HERE" "$BACKUP"
echo 'No programs or motors started. Keep the matching v29/v29.1 MOT4 FPGA bitstream; this update does not program FPGA. Use the original dashboard command.'
