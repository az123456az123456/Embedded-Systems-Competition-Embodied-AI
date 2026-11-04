#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
. "$HERE/common.sh"
[ "$#" = 1 ] || die 'Usage: sh rollback.sh /root/robot-phone-v32-live-sonar-20261028-backup.XXXXXX'
setup_paths
verify_package
assert_stopped
BACKUP=$1
[ -d "$BACKUP" ] && [ ! -L "$BACKUP" ] || die 'Backup must be a real directory.'
BACKUP=$(CDPATH= cd -- "$BACKUP" && pwd -P)
case "$BACKUP" in "$BASE"/robot-phone-v32-live-sonar-20261028-backup.*) ;; *) die 'Backup is outside the v32-live-sonar-20261028 backup location.' ;; esac
for file in receipt.txt perception-dashboard.aarch64 robot-navigation.aarch64 dashboard.sha256; do regular "$BACKUP/$file"; done
[ "$(metadata format "$BACKUP")" = robot-phone-v32-live-sonar-20261028 ] && [ "$(metadata mode "$BACKUP")" = "$MODE" ] || die 'Backup format/mode mismatch.'
[ "$(metadata dashboard_target "$BACKUP")" = "$DASH" ] && [ "$(metadata navigation_target "$BACKUP")" = "$NAV" ] && [ "$(metadata pair_target "$BACKUP")" = "$PAIR" ] || die 'Backup target paths mismatch.'
OLD_DASH=$(metadata old_dashboard "$BACKUP"); OLD_NAV=$(metadata old_navigation "$BACKUP"); OLD_PAIR=$(metadata old_pair "$BACKUP")
known dashboard "$OLD_DASH" && known navigation "$OLD_NAV" && known_pair "$OLD_DASH" "$OLD_NAV" && valid_hash "$OLD_PAIR" || die 'Unrecognized old baseline pair.'
EXPECTED_NEW_NAV=$NEW_NAV
[ "$(metadata new_dashboard "$BACKUP")" = "$NEW_DASH" ] && [ "$(metadata new_navigation "$BACKUP")" = "$EXPECTED_NEW_NAV" ] && [ "$(metadata new_pair "$BACKUP")" = "$NEW_PAIR" ] || die 'Backup belongs to a different update.'
verify_copy "$BACKUP" "$OLD_DASH" "$OLD_PAIR" "$OLD_NAV" || die 'Backup checksum mismatch.'
old_pair_expected=$(printf '%s\n' "$OLD_DASH" | sha256sum | awk '{print $1}')
[ "$OLD_PAIR" = "$old_pair_expected" ] || die 'Old backup pair is inconsistent.'
[ "$(metadata config_policy "$BACKUP")" = never_replaced_allow_later_edits_on_rollback ] || die 'Unexpected configuration policy.'
read_current
assert_launchers
[ "$(metadata dashboard_launcher_sha256 "$BACKUP")" = "$CUR_ENTRY" ] && [ "$(metadata navigation_launcher_sha256 "$BACKUP")" = "$CUR_ORCH" ] || die 'Launchers changed after installation.'
if [ "$CUR_DASH" = "$OLD_DASH" ] && [ "$CUR_NAV" = "$OLD_NAV" ] && [ "$CUR_PAIR" = "$OLD_PAIR" ]; then echo 'Original complete pair already restored; current config retained.'; exit 0; fi
[ "$CUR_DASH" = "$NEW_DASH" ] && [ "$CUR_NAV" = "$EXPECTED_NEW_NAV" ] && [ "$CUR_PAIR" = "$NEW_PAIR" ] || die 'Runtime changed after v32-live-sonar-20261028; refusing to overwrite newer or mixed files.'
snapshot
take_lock
make_stages
cp -p "$BACKUP/perception-dashboard.aarch64" "$STAGE_DASH"; cp -p "$BACKUP/dashboard.sha256" "$STAGE_PAIR"
cp -p "$BACKUP/robot-navigation.aarch64" "$STAGE_NAV"
assert_stopped
assert_current
RECOVERY=$(mktemp -d "$BACKUP/.rollback-recovery.XXXXXX") || die 'Cannot create rollback recovery.'
copy_current "$RECOVERY"
verify_copy "$RECOVERY" "$EXPECT_DASH" "$EXPECT_PAIR" "$EXPECT_NAV" || die 'Rollback recovery checksum failed.'
RECOVER_DASH=$EXPECT_DASH; RECOVER_PAIR=$EXPECT_PAIR; RECOVER_NAV=$EXPECT_NAV
DEST_DASH=$OLD_DASH; DEST_PAIR=$OLD_PAIR; DEST_NAV=$OLD_NAV
assert_stopped
assert_current
IN_TX=1
replace_stages
IN_TX=0
rm -f "$RECOVERY/perception-dashboard.aarch64" "$RECOVERY/dashboard.sha256"
rm -f "$RECOVERY/robot-navigation.aarch64"
rmdir "$RECOVERY"
echo 'Original complete dashboard/navigation pair restored. Current config/tokens/launchers retained; no program started.'
