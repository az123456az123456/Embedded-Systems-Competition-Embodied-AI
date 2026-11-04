#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
. "$HERE/common.sh"
[ "$#" = 1 ] || die 'Usage: sh rollback.sh /root/robot-display-v19.2-backup.XXXXXX'
setup_paths
verify_package
assert_stopped
BACKUP=$1
[ -d "$BACKUP" ] && [ ! -L "$BACKUP" ] || die 'Backup must be a real directory.'
BACKUP=$(CDPATH= cd -- "$BACKUP" && pwd -P)
case "$BACKUP" in "$BASE"/robot-display-v19.2-backup.*) ;; *) die 'Backup is outside this update backup location.' ;; esac
[ -f "$BACKUP/receipt.txt" ] && [ ! -L "$BACKUP/receipt.txt" ] || die 'Missing regular backup receipt.'
[ "$(metadata_value format "$BACKUP")" = robot-display-hdmi-v19.2 ] || die 'Not a display fix backup.'
[ "$(metadata_value target "$BACKUP")" = "$TARGET" ] || die 'Backup belongs to another application path.'
[ "$(metadata_value old_sha256 "$BACKUP")" = "$BASELINE_SHA" ] || die 'Unexpected backup baseline.'
[ "$(metadata_value new_sha256 "$BACKUP")" = "$NEW_SHA" ] || die 'Backup belongs to a different update.'
[ -f "$BACKUP/perception-dashboard.aarch64" ] && [ ! -L "$BACKUP/perception-dashboard.aarch64" ] || die 'Missing regular backup binary.'
[ "$(hash "$BACKUP/perception-dashboard.aarch64")" = "$BASELINE_SHA" ] || die 'Backup binary checksum failed.'
CURRENT_SHA=$(hash "$TARGET")
if [ "$CURRENT_SHA" = "$BASELINE_SHA" ]; then
    echo 'The baseline dashboard is already restored. No files changed.'
    exit 0
fi
[ "$CURRENT_SHA" = "$NEW_SHA" ] || die 'The dashboard was updated again after this fix; rollback refuses to overwrite it.'
take_lock
make_stage
cp -p "$BACKUP/perception-dashboard.aarch64" "$STAGE"
[ "$(hash "$STAGE")" = "$BASELINE_SHA" ] || die 'Rollback staging checksum failed.'
assert_stopped
[ "$(hash "$TARGET")" = "$NEW_SHA" ] || die 'Installed program changed during rollback; no replacement attempted.'
# Keep the current payload independently until the restored file verifies.
RECOVERY=$(mktemp "$BACKUP/.current.XXXXXX") || die 'Cannot create rollback recovery file.'
cp -p "$TARGET" "$RECOVERY"
[ "$(hash "$RECOVERY")" = "$NEW_SHA" ] || die 'Cannot verify rollback recovery file.'
REPLACE_FAILED=0
mv -f "$STAGE" "$TARGET" || REPLACE_FAILED=1
if [ "$REPLACE_FAILED" = 1 ] && [ "$(hash "$TARGET")" = "$NEW_SHA" ]; then
    rm -f "$RECOVERY"
    die 'Atomic rollback failed; the display fix remains installed.'
fi
if [ "$REPLACE_FAILED" = 1 ] || [ "$(hash "$TARGET")" != "$BASELINE_SHA" ]; then
    [ -z "$STAGE" ] || rm -f -- "$STAGE"
    make_stage
    cp -p "$RECOVERY" "$STAGE"
    [ "$(hash "$STAGE")" = "$NEW_SHA" ] || die "Cannot stage rollback recovery: $RECOVERY"
    mv -f "$STAGE" "$TARGET" || die "Rollback recovery failed: $RECOVERY"
    STAGE=
    [ "$(hash "$TARGET")" = "$NEW_SHA" ] || die "Rollback recovery checksum failed: $RECOVERY"
    rm -f "$RECOVERY"
    die 'Rollback verification failed; the display fix was automatically restored.'
fi
STAGE=
rm -f "$RECOVERY"
echo 'The previous v19.1 dashboard was restored. The verified backup is retained.'
