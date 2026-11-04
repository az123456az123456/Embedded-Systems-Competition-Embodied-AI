#!/bin/sh
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
. "$HERE/common.sh"
CHECK_ONLY=0
case "${1:-}" in '') ;; --check-only) CHECK_ONLY=1 ;; *) die 'Usage: sh install.sh [--check-only]' ;; esac
[ "$#" -le 1 ] || die 'Usage: sh install.sh [--check-only]'
setup_paths
verify_package
assert_stopped
CURRENT_SHA=$(hash "$TARGET")
if [ "$CURRENT_SHA" = "$NEW_SHA" ]; then
    echo 'This HDMI display fix is already installed. No files or backups changed.'
    exit 0
fi
[ "$CURRENT_SHA" = "$BASELINE_SHA" ] || die 'Unrecognized installed dashboard. This update accepts only the v19.1 baseline; no files changed.'
if [ "$CHECK_ONLY" = 1 ]; then
    echo 'Preflight passed. No files changed.'
    exit 0
fi
take_lock
make_stage
cp "$HERE/perception-dashboard.aarch64" "$STAGE"
chmod 755 "$STAGE"
[ "$(hash "$STAGE")" = "$NEW_SHA" ] || die 'Staged payload checksum failed; installed program is unchanged.'
# Recheck immediately before backup / replacement to catch another updater or
# a dashboard started after the initial preflight.
assert_stopped
[ "$(hash "$TARGET")" = "$BASELINE_SHA" ] || die 'Installed program changed during preflight; no replacement attempted.'
BACKUP=$(mktemp -d "$BASE/robot-display-v19.2-backup.XXXXXX") || die 'Cannot create backup.'
cp -p "$TARGET" "$BACKUP/perception-dashboard.aarch64"
[ "$(hash "$BACKUP/perception-dashboard.aarch64")" = "$BASELINE_SHA" ] || die 'Backup checksum failed; installed program is unchanged.'
{
    printf 'format=robot-display-hdmi-v19.2\n'
    printf 'target=%s\nold_sha256=%s\nnew_sha256=%s\n' "$TARGET" "$BASELINE_SHA" "$NEW_SHA"
    printf 'created_utc=%s\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')"
} > "$BACKUP/receipt.txt"
chmod 600 "$BACKUP/receipt.txt"
REPLACE_FAILED=0
mv -f "$STAGE" "$TARGET" || REPLACE_FAILED=1
if [ "$REPLACE_FAILED" = 1 ] && [ "$(hash "$TARGET")" = "$BASELINE_SHA" ]; then
    die "Atomic replacement failed; the installed program is unchanged. Verified backup: $BACKUP"
fi
if [ "$REPLACE_FAILED" = 1 ] || [ "$(hash "$TARGET")" != "$NEW_SHA" ]; then
    [ -z "$STAGE" ] || rm -f -- "$STAGE"
    make_stage
    cp -p "$BACKUP/perception-dashboard.aarch64" "$STAGE"
    [ "$(hash "$STAGE")" = "$BASELINE_SHA" ] || die "Verification failed and automatic restoration could not be staged. Backup: $BACKUP"
    mv -f "$STAGE" "$TARGET" || die "Verification failed and automatic restoration failed. Backup: $BACKUP"
    STAGE=
    [ "$(hash "$TARGET")" = "$BASELINE_SHA" ] || die "Automatic restoration checksum failed. Backup: $BACKUP"
    die 'Installed payload verification failed; the previous dashboard was automatically restored.'
fi
STAGE=
printf 'Installed HDMI display fix.\nBackup: %s\n' "$BACKUP"
printf 'Rollback: sh %s/rollback.sh %s\n' "$HERE" "$BACKUP"
echo 'Start: sh /root/perception-dashboard/run-dashboard.sh -0'
