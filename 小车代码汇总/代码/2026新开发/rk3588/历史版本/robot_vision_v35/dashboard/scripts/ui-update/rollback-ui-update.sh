#!/bin/sh
set -eu
umask 077
[ "$#" = 1 ] || { echo "Usage: sh $0 /root/robot-perception/.ui-v12.4-backup.XXXXXX" >&2;exit 2; }
BACKUP=$(CDPATH= cd -- "$1" && pwd -P)
[ -f "$BACKUP/format" ] && [ "$(cat "$BACKUP/format")" = ROBOT_UI_V124 ] || {
    echo 'Not a v12.4 UI backup' >&2;exit 2;
}
APP=$(dirname -- "$BACKUP")
TARGET=$(cat "$BACKUP/target")
[ "$TARGET" = "$APP/perception-dashboard.aarch64" ] || { echo 'Backup target does not match its application directory' >&2;exit 2; }
[ -f "$BACKUP/previous.aarch64" ] && [ ! -L "$BACKUP/previous.aarch64" ] || { echo 'Backup executable is missing' >&2;exit 2; }
[ ! -L "$TARGET" ] || { echo 'Executable target cannot be a symlink' >&2;exit 2; }
(cd "$BACKUP" && sha256sum -c previous.sha256) || { echo 'Backup checksum failed; nothing restored' >&2;exit 1; }
LOCK=$APP/.ui-update-lock
mkdir "$LOCK" 2>/dev/null || { echo 'Another UI update is active' >&2;exit 1; }
STAGE=
cleanup() {
    [ -z "$STAGE" ] || rm -f "$STAGE"
    rmdir "$LOCK" 2>/dev/null || :
}
trap cleanup 0
trap 'exit 130' HUP INT TERM
STAGE=$(mktemp "$APP/.ui-restore.XXXXXX")
cp -p "$BACKUP/previous.aarch64" "$STAGE"
mv -f "$STAGE" "$TARGET"
STAGE=
printf 'Previous UI restored: %s\n' "$TARGET"
printf '%s\n' 'Restart your usual launcher after quitting the current window.'
