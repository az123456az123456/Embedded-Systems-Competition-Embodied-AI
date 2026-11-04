#!/bin/sh
# Update the existing unified executable; keep model, camera and private state.
set -eu
umask 077
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
APP_INPUT=${ROBOT_APP_DIR:-${ROBOT_ROOT:-/root}/robot-perception}
case "$APP_INPUT" in /*) ;; *) echo 'ROBOT_APP_DIR must be absolute' >&2;exit 2;; esac
[ -d "$APP_INPUT" ] || { echo "Existing unified application not found: $APP_INPUT" >&2;exit 2; }
APP=$(CDPATH= cd -- "$APP_INPUT" && pwd -P)
TARGET=$APP/perception-dashboard.aarch64
[ -f "$TARGET" ] && [ ! -L "$TARGET" ] || { echo 'Existing executable must be a regular file' >&2;exit 2; }
[ -f "$HERE/payload/perception-dashboard.aarch64" ] && [ -f "$HERE/SHA256SUMS" ] || {
    echo 'Unpack the complete UI update first' >&2;exit 2;
}
(cd "$HERE" && sha256sum -c SHA256SUMS) || { echo 'Checksum verification failed; nothing installed' >&2;exit 1; }
LOCK=$APP/.ui-update-lock
mkdir "$LOCK" 2>/dev/null || { echo 'Another UI update is active' >&2;exit 1; }
STAGE=
cleanup() {
    [ -z "$STAGE" ] || rm -f "$STAGE"
    rmdir "$LOCK" 2>/dev/null || :
}
trap cleanup 0
trap 'exit 130' HUP INT TERM
BACKUP=$(mktemp -d "$APP/.ui-v12.4-backup.XXXXXX")
cp -p "$TARGET" "$BACKUP/previous.aarch64"
printf '%s\n' "$TARGET" > "$BACKUP/target"
(cd "$BACKUP" && sha256sum previous.aarch64 > previous.sha256)
printf '%s\n' ROBOT_UI_V124 > "$BACKUP/format"
STAGE=$(mktemp "$APP/.ui-new.XXXXXX")
cp "$HERE/payload/perception-dashboard.aarch64" "$STAGE"
chmod 755 "$STAGE"
mv -f "$STAGE" "$TARGET"
STAGE=
printf 'UI updated: %s\nBackup: %s\n' "$TARGET" "$BACKUP"
printf '%s\n' 'Restart your usual run-dashboard.sh -0 or -90 after quitting the previous window.'
