#!/bin/sh
set -eu
DASH=${DASHBOARD_DIR:-/root/perception-dashboard}
DASH=$(CDPATH= cd -- "$DASH" && pwd -P)
BACKUP=${1:-}
if [ -z "$BACKUP" ]; then
    [ -r "$DASH/.preview-v8-last-backup" ] || { echo 'Pass the backup directory printed by install.sh.' >&2;exit 2; }
    BACKUP=$(cat "$DASH/.preview-v8-last-backup")
fi
BACKUP=$(CDPATH= cd -- "$BACKUP" && pwd -P)
[ -f "$BACKUP/perception-dashboard.aarch64" ] && [ -f "$BACKUP/SHA256SUMS" ] || { echo 'Invalid backup.' >&2;exit 2; }
for directory in /proc/[0-9]*; do
    executable=$(readlink -f "$directory/exe" 2>/dev/null || true)
    case "$executable" in "$DASH/perception-dashboard.aarch64"|"$DASH/build/perception-dashboard.aarch64"|"$DASH/perception-dashboard"|"$DASH/build/perception-dashboard")
        echo 'Close the YOLO/IMU window before restoring.' >&2;exit 2 ;;
    esac
done
(cd "$BACKUP" && sha256sum -c SHA256SUMS)
TEMP=$(mktemp "$DASH/.preview-v8-restore.XXXXXX")
trap 'rm -f "$TEMP"' EXIT HUP INT TERM
cp -p "$BACKUP/perception-dashboard.aarch64" "$TEMP"
mv -f "$TEMP" "$DASH/perception-dashboard.aarch64"
printf 'Restored: %s\n' "$BACKUP"
printf 'Start: cd "%s" && ./run-perception.sh\n' "$DASH"
