#!/bin/sh
set -eu
DASH=${DASHBOARD_DIR:-/root/perception-dashboard}
DASH=$(CDPATH= cd -- "$DASH" && pwd -P)
BACKUP=${1:-}
if [ -z "$BACKUP" ]; then
 [ -r "$DASH/.v81-last-backup" ] || { echo 'Pass the backup directory printed by install.sh.' >&2;exit 2; }
 BACKUP=$(cat "$DASH/.v81-last-backup")
fi
BACKUP=$(CDPATH= cd -- "$BACKUP" && pwd -P)
[ -f "$BACKUP/perception-dashboard.aarch64" ] && [ -f "$BACKUP/MISSING" ] && [ -f "$BACKUP/SHA256SUMS" ] || { echo 'Invalid backup.' >&2;exit 2; }
for directory in /proc/[0-9]*; do
 executable=$(readlink -f "$directory/exe" 2>/dev/null || true)
 case "$executable" in "$DASH/perception-dashboard.aarch64"|"$DASH/build/perception-dashboard.aarch64"|"$DASH/perception-dashboard"|"$DASH/build/perception-dashboard")
  echo 'Close the YOLO/IMU window before restoring.' >&2;exit 2 ;;
 esac
done
(cd "$BACKUP" && sha256sum -c SHA256SUMS)
STAGE=$(mktemp -d "$DASH/.v81-restore.XXXXXX")
trap 'rm -rf "$STAGE"' EXIT
trap 'exit 1' HUP INT TERM
for name in perception-dashboard.aarch64 run-dashboard.sh; do
 if [ -f "$BACKUP/$name" ]; then cp -p "$BACKUP/$name" "$STAGE/$name"
 elif ! grep -Fx "$name" "$BACKUP/MISSING" >/dev/null; then echo "Incomplete backup: $name" >&2;exit 2; fi
done
for name in perception-dashboard.aarch64 run-dashboard.sh; do
 if [ -f "$STAGE/$name" ]; then mv -f "$STAGE/$name" "$DASH/$name"
 else rm -f "$DASH/$name"; fi
done
printf 'Restored binary and previous launch script: %s\n' "$BACKUP"
