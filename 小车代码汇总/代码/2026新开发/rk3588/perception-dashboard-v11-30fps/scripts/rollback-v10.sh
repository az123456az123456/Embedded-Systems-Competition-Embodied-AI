#!/bin/sh
set -eu
DASH=${DASHBOARD_DIR:-/root/perception-dashboard}
DASH=$(CDPATH= cd -- "$DASH" && pwd -P)
BACKUP=${1:-$(cat "$DASH/.v10-last-backup")}
BACKUP=$(CDPATH= cd -- "$BACKUP" && pwd -P)
case "$BACKUP" in "$DASH"/v10-backup.*) ;; *) echo 'Backup does not belong to this installation.' >&2;exit 2 ;; esac
for proc in /proc/[0-9]*/exe; do
 exe=$(readlink "$proc" 2>/dev/null || true)
 case "$exe" in "$DASH/perception-dashboard.aarch64"*|"$DASH/build/perception-dashboard.aarch64"*)
  echo 'Close the YOLO/IMU window before restoring.' >&2;exit 2 ;;
 esac
done
(cd "$BACKUP" && sha256sum -c SHA256SUMS)
STAGE=$(mktemp -d "$DASH/.v10-restore.XXXXXX")
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
printf 'Restored previous binary/wrapper from %s\n' "$BACKUP"
