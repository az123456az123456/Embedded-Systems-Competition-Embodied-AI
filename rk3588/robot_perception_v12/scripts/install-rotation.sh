#!/bin/sh
# Run from the rotation release; preserve the board's working launcher.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
DASH=${DASHBOARD_DIR:-/root/perception-dashboard}
case $(uname -m) in aarch64|arm64) ;; *) echo 'Run this installer on the ARM64 RK3588.' >&2;exit 2 ;; esac
cd "$BASE"
sha256sum -c SHA256SUMS
[ -f "$DASH/perception-dashboard.aarch64" ] && [ -f "$DASH/run-perception.sh" ] || {
    echo "Existing dashboard missing: $DASH; set DASHBOARD_DIR." >&2;exit 2;
}
for directory in /proc/[0-9]*; do
    executable=$(readlink -f "$directory/exe" 2>/dev/null || true)
    case "$executable" in "$DASH/perception-dashboard.aarch64"|"$DASH/build/perception-dashboard.aarch64"|"$DASH/perception-dashboard"|"$DASH/build/perception-dashboard")
        echo 'Close the YOLO/IMU window before updating.' >&2;exit 2 ;;
    esac
done
BACKUP=$(mktemp -d "$DASH/rotation-backup.XXXXXX")
for name in perception-dashboard.aarch64 run-normal.sh run-rotate90.sh; do
    if [ -e "$DASH/$name" ]; then cp -p "$DASH/$name" "$BACKUP/$name"; fi
    cp "$BASE/$name" "$DASH/$name.rotation-new"
    chmod 755 "$DASH/$name.rotation-new"
done
for name in perception-dashboard.aarch64 run-normal.sh run-rotate90.sh; do
    mv -f "$DASH/$name.rotation-new" "$DASH/$name"
done
printf 'Installed. Backup: %s\n' "$BACKUP"
printf 'Normal: cd "%s" && ./run-normal.sh\n' "$DASH"
printf 'Rotate 90: cd "%s" && ./run-rotate90.sh\n' "$DASH"
