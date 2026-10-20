#!/bin/sh
# Install the IMU calibration fix, preserving the board's working launcher.
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
BACKUP=$(mktemp -d "$DASH/imu-bias-backup.XXXXXX")
for name in perception-dashboard.aarch64 run-normal.sh run-rotate90.sh; do
    if [ -e "$DASH/$name" ]; then cp -p "$DASH/$name" "$BACKUP/$name"; fi
    cp "$BASE/$name" "$DASH/$name.imu-new"
    chmod 755 "$DASH/$name.imu-new"
done
for name in perception-dashboard.aarch64 run-normal.sh run-rotate90.sh; do
    mv -f "$DASH/$name.imu-new" "$DASH/$name"
done
printf 'Installed IMU bias fix v7. Backup: %s\n' "$BACKUP"
printf 'Start: cd "%s" && ./run-perception.sh\n' "$DASH"
printf '%s\n' 'Keep still until calibration completes, then press heading zero.'
printf '%s\n' 'Optional orientation: ./run-normal.sh or ./run-rotate90.sh'
