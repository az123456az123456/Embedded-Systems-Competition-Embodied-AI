#!/bin/sh
# Binary-only update; keep the board's working launcher, model, ISP and libraries.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
DASH=${DASHBOARD_DIR:-/root/perception-dashboard}
case $(uname -m) in aarch64|arm64) ;; *) echo 'Run this installer on the ARM64 RK3588.' >&2;exit 2 ;; esac
cd "$BASE"
sha256sum -c SHA256SUMS
[ -d "$DASH" ] || { echo "Dashboard directory not found: $DASH" >&2;exit 2; }
DASH=$(CDPATH= cd -- "$DASH" && pwd -P)
[ -f "$DASH/perception-dashboard.aarch64" ] && [ -f "$DASH/run-perception.sh" ] || {
    echo "Existing dashboard missing: $DASH; set DASHBOARD_DIR." >&2;exit 2;
}
for directory in /proc/[0-9]*; do
    executable=$(readlink -f "$directory/exe" 2>/dev/null || true)
    case "$executable" in "$DASH/perception-dashboard.aarch64"|"$DASH/build/perception-dashboard.aarch64"|"$DASH/perception-dashboard"|"$DASH/build/perception-dashboard")
        echo 'Close the YOLO/IMU window before updating.' >&2;exit 2 ;;
    esac
done
BACKUP=$(mktemp -d "$DASH/preview-v8-backup.XXXXXX")
cp -p "$DASH/perception-dashboard.aarch64" "$BACKUP/perception-dashboard.aarch64"
(cd "$BACKUP" && sha256sum perception-dashboard.aarch64 > SHA256SUMS)
TEMP=$(mktemp "$DASH/.preview-v8-binary.XXXXXX")
trap 'rm -f "$TEMP"' EXIT HUP INT TERM
cp "$BASE/perception-dashboard.aarch64" "$TEMP"
chmod 755 "$TEMP"
# Rename within the target filesystem so readers never see a partial executable.
mv -f "$TEMP" "$DASH/perception-dashboard.aarch64"
printf '%s\n' "$BACKUP" > "$DASH/.preview-v8-last-backup"
printf 'Installed v8 independent preview + stage profiling. Backup: %s\n' "$BACKUP"
printf 'Start: cd "%s" && ./run-perception.sh\n' "$DASH"
printf '%s\n' 'Working launcher, orientation scripts, model, libraries, ISP and FPGA are preserved.'
printf 'Rollback after closing the window: sh "%s/rollback.sh"\n' "$BASE"
