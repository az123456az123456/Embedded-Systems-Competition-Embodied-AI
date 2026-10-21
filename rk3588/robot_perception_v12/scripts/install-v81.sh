#!/bin/sh
# v8.1: confirmed IMU calibration gate fix + one orientation-selecting entry point.
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
BACKUP=$(mktemp -d "$DASH/v81-backup.XXXXXX")
: > "$BACKUP/MISSING"
for name in perception-dashboard.aarch64 run-dashboard.sh; do
 if [ -e "$DASH/$name" ]; then cp -p "$DASH/$name" "$BACKUP/$name"
 else printf '%s\n' "$name" >> "$BACKUP/MISSING"; fi
done
(cd "$BACKUP" && sha256sum MISSING perception-dashboard.aarch64 > SHA256SUMS &&
 for name in run-dashboard.sh; do
  if [ -f "$name" ]; then sha256sum "$name" >> SHA256SUMS; fi
 done)
STAGE=$(mktemp -d "$DASH/.v81-stage.XXXXXX")
trap 'rm -rf "$STAGE"' EXIT
trap 'exit 1' HUP INT TERM
for name in perception-dashboard.aarch64 run-dashboard.sh; do
 cp "$BASE/$name" "$STAGE/$name"
 chmod 755 "$STAGE/$name"
done
printf '%s\n' "$BACKUP" > "$DASH/.v81-last-backup"
for name in perception-dashboard.aarch64 run-dashboard.sh; do
 mv -f "$STAGE/$name" "$DASH/$name"
done
printf 'Installed v8.1. Backup: %s\n' "$BACKUP"
printf 'Normal fullscreen: cd "%s" && ./run-dashboard.sh -0\n' "$DASH"
printf 'Clockwise 90 degrees: cd "%s" && ./run-dashboard.sh -90\n' "$DASH"
printf '%s\n' 'Keep the car still until IMU[v8.1] says cal=done progress=100% bias_valid=1, then zero heading.'
printf '%s\n' 'Original launcher, model, libraries, ISP and FPGA were preserved.'
printf 'Rollback after closing the window: sh "%s/rollback.sh"\n' "$BASE"
