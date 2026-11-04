#!/bin/sh
# Run from the released package, after closing the old dashboard.
# Updates the dedicated ISP runtime only; preserves launchers, system libraries,
# IQ, model, kernel, and the board's existing device-node configuration.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
DASH=${DASHBOARD_DIR:-/root/perception-dashboard}
ISP=${ISP_DIR:-/opt/fpga-camera-isp-1080}
MODE=${1:-all}
case "$MODE" in all|--dashboard-only) ;; *) echo "Usage: $0 [--dashboard-only]" >&2;exit 2 ;; esac
case $(uname -m) in aarch64|arm64) ;; *) echo 'Install on the ARM64 RK3588 board.' >&2;exit 2 ;; esac
cd "$BASE"
sha256sum -c SHA256SUMS
[ -f "$DASH/perception-dashboard.aarch64" ] || { echo "Dashboard not found: $DASH; set DASHBOARD_DIR." >&2;exit 2; }
for directory in /proc/[0-9]*; do
 [ -r "$directory/comm" ] || continue
 executable=$(readlink -f "$directory/exe" 2>/dev/null || true)
 case "$executable" in "$DASH/perception-dashboard.aarch64"|"$DASH/build/perception-dashboard.aarch64"|"$DASH/perception-dashboard"|"$DASH/build/perception-dashboard")
  echo 'Close the old YOLO/IMU dashboard before updating.' >&2;exit 2 ;; esac
done
pids=
if [ "$MODE" = all ]; then
 [ -f "$ISP/isp-probe" ] && [ -f "$ISP/lib/librkaiq.so" ] || { echo "Existing ISP runtime not found: $ISP; set ISP_DIR." >&2;exit 2; }
 for directory in /proc/[0-9]*; do
  [ -r "$directory/comm" ] || continue
  name=$(cat "$directory/comm" 2>/dev/null || true)
  [ "$name" = isp-probe ] || continue
  executable=$(readlink -f "$directory/exe" 2>/dev/null || true)
  [ "$executable" = "$ISP/isp-probe" ] || {
   echo "A different ISP binary is active: $executable. Stop it before updating." >&2;exit 2;
  }
  pids="$pids ${directory##*/}"
 done
fi
backup=$(mktemp -d "$DASH/fps-backup.XXXXXX")
cp -p "$DASH/perception-dashboard.aarch64" "$backup/perception-dashboard.aarch64"
if [ "$MODE" = all ]; then
 cp -p "$ISP/isp-probe" "$backup/isp-probe"
 cp -p "$ISP/lib/librkaiq.so" "$backup/librkaiq.so"
 for pid in $pids; do kill -INT "$pid" 2>/dev/null || true; done
 for pid in $pids; do
  i=0
  while kill -0 "$pid" 2>/dev/null; do
   i=$((i+1));[ "$i" -le 15 ] || { echo "ISP $pid did not exit; update cancelled. Backup: $backup" >&2;exit 2; }
   sleep 1
  done
 done
 cp "$BASE/rk3588/isp-probe" "$ISP/isp-probe.fps-new"
 chmod 755 "$ISP/isp-probe.fps-new"
 cp "$BASE/rk3588/lib/librkaiq.so" "$ISP/lib/librkaiq.so.fps-new"
 chmod 755 "$ISP/lib/librkaiq.so.fps-new"
fi
cp "$BASE/rk3588/perception-dashboard.aarch64" "$DASH/perception-dashboard.aarch64.fps-new"
chmod 755 "$DASH/perception-dashboard.aarch64.fps-new"
if [ "$MODE" = all ]; then
 mv -f "$ISP/lib/librkaiq.so.fps-new" "$ISP/lib/librkaiq.so"
 mv -f "$ISP/isp-probe.fps-new" "$ISP/isp-probe"
fi
mv -f "$DASH/perception-dashboard.aarch64.fps-new" "$DASH/perception-dashboard.aarch64"
printf 'Installed. Backup: %s\n' "$backup"
printf 'Start with: cd "%s" && ./run-perception.sh\n' "$DASH"
printf '%s\n' 'Compare camera-only mode with ./run-perception.sh --no-detect; CAP=capture, AI=recognition FPS.'
