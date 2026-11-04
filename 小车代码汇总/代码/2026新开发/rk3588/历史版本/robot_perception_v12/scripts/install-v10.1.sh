#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
DASH=${DASHBOARD_DIR:-/root/perception-dashboard}
case $(uname -m) in aarch64|arm64) ;; *) echo 'Run this installer on RK3588 ARM64.' >&2;exit 2 ;; esac
(cd "$BASE" && sha256sum -c SHA256SUMS)
DASH=$(CDPATH= cd -- "$DASH" && pwd -P)
[ -f "$DASH/perception-dashboard.aarch64" ] && [ -f "$DASH/run-perception.sh" ] || {
 echo 'Existing dashboard binary/launcher missing. Set DASHBOARD_DIR to the working installation.' >&2;exit 2;
}
for proc in /proc/[0-9]*/exe; do
 exe=$(readlink "$proc" 2>/dev/null || true)
 case "$exe" in "$DASH/perception-dashboard.aarch64"*|"$DASH/build/perception-dashboard.aarch64"*)
  echo 'Close the YOLO/IMU window before updating.' >&2;exit 2 ;;
 esac
done
BACKUP=$(mktemp -d "$DASH/v10.1-backup.XXXXXX")
: > "$BACKUP/MISSING"
for name in perception-dashboard.aarch64 run-dashboard.sh; do
 [ ! -L "$DASH/$name" ] || { echo "Refusing symbolic link: $name" >&2;exit 2; }
 if [ -f "$DASH/$name" ]; then cp -p "$DASH/$name" "$BACKUP/$name"
 else printf '%s\n' "$name" >> "$BACKUP/MISSING"; fi
done
(cd "$BACKUP" && sha256sum MISSING perception-dashboard.aarch64 > SHA256SUMS &&
 if [ -f run-dashboard.sh ]; then sha256sum run-dashboard.sh >> SHA256SUMS; fi)
STAGE=$(mktemp -d "$DASH/.v10.1-stage.XXXXXX")
committed=0
cleanup() {
 rc=$?
 trap - EXIT HUP INT TERM
 if [ "$committed" = 0 ]; then
  for name in perception-dashboard.aarch64 run-dashboard.sh; do
   if [ -f "$BACKUP/$name" ]; then cp -p "$BACKUP/$name" "$DASH/$name.restore.$$" && mv -f "$DASH/$name.restore.$$" "$DASH/$name"
   else rm -f "$DASH/$name"; fi
  done
  echo 'Update incomplete; previous binary and wrapper restored.' >&2
  [ "$rc" != 0 ] || rc=1
 fi
 rm -rf "$STAGE"
 exit "$rc"
}
trap cleanup EXIT
trap 'exit 1' HUP INT TERM
for name in perception-dashboard.aarch64 run-dashboard.sh; do
 cp "$BASE/$name" "$STAGE/$name"
 chmod 755 "$STAGE/$name"
done
for name in perception-dashboard.aarch64 run-dashboard.sh; do mv -f "$STAGE/$name" "$DASH/$name";done
printf '%s\n' "$BACKUP" > "$DASH/.v10.1-last-backup"
committed=1
printf 'Installed v10.1. Backup: %s\n' "$BACKUP"
printf 'Start: cd "%s" && ./run-dashboard.sh -0\n' "$DASH"
printf '%s\n' 'Use -90 for the rotated fullscreen window. Existing model, IMU, ISP and launcher are retained.'
