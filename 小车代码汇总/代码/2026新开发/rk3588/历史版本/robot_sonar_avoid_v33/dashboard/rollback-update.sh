#!/bin/sh
set -eu
[ "$#" = 1 ] || { echo "Usage: sh $0 /root/robot-perception-v12-backup.XXXXXX" >&2;exit 2; }
BACKUP=$1
[ -f "$BACKUP/format" ] && [ "$(cat "$BACKUP/format")" = ROBOT_PERCEPTION_V12 ] || { echo 'Not a v12 application backup' >&2;exit 2; }
ROOT=$(cat "$BACKUP/root");ISP=$(cat "$BACKUP/isp");SKIP_ISP=$(cat "$BACKUP/skip-isp")
case "$ROOT" in /*) ;; *) echo 'Invalid backup root' >&2;exit 2 ;; esac
case "$ISP" in /*) ;; *) echo 'Invalid backup ISP path' >&2;exit 2 ;; esac
[ "$ROOT" != / ] && [ "$ROOT" != '' ] || { echo 'Invalid backup root' >&2;exit 2; }
TARGET=$ROOT/robot-perception
[ ! -L "$TARGET" ] || { echo 'Application target cannot be a symlink' >&2;exit 2; }
if [ -d "$BACKUP/app" ]; then
 STAGE=$(mktemp -d "$ROOT/.robot-perception-restore.XXXXXX")
 cp -a "$BACKUP/app/." "$STAGE/"
 rm -rf "$TARGET";mv "$STAGE" "$TARGET"
else rm -rf "$TARGET";fi
rm -f "$ROOT/run-yolo.sh"
if [ -e "$BACKUP/run-yolo.sh" ] || [ -L "$BACKUP/run-yolo.sh" ]; then cp -a "$BACKUP/run-yolo.sh" "$ROOT/run-yolo.sh";fi
if [ -f "$BACKUP/legacy-entry-state" ] && [ -d "$ROOT/perception-dashboard" ] && [ ! -L "$ROOT/perception-dashboard" ]; then
 rm -f "$ROOT/perception-dashboard/run-dashboard.sh"
 if [ "$(cat "$BACKUP/legacy-entry-state")" = present ]; then
  cp -a "$BACKUP/legacy-run-dashboard.sh" "$ROOT/perception-dashboard/run-dashboard.sh"
 fi
fi
if [ "$SKIP_ISP" = 0 ]; then
 if [ -e "$BACKUP/isp-probe" ] || [ -L "$BACKUP/isp-probe" ]; then
  RECEIVER=$(mktemp "$ISP/.isp-probe-restore.XXXXXX")
  rm -f "$RECEIVER"
  cp -a "$BACKUP/isp-probe" "$RECEIVER";mv -f "$RECEIVER" "$ISP/isp-probe"
 else rm -f "$ISP/isp-probe";fi
fi
echo "Restored application from $BACKUP"
echo 'Restart the camera ISP session to load the restored receiver. FPGA bitstreams are restored separately.'
