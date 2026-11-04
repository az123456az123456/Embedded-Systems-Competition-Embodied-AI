#!/bin/sh
# Install only the unified application and camera receiver; preserve system libraries.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ROOT=${ROBOT_ROOT:-/root}
ISP=${ROBOT_ISP_DIR:-/opt/fpga-camera-isp-1080}
SKIP_ISP=${SKIP_ISP_UPDATE:-0}
case "$ROOT" in /*) ;; *) echo 'ROBOT_ROOT must be absolute' >&2;exit 2 ;; esac
case "$ISP" in /*) ;; *) echo 'ROBOT_ISP_DIR must be absolute' >&2;exit 2 ;; esac
case "$SKIP_ISP" in 0|1) ;; *) echo 'SKIP_ISP_UPDATE must be 0 or 1' >&2;exit 2 ;; esac
[ "$ROOT" != / ] && [ "$ROOT" != '' ] || { echo 'Invalid application root' >&2;exit 2; }
TARGET=$ROOT/robot-perception
[ ! -L "$TARGET" ] || { echo 'Application target cannot be a symlink' >&2;exit 2; }
[ ! -e "$TARGET" ] || [ -d "$TARGET" ] || { echo 'Application target must be a directory' >&2;exit 2; }
[ -r "$HERE/SHA256SUMS" ] && [ -r "$HERE/rollback-update.sh" ] || { echo 'Unpack the complete release first' >&2;exit 2; }
(cd "$HERE" && sha256sum -c SHA256SUMS) || { echo 'Release checksum verification failed; nothing installed' >&2;exit 1; }
for name in perception-dashboard.aarch64 run-perception.sh run-dashboard.sh model/yolo11s-int8-rk3588.rknn model/coco_80_labels_list.txt; do
 [ -f "$HERE/payload/app/$name" ] || { echo "Missing release file: $name" >&2;exit 1; }
done
if [ "$SKIP_ISP" = 0 ]; then
 [ -d "$ISP" ] && [ -f "$ISP/run_isp_camera.sh" ] && [ -f "$HERE/payload/isp-probe" ] || {
  echo 'Existing camera ISP installation not found; set ROBOT_ISP_DIR to its working directory' >&2;exit 1;
 }
fi
mkdir -p "$ROOT"
BACKUP=$(mktemp -d "$ROOT/robot-perception-v12-backup.XXXXXX")
printf '%s\n' ROBOT_PERCEPTION_V12 > "$BACKUP/format"
printf '%s\n' "$ROOT" > "$BACKUP/root"
printf '%s\n' "$ISP" > "$BACKUP/isp"
printf '%s\n' "$SKIP_ISP" > "$BACKUP/skip-isp"
if [ -e "$TARGET" ]; then cp -a "$TARGET" "$BACKUP/app";fi
if [ -e "$ROOT/run-yolo.sh" ] || [ -L "$ROOT/run-yolo.sh" ]; then cp -a "$ROOT/run-yolo.sh" "$BACKUP/run-yolo.sh";fi
# Preserve the user's established entry point while forwarding it to this one
# application. Do not replace the old binary, model or camera installation.
LEGACY=$ROOT/perception-dashboard
if [ -d "$LEGACY" ] && [ ! -L "$LEGACY" ]; then
 if [ -e "$LEGACY/run-dashboard.sh" ] || [ -L "$LEGACY/run-dashboard.sh" ]; then
  cp -a "$LEGACY/run-dashboard.sh" "$BACKUP/legacy-run-dashboard.sh"
  printf '%s\n' present > "$BACKUP/legacy-entry-state"
 else printf '%s\n' absent > "$BACKUP/legacy-entry-state";fi
fi
if [ "$SKIP_ISP" = 0 ] && { [ -e "$ISP/isp-probe" ] || [ -L "$ISP/isp-probe" ]; }; then cp -a "$ISP/isp-probe" "$BACKUP/isp-probe";fi
STAGE=$(mktemp -d "$ROOT/.robot-perception-stage.XXXXXX")
MUTATED=0
cleanup() {
 status=$?
 trap - 0 HUP INT TERM
 if [ "$status" != 0 ] && [ "$MUTATED" = 1 ]; then
  echo 'Install interrupted; restoring saved application...' >&2
  sh "$HERE/rollback-update.sh" "$BACKUP" || echo "Automatic restore failed; backup remains at $BACKUP" >&2
 fi
 [ ! -d "$STAGE" ] || rm -rf "$STAGE"
 exit "$status"
}
trap cleanup 0
trap 'exit 130' HUP INT TERM
cp -a "$HERE/payload/app/." "$STAGE/"
for candidate in "$TARGET/pair-token.txt" "$ROOT/robot-phone/pair-token.txt" "$ROOT/perception-dashboard/pair-token.txt"; do
 if [ -f "$candidate" ]; then cp "$candidate" "$STAGE/pair-token.txt";chmod 600 "$STAGE/pair-token.txt";break;fi
done
chmod 755 "$STAGE/perception-dashboard.aarch64" "$STAGE/run-dashboard.sh" "$STAGE/run-perception.sh"
MUTATED=1
if [ -e "$TARGET" ]; then mv "$TARGET" "$BACKUP/replaced-app";fi
mv "$STAGE" "$TARGET"
WRAPPER=$(mktemp "$ROOT/.run-yolo.XXXXXX")
SAFE_TARGET=$(printf '%s' "$TARGET" | sed "s/'/'\\\\''/g")
printf '%s\n' '#!/bin/sh' 'set -eu' > "$WRAPPER"
printf '%s%s%s\n' "exec sh '" "$SAFE_TARGET" "/run-dashboard.sh' \"\$@\"" >> "$WRAPPER"
chmod 755 "$WRAPPER";mv -f "$WRAPPER" "$ROOT/run-yolo.sh"
if [ -f "$BACKUP/legacy-entry-state" ]; then
 LEGACY_WRAPPER=$(mktemp "$LEGACY/.run-dashboard-v121.XXXXXX")
 cp "$ROOT/run-yolo.sh" "$LEGACY_WRAPPER";chmod 755 "$LEGACY_WRAPPER"
 mv -f "$LEGACY_WRAPPER" "$LEGACY/run-dashboard.sh"
fi
if [ "$SKIP_ISP" = 0 ]; then
 RECEIVER=$(mktemp "$ISP/.isp-probe-v12.XXXXXX")
 cp "$HERE/payload/isp-probe" "$RECEIVER";chmod 755 "$RECEIVER";mv -f "$RECEIVER" "$ISP/isp-probe"
fi
MUTATED=0
[ ! -d "$BACKUP/replaced-app" ] || rm -rf "$BACKUP/replaced-app"
echo "Installed: $TARGET"
echo "Backup: $BACKUP"
echo "Start normal: sh $ROOT/run-yolo.sh -0"
echo "Start rotated: sh $ROOT/run-yolo.sh -90"
[ ! -f "$BACKUP/legacy-entry-state" ] || echo "Compatible entry: sh $LEGACY/run-dashboard.sh -0"
echo 'Quit the previous YOLO and robot-phone-server before starting the unified app.'
echo 'Flash the matching robot_fmc_remote_30fps_v12.fs; motors stay disabled until explicitly armed.'
