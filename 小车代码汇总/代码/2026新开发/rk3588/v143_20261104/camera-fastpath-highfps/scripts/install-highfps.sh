#!/bin/sh
# Updates the isolated receiver, two narrowly patched launchers and FPS metadata.
# The FPGA sensor timing controls real frame generation.
set -eu
. "$(dirname -- "$0")/highfps-common.sh"
[ "$#" -le 1 ] || fail "Usage: $0 [30|35|40]"
target=${1:-40}
case "$target" in 30|35|40) ;; *) fail 'Select 30, 35 or 40.' ;; esac
check_board
(cd "$HERE" && sha256sum -c SHA256SUMS) || fail 'Package checksum verification failed.'
[ "$(sha "$ISP/lib/librkaiq.so")" = bd20b922212d5b22cc155008772c5ee8efc49c34d031e35425d1691f3e8a27b6 ] || fail 'This update requires the verified isolated FakeCam v4 library; no system library was changed.'
[ -s "$HERE/isp-probe" ] || fail 'Missing packaged receiver.'
check_consumers
stage=$(mktemp -d "$ISP/.camera-highfps-stage.XXXXXX")
appstage=$(mktemp "$DASH/.camera-highfps-launch.XXXXXX")
committing=0
backup=
cleanup() {
 code=$?
 trap - EXIT HUP INT TERM
 if [ "$committing" = 1 ]; then
  if restore_backup; then
   printf 'Update failed; original camera files restored. Backup: %s\n' "$backup" >&2
  else
   printf 'Update failed and restore needs attention. Keep programs closed. Backup: %s\n' "$backup" >&2
  fi
  code=2
 fi
 rm -f "$appstage"
 rm -rf "$stage"
 exit "$code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' HUP TERM
awk -v kind=isp -f "$HERE/patch-fps-launcher.awk" "$ISP/run_isp_camera.sh" > "$stage/run_isp_camera.sh" || fail 'ISP launcher does not match the supported FPS statements.'
awk -v kind=app -f "$HERE/patch-fps-launcher.awk" "$DASH/run-perception.sh" > "$appstage" || fail 'Dashboard launcher does not match the supported FPS statements.'
sh -n "$stage/run_isp_camera.sh" "$appstage"
cp "$HERE/isp-probe" "$stage/isp-probe"
cmp -s "$HERE/isp-probe" "$stage/isp-probe" || fail 'Staged receiver mismatch.'
printf '%s\n' "$target" > "$stage/camera-target-fps"
chmod 755 "$stage/isp-probe" "$stage/run_isp_camera.sh" "$appstage"
chmod 644 "$stage/camera-target-fps"
isp_launch_sha=$(sha "$stage/run_isp_camera.sh")
app_launch_sha=$(sha "$appstage")
backup=$(mktemp -d "$ISP/camera-highfps-backup.XXXXXX")
chmod 700 "$backup"
cp -p "$ISP/isp-probe" "$ISP/run_isp_camera.sh" "$backup/"
cp -p "$DASH/run-perception.sh" "$backup/run-perception.sh"
printf '%s\n' "$ISP" > "$backup/isp_path"
printf '%s\n' "$DASH" > "$backup/dash_path"
if [ -f "$ISP/camera-target-fps" ]; then
 cp -p "$ISP/camera-target-fps" "$backup/camera-target-fps"
 printf '%s\n' present > "$backup/config_state"
else printf '%s\n' absent > "$backup/config_state"; fi
(cd "$backup" && sha256sum isp-probe run_isp_camera.sh run-perception.sh isp_path dash_path config_state > SHA256SUMS)
if [ -f "$backup/camera-target-fps" ]; then (cd "$backup" && sha256sum camera-target-fps >> SHA256SUMS); fi
verify_backup
stop_receiver
committing=1
mv -f "$stage/isp-probe" "$ISP/isp-probe"
mv -f "$stage/run_isp_camera.sh" "$ISP/run_isp_camera.sh"
mv -f "$appstage" "$DASH/run-perception.sh"
mv -f "$stage/camera-target-fps" "$ISP/camera-target-fps"
[ "$(sha "$ISP/isp-probe")" = "$(sha "$HERE/isp-probe")" ] || fail 'Installed receiver verification failed.'
[ "$(sha "$ISP/run_isp_camera.sh")" = "$isp_launch_sha" ] &&
[ "$(sha "$DASH/run-perception.sh")" = "$app_launch_sha" ] &&
[ "$(cat "$ISP/camera-target-fps")" = "$target" ] || fail 'Installed camera configuration verification failed.'
committing=0
printf 'Camera receiver installed; nominal target=%s FPS. Backup: %s\n' "$target" "$backup"
printf '%s\n' 'Program the matching FPGA image, then use your existing yolo or run-dashboard.sh command.'
printf '%s\n' 'Actual camera FPS is FASTPATH rx_fps/output_fps; nominal V4L2 intervals never create frames.'
