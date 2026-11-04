#!/bin/sh
# Read-only checks: no kernel, firmware, routing or partition changes.
set -eu
APP=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ -f "$APP/../isp/isp-probe" ]; then ISP=$APP/../isp
else ISP=${ROBOT_ISP_DIR:-/opt/fpga-camera-isp-1080}; fi
MODE=${1:---diagnose}
case "$MODE" in --install|--run|--diagnose) ;; *) echo 'Usage: check-board.sh [--install|--run|--diagnose]' >&2; exit 2;; esac
failures=0
bad() { echo "FAIL: $*" >&2; failures=$((failures+1)); }
note() { echo "INFO: $*"; }
case $(uname -m) in aarch64|arm64) ;; *) bad 'This package requires aarch64 RK3588.';; esac
[ "$(uname -r)" = 6.1.75 ] || bad 'Bundled camera module requires Linux 6.1.75. Do not force-load it.'
[ "$(id -u)" = 0 ] || bad 'Run on the board as root.'
if [ -r /proc/config.gz ]; then
 config=$(zcat /proc/config.gz | sha256sum); config=${config%% *}
 [ "$config" = 7114825305fb4b5f032a523d009ae4eaa321f1c816ff60f44eeaca3b9a0ec20f ] || bad "Kernel configuration differs from the original BSP: $config"
else bad '/proc/config.gz is unavailable; cannot verify the bundled kernel module baseline.'; fi
for tool in ip media-ctl v4l2-ctl insmod sha256sum readlink mktemp; do
 command -v "$tool" >/dev/null 2>&1 || bad "Missing BSP tool: $tool"
done
command -v weston >/dev/null 2>&1 || bad 'BSP Weston compositor is missing.'
if [ -x /lib/ld-linux-aarch64.so.1 ]; then
 for program in "$APP/perception-dashboard.aarch64" "$ISP/isp-probe"; do
  [ -f "$program" ] || { bad "Missing runtime: $program"; continue; }
  if output=$(LD_LIBRARY_PATH="$APP/lib:$ISP/lib" /lib/ld-linux-aarch64.so.1 --list "$program" 2>&1); then
   note "Dynamic libraries resolved: ${program##*/}"
  else bad "$output"; fi
 done
else bad 'Missing aarch64 glibc loader /lib/ld-linux-aarch64.so.1'; fi
IFACE=
for path in /sys/class/net/*; do
 device=$(readlink -f "$path/device" 2>/dev/null || true)
 case "$device" in */fe1c0000.ethernet|*/ethernet@fe1c0000) IFACE=${path##*/}; break;; esac
done
if [ -n "$IFACE" ]; then note "Internal FPGA GMAC1: $IFACE"
elif [ "$MODE" = --run ]; then bad 'Internal fe1c0000 GMAC1 is missing. Restore matching RGMII BSP configuration first; this package does not flash boot.'
else note 'Internal GMAC1 not found. Installation can finish, but camera/IMU/control require matching RGMII BSP.'; fi
if [ "$MODE" = --run ]; then
 for spec in video22:rkisp_mainpath video27:rkisp_rawrd2; do
  node=${spec%%:*}; pattern=${spec#*:}
  name=$(cat "/sys/class/video4linux/$node/name" 2>/dev/null || true)
  case "$name" in *"$pattern"*) note "$node: $name";; *) bad "$node must be $pattern; got '$name'. Existing ISP binary requires this BSP node layout.";; esac
 done
 if topology=$(media-ctl -d /dev/media2 -p 2>/dev/null); then
  case "$topology" in *rkisp-isp-subdev*) ;; *) bad '/dev/media2 has no rkisp-isp-subdev.';; esac
  case "$topology" in *rkisp_rawrd2_s*) ;; *) bad '/dev/media2 has no rkisp_rawrd2_s.';; esac
 else bad 'Cannot read /dev/media2. Matching camera/ISP BSP configuration is required.'; fi
fi
if [ "$failures" -ne 0 ]; then
 echo "Preflight failed ($failures). No system image or device tree was changed." >&2
 exit 1
fi
echo "PASS: $MODE"
