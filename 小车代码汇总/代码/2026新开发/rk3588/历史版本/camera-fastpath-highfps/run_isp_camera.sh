#!/bin/sh
# Board-specific experimental RAW/RGMII -> ISP -> V4L2/Wayland launcher.
# Program the matching single/dual RAW FPGA SRAM image before running.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MODE=${1:-single}
FRAMES=${2:-100000}
# The real sensor rate is controlled by the programmed FPGA sensor table.
# Validate before checking devices, loading modules or changing network links.
# CAMERA_HIGHFPS_CONFIG_V20
if [ "${CAMERA_TARGET_FPS+x}" != x ]; then
    FPS_CONFIG=${CAMERA_FPS_CONFIG:-$BASE/camera-target-fps}
    if [ -r "$FPS_CONFIG" ]; then CAMERA_TARGET_FPS=$(cat "$FPS_CONFIG")
    else CAMERA_TARGET_FPS=40; fi
fi
if [ "${CAMERA_TARGET_FPS+x}" = x ]; then
 case "$CAMERA_TARGET_FPS" in 15|20|30|35|40) ;; *) echo 'CAMERA_TARGET_FPS must be 15, 20, 30, 35 or 40' >&2; exit 2 ;; esac
 export CAMERA_TARGET_FPS
fi
case "$MODE" in
 single) WIDTH=640; HEIGHT=480; MTU=1500; ARG=--camera ;;
 dual) WIDTH=1280; HEIGHT=480; MTU=1500; ARG=--dual-camera ;;
 single-1080) WIDTH=1920; HEIGHT=1080; MTU=4000; ARG=--camera-1080 ;;
 dual-1080) WIDTH=1920; HEIGHT=1080; MTU=4000; ARG=--dual-camera-1080 ;;
 *) echo "Usage: $0 single|dual|single-1080|dual-1080 [frames:1..100000]" >&2; exit 2 ;;
esac
case "$FRAMES" in ''|*[!0-9]*) exit 2;; esac
[ "$FRAMES" -ge 1 ] && [ "$FRAMES" -le 100000 ] || exit 2
[ "$(id -u)" = 0 ] || { echo 'Run as root.' >&2; exit 2; }
IQ=${ISP_IQ_FILE:-ov5647_ACG138-OV5647-CSI0_default.json}
for file in "$BASE/isp-probe" "$BASE/lib/librkaiq.so" "/etc/iqfiles/$IQ"; do
 [ -f "$file" ] || { echo "Missing $file" >&2; exit 2; }
done
LOCK=/tmp/fpga-isp.lock
mkdir "$LOCK" 2>/dev/null || { echo "ISP launcher already active; inspect $LOCK" >&2; exit 2; }
receiver=; viewer=
cleanup() {
 trap - EXIT INT TERM
 if [ -n "$viewer" ]; then kill -INT "$viewer" 2>/dev/null || :; wait "$viewer" 2>/dev/null || :; fi
 if [ -n "$receiver" ]; then kill -TERM "$receiver" 2>/dev/null || :; wait "$receiver" 2>/dev/null || :; fi
 rm -f "$LOCK/pid"; rmdir "$LOCK"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo $$ >"$LOCK/pid"
echo "Camera target: ${CAMERA_TARGET_FPS:-binary build default} FPS (nominal V4L2 interval; actual rate is FASTPATH rx_fps/output_fps)"
if [ ! -d /sys/module/v4l2loopback ]; then
 insmod "$BASE/v4l2loopback.ko" devices=3 video_nr=60,61,62 \
  card_label=FPGA_Combined,FPGA_Left,FPGA_Right exclusive_caps=1,1,1 max_width=1920 max_height=1080
fi
for spec in "max_width:$WIDTH" "max_height:$HEIGHT"; do
 parameter=${spec%:*}; required=${spec#*:}
 available=$(cat "/sys/module/v4l2loopback/parameters/$parameter")
 [ "$available" -ge "$required" ] || { echo "v4l2loopback $parameter=$available is too small; stop its consumers, unload it, and rerun." >&2; exit 2; }
done
IFACE=${FPGA_INTERFACE:-}
if [ -z "$IFACE" ]; then
 for path in /sys/class/net/*; do
  device=$(readlink -f "$path/device" 2>/dev/null || true)
  case "$device" in */fe1c0000.ethernet) IFACE=${path##*/}; break ;; esac
 done
fi
[ -n "$IFACE" ] && [ -d "/sys/class/net/$IFACE" ] || {
 echo 'Cannot identify FPGA GMAC1; set FPGA_INTERFACE to the internal fe1c0000.ethernet interface.' >&2; exit 2;
}
echo "Internal FPGA interface: $IFACE"
ip link set "$IFACE" down
ip link set "$IFACE" mtu "$MTU"
ip addr replace 192.168.0.3/32 dev "$IFACE"
ip link set "$IFACE" up
ip route replace 192.168.0.2/32 dev "$IFACE"
# Must run immediately before AIQ init: deinit disconnects this link.
media-ctl -d /dev/media2 -V "\"rkisp-isp-subdev\":0[fmt:SBGGR8_1X8/${WIDTH}x${HEIGHT} crop:(0,0)/${WIDTH}x${HEIGHT}]"
media-ctl -d /dev/media2 -l '"rkisp_rawrd2_s":0 -> "rkisp-isp-subdev":0[1]'
ISP_MANUAL_WB=1 ISP_SHARP_OFF=1 ISP_GAMMA_LINEAR=1 ISP_V4L2_OUTPUT=/dev/video60 LD_LIBRARY_PATH="$BASE/lib" \
 "$BASE/isp-probe" "$IQ" "$ARG" "$FRAMES" >/tmp/fpga-isp.log 2>&1 &
receiver=$!
if [ "${ISP_DISPLAY:-1}" = 1 ]; then
 sleep 2
 kill -0 "$receiver" 2>/dev/null || { wait "$receiver"; exit $?; }
 XDG_RUNTIME_DIR=/run WAYLAND_DISPLAY=wayland-0 gst-launch-1.0 \
  v4l2src device=/dev/video60 ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! waylandsink sync=false >/tmp/fpga-isp-display.log 2>&1 &
 viewer=$!
fi
rc=0
wait "$receiver" || rc=$?
receiver=
if [ -n "$viewer" ] && ! kill -0 "$viewer" 2>/dev/null; then
 echo 'Display exited before the capture session finished; inspect /tmp/fpga-isp-display.log' >&2
 wait "$viewer" 2>/dev/null || :
 viewer=
 rc=8
fi
tail -n 8 /tmp/fpga-isp.log
exit "$rc"
