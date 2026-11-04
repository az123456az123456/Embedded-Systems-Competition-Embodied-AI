#!/bin/sh
# Retry dh until application entry. No replay, ARM, goal or --log.
set -eu
BASE=${ROBOT_STACK:-/root/robot-fpga-integration-20261102}
ROOTFS=${ROOTFS_DIR:-/acg138-ros2/rootfs}
ready() {
 [ -d /sys/class/net/eth0 ] && [ -d "$ROOTFS/opt/acg138_exploration_20261102/install" ] || return 1
 [ -c /dev/rplidar_c1 ] || [ -c /dev/ttyUSB0 ] || return 1
 [ -S /run/wayland-0 ] || [ -S /run/user/0/wayland-0 ] || return 1
 [ -x "$BASE/bin/robot-fpga-gateway.aarch64" ] && [ -x "$BASE/perception/perception-dashboard.aarch64" ]
}
if [ "${1:-}" = --check ]; then
 if ready;then echo 'Boot prerequisites ready; no application started';exit 0;fi
 echo 'Waiting for Ethernet, C1, Weston, ROS overlay or program files';exit 1
fi
[ "$#" = 0 ] || exit 2
while ! ready;do
 sleep 1
done
export XDG_RUNTIME_DIR=/run WAYLAND_DISPLAY=wayland-0
if [ ! -S /run/wayland-0 ];then XDG_RUNTIME_DIR=/run/user/0;export XDG_RUNTIME_DIR;fi
# dh resolves to /root/run-yolo.sh (not the missing deploy/run-yolo.sh).
# The retry worker preserves stack flock and child cleanup, and stops retrying
# after UI/phone entry, including when the user deliberately exits afterward.
export PERCEPTION_CAMERA_PROBE_BINARY=${PERCEPTION_CAMERA_PROBE_BINARY:-/root/robot-perception/camera-link-probe.aarch64}
exec sh "$BASE/deploy/retry-dh-until-ready.sh"
