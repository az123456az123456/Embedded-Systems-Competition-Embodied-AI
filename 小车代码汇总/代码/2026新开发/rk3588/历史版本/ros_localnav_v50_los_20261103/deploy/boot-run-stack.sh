#!/bin/sh
# One boot launch of the same foreground entry. No replay, ARM, goal or --log.
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
n=0
while ! ready;do
 n=$((n+1));[ "$n" -le 180 ] || { echo 'Boot prerequisites timed out; no application started';exit 1; }
 sleep 1
done
export XDG_RUNTIME_DIR=/run WAYLAND_DISPLAY=wayland-0
if [ ! -S /run/wayland-0 ];then XDG_RUNTIME_DIR=/run/user/0;export XDG_RUNTIME_DIR;fi
# Keep run-stack's existing flock and orderly child/mount cleanup. Do not restart
# after a deliberate exit or component failure; user can inspect and restart.
exec sh "$BASE/deploy/run-yolo.sh"
