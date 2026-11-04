#!/bin/sh
# Existing YOLO UI, task-only Gateway and idle ROS workers in one owned session.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "${1:-}" = --help ]; then
    echo 'run-stack.sh [existing YOLO dashboard options]'
    echo 'Starts idle. Phone selects an endpoint to preview; explicit Start executes.'
    echo 'Uses existing candidate26; never loads or modifies an FPGA image.'
    exit 0
fi
[ "$(id -u)" = 0 ] && [ "$(uname -m)" = aarch64 ] || { echo 'RK ARM64 root runtime required.' >&2; exit 1; }
ROOTFS_DIR=${ROOTFS_DIR:-/acg138-ros2/rootfs}
export ROOTFS_DIR
ROS_OVERLAY=$ROOTFS_DIR/opt/acg138_exploration_20261102/install/setup.bash
[ -r "$ROS_OVERLAY" ] || { echo 'The isolated Jazzy overlay must be built and verified first.' >&2; exit 1; }
GATEWAY=$BASE/bin/robot-fpga-gateway.aarch64
PERCEPTION_BINARY=$BASE/perception/perception-dashboard.aarch64
[ -x "$GATEWAY" ] && [ -x "$PERCEPTION_BINARY" ] || { echo 'Missing verified ARM64 binaries.' >&2; exit 1; }
PAIR_TOKEN_FILE=${PAIR_TOKEN_FILE:-/root/robot-perception/pair-token.txt}
[ -s "$PAIR_TOKEN_FILE" ] || { echo 'Existing pairing token required; no replacement token is generated.' >&2; exit 1; }
MODEL_PATH=${MODEL_PATH:-/root/robot-perception/model/acg138-last-e0e54560-int8-rk3588.rknn}
LABELS_PATH=${LABELS_PATH:-/root/robot-perception/model/acg138-last-e0e54560-labels.txt}
[ -r "$MODEL_PATH" ] && [ -r "$LABELS_PATH" ] || { echo 'Install the verified last.pt model first.' >&2; exit 1; }
SERIAL_DEVICE=${SERIAL_DEVICE:-/dev/rplidar_c1}
if [ ! -c "$SERIAL_DEVICE" ] && [ "$SERIAL_DEVICE" = /dev/rplidar_c1 ]; then SERIAL_DEVICE=/dev/ttyUSB0; fi
[ -c "$SERIAL_DEVICE" ] || { echo 'RPLIDAR C1 serial device missing.' >&2; exit 1; }
command -v setsid >/dev/null
mkdir -p /run/lock /tmp/robot-navigation
exec 9>/run/lock/acg138-fpga-stack.lock
flock -n 9 || { echo 'This stack already owns the runtime.' >&2; exit 1; }
# Avoid sharing the old executor. Do not kill unrelated or rollback processes.
for p in /proc/[0-9]*/comm; do
    name=$(cat "$p" 2>/dev/null || :)
    case "$name" in robot-navigatio*) echo 'Stop the old robot-navigation before this stack.' >&2; exit 1;; esac
done
ip link set eth0 mtu 4000 up
export SERIAL_DEVICE MODEL_PATH LABELS_PATH PAIR_TOKEN_FILE PERCEPTION_BINARY
export PERCEPTION_NAVIGATION_SOURCE=fpga
CHILDREN=
cleanup() {
    trap - EXIT INT TERM HUP
    # Every PID below is a process group this launcher created with setsid.
    for pid in $CHILDREN; do kill -TERM "-$pid" 2>/dev/null || :; done
    for pid in $CHILDREN; do wait "$pid" 2>/dev/null || :; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
rotate() { [ ! -f "$1" ] || [ "$(wc -c < "$1")" -lt 8388608 ] || mv -f "$1" "$1.previous"; }
rotate /tmp/robot-ros-stack.log;rotate /tmp/robot-fpga-gateway.log;rotate /tmp/robot-nav-dashboard.log
setsid sh "$BASE/deploy/run-chroot.sh" ros2 launch acg138_frontier integrated.launch.py dry_run:=false > /tmp/robot-ros-stack.log 2>&1 &
ROS_PID=$!;CHILDREN="$ROS_PID"
setsid "$GATEWAY" --token-file "$PAIR_TOKEN_FILE" --port 8080 --backend-port 8082 > /tmp/robot-fpga-gateway.log 2>&1 &
GATEWAY_PID=$!;CHILDREN="$GATEWAY_PID $CHILDREN"
setsid sh "$BASE/perception/run-dashboard.sh" "$@" --phone-bind 127.0.0.1 --phone-port 8082 --npu-workers 3 > /tmp/robot-nav-dashboard.log 2>&1 &
UI_PID=$!;CHILDREN="$UI_PID $CHILDREN"
# No ARM, GOAL, START, NVA or old target restoration in this launcher.
while kill -0 "$ROS_PID" 2>/dev/null && kill -0 "$GATEWAY_PID" 2>/dev/null && kill -0 "$UI_PID" 2>/dev/null; do sleep 1; done
echo 'A component exited; stopping only this owned session. See the three logs.' >&2
exit 1
