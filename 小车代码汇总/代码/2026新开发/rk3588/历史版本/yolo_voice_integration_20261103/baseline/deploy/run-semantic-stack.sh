#!/bin/sh
# Semantic preview mode; YOLO contexts are not created in this session.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ "${1:-}" = --help ]; then
    echo 'run-semantic-stack.sh [existing dashboard options]'
    echo 'SmolVLM2-500M owns the NPU; YOLO is disabled; semantic goals default to preview.'
    echo 'Starts idle. Phone selects an endpoint to preview; explicit Start executes.'
    echo 'Uses existing candidate26; never loads or modifies an FPGA image.'
    exit 0
fi
# Final option wins; no paused YOLO process retains NPU contexts or memory.
set -- "$@" --no-detect
[ "$(id -u)" = 0 ] && [ "$(uname -m)" = aarch64 ] || { echo 'RK ARM64 root runtime required.' >&2; exit 1; }
ROOTFS_DIR=${ROOTFS_DIR:-/acg138-ros2/rootfs}
export ROOTFS_DIR
[ -x "$BASE/semantic/serve.sh" ] && [ -x "$ROOTFS_DIR/opt/acg138_semantic_20261102/run-ros.sh" ] || {
    echo 'Install the verified semantic runtime first.' >&2; exit 1;
}
ROS_OVERLAY=$ROOTFS_DIR/opt/acg138_exploration_20261102/install/setup.bash
[ -r "$ROS_OVERLAY" ] || { echo 'The isolated Jazzy overlay must be built and verified first.' >&2; exit 1; }
GATEWAY=$BASE/bin/robot-fpga-gateway.aarch64
PERCEPTION_BINARY=$BASE/perception/perception-dashboard.aarch64
[ -x "$GATEWAY" ] && [ -x "$PERCEPTION_BINARY" ] || { echo 'Missing verified ARM64 binaries.' >&2; exit 1; }
case "${PPE_VOICE_ENABLED:-1}" in 0|1) ;; *) echo 'PPE_VOICE_ENABLED must be 0 or 1.' >&2; exit 2 ;; esac
if [ "${PPE_VOICE_ENABLED:-1}" = 1 ]; then
    [ -x "$BASE/alerts/ppe-voice.aarch64" ] && [ -x "$BASE/alerts/play-hdmi.sh" ] && [ -r "$BASE/alerts/config.json" ] || {
        echo 'PPE HDMI voice runtime is missing from this integration package.' >&2; exit 1;
    }
fi
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
PERCEPTION_CAMERA_PROBE_BINARY=${PERCEPTION_CAMERA_PROBE_BINARY:-/root/robot-perception/camera-link-probe.aarch64}
export PERCEPTION_CAMERA_PROBE_BINARY
export PERCEPTION_NAVIGATION_SOURCE=fpga
VOICE_PID=
SEMANTIC_PID=
CHILDREN=
cleanup() {
    trap - EXIT INT TERM HUP
    # Every PID below is a process group this launcher created with setsid.
    for pid in $CHILDREN; do kill -TERM "-$pid" 2>/dev/null || :; done
    for pid in $CHILDREN; do wait "$pid" 2>/dev/null || :; done
    rm -f /run/acg138-ros-bridge/semantic/token
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
rotate() { [ ! -f "$1" ] || [ "$(wc -c < "$1")" -lt 8388608 ] || mv -f "$1" "$1.previous"; }
LOG_DIR=${DATA_DIR:-/acg138-ros2/data}/logs/integration
(umask 077; mkdir -p "$LOG_DIR")
for name in robot-ros-stack.log robot-fpga-gateway.log robot-nav-dashboard.log robot-ppe-voice.log robot-semantic.log; do
    rotate "$LOG_DIR/$name"
    # Keep the old diagnostic paths while retaining the real files over reboot.
    if [ -f "/tmp/$name" ] && [ ! -L "/tmp/$name" ]; then cp -p "/tmp/$name" "$LOG_DIR/$name.before-persistent-logging"; fi
    ln -sfn "$LOG_DIR/$name" "/tmp/$name"
done
# The RAW ISP launcher reconfigures eth0. Finish that before opening ROS NAV3,
# SNS1 or IMU sockets. This pass never starts the native dashboard or sends MOT.
# Close fd 9 here too: the ISP receiver can outlive the supervisor.
PERCEPTION_PREPARE_ONLY=1 sh "$BASE/perception/run-dashboard.sh" "$@" 9>&-
BRIDGE_LOG=$ROOTFS_DIR/run/acg138-mapping/map-bridge.log
BRIDGE_OFFSET=1
if [ -f "$BRIDGE_LOG" ]; then BRIDGE_OFFSET=$(( $(wc -c < "$BRIDGE_LOG") + 1 )); fi
# The supervisor alone owns fd 9. Camera helpers may daemonize inside the UI
# launcher; inheriting the open file description would retain flock after exit.
(umask 077; mkdir -p /run/acg138-ros-bridge/semantic)
chmod 700 /run/acg138-ros-bridge /run/acg138-ros-bridge/semantic
cp "$PAIR_TOKEN_FILE" /run/acg138-ros-bridge/semantic/token
chmod 600 /run/acg138-ros-bridge/semantic/token
setsid sh "$BASE/deploy/run-chroot.sh" /opt/acg138_semantic_20261102/run-ros.sh dry_run:=false 9>&- > /tmp/robot-ros-stack.log 2>&1 &
ROS_PID=$!;CHILDREN="$ROS_PID"
# SESSION_BIND must finish before the native dashboard starts its legacy MOT
# status probes. They revoke a binding in progress even when their speed is zero.
# Read only newly appended lines; a previous successful session is not evidence.
ATTEMPT=0
while :; do
    kill -0 "$ROS_PID" 2>/dev/null || { echo 'ROS exited before the idle FPGA handshake.' >&2; exit 1; }
    if [ -f "$BRIDGE_LOG" ]; then
        if tail -c +"$BRIDGE_OFFSET" "$BRIDGE_LOG" | grep -q 'NAV3 handshake complete;'; then break; fi
        if tail -c +"$BRIDGE_OFFSET" "$BRIDGE_LOG" | grep -q 'NAV3 stopped:'; then echo 'Idle FPGA handshake failed; native MOT probes not started.' >&2; exit 1; fi
    fi
    ATTEMPT=$((ATTEMPT+1))
    [ "$ATTEMPT" -lt 200 ] || { echo 'Idle FPGA handshake deadline; native MOT probes not started.' >&2; exit 1; }
    sleep 0.1
done
setsid "$GATEWAY" --token-file "$PAIR_TOKEN_FILE" --port 8080 --backend-port 8082 9>&- > /tmp/robot-fpga-gateway.log 2>&1 &
GATEWAY_PID=$!;CHILDREN="$GATEWAY_PID $CHILDREN"
setsid env PERCEPTION_PREPARE_ISP=0 PERCEPTION_PREPARE_ONLY=0 sh "$BASE/perception/run-dashboard.sh" "$@" --phone-bind 127.0.0.1 --phone-port 8082 --npu-workers 3 9>&- > /tmp/robot-nav-dashboard.log 2>&1 &
UI_PID=$!;CHILDREN="$UI_PID $CHILDREN"

setsid sh "$BASE/semantic/serve.sh" 9>&- > /tmp/robot-semantic.log 2>&1 &
SEMANTIC_PID=$!;CHILDREN="$SEMANTIC_PID $CHILDREN"

# PPE voice is owned by this integration session, alongside the existing UI.
# It only reads authenticated video metadata and plays local HDMI audio.
if [ "${PPE_VOICE_ENABLED:-1}" = 1 ]; then
    setsid nice -n 10 "$BASE/alerts/ppe-voice.aarch64" --config "$BASE/alerts/config.json" 9>&- > /tmp/robot-ppe-voice.log 2>&1 &
    VOICE_PID=$!;CHILDREN="$VOICE_PID $CHILDREN"
fi
# No ARM, GOAL, START, NVA or old target restoration in this launcher.
while kill -0 "$ROS_PID" 2>/dev/null && kill -0 "$GATEWAY_PID" 2>/dev/null && kill -0 "$UI_PID" 2>/dev/null; do
    if ! kill -0 "$SEMANTIC_PID" 2>/dev/null; then break; fi
    if [ -n "$VOICE_PID" ] && ! kill -0 "$VOICE_PID" 2>/dev/null; then break; fi
    sleep 1
done
echo 'A component exited; stopping only this owned session. See integration logs, including robot-semantic.log.' >&2
exit 1
