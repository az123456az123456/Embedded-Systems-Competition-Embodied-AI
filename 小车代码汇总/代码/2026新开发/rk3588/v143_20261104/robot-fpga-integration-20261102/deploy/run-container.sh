#!/bin/sh
set -eu
IMAGE=${ROS_IMAGE:-acg138-ros2:jazzy-v37}
DATA=${DATA_DIR:-/userdata/acg138-ros2}
SERIAL=${SERIAL_DEVICE:-/dev/rplidar_c1}
NAME=${CONTAINER_NAME:-acg138-ros2-v37}
command -v docker >/dev/null 2>&1 || { echo 'Docker is not installed; use run-chroot.sh with the exported ARM64 userspace.' >&2; exit 1; }
[ "$(uname -m)" = aarch64 ] || { echo 'This runtime command is for RK3588/aarch64.' >&2; exit 1; }
[ -c "$SERIAL" ] || { echo "Missing serial device: $SERIAL" >&2; exit 1; }
mkdir -p "$DATA/maps" "$DATA/logs" "$DATA/bags" /run/lock
# NET_RAW is needed by the read-only SNS1/IMU odometry receiver and optional map
# uploader. No NET_ADMIN, SYS_ADMIN, --privileged, docker.sock or /dev/mem.
# Existing YOLO/phone processes and their files are not mounted or modified.
if [ "$#" -eq 0 ]; then set -- ros2 launch acg138_bringup mapping.launch.py; fi
exec docker run --rm --name "$NAME" --network host --cap-drop ALL --cap-add NET_RAW \
    --security-opt no-new-privileges:true --user 0:0 \
    --device "$SERIAL:/dev/rplidar_c1:rw" \
    --mount "type=bind,src=$DATA/maps,dst=/data/maps" \
    --mount "type=bind,src=$DATA/logs,dst=/data/logs" \
    --mount "type=bind,src=$DATA/bags,dst=/data/bags" \
    --mount type=bind,src=/proc,dst=/host_proc,readonly \
    --mount type=bind,src=/run/lock,dst=/run/lock \
    --tmpfs /tmp:rw,nosuid,nodev,size=128m --shm-size 128m \
    --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-37}" \
    --env ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}" \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --env ACG138_HOST_PROC_ROOT=/host_proc \
    --env OMP_NUM_THREADS=2 --env OPENBLAS_NUM_THREADS=1 \
    --log-opt max-size=5m --log-opt max-file=3 "$IMAGE" "$@"
