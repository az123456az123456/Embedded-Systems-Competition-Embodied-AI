#!/bin/sh
# Execute a CLI inside the existing private mount session, without re-mounting.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CONFIG=${ACG138_SERVICE_CONFIG:-/etc/acg138-ros2.conf}
[ ! -r "$CONFIG" ] || . "$CONFIG"
ROOTFS=${ROOTFS_DIR:-/acg138-ros2/rootfs}
PIDFILE=${CHROOT_LOCK_DIR:-/run/acg138-ros2-chroot.lock}/mountns.pid
[ -r "$PIDFILE" ] || { echo 'No running chroot mapping session.' >&2; exit 1; }
read -r PID START_TICKS < "$PIDFILE"
case "$PID" in ''|*[!0-9]*) echo 'Invalid session PID.' >&2; exit 1;; esac
[ -r "/proc/$PID/ns/mnt" ] || { echo 'Session has exited; start run-chroot.sh first.' >&2; exit 1; }
[ "$(awk '{sub(/^.*\) /, ""); print $20}' "/proc/$PID/stat")" = "$START_TICKS" ] || {
    echo 'Stale session PID: process identity changed.' >&2; exit 1;
}
[ "$#" -gt 0 ] || { echo 'Usage: exec-chroot.sh ros2 topic hz /scan' >&2; exit 2; }
exec "$BASE/bin/mountns-exec.aarch64" --join "$PID" \
    chroot "$ROOTFS" /usr/bin/env -i \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    HOME=/root LANG=C.UTF-8 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-37}" \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    ACG138_HOST_PROC_ROOT=/proc \
    /usr/bin/setpriv --bounding-set=-all,+net_raw --inh-caps=-all,+net_raw \
    --ambient-caps=-all,+net_raw --no-new-privs \
    /opt/acg138_exploration_20261102/src/deploy/ros-entrypoint.sh /bin/bash -c 'set +u; source /opt/ros/jazzy/setup.bash; source /opt/acg138_ros/setup.bash; source /opt/acg138_exploration_20261102/install/setup.bash; ros2 daemon stop >/dev/null 2>&1 || :; exec "$@"' bash "$@"
