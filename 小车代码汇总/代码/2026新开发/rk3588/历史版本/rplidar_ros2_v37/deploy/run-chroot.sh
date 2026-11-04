#!/bin/sh
# Trusted userspace isolation; chroot is not a security boundary.
# No Docker daemon, cgroups, bridge, veth, systemd or changed kernel required.
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ROOTFS=${ROOTFS_DIR:-/userdata/acg138-ros2/rootfs}
DATA=${DATA_DIR:-/userdata/acg138-ros2}
SERIAL=${SERIAL_DEVICE:-/dev/rplidar_c1}
LOCK=${CHROOT_LOCK_DIR:-/run/acg138-ros2-chroot.lock}
MOUNTS=
[ "$(id -u)" -eq 0 ] || { echo 'Root required to mount isolated userspace.' >&2; exit 1; }
[ "$(uname -m)" = aarch64 ] || { echo 'This is an ARM64 board runtime, not a PC runtime.' >&2; exit 1; }
[ -f "$ROOTFS/etc/acg138-ros2-rootfs" ] || { echo "Not an installed v37 userspace: $ROOTFS" >&2; exit 1; }
[ -c "$SERIAL" ] || { echo "Serial device missing: $SERIAL" >&2; exit 1; }
case "$ROOTFS$DATA$SERIAL" in *[[:space:]]*) echo 'Paths containing whitespace are unsupported.' >&2; exit 1;; esac
if [ "${ACG138_ROS2_MOUNTNS:-0}" != 1 ]; then
    HELPER=$BASE/bin/mountns-exec.aarch64
    [ -x "$HELPER" ] || { echo "Missing private mount helper: $HELPER" >&2; exit 1; }
    exec "$HELPER" sh "$0" "$@"
fi
mkdir "$LOCK" 2>/dev/null || { echo 'An acg138 chroot session is already running (or lock needs inspection).' >&2; exit 1; }
START_TICKS=$(awk '{sub(/^.*\) /, ""); print $20}' "/proc/$$/stat")
printf '%s %s\n' "$$" "$START_TICKS" > "$LOCK/mountns.pid"
cleanup() {
    trap - EXIT INT TERM HUP
    BUSY=0
    # Reverse order; do not unmount another program's or a pre-existing mount.
    for p in $MOUNTS; do umount "$p" 2>/dev/null || { echo "Mount remains busy: $p" >&2; BUSY=1; }; done
    if [ "$BUSY" -eq 0 ]; then rm -f "$LOCK/mountns.pid"; rmdir "$LOCK" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
bind_file() {
    src=$1; dst=$2
    [ -e "$dst" ] || : > "$dst"
    mount --bind "$src" "$dst"
    MOUNTS="$dst $MOUNTS"
}
mkdir -p "$DATA/maps" "$DATA/logs" "$DATA/bags" "$ROOTFS/proc" "$ROOTFS/dev" "$ROOTFS/dev/shm" "$ROOTFS/data" "$ROOTFS/tmp" "$ROOTFS/run/lock" /run/lock
mount -t proc proc "$ROOTFS/proc"; MOUNTS="$ROOTFS/proc $MOUNTS"
mount -t tmpfs -o mode=1777,nosuid,nodev,size=128m tmpfs "$ROOTFS/dev/shm"; MOUNTS="$ROOTFS/dev/shm $MOUNTS"
mount -t tmpfs -o mode=1777,nosuid,nodev,size=128m tmpfs "$ROOTFS/tmp"; MOUNTS="$ROOTFS/tmp $MOUNTS"
for p in null zero random urandom; do bind_file "/dev/$p" "$ROOTFS/dev/$p"; done
bind_file "$SERIAL" "$ROOTFS/dev/rplidar_c1"
# Only sensor serial and generic character devices, never the entire host /dev.
for p in maps logs bags; do
    mkdir -p "$ROOTFS/data/$p"
    mount --bind "$DATA/$p" "$ROOTFS/data/$p"; MOUNTS="$ROOTFS/data/$p $MOUNTS"
done
mount --bind /run/lock "$ROOTFS/run/lock"; MOUNTS="$ROOTFS/run/lock $MOUNTS"
if [ -f /etc/resolv.conf ]; then
    rm -f "$ROOTFS/etc/resolv.conf"
    cp /etc/resolv.conf "$ROOTFS/etc/resolv.conf"
fi
if [ "$#" -eq 0 ]; then set -- ros2 launch acg138_bringup mapping.launch.py; fi
# setpriv belongs to the exported Ubuntu userspace. Drop all capabilities except
# NET_RAW before the ROS nodes: SNS1+IMU receive and optional map transmit only.
# It cannot enable an interface or reconfigure the host network.
chroot "$ROOTFS" /usr/bin/env -i \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    HOME=/root LANG=C.UTF-8 \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-37}" \
    ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}" \
    ACG138_HOST_PROC_ROOT=/proc \
    /usr/bin/setpriv \
    --bounding-set=-all,+net_raw --inh-caps=-all,+net_raw \
    --ambient-caps=-all,+net_raw --no-new-privs \
    /usr/local/bin/acg138-ros-entrypoint "$@" &
CHILD=$!
terminate() { kill -TERM "$CHILD" 2>/dev/null || true; wait "$CHILD" 2>/dev/null || true; }
trap 'terminate; exit 130' INT
trap 'terminate; exit 143' TERM HUP
wait "$CHILD"
