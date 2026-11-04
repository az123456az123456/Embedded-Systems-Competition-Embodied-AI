#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CONFIG=${ACG138_SERVICE_CONFIG:-/etc/acg138-ros2.conf}
[ ! -r "$CONFIG" ] || . "$CONFIG"
export ROS_IMAGE ROOTFS_DIR DATA_DIR SERIAL_DEVICE ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE
RUNTIME=${ROS_RUNTIME:-chroot}
case "$RUNTIME" in chroot) RUN=$BASE/run-chroot.sh ;; docker) RUN=$BASE/run-container.sh ;; *) echo 'ROS_RUNTIME must be chroot or docker' >&2; exit 1;; esac
LOG=${DATA_DIR:-/userdata/acg138-ros2}/logs/service.log
mkdir -p "$(dirname "$LOG")"
STOP=0; CHILD=
finish() { STOP=1; [ -z "$CHILD" ] || kill -TERM "$CHILD" 2>/dev/null || true; }
trap finish TERM INT HUP
DELAY=2
while [ "$STOP" -eq 0 ]; do
    if [ -f "$LOG" ] && [ "$(wc -c < "$LOG")" -gt 5242880 ]; then mv -f "$LOG" "$LOG.1"; fi
    printf '%s starting sensor/mapping service; motor_enable=never runtime=%s\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$RUNTIME" >> "$LOG"
    "$RUN" >> "$LOG" 2>&1 & CHILD=$!
    CODE=0; wait "$CHILD" || CODE=$?
    # An interrupted wait does not mean the child has completed.
    if [ "$STOP" -ne 0 ]; then wait "$CHILD" 2>/dev/null || true; break; fi
    CHILD=
    printf '%s exit=%s; retry_in=%ss; no automatic motor enable\n' "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$CODE" "$DELAY" >> "$LOG"
    sleep "$DELAY" & CHILD=$!; wait "$CHILD" || true; CHILD=
    [ "$DELAY" -ge 30 ] || DELAY=$((DELAY * 2))
    [ "$DELAY" -le 30 ] || DELAY=30
done
