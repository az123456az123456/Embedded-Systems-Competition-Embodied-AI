#!/bin/sh
# Use the running desktop; never start/stop Weston or modify a network interface.
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ -z "${WAYLAND_DISPLAY:-}" ]; then
    for directory in "${XDG_RUNTIME_DIR:-/var/run}" /var/run /run/user/0 /run/user/1000 /tmp; do
        for socket in "$directory"/wayland-*; do
            if [ -S "$socket" ]; then
                XDG_RUNTIME_DIR=$directory
                WAYLAND_DISPLAY=${socket##*/}
                break 2
            fi
        done
    done
fi
if [ -z "${XDG_RUNTIME_DIR:-}" ]; then XDG_RUNTIME_DIR=/var/run; fi
export XDG_RUNTIME_DIR
if [ -n "${WAYLAND_DISPLAY:-}" ]; then export WAYLAND_DISPLAY; fi
if [ -x "$APP_DIR/imu-dashboard.aarch64" ]; then
    exec "$APP_DIR/imu-dashboard.aarch64" "$@"
fi
exec "$APP_DIR/build/imu-dashboard.aarch64" "$@"
