#!/bin/sh
# v23: keep the familiar entry point; the supervisor owns both services.
set -eu
APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
NAV_DIR=${NAVIGATION_HOME:-/root/robot-navigation}
[ -f "$NAV_DIR/run-navigation.sh" ] || {
    echo 'Unified navigation launcher is missing. Install the complete v23 RK update.' >&2
    exit 1
}
NAV_PERCEPTION_DIR=$APP_DIR
export NAV_PERCEPTION_DIR
exec sh "$NAV_DIR/run-navigation.sh" "$@"
