#!/bin/sh
set -eu
ROBOT_APP=${ROBOT_APP:-/root/robot-perception}
exec sh "$ROBOT_APP/run-dashboard.sh" "$@"
