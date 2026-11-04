#!/bin/sh
# Familiar entry point for the integrated YOLO + ROS program; idle at startup.
set -eu
ROBOT_STACK=${ROBOT_STACK:-/root/robot-fpga-integration-20261102}
exec sh "$ROBOT_STACK/deploy/run-stack.sh" "$@"
