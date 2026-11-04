#!/bin/bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/acg138_ros/setup.bash
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
export OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-1}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/data/logs}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-37}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
mkdir -p "$ROS_LOG_DIR"
# No motor enable, /cmd_vel producer, goal submission or FPGA PLAN command here.
exec "$@"
