#!/bin/bash
set -e
set +u
source /opt/ros/jazzy/setup.bash
source /opt/acg138_ros/setup.bash
source /opt/acg138_exploration_20261102/install/setup.bash
source /opt/acg138_semantic_20261103/install/setup.bash
export ROS_DOMAIN_ID=187 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
exec python3 -B /opt/acg138_semantic_20261103/native-test.py
