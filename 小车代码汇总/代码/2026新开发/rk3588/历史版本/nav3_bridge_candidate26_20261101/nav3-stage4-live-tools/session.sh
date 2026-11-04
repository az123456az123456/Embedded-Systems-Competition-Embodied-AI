#!/bin/bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /opt/acg138_ros/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 daemon stop
ros2 launch /opt/acg138_ws/nav3-stage3-live-tools/static_diagnostic.launch.py enable_fpga_bridge:=false interface:=eth0 > /data/logs/nav3-stage4-mapping.log 2>&1 &
mapping=$!
ros2 run acg138_fpga_map_bridge fpga_map_bridge --ros-args -p protocol:=nav3 -p dry_run:=false -p exclusive_nav2_owner:=true -p interface:=eth0 -p nav3_frame:=map -p enable_plan:=true -p metadata_path:=/data/logs/nav3-stage4-metadata.json &
bridge=$!
cleanup(){ kill -INT "$mapping" "$bridge" 2>/dev/null || true; wait "$mapping" "$bridge" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 0' INT TERM
python3 /opt/acg138_ws/nav3-stage4-live-tools/observe.py > /data/logs/nav3-stage4-observer.jsonl 2> /data/logs/nav3-stage4-observer.err &
wait -n "$mapping" "$bridge"
