#!/bin/bash
# Execute INSIDE the built ROS userspace. No serial/raw socket or motor involved.
set -eo pipefail
mkdir -p /data/logs /data/maps
ros2 launch acg138_bringup simulation.launch.py > /data/logs/ros-smoke-launch.log 2>&1 &
CHILD=$!
trap 'kill -INT "$CHILD" 2>/dev/null || true; wait "$CHILD" 2>/dev/null || true' EXIT
ros2 run acg138_bringup check_map.py --timeout 45 --output /data/logs/ros-map-smoke.json
ros2 run nav2_map_server map_saver_cli -f /data/maps/smoke --ros-args -p save_map_timeout:=10.0
[ -s /data/maps/smoke.yaml ] && [ -s /data/maps/smoke.pgm ]
echo 'PASS simulated ROS2 LaserScan -> slam_toolbox -> OccupancyGrid -> saved map'
