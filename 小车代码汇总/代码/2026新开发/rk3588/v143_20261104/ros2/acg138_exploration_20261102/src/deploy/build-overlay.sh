#!/bin/bash
# Run inside the existing Jazzy chroot. Writes only this independent /opt tree.
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /opt/acg138_ros/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
WORK=/opt/acg138_exploration_20261102
cd "$WORK"
colcon build --merge-install --executor sequential \
    --base-paths src/acg138_nav_interfaces src/acg138_fpga_map_bridge src/acg138_frontier src/acg138_ros_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$WORK/install/setup.bash"
ros2 pkg executables acg138_fpga_map_bridge
ros2 pkg executables acg138_frontier
ros2 pkg executables acg138_ros_bridge
# No launch, service call, map upload, START or motor authorization.
