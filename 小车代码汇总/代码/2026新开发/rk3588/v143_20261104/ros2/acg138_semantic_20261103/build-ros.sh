#!/bin/bash
set -e
set +u
source /opt/ros/jazzy/setup.bash
source /opt/acg138_ros/setup.bash
source /opt/acg138_exploration_20261102/install/setup.bash
cd /opt/acg138_semantic_20261103
export MAKEFLAGS=-j1; export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --merge-install --executor sequential --packages-select explore_lite_msgs explore_lite acg138_semantic acg138_frontier acg138_ros_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_CXX_FLAGS_RELEASE="-O1 -DNDEBUG"
