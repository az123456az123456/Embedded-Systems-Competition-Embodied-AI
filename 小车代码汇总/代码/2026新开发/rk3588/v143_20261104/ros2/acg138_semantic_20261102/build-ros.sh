#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/acg138_exploration_20261102/install/setup.bash
cd /opt/acg138_semantic_20261102
colcon build --base-paths src --executor sequential --event-handlers console_direct+
