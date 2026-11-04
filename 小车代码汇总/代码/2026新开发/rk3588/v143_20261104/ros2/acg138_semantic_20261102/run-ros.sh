#!/bin/bash
set -eo pipefail
source /opt/acg138_semantic_20261102/install/setup.bash
exec ros2 launch acg138_semantic_navigation integrated_semantic.launch.py "$@"
