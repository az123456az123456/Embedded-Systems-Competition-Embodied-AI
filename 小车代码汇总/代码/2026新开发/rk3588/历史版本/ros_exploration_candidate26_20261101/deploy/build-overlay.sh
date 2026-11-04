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
    --base-paths src/acg138_nav_interfaces src/acg138_base_telemetry src/acg138_fpga_map_bridge src/acg138_frontier src/acg138_ros_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$WORK/install/setup.bash"
# A board clock older than an archived source tree can make setuptools retain a
# future-dated build/lib copy. Never accept colcon's exit code as proof of identity.
python3 - <<'PY'
import hashlib
from pathlib import Path
import acg138_frontier
source=Path('/opt/acg138_exploration_20261102/src/acg138_frontier/acg138_frontier')
installed=Path(acg138_frontier.__file__).parent
for path in source.rglob('*.py'):
    target=installed/path.relative_to(source)
    if not target.is_file() or hashlib.sha256(path.read_bytes()).digest()!=hashlib.sha256(target.read_bytes()).digest():
        raise SystemExit('Installed ROS Python differs from source; preserve and clean the isolated build/install trees before rebuilding: '+str(target))
print('ROS_PYTHON_SOURCE_IDENTITY=PASS')
PY
ros2 pkg executables acg138_fpga_map_bridge
ros2 pkg executables acg138_frontier
ros2 pkg executables acg138_ros_bridge
# No launch, service call, map upload, START or motor authorization.
