set -eu
ls -l /acg138-ros2/rootfs/opt
cat /acg138-ros2/rootfs/usr/local/bin/acg138-ros-entrypoint
cat > /acg138-ros2/rootfs/opt/acg138_ws/nav3-stage0-delivery/build-board.sh <<'BUILD'
#!/bin/bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
cd /opt/acg138_ws
colcon build --merge-install --packages-select acg138_fpga_map_bridge --executor sequential --install-base /opt/acg138_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
./build/acg138_fpga_map_bridge/test_nav3_wire /opt/acg138_ws/nav3-stage0-delivery/board-test-results
ctest --test-dir build/acg138_fpga_map_bridge -R '^nav3_wire_golden$' --output-on-failure
sha256sum install/lib/acg138_fpga_map_bridge/fpga_map_bridge build/acg138_fpga_map_bridge/test_nav3_wire
BUILD
cat > /root/nav3-stage0-build.sh <<'WRAP'
#!/bin/sh
chroot /acg138-ros2/rootfs /usr/bin/env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin HOME=/root LANG=C.UTF-8 /bin/bash /opt/acg138_ws/nav3-stage0-delivery/build-board.sh
code=$?
printf '%s\n' "$code" > /root/nav3-stage0-build.exit
exit "$code"
WRAP
nohup sh /root/nav3-stage0-build.sh > /root/nav3-stage0-build.log 2>&1 < /dev/null &
echo $! > /root/nav3-stage0-build.pid
cat /root/nav3-stage0-build.pid
