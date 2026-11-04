cat /root/nav3-stage0-build.log
cat /root/nav3-stage0-build.exit
cat /acg138-ros2/rootfs/opt/acg138_ws/nav3-stage0-delivery/board-test-results/summary.json
chroot /acg138-ros2/rootfs /bin/sh -c 'readlink -f /opt/acg138_ros; sha256sum /opt/acg138_ws/install/lib/acg138_fpga_map_bridge/fpga_map_bridge /opt/acg138_ros/lib/acg138_fpga_map_bridge/fpga_map_bridge'
