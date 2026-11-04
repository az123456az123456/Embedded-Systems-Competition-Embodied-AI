set -eu
cd /acg138-ros2/rootfs/opt/acg138_ws
test ! -e nav3-stage0-delivery
printf '%s\n' 'be31fef90aa0e86d7731d785746f026333971a769b9b600c3f17eeb706d2e638  /root/nav3-stage0-be31fef90aa0e86d.tar.gz' | sha256sum -c -
tar -xzf /root/nav3-stage0-be31fef90aa0e86d.tar.gz
chroot /acg138-ros2/rootfs /usr/bin/python3 /opt/acg138_ws/nav3-stage0-delivery/install_stage0.py
cat log/latest_build/acg138_fpga_map_bridge/stdout_stderr.log | tail -n 8
ls -l /acg138-ros2/rootfs/opt/acg138_ros/setup.bash /dev/ttyUSB0 /dev/rplidar_c1
