cat /root/nav3-stage0-build.log
cat /root/nav3-stage0-build.exit 2>/dev/null
ps -eo pid,comm | grep -E 'cc1|cmake|make|colcon|test_nav3'
tail -n 12 /acg138-ros2/rootfs/opt/acg138_ws/log/latest_build/acg138_fpga_map_bridge/stdout_stderr.log
