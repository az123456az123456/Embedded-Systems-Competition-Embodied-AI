set -eu
for name in Pause SaveMap; do
    printf '\nSLAM_SERVICE_%s\n' "$name"
    cat "/acg138-ros2/rootfs/opt/ros/jazzy/share/slam_toolbox/srv/$name.srv"
done
printf '\nBOARD_PYTHON_PACKAGES\n'
ls /acg138-ros2/rootfs/usr/lib/python3/dist-packages/yaml/__init__.py
printf '\nBUILD_HELPER\n'
ls -l /root/rplidar-ros2-v37/deploy/bin/mountns-exec.aarch64
ls -l /acg138-ros2/rootfs/usr/include/nlohmann/json.hpp 2>/dev/null || true
