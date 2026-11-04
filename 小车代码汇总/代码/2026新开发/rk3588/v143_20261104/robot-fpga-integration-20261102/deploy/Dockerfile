# Build on a Linux ARM64 PC, or with buildx --platform linux/arm64 on x86.
# Do not build the whole image on the 2 GiB RK3588 while YOLO is running.
# For a release, set ROS_BASE to the inspected official image @sha256 digest.
ARG ROS_BASE=ros:jazzy-ros-base-noble
FROM ${ROS_BASE} AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-robot-localization \
    ros-jazzy-nav2-amcl ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-rosbag2-storage-default-plugins python3-colcon-common-extensions \
    build-essential cmake util-linux ca-certificates && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/acg138_ws
COPY ros2_ws/src/ src/
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --merge-install --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' > install/debian-packages.tsv && \
    rm -rf build log
RUN mkdir -p /data/maps /data/logs /data/bags /tmp/robot-navigation /run/lock /host_proc && \
    printf '%s\n' jazzy-v37 > /etc/acg138-ros2-rootfs && \
    ln -s /opt/acg138_ws/install /opt/acg138_ros
COPY deploy/ros-entrypoint.sh /usr/local/bin/acg138-ros-entrypoint
RUN chmod 755 /usr/local/bin/acg138-ros-entrypoint
ENV ROS_DOMAIN_ID=37 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=/data/logs \
    OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
ENTRYPOINT ["/usr/local/bin/acg138-ros-entrypoint"]
CMD ["ros2", "launch", "acg138_bringup", "mapping.launch.py"]
