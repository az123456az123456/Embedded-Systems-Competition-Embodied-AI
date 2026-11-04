#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
: "${ACG138_TOOLCHAIN_BIN:?Verified aarch64-none-linux-gnu toolchain bin required}"
: "${ACG138_SYSROOT:?BSP target sysroot including native dependencies required}"
: "${ACG138_ROS_UNDERLAY:?ARM64 ROS built for this exact Buildroot ABI required}"
[ -f "$ACG138_ROS_UNDERLAY/share/rclcpp/cmake/rclcppConfig.cmake" ] || {
  echo 'Missing Buildroot-native ROS underlay; this script does not build all ROS dependencies.' >&2; exit 1;
}
export AMENT_PREFIX_PATH="$ACG138_ROS_UNDERLAY"
export CMAKE_PREFIX_PATH="$ACG138_ROS_UNDERLAY"
export PKG_CONFIG_SYSROOT_DIR="$ACG138_SYSROOT"
export PKG_CONFIG_LIBDIR="$ACG138_SYSROOT/usr/lib/pkgconfig:$ACG138_SYSROOT/usr/share/pkgconfig"
unset PKG_CONFIG_PATH
cd "$BASE/ros2_ws"
exec colcon build --merge-install --executor sequential \
  --build-base build-native --install-base install-native \
  --cmake-args -DCMAKE_TOOLCHAIN_FILE="$BASE/deploy/native/acg138-aarch64.cmake" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
