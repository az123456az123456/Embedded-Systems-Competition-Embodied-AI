# Native Buildroot integration template. Requires a complete ARM64 ROS underlay
# built with the SAME BSP/sysroot, not /opt/ros copied from Ubuntu Noble.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
if(NOT DEFINED ENV{ACG138_TOOLCHAIN_BIN} OR NOT DEFINED ENV{ACG138_SYSROOT})
  message(FATAL_ERROR "Set ACG138_TOOLCHAIN_BIN and ACG138_SYSROOT to the verified BSP SDK")
endif()
set(CMAKE_C_COMPILER "$ENV{ACG138_TOOLCHAIN_BIN}/aarch64-none-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "$ENV{ACG138_TOOLCHAIN_BIN}/aarch64-none-linux-gnu-g++")
set(CMAKE_SYSROOT "$ENV{ACG138_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH "$ENV{ACG138_SYSROOT};$ENV{ACG138_ROS_UNDERLAY}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# Run host Python/code generators; install matching target Python and libraries
# separately in the underlay. Never execute a target Python during a host build.
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
