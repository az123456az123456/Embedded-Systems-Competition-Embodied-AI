################################################################################
# Advanced overlay example. ROS core/dependencies are prerequisites, not supplied.
################################################################################
ACG138_RPLIDAR_VERSION = 0.37.0
ACG138_RPLIDAR_SITE = $(call qstrip,$(BR2_PACKAGE_ACG138_RPLIDAR_SOURCE))
ACG138_RPLIDAR_SITE_METHOD = local
ACG138_RPLIDAR_LICENSE = BSD-2-Clause
ACG138_RPLIDAR_LICENSE_FILES = vendor/LICENSE-Slamtec.txt
ACG138_RPLIDAR_DEPENDENCIES = host-python3
ACG138_RPLIDAR_INSTALL_STAGING = YES
ACG138_RPLIDAR_CONF_ENV = PYTHONPATH=$(call qstrip,$(BR2_PACKAGE_ACG138_ROS_HOST_PYTHONPATH))
ACG138_RPLIDAR_CONF_OPTS = -DBUILD_TESTING=OFF -DACG138_CORE_ONLY=OFF \
	-DCMAKE_PREFIX_PATH=$(call qstrip,$(BR2_PACKAGE_ACG138_ROS_TARGET_UNDERLAY)) \
	-DPython3_EXECUTABLE=$(HOST_DIR)/bin/python3

define ACG138_RPLIDAR_CHECK_UNDERLAY
	test -f $(call qstrip,$(BR2_PACKAGE_ACG138_ROS_TARGET_UNDERLAY))/share/rclcpp/cmake/rclcppConfig.cmake || \
		{ echo 'Port ROS 2 core into this same Buildroot ABI first; Ubuntu ROS binaries are not a native underlay.'; exit 1; }
endef
ACG138_RPLIDAR_PRE_CONFIGURE_HOOKS += ACG138_RPLIDAR_CHECK_UNDERLAY

$(eval $(cmake-package))
