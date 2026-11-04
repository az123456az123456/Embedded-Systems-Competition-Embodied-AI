// SPDX-License-Identifier: BSD-2-Clause
// Compiles against the real vendored SDK headers; no mock SDK or ROS stubs.
#include "sl_lidar.h"
#include <cstddef>
#include <type_traits>

static_assert(SL_LIDAR_SDK_VERSION_MAJOR == 2, "Expect the user's sl SDK v2 API");
static_assert(SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT == 2, "HQ quality format changed");
static_assert(std::is_same<decltype(sl_lidar_response_measurement_node_hq_t{}.angle_z_q14),sl_u16>::value,
  "HQ angle must be unsigned 16-bit quarter-turn Q14");

void sdk_compile_contract(sl::ILidarDriver& driver, sl::IChannel& channel) {
  auto factory = sl::createLidarDriver();
  auto serial = sl::createSerialPortChannel("/dev/ttyUSB0", 460800);
  (void)factory; (void)serial;
  sl::LidarScanMode mode{};
  sl::MotorCtrlSupport support{};
  std::vector<sl::LidarScanMode> modes;
  sl_lidar_response_measurement_node_hq_t points[8192]{};
  std::size_t count = 8192;
  (void)driver.connect(&channel);
  (void)driver.checkMotorCtrlSupport(support);
  (void)driver.setMotorSpeed(600);
  (void)driver.startScan(false, true, 0, &mode);
  (void)driver.getAllSupportedScanModes(modes);
  (void)driver.startScanExpress(false, 0, 0, &mode);
  (void)driver.grabScanDataHq(points, count, 500);
  (void)driver.ascendScanData(points, count);
  (void)driver.stop(500);
  driver.disconnect();
}
