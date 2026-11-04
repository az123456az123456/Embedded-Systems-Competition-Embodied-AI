// SPDX-License-Identifier: BSD-2-Clause
// Native Buildroot diagnostic: no ROS, FPGA network sockets, or vehicle controls.
#include "sl_lidar.h"
#include "acg138_rplidar/scan_core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
volatile std::sig_atomic_t stopping = 0;
void interrupt_handler(int) { stopping = 1; }
using Clock = std::chrono::steady_clock;
constexpr int kBaud = 460800;

void require_ok(sl_result result, const char* context) {
  if (SL_IS_FAIL(result)) {
    char error[256];
    std::snprintf(error, sizeof(error), "%s failed SDK_result=0x%08x", context,
      static_cast<unsigned>(result));
    throw std::runtime_error(error);
  }
}

struct Session {
  std::unique_ptr<sl::IChannel> channel;
  std::unique_ptr<sl::ILidarDriver> driver;
  bool connected = false;
  ~Session() {
    if (driver && connected) {
      // This stops the LIDAR scanner only; there is no vehicle motor interface.
      (void)driver->stop(500);
      (void)driver->setMotorSpeed(0);
      driver->disconnect();
    }
  }
};

sl::LidarScanMode begin_scan(sl::ILidarDriver& driver) {
  sl::MotorCtrlSupport support = sl::MotorCtrlSupportNone;
  const auto capability = driver.checkMotorCtrlSupport(support);
  sl_result motor;
  if (SL_IS_OK(capability) && support == sl::MotorCtrlSupportRpm) {
    motor = driver.setMotorSpeed(600);
    std::printf("LIDAR_MOTOR control=rpm requested_rpm=600 measured=no\n");
  } else {
    motor = driver.setMotorSpeed();
    std::printf("LIDAR_MOTOR control=sdk_default requested_hz=10 measured=no\n");
  }
  if (SL_IS_FAIL(motor))
    std::fprintf(stderr, "WARN LIDAR motor command result=0x%08x; attempting scan\n", static_cast<unsigned>(motor));
  sl::LidarScanMode mode{};
  auto result = driver.startScan(false, true, 0, &mode);
  if (SL_IS_FAIL(result)) {
    std::fprintf(stderr, "WARN typical scan failed=0x%08x; examining advertised mode 0\n", static_cast<unsigned>(result));
    (void)driver.stop(500);
    std::vector<sl::LidarScanMode> modes;
    if (SL_IS_OK(driver.getAllSupportedScanModes(modes))) {
      const auto zero = std::find_if(modes.begin(), modes.end(),
        [](const sl::LidarScanMode& item) { return item.id == 0; });
      if (zero != modes.end()) result = driver.startScanExpress(false, zero->id, 0, &mode);
    }
    if (SL_IS_FAIL(result)) {
      (void)driver.stop(500);
      mode = sl::LidarScanMode{};
      result = driver.startScan(false, false, 0, &mode);
    }
  }
  require_ok(result, "start C1 scan");
  return mode;
}

void usage() {
  std::puts("Usage: lidar-scan-probe.aarch64 [--port /dev/ttyUSB0] [--seconds 10]\n"
    "       lidar-scan-probe.aarch64 --version\n"
    "C1 baudrate is fixed at 460800. No ROS, FPGA network access, or vehicle motor command.\n"
    "The tool starts/stops the LIDAR scanner and reports actual received scans, not a fake LaserScan.");
}
}

int main(int argc, char** argv) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  try {
    std::string port = "/dev/ttyUSB0";
    double seconds = 10.0;
    for (int i=1; i<argc; ++i) {
      if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
        usage(); return 0;
      }
      if (std::strcmp(argv[i], "--version") == 0) {
#if defined(__aarch64__)
        constexpr const char* architecture = "aarch64";
#else
        constexpr const char* architecture = "host-test";
#endif
        std::printf("lidar-scan-probe v37 SDK=%s baud=460800 architecture=%s\n",
          SL_LIDAR_SDK_VERSION, architecture);
        return 0;
      }
      if (std::strcmp(argv[i], "--port") == 0 && i+1<argc) {
        port = argv[++i];
      } else if (std::strcmp(argv[i], "--seconds") == 0 && i+1<argc) {
        char* end = nullptr;
        seconds = std::strtod(argv[++i], &end);
        if (!end || *end || !std::isfinite(seconds) || seconds<1.0 || seconds>3600.0)
          throw std::invalid_argument("--seconds must be in [1,3600]");
      } else {
        throw std::invalid_argument(std::string("unknown or incomplete argument: ") + argv[i]);
      }
    }
    if (port.empty() || port.front() != '/') throw std::invalid_argument("--port must be an absolute device path");
    std::signal(SIGINT, interrupt_handler);
    std::signal(SIGTERM, interrupt_handler);
    Session session;
    auto channel = sl::createSerialPortChannel(port, kBaud);
    if (!channel || *channel == nullptr) throw std::runtime_error("create serial channel failed");
    session.channel.reset(*channel);
    auto driver = sl::createLidarDriver();
    if (!driver || *driver == nullptr) throw std::runtime_error("create SDK driver failed");
    session.driver.reset(*driver);
    require_ok(session.driver->connect(session.channel.get()), "connect serial @460800");
    session.connected = true;
    sl_lidar_response_device_info_t info{};
    require_ok(session.driver->getDeviceInfo(info), "getDeviceInfo");
    sl_lidar_response_device_health_t health{};
    require_ok(session.driver->getHealth(health), "getHealth");
    if (health.status != SL_LIDAR_STATUS_OK && health.status != SL_LIDAR_STATUS_WARNING)
      throw std::runtime_error("LIDAR internal health error");
    std::printf("PROBE version=v37 SDK=%s port=%s baud=460800 seconds=%.3f model=0x%02x firmware=%u.%u health=%u\n",
      SL_LIDAR_SDK_VERSION, port.c_str(), seconds, static_cast<unsigned>(info.model),
      static_cast<unsigned>(info.firmware_version >> 8), static_cast<unsigned>(info.firmware_version & 255),
      static_cast<unsigned>(health.status));
    if (stopping) return 130;
    const auto mode = begin_scan(*session.driver);
    std::printf("MODE id=%u sample_us=%.3f mode_max_distance_m=%.3f\n",
      static_cast<unsigned>(mode.id), mode.us_per_sample, mode.max_distance);
    const auto started = Clock::now();
    std::array<sl_lidar_response_measurement_node_hq_t, 8192> nodes{};
    std::size_t scans=0, scans_with_echo=0, points=0, valid=0, failures=0;
    while (!stopping && std::chrono::duration<double>(Clock::now()-started).count()<seconds) {
      std::size_t count = nodes.size();
      const auto result = session.driver->grabScanDataHq(nodes.data(), count, 500);
      if (SL_IS_FAIL(result) || count<4 || count>=nodes.size()) {
        ++failures;
        std::fprintf(stderr, "SCAN_WAIT result=0x%08x count=%zu failures=%zu\n", static_cast<unsigned>(result), count, failures);
        continue;
      }
      const auto sorted = session.driver->ascendScanData(nodes.data(), count);
      if (SL_IS_FAIL(sorted) && sorted != SL_RESULT_OPERATION_FAIL)
        require_ok(sorted, "ascendScanData");
      ++scans;
      points += count;
      std::size_t valid_this=0;
      double first_sdk_deg=0, first_ros_deg=0, first_distance_m=0;
      for (std::size_t i=0; i<count; ++i) {
        const double distance = nodes[i].dist_mm_q2 / 4000.0;
        if ((nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) == 0 || distance<0.05 || distance>12.0)
          continue;
        if (!valid_this) {
          const double sdk_rad = acg138_rplidar::sdk_clockwise_radians(nodes[i].angle_z_q14);
          first_sdk_deg = sdk_rad * 180.0 / acg138_rplidar::kPi;
          first_ros_deg = acg138_rplidar::wrap_pi(-sdk_rad) * 180.0 / acg138_rplidar::kPi;
          first_distance_m = distance;
        }
        ++valid_this;
      }
      if (valid_this) ++scans_with_echo;
      valid += valid_this;
      const double elapsed = std::chrono::duration<double>(Clock::now()-started).count();
      std::printf("SCAN index=%zu raw_points=%zu valid_points=%zu elapsed_s=%.3f actual_receipt_hz=%.3f first_valid_sdk_clockwise_deg=%.3f first_valid_ROS_ccw_deg=%.3f first_valid_range_m=%.3f\n",
        scans, count, valid_this, elapsed, scans/elapsed, first_sdk_deg, first_ros_deg, first_distance_m);
    }
    const double elapsed = std::chrono::duration<double>(Clock::now()-started).count();
    std::printf("SUMMARY scans=%zu scans_with_echo=%zu raw_points=%zu valid_points=%zu read_failures=%zu elapsed_s=%.3f actual_receipt_hz=%.3f interrupted=%s\n",
      scans, scans_with_echo, points, valid, failures, elapsed, elapsed>0?scans/elapsed:0.0, stopping?"yes":"no");
    return scans ? 0 : 2;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "ERROR %s\n", error.what());
    return 1;
  }
}
