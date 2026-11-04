// SPDX-License-Identifier: BSD-2-Clause
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "acg138_rplidar/scan_core.hpp"
#include "sl_lidar.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::string sdk_error(sl_result result) {
  std::ostringstream message;
  message << "SDK result 0x" << std::hex << static_cast<std::uint32_t>(result);
  return message.str();
}
using Steady = std::chrono::steady_clock;
}

class C1Node final : public rclcpp::Node {
 public:
  C1Node() : Node("acg138_rplidar") {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    const int baudrate = declare_parameter<int>("baudrate", 460800);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    expected_hz_ = declare_parameter<double>("scan_frequency", 10.0);
    const int bins = declare_parameter<int>("scan_bins", 500);
    scan_config_.range_min_m = static_cast<float>(declare_parameter<double>("range_min", 0.05));
    scan_config_.range_max_m = static_cast<float>(declare_parameter<double>("range_max", 12.0));
    grab_timeout_ms_ = declare_parameter<int>("grab_timeout_ms", 500);
    max_timeouts_ = declare_parameter<int>("max_consecutive_timeouts", 5);
    request_rpm_ = declare_parameter<bool>("request_motor_rpm", true);
    if (baudrate != 460800) throw std::invalid_argument("C1 baudrate must be 460800; no baud fallback is used");
    if (serial_port_.empty() || frame_id_.empty() || bins < 0 || bins > 8192 ||
        (bins != 0 && bins < 4) || grab_timeout_ms_ < 100 || grab_timeout_ms_ > 5000 ||
        max_timeouts_ < 1 || max_timeouts_ > 100) {
      throw std::invalid_argument("invalid serial/frame/bins/timeout parameter");
    }
    scan_config_.bins = static_cast<std::size_t>(bins);
    if (!std::isfinite(expected_hz_) || expected_hz_ < 8.0 || expected_hz_ > 12.0 ||
        scan_config_.range_min_m < 0.05F || scan_config_.range_max_m > 12.0F) {
      throw std::invalid_argument("C1: scan_frequency in [8,12], range_min >=0.05m, range_max <=12m");
    }
    auto check = scan_config_;
    if (check.bins == 0) check.bins = 500;
    (void)acg138_rplidar::make_scan({}, check);
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  }

  void connect_device() {
    auto channel_result = sl::createSerialPortChannel(serial_port_, 460800);
    if (!channel_result || *channel_result == nullptr)
      throw std::runtime_error("createSerialPortChannel: " + sdk_error(channel_result.err));
    channel_.reset(*channel_result);
    auto driver_result = sl::createLidarDriver();
    if (!driver_result || *driver_result == nullptr)
      throw std::runtime_error("createLidarDriver: " + sdk_error(driver_result.err));
    driver_.reset(*driver_result);
    require_ok(driver_->connect(channel_.get()), "connect " + serial_port_ + " @460800");
    connected_ = true;
    sl_lidar_response_device_info_t info{};
    require_ok(driver_->getDeviceInfo(info), "getDeviceInfo");
    sl_lidar_response_device_health_t health{};
    require_ok(driver_->getHealth(health), "getHealth");
    if (health.status == SL_LIDAR_STATUS_ERROR)
      throw std::runtime_error("C1 internal health error; check power/device before restart");
    if (health.status != SL_LIDAR_STATUS_OK && health.status != SL_LIDAR_STATUS_WARNING)
      throw std::runtime_error("unknown C1 health status");
    RCLCPP_INFO(get_logger(), "C1 port=%s baud=460800 SDK=%s model=0x%02x fw=%u.%u health=%u",
      serial_port_.c_str(), SL_LIDAR_SDK_VERSION, static_cast<unsigned>(info.model),
      static_cast<unsigned>(info.firmware_version >> 8),
      static_cast<unsigned>(info.firmware_version & 255), static_cast<unsigned>(health.status));

    sl::MotorCtrlSupport support = sl::MotorCtrlSupportNone;
    const auto capability = driver_->checkMotorCtrlSupport(support);
    sl_result motor_result;
    if (SL_IS_OK(capability) && support == sl::MotorCtrlSupportRpm && request_rpm_) {
      const auto rpm = static_cast<sl_u16>(std::lround(expected_hz_ * 60.0));
      motor_result = driver_->setMotorSpeed(rpm);
      RCLCPP_INFO(get_logger(), "SDK reports RPM control; requested %u rpm (not a measured frequency)",
        static_cast<unsigned>(rpm));
    } else {
      // SDK ultra_simple uses this; in PWM mode a Hertz value must not be passed as PWM.
      motor_result = driver_->setMotorSpeed();
      RCLCPP_INFO(get_logger(), "Using SDK default motor speed; scan_frequency=%.2f is an expectation",
        expected_hz_);
    }
    if (SL_IS_FAIL(motor_result)) {
      RCLCPP_WARN(get_logger(), "Motor command unsupported/failed (%s); trying proven scan API",
        sdk_error(motor_result).c_str());
    }
    start_scan();
  }

  ~C1Node() override {
    if (driver_ && connected_) {
      // Stop before destroying the channel; at most SDK's bounded command timeout.
      (void)driver_->stop(500);
      (void)driver_->setMotorSpeed(0);
      driver_->disconnect();
    }
  }

  void run() {
    connect_device();
    std::array<sl_lidar_response_measurement_node_hq_t, 8192> raw{};
    unsigned consecutive_timeouts = 0;
    std::uint64_t published = 0;
    auto last_receipt = Steady::time_point{};
    auto report_start = Steady::now();
    std::size_t report_scans = 0;
    std::size_t report_valid_bins = 0;
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      std::size_t count = raw.size();
      const auto result = driver_->grabScanDataHq(raw.data(), count,
        static_cast<sl_u32>(grab_timeout_ms_));
      const auto receipt_steady = Steady::now();
      const auto receipt_ros = now();  // Node clock also respects an explicit use_sim_time parameter.
      if (!rclcpp::ok()) break;
      if (SL_IS_FAIL(result) || count < 4 || count >= raw.size()) {
        ++consecutive_timeouts;
        RCLCPP_WARN(get_logger(), "No publishable complete revolution: %s count=%zu timeout_streak=%u",
          sdk_error(result).c_str(), count, consecutive_timeouts);
        // Do not repeat the preceding LaserScan or manufacture free space.
        if (consecutive_timeouts >= static_cast<unsigned>(max_timeouts_))
          throw std::runtime_error("C1 stream unavailable; supervisor may restart node");
        continue;
      }
      consecutive_timeouts = 0;
      const auto first_angle = raw.front().angle_z_q14;  // Preserve chronology before sorting.
      const double inter_receipt = last_receipt == Steady::time_point{} ? 0.0 :
        std::chrono::duration<double>(receipt_steady - last_receipt).count();
      last_receipt = receipt_steady;
      const double period = acg138_rplidar::scan_period_s(count, mode_.us_per_sample,
        inter_receipt, expected_hz_);
      const auto ascend_result = driver_->ascendScanData(raw.data(), count);
      if (SL_IS_FAIL(ascend_result) && ascend_result != SL_RESULT_OPERATION_FAIL)
        throw std::runtime_error("ascendScanData: " + sdk_error(ascend_result));
      // SL_RESULT_OPERATION_FAIL means all nodes invalid; publish one all-inf scan.
      std::vector<acg138_rplidar::HqPoint> points;
      points.reserve(count);
      for (std::size_t i = 0; i < count; ++i)
        points.push_back({raw[i].angle_z_q14, raw[i].dist_mm_q2, raw[i].quality});
      auto uniform = acg138_rplidar::make_scan(points, scan_config_);
      sensor_msgs::msg::LaserScan message;
      // With angle-sorted, CCW output the actual samples were collected in reverse
      // temporal order. There is no honest positive per-bin time_increment after
      // nearest-bin resampling, so publish 0 (no per-ray deskew claim).
      const double ray_delay = acg138_rplidar::first_output_ray_delay_s(first_angle, period);
      const auto estimated_ns = receipt_ros.nanoseconds() -
        static_cast<std::int64_t>(std::llround(ray_delay * 1e9));
      message.header.stamp = rclcpp::Time(std::max<std::int64_t>(0, estimated_ns),
        receipt_ros.get_clock_type());
      message.header.frame_id = frame_id_;
      message.angle_min = uniform.angle_min;
      message.angle_max = uniform.angle_max;
      message.angle_increment = uniform.angle_increment;
      message.scan_time = static_cast<float>(period);
      message.time_increment = 0.0F;
      message.range_min = scan_config_.range_min_m;
      message.range_max = scan_config_.range_max_m;
      message.ranges = std::move(uniform.ranges);
      message.intensities = std::move(uniform.intensities);
      publisher_->publish(message);
      ++published;
      ++report_scans;
      report_valid_bins += uniform.valid_bins;
      const double report_s = std::chrono::duration<double>(receipt_steady - report_start).count();
      if (report_s >= 5.0) {
        RCLCPP_INFO(get_logger(),
          "C1_SCAN published=%llu receipt_hz=%.2f sample_period_hz=%.2f raw_points=%zu bins=%zu valid_bins_mean=%.1f timestamp=host-estimated time_increment=0",
          static_cast<unsigned long long>(published), report_scans / report_s, 1.0 / period,
          count, message.ranges.size(), static_cast<double>(report_valid_bins) / report_scans);
        report_start = receipt_steady;
        report_scans = 0;
        report_valid_bins = 0;
      }
    }
  }

 private:
  static void require_ok(sl_result result, const std::string& context) {
    if (SL_IS_FAIL(result)) throw std::runtime_error(context + ": " + sdk_error(result));
  }

  void start_scan() {
    mode_ = sl::LidarScanMode{};
    auto result = driver_->startScan(false, true, 0, &mode_);
    if (SL_IS_FAIL(result)) {
      RCLCPP_WARN(get_logger(), "Typical scan failed (%s); checking advertised mode 0",
        sdk_error(result).c_str());
      (void)driver_->stop(500);
      std::vector<sl::LidarScanMode> modes;
      if (SL_IS_OK(driver_->getAllSupportedScanModes(modes))) {
        const auto zero_mode = std::find_if(modes.begin(), modes.end(),
          [](const sl::LidarScanMode& item) { return item.id == 0; });
        if (zero_mode != modes.end())
          result = driver_->startScanExpress(false, zero_mode->id, 0, &mode_);
      }
      if (SL_IS_FAIL(result)) {
        (void)driver_->stop(500);
        mode_ = sl::LidarScanMode{};
        result = driver_->startScan(false, false, 0, &mode_);
      }
    }
    require_ok(result, "all supported C1 scan startup attempts failed");
    if (std::isfinite(mode_.max_distance) && mode_.max_distance > scan_config_.range_min_m)
      scan_config_.range_max_m = std::min(scan_config_.range_max_m, mode_.max_distance);
    RCLCPP_INFO(get_logger(), "Scanning mode_id=%u sample_us=%.3f configured_range=[%.3f,%.3f]m",
      static_cast<unsigned>(mode_.id), mode_.us_per_sample,
      scan_config_.range_min_m, scan_config_.range_max_m);
  }

  std::string serial_port_;
  std::string frame_id_;
  double expected_hz_ = 10.0;
  int grab_timeout_ms_ = 500;
  int max_timeouts_ = 5;
  bool connected_ = false;
  bool request_rpm_ = true;
  acg138_rplidar::ScanConfig scan_config_;
  sl::LidarScanMode mode_{};
  // The driver must be destroyed before the channel, as required by SDK ownership.
  std::unique_ptr<sl::IChannel> channel_;
  std::unique_ptr<sl::ILidarDriver> driver_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<C1Node>();
    node->run();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("acg138_rplidar"), "%s", error.what());
    status = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
