// SPDX-License-Identifier: BSD-2-Clause
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace acg138_rplidar {

constexpr double kPi = 3.141592653589793238462643383279502884;

// An explicit SDK-neutral representation makes conversion independently testable.
struct HqPoint {
  std::uint16_t angle_z_q14;
  std::uint32_t dist_mm_q2;
  std::uint8_t quality;
};

struct ScanConfig {
  std::size_t bins = 500;  // 5000 samples/s / 10 revolutions/s, C1 typical.
  float range_min_m = 0.05F;
  float range_max_m = 12.0F;
  // The C1 mechanical forward axis is laser +X. Use TF for installation yaw.
};

struct UniformScan {
  float angle_min;
  float angle_max;
  float angle_increment;
  std::vector<float> ranges;
  std::vector<float> intensities;
  std::size_t valid_bins = 0;
  std::size_t collisions = 0;
};

inline double sdk_clockwise_radians(std::uint16_t angle_z_q14) {
  // Official SL SDK / ultra_simple: degrees = angle_z_q14 * 90 / 16384.
  // Dividing by 16384 alone (degrees) gives a 90-fold angular error.
  return static_cast<double>(angle_z_q14) * kPi / 32768.0;
}

inline double wrap_pi(double radians) {
  double wrapped = std::fmod(radians + kPi, 2.0 * kPi);
  if (wrapped < 0.0) wrapped += 2.0 * kPi;
  return wrapped - kPi;
}

inline std::size_t ros_bin(std::uint16_t sdk_angle, std::size_t bins) {
  if (bins < 4 || bins > 8192) throw std::invalid_argument("scan_bins outside [4,8192]");
  // C1: clockwise positive; ROS REP-103: +X forward, +Y left, CCW positive.
  const double angle = wrap_pi(-sdk_clockwise_radians(sdk_angle));
  const auto index = static_cast<std::size_t>(std::llround(
      (angle + kPi) * static_cast<double>(bins) / (2.0 * kPi)));
  return index % bins;
}

inline UniformScan make_scan(const std::vector<HqPoint>& points, ScanConfig config) {
  if (config.bins == 0) config.bins = points.size();
  if (config.bins < 4 || config.bins > 8192 ||
      !std::isfinite(config.range_min_m) || !std::isfinite(config.range_max_m) ||
      config.range_min_m <= 0 || config.range_max_m <= config.range_min_m) {
    throw std::invalid_argument("invalid LaserScan configuration");
  }
  UniformScan out;
  out.angle_min = static_cast<float>(-kPi);
  out.angle_increment = static_cast<float>(2.0 * kPi / config.bins);
  // Do not duplicate the -pi ray at +pi. N bins cover one turn, N-1 intervals.
  out.angle_max = out.angle_min + static_cast<float>(config.bins - 1) * out.angle_increment;
  out.ranges.assign(config.bins, std::numeric_limits<float>::infinity());
  out.intensities.assign(config.bins, 0.0F);
  for (const auto& point : points) {
    const std::uint8_t quality = point.quality >> 2;
    const float range_m = static_cast<float>(static_cast<double>(point.dist_mm_q2) / 4000.0);
    if (quality == 0 || point.dist_mm_q2 == 0 ||
        range_m < config.range_min_m || range_m > config.range_max_m) continue;
    const auto bin = ros_bin(point.angle_z_q14, config.bins);
    if (std::isfinite(out.ranges[bin])) ++out.collisions;
    // Never average a near obstacle away or invent an echo in an empty bin.
    if (range_m < out.ranges[bin]) {
      if (!std::isfinite(out.ranges[bin])) ++out.valid_bins;
      out.ranges[bin] = range_m;
      out.intensities[bin] = quality;
    }
  }
  return out;
}

inline double scan_period_s(std::size_t node_count, double us_per_sample,
                            double inter_receipt_s, double expected_hz) {
  // A complete SDK scan has every measurement including invalid returns.
  // Its sample period is preferable to USB/scheduler wait duration.
  const double sample_period = static_cast<double>(node_count) * us_per_sample * 1e-6;
  if (std::isfinite(sample_period) && sample_period >= 0.04 && sample_period <= 0.5)
    return sample_period;
  if (std::isfinite(inter_receipt_s) && inter_receipt_s >= 0.04 && inter_receipt_s <= 0.5)
    return inter_receipt_s;
  if (!std::isfinite(expected_hz) || expected_hz < 8.0 || expected_hz > 12.0)
    throw std::invalid_argument("C1 expected scan_frequency must be [8,12] Hz");
  return 1.0 / expected_hz;
}

inline double first_output_ray_delay_s(std::uint16_t first_sdk_angle, double period_s) {
  // SDK first point is scan start. Output bin zero is ROS -pi => SDK +pi.
  double clockwise_offset = std::fmod(kPi - sdk_clockwise_radians(first_sdk_angle)
                                      + 2.0 * kPi, 2.0 * kPi);
  // Receipt is approximately end of the complete revolution. This is an estimate;
  // grabScanDataHq has no device timestamp or USB latency compensation.
  return period_s * (1.0 - clockwise_offset / (2.0 * kPi));
}

}  // namespace acg138_rplidar
