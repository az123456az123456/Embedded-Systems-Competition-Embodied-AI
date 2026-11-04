// SPDX-License-Identifier: BSD-2-Clause
#include "acg138_rplidar/scan_core.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>

using namespace acg138_rplidar;
namespace {
std::size_t checks = 0;
void check(bool value, const char* message) {
  ++checks;
  if (!value) throw std::runtime_error(message);
}
void close(double a, double b, double tolerance, const char* message) {
  check(std::fabs(a - b) < tolerance, message);
}
template<class F> void throws(F fn, const char* message) {
  bool caught = false;
  try { fn(); } catch (const std::invalid_argument&) { caught = true; }
  check(caught, message);
}
}
int main() {
  try {
    close(sdk_clockwise_radians(16384), kPi / 2, 1e-12, "Q14 quarter-turn conversion");
    close(sdk_clockwise_radians(32768), kPi, 1e-12, "Q14 half-turn conversion");
    close(sdk_clockwise_radians(49152), 3*kPi/2, 1e-12, "Q14 three-quarter-turn conversion");
    check(ros_bin(0, 500) == 250, "hardware forward -> ROS +X");
    check(ros_bin(16384, 500) == 125, "hardware clockwise90 -> ROS right -90");
    check(ros_bin(32768, 500) == 0, "rear wraps to -pi");
    check(ros_bin(49152, 500) == 375, "hardware clockwise270 -> ROS left +90");
    auto scan = make_scan({{0,4000,80}, {16384,8000,160}, {32768,12000,240},
                            {49152,16000,252}}, {});
    check(scan.ranges.size() == 500, "fixed uniform bin count");
    close(scan.ranges[250], 1.0, 1e-6, "quarter-mm to metres");
    close(scan.ranges[125], 2.0, 1e-6, "right range");
    close(scan.ranges[0], 3.0, 1e-6, "rear range");
    close(scan.ranges[375], 4.0, 1e-6, "left range");
    check(scan.valid_bins == 4, "valid bin count");
    close(scan.intensities[250], 20.0, 1e-6, "quality is six high bits");
    close(scan.intensities[375], 63.0, 1e-6, "max quality");
    close(scan.angle_increment, 2*kPi/500, 1e-7, "uniform angles");
    close(scan.angle_max-scan.angle_min, scan.angle_increment*499, 1e-6, "N bins N-1 intervals");
    check(scan.angle_max < kPi, "no duplicated rear endpoint");
    auto nearest = make_scan({{0,12000,240}, {0,4000,80}, {1,8000,160}}, {});
    close(nearest.ranges[250], 1.0, 1e-6, "nearest echo wins bin collision");
    close(nearest.intensities[250], 20.0, 1e-6, "quality belongs to selected echo");
    check(nearest.valid_bins == 1 && nearest.collisions == 2, "collisions counted");
    auto invalid = make_scan({{0,0,252}, {16384,4000,0}, {32768,4000,3},
                              {49152,196,252}, {10000,48004,252}}, {});
    for (const auto range : invalid.ranges) check(std::isinf(range), "invalid is +inf");
    check(invalid.valid_bins == 0, "all invalid valid count");
    auto boundaries = make_scan({{0,200,252}, {16384,48000,252}}, {});
    close(boundaries.ranges[250], 0.05, 1e-7, "range_min included");
    close(boundaries.ranges[125], 12.0, 1e-6, "range_max included");
    ScanConfig dynamic;
    dynamic.bins = 0;
    auto variable = make_scan({{0,4000,4},{16384,4000,4},{32768,4000,4},{49152,4000,4}}, dynamic);
    check(variable.ranges.size() == 4 && variable.valid_bins == 4, "native count option");
    throws([&] { make_scan({}, dynamic); }, "empty dynamic scan fails");
    ScanConfig bad;
    bad.bins = 3;
    throws([&] { make_scan({}, bad); }, "too few bins fail");
    bad.bins = 8193;
    throws([&] { make_scan({}, bad); }, "oversized bins fail");
    bad = ScanConfig{}; bad.range_min_m = std::numeric_limits<float>::quiet_NaN();
    throws([&] { make_scan({}, bad); }, "NaN range fails");
    bad = ScanConfig{}; bad.range_max_m = 0.01F;
    throws([&] { make_scan({}, bad); }, "range order fails");
    for (std::uint32_t code=0; code<65536; ++code) {
      const auto bin=ros_bin(static_cast<std::uint16_t>(code),500);
      check(bin<500, "full uint16 angle domain stays in bounds");
      const double represented=-kPi+bin*(2*kPi/500);
      const double physical=wrap_pi(-sdk_clockwise_radians(static_cast<std::uint16_t>(code)));
      check(std::fabs(wrap_pi(represented-physical))<=kPi/500+1e-12,
        "nearest bin angular error at most half a bin");
    }
    close(scan_period_s(500,200.0,0.2,10),0.1,1e-12,"sample count period precedes scheduler wait");
    close(scan_period_s(0,0.0,0.125,10),0.125,1e-12,"receipt period fallback");
    close(scan_period_s(0,0.0,10,10),0.1,1e-12,"first sample configured fallback");
    throws([&] { scan_period_s(0,0,0,20); }, "C1 unsupported expectation rejected");
    close(first_output_ray_delay_s(0,0.1),0.05,1e-12,"rear ray acquired midway through scan");
    close(first_output_ray_delay_s(32768,0.1),0.1,1e-12,"rear scan start estimate");
    for (std::uint32_t code=0; code<65536; ++code) {
      const auto delay=first_output_ray_delay_s(static_cast<std::uint16_t>(code),0.1);
      check(delay>=0 && delay<=0.1,"timestamp estimate remains inside one revolution");
    }
    std::cout << "PASS scan_core checks=" << checks << " q14=90/16384 ROS=counterclockwise bins=uniform missing=inf nearest_echo=yes\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL scan_core checks=" << checks << " reason=" << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
