#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace acg138 {
constexpr unsigned kSide = 64, kCells = kSide * kSide;
using Bitmap = std::array<uint8_t, kCells / 8>;
struct Pose { double x = 0, y = 0, yaw = 0; };
struct Grid {
  uint32_t width = 0, height = 0;
  double resolution = .05;
  Pose origin;
  std::vector<int8_t> cells;  // ROS convention: -1 unknown, 0..100 occupancy.
};
struct CropConfig {
  double resolution = .1;
  // User-selected total map inflation: 0.15 m, with no added margin.
  // This is a planning parameter, not a measured body dimension.
  double footprint_radius = .15, safety_margin = 0.;
  int free_threshold = 25;
  double body_width = .30, body_front = .1589, body_rear = .1942;
  bool self_footprint_exemption = true;
};
struct LocalGrid {
  Bitmap blocked{};
  double resolution = .1, origin_x = 0, origin_y = 0;
  unsigned free_cells = 0;
  std::vector<int8_t> occupancy() const;
  bool is_blocked(unsigned cell) const;
  uint16_t cell(double x, double y) const;
  Pose center(uint16_t cell) const;
};
// Conservative area-overlap resampling. Every source cell intersecting a
// target cell must be observed free. Unknown, uncertain and out-of-map cells
// stay blocked. Observed-free squares overlapping the current body are exempt
// from inflation only; original occupied/unknown squares are never cleared.
LocalGrid crop_and_inflate(const Grid &, const Pose &, const CropConfig &);
std::string json_escape(const std::string &);
}  // namespace acg138
