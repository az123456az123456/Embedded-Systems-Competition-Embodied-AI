#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace acg138 {
// A constant mounting/reference correction, applied ONCE at the navigation
// input boundary. Map XY, target coordinates, source stamps and TF stay intact.
// Zero is the generic ROS default. This vehicle's confirmed camera-facing
// forward axis is TF yaw + pi, matching its existing phone/RK arrow display.
class NavigationHeading {
 public:
  static constexpr double pi=3.14159265358979323846;
  explicit NavigationHeading(double offset=0):offset_(offset){
    if(!std::isfinite(offset)||std::abs(offset)>pi)
      throw std::invalid_argument("navigation_heading_offset_rad must be finite in [-pi,pi]");
  }
  double radians(double tf_yaw)const{
    if(!std::isfinite(tf_yaw))throw std::invalid_argument("nonfinite TF yaw");
    double a=std::remainder(tf_yaw+offset_,2*pi);
    return a>=pi?a-2*pi:a;
  }
  std::int32_t microradians(double tf_yaw)const{
    return static_cast<std::int32_t>(std::lround(radians(tf_yaw)*1000000.));
  }
  double offset()const{return offset_;}
 private:
  double offset_;
};
}
