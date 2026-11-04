#include "acg138_fpga_map_bridge/navigation_heading.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using acg138::NavigationHeading;
int main(){
 const double pi=NavigationHeading::pi;
 NavigationHeading zero,vehicle(pi);
 unsigned samples=0;
 // Independent oracle: existing phone's screen arrow is (-cos(yaw),+sin(yaw));
 // map screen Y points down. A physical map heading must be (-cos,-sin).
 for(int k=-3600;k<=3600;++k){
  double raw=k*pi/1800.,body=vehicle.radians(raw);
  assert(body>=-pi&&body<pi);
  assert(std::abs(std::cos(body)+std::cos(raw))<1e-12);
  assert(std::abs(std::sin(body)+std::sin(raw))<1e-12);
  auto wire=vehicle.microradians(raw);
  assert(wire>=-3141593&&wire<=3141593);
  assert(std::abs(std::atan2(std::sin(wire/1e6-body),std::cos(wire/1e6-body)))<=.500001e-6);
  assert(std::abs(std::cos(zero.radians(raw))-std::cos(raw))<1e-12);
  ++samples;
 }
 for(double bad:{std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),pi+.001}){
  bool rejected=false;try{NavigationHeading invalid(bad);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
 }
 bool rejected=false;try{vehicle.radians(std::numeric_limits<double>::quiet_NaN());}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
 std::cout<<"{\"case\":\"phone_arrow_to_NAV3_heading\",\"yaw_samples\":"<<samples<<",\"invalid_inputs_rejected\":4,\"pass\":true}\n";
}
