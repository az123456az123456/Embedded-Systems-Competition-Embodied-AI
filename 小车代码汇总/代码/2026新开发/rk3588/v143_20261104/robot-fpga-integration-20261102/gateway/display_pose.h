#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
namespace robotnav {
// FPGA reports the calibrated physical heading. Existing UI renderers expect
// the original TF heading and apply their own confirmed 180-degree arrow offset.
// Only matching-session metadata can undo that offset for display; never alter
// the FPGA state object used by authorization/control.
inline bool fpga_display_heading(double physical,double offset,uint64_t state_session,uint64_t metadata_session,double& yaw){
 constexpr double pi=3.14159265358979323846;
 if(!state_session||state_session!=metadata_session||!std::isfinite(physical)||!std::isfinite(offset)||std::abs(offset)>pi)return false;
 yaw=std::atan2(std::sin(physical-offset),std::cos(physical-offset));return true;
}
// Display only: choose the newest honest source stamp, never extrapolate.
// This cannot grant motor authority or alter the execution pose.
struct DisplayPose {
 double x=0,y=0,yaw=0;
 uint64_t age_ns=std::numeric_limits<uint64_t>::max();
 const char* source="unavailable";
 bool valid=false;
 void offer(double px,double py,double a,uint64_t age,const char* name){
  if(age>250000000||!std::isfinite(px)||!std::isfinite(py)||!std::isfinite(a))return;
  if(valid&&age>=age_ns)return;
  x=px;y=py;yaw=a;age_ns=age;source=name;valid=true;
 }
};
}
