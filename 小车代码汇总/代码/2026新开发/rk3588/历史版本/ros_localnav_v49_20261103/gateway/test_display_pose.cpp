#include "display_pose.h"
#include <cassert>
#include <cstdio>
int main(){
 robotnav::DisplayPose p;
 p.offer(0,0,0,200000000,"old_FPGA");p.offer(1,2,1.2,30000000,"live_IMU");
 assert(p.valid&&p.x==1&&p.yaw==1.2&&p.age_ns==30000000);
 p.offer(3,4,2,90000000,"late_scan_match");assert(p.x==1);
 p.offer(0,0,0,250000001,"stale");assert(p.x==1);
 p.offer(NAN,0,0,0,"nan");assert(p.x==1);
 robotnav::DisplayPose q;q.offer(1,2,3,250000001,"old");assert(!q.valid);
 q.offer(1,2,3,250000000,"boundary");assert(q.valid);
 constexpr double pi=3.14159265358979323846;
 unsigned headings=0;
 for(int k=-180;k<=180;++k){
  double raw=k*pi/180.,physical=std::atan2(std::sin(raw+pi),std::cos(raw+pi)),a=0;
  assert(robotnav::fpga_display_heading(physical,pi,123,123,a));
  assert(std::abs(std::atan2(std::sin(a-raw),std::cos(a-raw)))<1e-12);
  robotnav::DisplayPose r;r.offer(1,2,raw,20000000,"ROS");r.offer(1,2,a,10000000,"FPGA");
  assert(std::abs(std::cos(r.yaw)-std::cos(raw))<1e-12);++headings;
 }
 double y=9;assert(!robotnav::fpga_display_heading(0,pi,123,122,y));assert(y==9);
 assert(!robotnav::fpga_display_heading(0,pi,0,0,y));
 assert(!robotnav::fpga_display_heading(0,pi+.01,123,123,y));
 assert(!robotnav::fpga_display_heading(NAN,0,123,123,y));
 assert(robotnav::fpga_display_heading(.4,0,123,123,y));assert(std::abs(y-.4)<1e-12);
 std::printf("{\"passed\":true,\"age_checks\":6,\"heading_source_switch_checks\":%u,\"metadata_checks\":5,\"control_path_modified\":false}\n",headings);
}
