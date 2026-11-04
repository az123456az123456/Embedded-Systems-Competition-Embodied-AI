#include "acg138_base_telemetry/heading_history.hpp"
#include "acg138_base_telemetry/odometry.hpp"
#include <iostream>
#include <cstdlib>
int main(){
 int n=0;auto need=[&](bool good){++n;if(!good){std::cerr<<"failed check "<<n<<'\n';std::exit(1);}};
 constexpr double pi=3.14159265358979323846;
 acg138::HeadingHistory h;
 need(!h.at(100));need(h.push(1000000000,179*pi/180));need(h.push(1020000000,-179*pi/180));
 need(!h.push(1020000000,0));need(!h.push(1010000000,0));
 need(std::abs(std::abs(*h.at(1010000000))-pi)<1e-10);
 need(!h.at(999999999));need(!h.at(1020000001));
 need(h.push(1400000000,0));need(!h.at(1200000000));
 acg138::WheelOdometry odom;
 std::array<std::int32_t,4> counts{};
 need(odom.update(counts,100,1,0,0,1.2));
 need(odom.update(counts,150,2,0,0,1.5));
 need(std::abs(odom.state().yaw-.3)<1e-12); // Pushing/rotating without wheel turns.
 need(std::abs(odom.state().x)<1e-12);
 counts={{-1000,-1000,-1000,-1000}};
 need(odom.update(counts,200,3,0,0,1.5));
 need(std::abs(odom.state().yaw-.3)<1e-12); // Wheel skid does not steer IMU yaw.
 need(!odom.update(counts,200,3,0,0,2.));need(std::abs(odom.state().yaw-.3)<1e-12);
 odom.reset();counts={};need(odom.update(counts,100,1,0,0,179*pi/180));
 need(odom.update(counts,150,2,0,0,-179*pi/180));need(std::abs(odom.state().yaw-2*pi/180)<1e-10);
 odom.reset();need(odom.update(counts,100,1,0,0,0.));
 counts={{-10000,10000,-10000,10000}};need(odom.update(counts,200,2,0,0,.5));
 const double d=10000*.20507/60527.7;
 need(std::abs(odom.state().x-d*std::sin(.5)/.5)<1e-12);
 need(std::abs(odom.state().y-d*(1-std::cos(.5))/.5)<1e-12);
 need(!odom.update(counts,250,3,0,0));need(odom.state().frozen); // No silent fallback.
 odom.reset();need(odom.update({},100,1,0,0,0.));
 need(!odom.update({},150,2,0,1,0.));need(odom.state().frozen);
 std::cout<<"{\"passed\":true,\"checks\":"<<n<<",\"motion_sent\":false}\n";
}
