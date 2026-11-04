#include "acg138_base_telemetry/rx_stamp.hpp"
#include "acg138_base_telemetry/odometry.hpp"
#include <cassert>
#include <iostream>
int main(){
 acg138::OdometryPublicationClock clock;
 // Exact live seq13244/13245 timestamps, retained in timestamp-probe.console.
 const std::int64_t shared=1609513672188301666LL;
 assert(clock.observe(shared)==1);
 acg138::WheelOdometry odom(acg138::OdometryConfig{});
 assert(odom.update({0,0,0,0},100,13244,0,0));
 assert(odom.update({-10,10,-10,10},120,13245,0,0));
 assert(clock.observe(shared)==0&&!odom.state().frozen);
 const double integrated=odom.state().x;assert(integrated>0);
 assert(odom.update({-20,20,-20,20},140,13246,0,0));
 assert(clock.observe(shared+20000000)==1&&odom.state().x>integrated);
 assert(clock.observe(shared+19999999)==-1); // Even 1ns back is still rejected.
 assert(clock.observe(shared+40000000)==1);
 assert(clock.observe(shared+40000000)==0);
 std::cout<<"{\"cases\":10,\"failed\":0,\"live_timestamp_pair_replayed\":true,\"backwards_still_rejected\":true,\"fake_now_used\":false}\n";
}
