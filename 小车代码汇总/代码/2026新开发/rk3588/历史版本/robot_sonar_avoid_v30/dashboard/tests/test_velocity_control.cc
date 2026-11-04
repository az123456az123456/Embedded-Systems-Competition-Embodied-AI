#include "../phone/controller.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
using namespace robotphone;
int main(){
 Config config;config.left_motor_mask=10;Controller c(config);
 c.updateMotorLink(1000,true);c.updateVelocityProtocol(false,false);assert(c.arm(1,1000));
 assert(!c.driveVelocity(2,.3,0,1000));assert(!c.state().armed);assert(c.state().reason=="fpga_v28_velocity_required");
 c.updateMotorLink(1100,true);c.updateVelocityProtocol(true,false);assert(c.arm(3,1100));
 assert(c.driveVelocity(4,0,0,1100));assert(c.state().twist_mode&&c.state().armed);
 c.updateMotorLink(1300,true);c.updateVelocityProtocol(true,false);assert(c.tick(1300).armed); // zero is not a stall
 assert(!c.driveVelocity(5,.3,0,1300));assert(!c.state().armed&&c.state().reason=="softcore_not_ready");
 c.updateMotorLink(1400,true);c.updateVelocityProtocol(true,true);assert(c.arm(6,1400));
 assert(c.driveVelocity(7,.3,-1,1400));auto s=c.tick(1420);assert(s.twist_mode&&s.linear_mps==.3&&s.angular_rps==-1&&s.left==0&&s.right==0);
 assert(!c.driveVelocity(7,0,0,1420));assert(c.state().linear_mps==.3); // replay cannot rewrite target
 c.updateVelocityProtocol(true,false);assert(!c.state().armed&&c.state().linear_mps==0&&c.state().angular_rps==0);
 c.updateMotorLink(1500,true);c.updateVelocityProtocol(true,true);assert(c.arm(8,1500));assert(c.driveVelocity(9,.2,.1,1500));
 assert(c.driveWheels(10,.1,.2,1520));s=c.tick(1540);assert(!s.twist_mode&&s.linear_mps==0&&s.angular_rps==0); // manual takeover clears MOT4
 c.disarm(1560);assert(!c.state().twist_mode&&c.state().left==0&&c.state().right==0);
 c.updateMotorLink(1600,true);assert(c.arm(11,1600));assert(c.driveVelocity(12,0,0,1600));
 c.updateMotorLink(1900,true);assert(c.tick(1951).reason=="lease_expired");assert(!c.state().armed&&!c.state().twist_mode);
 c.updateMotorLink(2000,true);assert(c.arm(13,2000));assert(!c.driveVelocity(14,.3001,0,2000));assert(!c.state().armed);
 c.updateMotorLink(2100,true);assert(c.arm(15,2100));assert(!c.driveVelocity(16,0,std::numeric_limits<double>::quiet_NaN(),2100));assert(!c.state().armed);
 std::cout<<"PASS v28 physical velocity controller: capability, zero-ready handshake, ranges, stale/replay, lease, failure and manual takeover\n";
}
