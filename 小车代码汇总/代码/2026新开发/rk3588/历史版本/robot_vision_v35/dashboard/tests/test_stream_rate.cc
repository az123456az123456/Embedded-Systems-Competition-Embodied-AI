#include "stream_rate.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>
int main(){
 using robotphone::streamTargetFps;unsigned checks=0;
 auto same=[&](double expected,double actual){++checks;assert(std::abs(expected-actual)<1e-9);};
 same(30,streamTargetFps(true,30,40,40,0));same(40,streamTargetFps(true,40,40,40,0));
 same(15,streamTargetFps(true,40,40,40,15));same(8,streamTargetFps(true,8,40,40,15));
 same(30,streamTargetFps(false,0,30,40,0));same(40,streamTargetFps(false,0,0,40,0));
 same(1,streamTargetFps(true,0,40,40,0));same(1,streamTargetFps(true,-3,40,40,0));
 same(1,streamTargetFps(true,std::numeric_limits<double>::quiet_NaN(),40,40,0));
 same(1,streamTargetFps(true,std::numeric_limits<double>::infinity(),40,40,0));
 same(1,streamTargetFps(false,0,0,0,0));same(60,streamTargetFps(true,90,120,120,0));
 same(29.97,streamTargetFps(true,29.97,40,40,0));same(20,streamTargetFps(true,40,20,40,0));
 same(20,streamTargetFps(true,40,40,20,0));
 std::printf("PASS stream-rate %u policy checks: complete throughput, explicit caps, detect-off, fractional rate, invalid metrics; no network/hardware\n",checks);
}
