#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
namespace acg138 {
struct RearClearance {bool observed=false;std::uint32_t travel_mm=0;unsigned bins=0;};
// Every 3deg sector of the rear120deg fan needs a finite return. This matches
// the C1 500-bin scan without requiring an impossible 120/120 one-degree
// coverage after occasional missing beams. No empty sector or no-return scan
// certifies space; all near returns veto. Rear body +200mm reserve is excluded.
inline RearClearance rear_clearance(const std::vector<float>& ranges,double a0,double da,double rmin,double rmax,double sensor_yaw){
 constexpr double pi=3.14159265358979323846;std::array<bool,40> bins{};double minimum=1e9;
 if(!std::isfinite(a0)||!std::isfinite(da)||da==0||std::abs(da)>pi/90||rmin<0||rmax<=rmin||!std::isfinite(sensor_yaw))return {};
 for(size_t i=0;i<ranges.size();++i){double r=ranges[i];if(!std::isfinite(r)||r<rmin||r>rmax)continue;
  double a=std::remainder(a0+i*da+sensor_yaw-pi,2*pi);
  if(a< -pi/3||a>=pi/3)continue;
  unsigned bin=unsigned((a+pi/3)*60/pi);if(bin>=40)continue;bins[bin]=true;minimum=std::min(minimum,r);
 }
 RearClearance out;for(bool b:bins)out.bins+=b;
 out.observed=out.bins==40;
 if(out.observed&&minimum>.3942)out.travel_mm=std::uint32_t(std::min(4500.,std::floor((minimum-.3942)*1000)));
 return out;
}
}
