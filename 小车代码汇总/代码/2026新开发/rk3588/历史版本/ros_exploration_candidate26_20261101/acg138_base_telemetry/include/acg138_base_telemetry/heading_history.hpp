#pragma once
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
namespace acg138 {
// Source time = kernel receive time minus FPGA-reported sensor age. No now()
// restamping, no extrapolation, and no direct subtraction of two device clocks.
class HeadingHistory {
 struct Sample {std::int64_t ns;double yaw;};
 std::deque<Sample> samples_;
 public:
 bool push(std::int64_t ns,double yaw){
  if(ns<=0||!std::isfinite(yaw)||(!samples_.empty()&&ns<=samples_.back().ns))return false;
  samples_.push_back({ns,yaw});
  while(samples_.size()>256||(!samples_.empty()&&ns-samples_.front().ns>2000000000LL))samples_.pop_front();
  return true;
 }
 std::int64_t first()const{return samples_.empty()?0:samples_.front().ns;}
 std::int64_t last()const{return samples_.empty()?0:samples_.back().ns;}
 std::optional<double> at(std::int64_t ns)const{
  if(samples_.empty()||ns<first()||ns>last())return {};
  for(std::size_t i=0;i<samples_.size();i++){
   const auto&b=samples_[i];if(ns==b.ns)return b.yaw;
   if(ns>b.ns)continue;
   const auto&a=samples_[i-1];const auto span=b.ns-a.ns;
   if(span>100000000LL)return {}; // Do not interpolate across missing IMU data.
   const double d=std::atan2(std::sin(b.yaw-a.yaw),std::cos(b.yaw-a.yaw));
   const double y=a.yaw+d*double(ns-a.ns)/span;
   return std::atan2(std::sin(y),std::cos(y));
  }
  return {};
 }
};
}
