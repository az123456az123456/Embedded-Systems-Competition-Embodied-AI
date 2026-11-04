#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
namespace acg138 {
struct OdometryConfig {
 double circumference_m=.20507,cpr=60527.7,track_m=.180635,max_wheel_mps=1.5,max_gap_s=.75;
 unsigned left_mask=10;
 std::array<int,4> signs{{-1,1,-1,1}};
};
struct OdometryState {double x=0,y=0,yaw=0,v=0,w=0,dt=0;bool initialized=false,frozen=false;std::string reason="waiting_encoder";};
class WheelOdometry {
 public:
 explicit WheelOdometry(OdometryConfig c={}):c_(c){
  unsigned count=0;for(unsigned b=0;b<4;b++)count+=(c.left_mask>>b)&1;
  if(count!=2||c.left_mask>=15||!std::isfinite(c.circumference_m)||!std::isfinite(c.cpr)||!std::isfinite(c.track_m)||c.circumference_m<=0||c.cpr<=0||c.track_m<=0||!std::isfinite(c.max_wheel_mps)||c.max_wheel_mps<=0||!std::isfinite(c.max_gap_s)||c.max_gap_s<=0)throw std::invalid_argument("invalid odometry geometry");
  for(int s:c.signs)if(s!=1&&s!=-1)throw std::invalid_argument("encoder signs must be +/-1");
 }
 void reset(){s_={};}
 void freeze(const std::string& why){s_.frozen=true;s_.v=s_.w=0;s_.reason=why;}
 const OdometryState& state()const{return s_;}
 // Counts are absolute signed32 snapshots. Modular subtraction handles natural overflow.
 bool update(const std::array<std::int32_t,4>& counts,std::uint32_t ms,std::uint32_t seq,unsigned invert,unsigned illegal){
  if(s_.frozen)return false;
  if(invert||illegal){freeze(invert?"unexpected_encoder_invert_mask":"encoder_illegal_transition");return false;}
  if(!s_.initialized){last_=counts;ms_=ms;seq_=seq;s_.initialized=true;s_.reason="encoder_baseline";return true;}
  const auto ds=seq-seq_;if(!ds||ds>=0x80000000u)return false;
  const auto elapsed=ms-ms_;if(!elapsed)return false;
  if(elapsed>=0x80000000u){freeze("fpga_clock_reset");return false;}
  const double dt=elapsed/1000.;if(dt>c_.max_gap_s){freeze("encoder_gap_requires_slam_restart");return false;}
  double left=0,right=0;
  for(unsigned i=0;i<4;i++){
   const auto u=std::uint32_t(counts[i])-std::uint32_t(last_[i]);
   const std::int64_t delta=u<=0x7fffffffu?u:std::int64_t(u)-4294967296LL;
   const double d=delta*c_.signs[i]*c_.circumference_m/c_.cpr;
   if(std::abs(d)>c_.max_wheel_mps*dt+.01){freeze("encoder_jump_requires_slam_restart");return false;}
   ((c_.left_mask>>i)&1?left:right)+=d*.5;
  }
  const double d=(left+right)*.5,dyaw=(right-left)/c_.track_m;
  // Exact constant-curvature SE(2) integration, stable at zero yaw.
  const double half=dyaw*.5,scale=std::abs(half)<1e-8?1.-half*half/6.:std::sin(half)/half;
  s_.x+=d*scale*std::cos(s_.yaw+half);s_.y+=d*scale*std::sin(s_.yaw+half);
  s_.yaw=std::atan2(std::sin(s_.yaw+dyaw),std::cos(s_.yaw+dyaw));s_.v=d/dt;s_.w=dyaw/dt;s_.dt=dt;s_.reason="valid";
  last_=counts;ms_=ms;seq_=seq;return true;
 }
 private:OdometryConfig c_;OdometryState s_;std::array<std::int32_t,4>last_{};std::uint32_t ms_=0,seq_=0;
};
}
