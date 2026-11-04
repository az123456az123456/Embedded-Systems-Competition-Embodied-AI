// Offline integration only: production navigation + the unchanged FPGA A* C
// source compiled for the host. A fully known simulator map is explicitly
// imported; this does not give the live three-sonar adapter a free-space prior.
#include "../navcore.h"
extern "C" {
#include "astar.h"
}
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace robot_nav;
namespace {
constexpr double dt=.02;
struct Scene {
  bool obstacle=false;
  std::vector<int8_t> cells()const{
    std::vector<int8_t> out(4096,0);
    if(obstacle)for(int y=0;y<64;++y)for(int x=0;x<64;++x){
      double wx=-3.2+(x+.5)*.1,wy=-3.2+(y+.5)*.1;
      if(wx>=.4&&wx<=.7&&std::abs(wy)<=.1)out[y*64+x]=1;
    }
    return out;
  }
  double range(const Pose&p,const std::array<double,3>&mount)const{
    if(!obstacle)return 4.5;
    double co=std::cos(p.yaw),si=std::sin(p.yaw);
    double sx=p.x+mount[0]*co-mount[1]*si,sy=p.y+mount[0]*si+mount[1]*co;
    double angle=p.yaw+mount[2],dx=std::cos(angle),dy=std::sin(angle);
    double enter=0,exit=4.5;
    for(auto slab:{std::array<double,4>{sx,dx,.4,.7},{sy,dy,-.1,.1}}){
      if(std::abs(slab[1])<1e-9){if(slab[0]<slab[2]||slab[0]>slab[3])return 4.5;continue;}
      double a=(slab[2]-slab[0])/slab[1],b=(slab[3]-slab[0])/slab[1];
      if(a>b)std::swap(a,b);
      enter=std::max(enter,a);exit=std::min(exit,b);
    }
    return enter<=exit&&enter>=.03?enter:4.5;
  }
};
std::vector<uint16_t> firmware_plan(const PlannerRequest&r){
  astar_context ctx{};std::copy(r.blocked.begin(),r.blocked.end(),ctx.occupied);
  int count=astar_plan(&ctx,r.start%64,r.start/64,r.goal%64,r.goal/64);
  if(count<0)throw std::runtime_error("actual firmware A* no route: "+std::to_string(count));
  return {ctx.path,ctx.path+count};
}
}
int main(){
  Config c;c.velocity_control=true;c.configured=false;c.drive_mps_per_full_pwm=0;c.sonar_latency_s=0;
  c.map_ttl_s=600;c.left_wheel_mask=12;
  { Navigator edge(c);SensorFrame f;f.imu_valid=true;
    edge.ingest(f);f.timestamp_s=.02;f.counts[0]=0x80000000U;
    assert(!edge.ingest(f)&&edge.reason()=="encoder_jump"); }
  Navigator nav(c);Scene world;double t=0,yaw=0;std::array<double,4> counts{};
  auto sample=[&]{
    SensorFrame frame;frame.timestamp_s=t;frame.device_timestamp_s=t;frame.imu_valid=true;frame.yaw_rad=wrap(yaw);
    frame.sonar_observation_ids_valid=true;
    for(unsigned i=0;i<4;++i)frame.counts[i]=uint32_t(int64_t(std::llround(counts[i])));
    for(unsigned i=0;i<3;++i){frame.sonar_m[i]=world.range(nav.pose(),c.sonar_mounts[i]);frame.sonar_observation_id[i]=uint32_t(std::llround(t*1000));}
    nav.ingest(frame,t);assert(nav.grid().import_observations(world.cells(),{-3.2,-3.2},t));
  };
  sample();t+=dt;sample();nav.set_goal(1.85,.05);assert(nav.start(t));
  Control out=nav.tick(t+.001);assert(out.planner_request);
  auto req=*out.planner_request;assert(nav.accept_plan(req.request_id,req.map_revision,firmware_plan(req),t+.002));out=nav.command();
  for(unsigned step=0;step<12000&&nav.state()!="ARRIVED";++step){
    const double left=out.linear_mps-out.angular_rps*c.track_m*.5,right=out.linear_mps+out.angular_rps*c.track_m*.5;
    for(unsigned i=0;i<4;++i)counts[i]+=((c.left_wheel_mask&(1U<<i))?left:right)*dt/c.wheel_circumference_m*c.encoder_ticks_per_turn*c.encoder_signs[i];
    yaw+=out.angular_rps*dt;t+=dt;sample();out=nav.tick(t+.001);
    assert(out.left==0&&out.right==0); // No PWM-calibration estimate in MOT4.
    assert(std::abs(out.linear_mps)<=.300001&&std::abs(out.angular_rps)<=1.000001);
    if(out.state=="FAULT")throw std::runtime_error(out.reason);
  }
  assert(nav.state()=="ARRIVED");assert(std::hypot(nav.pose().x-1.85,nav.pose().y-.05)<.13);
  // The historical immediate-detour fixture required A* before local motion.
  // v28 local .30m/.20m plus actual FPGA rejoin and geometric obstacle testing
  // lives in test_reactive_avoidance.cc, compiled for both velocity and PWM.
  Navigator blocked(c);SensorFrame near;near.imu_valid=true;near.sonar_m={.1,.1,.1};near.timestamp_s=t;
  blocked.ingest(near,t);near.timestamp_s=t+.02;blocked.ingest(near,t+.02);blocked.explore();assert(blocked.start(t+.02));
  for(unsigned i=0;i<200;++i){near.timestamp_s+=dt;blocked.ingest(near,near.timestamp_s);auto zero=blocked.tick(near.timestamp_s);assert(blocked.enabled());assert(zero.linear_mps==0&&zero.angular_rps==0);}
  assert(blocked.state()=="AVOID_WAIT");
  std::cout<<"PASS production physical velocity/fixed geometry/60527.7/signs/live mask12 -> actual FPGA A* -> ARRIVED, plus too-close local obstacle zero-and-enabled wait; simulated_s="<<t<<"\n";
}
