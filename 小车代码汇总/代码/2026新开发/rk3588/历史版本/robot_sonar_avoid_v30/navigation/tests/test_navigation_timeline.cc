// Full production geometry, actual FPGA A* firmware, 10 Hz new sonar observations
// and 50 Hz wheel/IMU feedback. No imported free map or synthetic map clearing.
// stdout is a CSV of state transitions; the virtual clock never moves hardware.
#include "../navcore.h"
extern "C" {
#include "astar.h"
}
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
using namespace robot_nav;
static unsigned checks=0;
static void ck(bool x,const char*msg){++checks;if(!x)throw std::runtime_error(msg);}
struct Sim {
 Config c;Navigator nav;double t=0,yaw=0;std::array<double,4>count{};Control out;std::array<double,3>ranges{4.5,4.5,4.5};
 unsigned invalid=0;double forced_age=0,forward_seconds=0,epoch_forward_seconds=0;std::string previous;bool freeze_m4=false,freeze_all=false;
 static Config configured(){Config c;c.configured=true;c.drive_mps_per_full_pwm=.6;c.left_wheel_mask=10;return c;}
 Sim(Config config=configured()):c(config),nav(c){sample();for(unsigned i=0;i<50;++i)step(false);}
 SensorFrame frame(){SensorFrame f;f.timestamp_s=t;f.device_timestamp_s=t;f.sequence=unsigned(t*1000);f.imu_valid=true;f.yaw_rad=wrap(yaw);f.sonar_observation_ids_valid=true;
  const auto obs=unsigned(std::floor((t+1e-8)*10))*100;
  for(unsigned i=0;i<4;++i)f.counts[i]=uint32_t(int64_t(std::llround(count[i])));
  for(unsigned i=0;i<3;++i){if(!(invalid&(1U<<i)))f.sonar_m[i]=ranges[i];f.sonar_age_s[i]=std::max(0.,t-obs/1000.)+forced_age;f.sonar_observation_id[i]=obs;}
  return f;
 }
 void sample(){nav.ingest(frame(),t);}
 void trace(const char*stage){const auto key=nav.state()+"/"+nav.reason()+"/"+std::to_string(nav.enabled());if(key==previous)return;previous=key;
  std::cout<<stage<<","<<std::fixed<<std::setprecision(3)<<t<<","<<nav.state()<<","<<nav.enabled()<<","<<out.left<<","<<out.right<<","<<nav.pose().x<<","<<nav.pose().y<<","<<nav.pose().yaw<<","<<forward_seconds<<","<<nav.reason()<<"\n";
 }
 void step(bool tick=true){const double dt=.02;
  if(tick&&nav.enabled()&&out.state=="BOOTSTRAP_ADVANCE"&&out.linear_mps>0){forward_seconds+=dt;epoch_forward_seconds+=dt;}
  double left=out.linear_mps-out.angular_rps*c.track_m*.5,right=out.linear_mps+out.angular_rps*c.track_m*.5;
  for(unsigned i=0;i<4;++i)if(!freeze_all&&!(freeze_m4&&i==3))count[i]+=((c.left_wheel_mask&(1U<<i))?left:right)*dt*c.encoder_ticks_per_turn/c.wheel_circumference_m*c.encoder_signs[i];
  yaw+=out.angular_rps*dt;t+=dt;sample();if(tick){out=nav.tick(t+.0001);
   if(nav.state()=="BOOTSTRAP_STOP"&&nav.reason()=="observe_before_bounded_forward_exploration")epoch_forward_seconds=0;
   if(nav.enabled())ck(std::abs(nav.bootstrap_motion_elapsed_s()-epoch_forward_seconds)<.0002,"production timer follows actual forward intervals");
  }
 }
 void start(){forward_seconds=epoch_forward_seconds=0;nav.explore();ck(nav.start(t),"nav start");out=nav.command();trace("start");}
 bool plan(){if(!out.planner_request)return false;const auto r=*out.planner_request;astar_context ctx{};std::copy(r.blocked.begin(),r.blocked.end(),ctx.occupied);const int n=astar_plan(&ctx,r.start%64,r.start/64,r.goal%64,r.goal/64);ck(n>1,"real firmware A* plan");ck(nav.accept_plan(r.request_id,r.map_revision,{ctx.path,ctx.path+n},t+.0002),"real firmware path accept");out=nav.command();trace("actual-Astar");return true;}
};
int main(){
 try{
  std::cout<<"event,time_s,state,enabled,left_pwm,right_pwm,pose_x_m,pose_y_m,yaw_rad,forward_command_s,reason\n";
  Sim baseline;baseline.start();bool moved=false,planned=false;
  for(unsigned i=0;i<2500&&!planned;++i){baseline.step();baseline.trace("baseline");ck(baseline.nav.enabled(),"baseline enabled");moved=moved||baseline.out.linear_mps>0;planned=baseline.plan();}
  ck(moved&&planned,"bounded bootstrap actual A* handoff");
  
  Sim obstacle;obstacle.start();for(unsigned i=0;i<250&&obstacle.out.linear_mps==0;++i){obstacle.step();obstacle.trace("advance");}
  ck(obstacle.out.linear_mps>0,"obstacle scenario moves first");
  for(unsigned i=0;i<50;++i)obstacle.step();
  obstacle.ranges={.2,.2,.2};
  bool waited=false;for(unsigned i=0;i<250;++i){obstacle.step();obstacle.trace("obstacle");if(obstacle.out.state=="AVOID_WAIT"){waited=true;break;}}
  ck(waited,"initial obstacle reaches WAIT");
  for(unsigned i=0;i<500;++i){obstacle.step();obstacle.trace("wait10sec");ck(obstacle.nav.enabled(),"WAIT not disarmed");ck(obstacle.out.left==0&&obstacle.out.right==0,"WAIT zeroPWM for10sec");}
  obstacle.ranges={4.5,4.5,4.5};bool resumed=false,after_planned=false;
  for(unsigned i=0;i<5000;++i){obstacle.step();obstacle.trace("physically-clear");ck(obstacle.nav.enabled(),"clear phase remains enabled");resumed=resumed||obstacle.out.linear_mps>0;if(obstacle.plan()){after_planned=true;break;}}
  ck(resumed&&after_planned,"physical obstacle removal naturally recovers and reaches actual FPGA planner");
  
  Sim invalid;invalid.start();invalid.invalid=1;invalid.step();invalid.trace("invalid");ck(invalid.nav.enabled()&&invalid.nav.reason()=="sonar_invalid_zero_wait","one invalid source keeps zero waiting session");
  for(unsigned i=0;i<50;++i){invalid.step();ck(invalid.nav.enabled()&&invalid.out.linear_mps==0,"one second source wait does not terminate");}
  invalid.invalid=0;bool resumed_source=false;for(unsigned i=0;i<120;++i){invalid.step();resumed_source=resumed_source||invalid.out.linear_mps>0;}
  ck(invalid.nav.enabled()&&resumed_source,"genuine stopped source recovery continues same task");
  invalid.invalid=1;for(unsigned i=0;i<270&&invalid.nav.enabled();++i)invalid.step();ck(!invalid.nav.enabled()&&invalid.nav.reason()=="sonar_unavailable_timeout","continuous five-second source failure terminates");
  invalid.invalid=0;for(unsigned i=0;i<100;++i)invalid.step();ck(!invalid.nav.enabled(),"source recovery cannot resurrect terminated task");
  Sim stale;stale.start();stale.forced_age=.8;stale.step();stale.trace("stale");ck(stale.nav.enabled()&&stale.nav.reason()=="sonar_stale_zero_wait","expired source zero wait");
  stale.forced_age=0;bool resumed_stale=false;for(unsigned i=0;i<120;++i){stale.step();resumed_stale=resumed_stale||stale.out.linear_mps>0;}ck(stale.nav.enabled()&&resumed_stale,"expired source safely recovers same authorized task");
  
  Sim encoder;encoder.start();encoder.freeze_m4=true;
  for(unsigned i=0;i<400&&encoder.nav.enabled();++i){encoder.step();encoder.trace("encoder");}
  ck(!encoder.nav.enabled()&&encoder.nav.reason()=="encoder_wheel_disagreement","M2/M4 same-side mismatch latches");encoder.freeze_m4=false;for(unsigned i=0;i<100;++i)encoder.step();ck(!encoder.nav.enabled(),"encoder recovery cannot autostart");

  // A shorter configured motion budget must still end bootstrap, including
  // across its ordinary stop/reobserve steps. We do not change map contents.
  Config limited=Sim::configured();limited.bootstrap_speed_mps=.01;limited.bootstrap_max_s=10;
  Sim budget(limited);budget.start();bool budget_wait=false,saw_step_stop=false;
  for(unsigned i=0;i<2000&&!budget_wait;++i){budget.step();budget.trace("motion-budget");
   ck(budget.nav.enabled(),"budget exhaustion safely waits without disarming");
   saw_step_stop=saw_step_stop||(budget.out.state=="BOOTSTRAP_STOP"&&budget.forward_seconds>0);
   budget_wait=budget.nav.reason()=="bounded_bootstrap_limit_waiting_verified_corridor";
  }
  ck(budget_wait&&saw_step_stop,"cumulative forward budget survives short observation stops");
  ck(budget.forward_seconds>=10-1e-8&&budget.forward_seconds<10.021,"10 second budget counts only actual forward command intervals");
  ck(budget.t-budget.forward_seconds>1.7,"ordinary stop observations excluded from motion budget");
  ck(budget.out.left==0&&budget.out.right==0,"budget limit PWM zero");

  // Long real waits do not consume budget. A completed local bypass starts a
  // new bounded exploration origin when there was no map route to rejoin;
  // its per-origin budget is tracked independently from lifetime forward time.
  Sim repeated(limited);repeated.start();
  for(unsigned round=0;round<2;++round){
   double real_forward_command_s=0;
   for(unsigned i=0;i<500&&real_forward_command_s<1.;++i){
    if(repeated.out.linear_mps>0)real_forward_command_s+=.02;
    repeated.step();repeated.trace("repeated-before-obstacle");
   }
   // Following the new map-independent .50m local departure is real motion
   // even though it must not be charged to the separate bootstrap budget.
   ck(real_forward_command_s>=1.,"each repeat actually commands forward motion (bootstrap or local departure)");
   repeated.ranges={.2,.2,.2};
   for(unsigned i=0;i<500&&repeated.nav.state()!="AVOID_WAIT";++i){repeated.step();repeated.trace("repeated-obstacle");}
   ck(repeated.nav.state()=="AVOID_WAIT","each repeated obstacle reaches safe wait");
   const double stopped_budget=repeated.forward_seconds;
   for(unsigned i=0;i<3500;++i){repeated.step();ck(repeated.nav.enabled(),"repeated wait stays enabled");ck(repeated.out.left==0&&repeated.out.right==0,"repeated wait PWM zero");}
   ck(std::abs(repeated.forward_seconds-stopped_budget)<1e-9,"70 second true-near wait consumes no motion budget");
   repeated.ranges={4.5,4.5,4.5};bool continued=false;
   for(unsigned i=0;i<4000&&!continued;++i){repeated.step();repeated.trace("repeated-clear");ck(repeated.nav.enabled(),"repeated clear remains enabled");continued=repeated.out.linear_mps>0;}
   ck(continued,"real front-clear observations permit local motion without a map-TTL wait");
  }
  ck(repeated.t>120&&repeated.forward_seconds<3.,"over two minutes of genuine safe waits do not exhaust ten second motion budget");
  bool repeat_budget_wait=false,repeat_planned=false;
  for(unsigned i=0;i<2000&&!repeat_budget_wait&&!repeat_planned;++i){repeated.step();repeated.trace("repeated-budget-finish");ck(repeated.nav.enabled(),"repeated final budget waits enabled");repeat_budget_wait=repeated.nav.reason()=="bounded_bootstrap_limit_waiting_verified_corridor";repeat_planned=repeated.plan();}
  ck(repeat_planned||(repeat_budget_wait&&repeated.epoch_forward_seconds>=10-1e-8&&repeated.epoch_forward_seconds<10.021),"post-bypass exploration either reaches actual FPGA A* from new observations or its bounded movement limit");

  repeated.start();ck(repeated.nav.enabled()&&repeated.nav.bootstrap_motion_elapsed_s()==0,"only explicit new Start resets the cumulative budget");
  Sim stalled;stalled.start();stalled.freeze_all=true;
  for(unsigned i=0;i<500&&stalled.nav.enabled();++i){stalled.step();stalled.trace("no-progress");}
  ck(!stalled.nav.enabled()&&stalled.nav.reason()=="no_odometry_progress","true commanded motion without measured progress still faults");

  Config distance_limited=Sim::configured();distance_limited.bootstrap_max_distance_m=.12;
  Sim distance_limit(distance_limited);distance_limit.start();
  for(unsigned i=0;i<1000;++i){distance_limit.step();distance_limit.trace("distance-budget");}
  ck(distance_limit.nav.enabled()&&distance_limit.out.state=="BOOTSTRAP_WAIT","independent measured-distance limit still waits safely");
  ck(distance_limit.out.left==0&&distance_limit.out.right==0&&distance_limit.nav.pose().x<=.14,"independent distance limit bounds real travel");
  std::cout<<"result,0,PASS,0,0,0,0,0,0,0,checks="<<checks<<"\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL timeline: "<<e.what()<<" checks="<<checks<<"\n";return 1;}
}
