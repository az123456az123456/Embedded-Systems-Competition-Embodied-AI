// Independent production-core tests for the bounded live-range manoeuvre.
// Counts and IMU are generated from the previous actual wheel command, never
// from a desired pose. Unknown-map fixtures never grant fictitious free cells.
#include "../navcore.h"
extern "C" {
#include "astar.h"
}
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
using namespace robot_nav;
namespace {
unsigned checks=0;
void ck(bool yes,const char* why){++checks;if(!yes)throw std::runtime_error(why);}
Config configured(){Config c;
#ifdef V28_TEST_VELOCITY
 c.velocity_control=true;c.configured=false;c.drive_mps_per_full_pwm=0;
#else
 c.configured=true;c.drive_mps_per_full_pwm=.5;
#endif
 return c;}
struct Box {double x0,x1,y0,y1;};
struct Robot {
 Config config;Navigator nav;double now=10,yaw=0;Control out;
 std::array<double,4> counts{};std::array<double,3> ranges{4.5,4.5,4.5};
 std::map<std::string,double> distance;std::vector<double> turn_segments;
 unsigned invalid_mask=0,stuck_mask=0;double extra_age=0;bool imu_valid=true,freeze=false,isolated_unknown_map=false;
 bool verbose=false;std::string previous;double turn_sum=0;std::vector<Box> boxes;
 explicit Robot(Config c=configured()):config(c),nav(c){sample();for(unsigned i=0;i<70;++i)step(false);}
 double ray(double x,double y,double angle)const{double answer=4.5;for(const auto& box:boxes){double lo=0,hi=4.5;bool miss=false;for(const auto& axis:{std::array<double,4>{x,std::cos(angle),box.x0,box.x1},std::array<double,4>{y,std::sin(angle),box.y0,box.y1}}){if(std::abs(axis[1])<1e-12){if(axis[0]<axis[2]||axis[0]>axis[3])miss=true;continue;}double a=(axis[2]-axis[0])/axis[1],b=(axis[3]-axis[0])/axis[1];if(a>b)std::swap(a,b);lo=std::max(lo,a);hi=std::min(hi,b);}if(!miss&&lo<=hi)answer=std::min(answer,std::max(config.sonar_min_m,lo));}return answer;}
 bool body_hits(const Box& box,double extra_yaw=0,double margin=0)const{const double orientation=yaw+extra_yaw;std::vector<Point> body;const auto p=nav.pose();for(double x:{-config.body_rear_m-margin,config.body_front_m+margin})for(double y:{-config.body_right_m-margin,config.body_left_m+margin})body.push_back({p.x+x*std::cos(orientation)-y*std::sin(orientation),p.y+x*std::sin(orientation)+y*std::cos(orientation)});const std::vector<Point> obstacle{{box.x0,box.y0},{box.x0,box.y1},{box.x1,box.y0},{box.x1,box.y1}};for(Point axis:{Point{1,0},Point{0,1},Point{std::cos(orientation),std::sin(orientation)},Point{-std::sin(orientation),std::cos(orientation)}}){double amin=1e9,amax=-1e9,bmin=1e9,bmax=-1e9;for(Point q:body){double t=q.first*axis.first+q.second*axis.second;amin=std::min(amin,t);amax=std::max(amax,t);}for(Point q:obstacle){double t=q.first*axis.first+q.second*axis.second;bmin=std::min(bmin,t);bmax=std::max(bmax,t);}if(amax<bmin-1e-9||bmax<amin-1e-9)return false;}return true;}
 void sample(){SensorFrame f;f.timestamp_s=now;f.device_timestamp_s=now;f.sequence=unsigned(now*1000);f.imu_valid=imu_valid;f.yaw_rad=wrap(yaw);f.sonar_observation_ids_valid=true;
  const unsigned obs=unsigned(std::floor((now+1e-8)*10))*100;
  for(unsigned i=0;i<4;++i)f.counts[i]=uint32_t(int64_t(std::llround(counts[i])));
  for(unsigned i=0;i<3;++i){double value=ranges[i];if(!boxes.empty()){const auto p=nav.pose();const auto m=config.sonar_mounts[i];const double x=p.x+m[0]*std::cos(yaw)-m[1]*std::sin(yaw),y=p.y+m[0]*std::sin(yaw)+m[1]*std::cos(yaw);value=4.5;for(int ray_index=-6;ray_index<=6;++ray_index)value=std::min(value,ray(x,y,yaw+m[2]+config.sonar_half_angle_rad*ray_index/6.));}if(!(invalid_mask&(1U<<i)))f.sonar_m[i]=value;f.sonar_age_s[i]=std::max(0.,now-obs/1000.)+extra_age;f.sonar_observation_id[i]=obs;}
  nav.ingest(f,now);
  if(isolated_unknown_map)ck(nav.grid().import_observations(std::vector<int8_t>(4096,-1),nav.grid().origin(),now),"explicit isolated map fixture for wrap/health control tests");
 }
 void step(bool control=true){constexpr double dt=.02;
  const double vl=config.velocity_control?out.linear_mps-out.angular_rps*config.track_m*.5:out.left*config.drive_mps_per_full_pwm;
  const double vr=config.velocity_control?out.linear_mps+out.angular_rps*config.track_m*.5:out.right*config.drive_mps_per_full_pwm;
  const double linear=(vl+vr)*.5,angular=(vr-vl)/config.track_m;
  if(!freeze){for(unsigned w=0;w<4;++w)if(!(stuck_mask&(1U<<w)))counts[w]+=((config.left_wheel_mask&(1U<<w))?vl:vr)*dt*config.encoder_ticks_per_turn/config.wheel_circumference_m*config.encoder_signs[w];yaw+=angular*dt;distance[out.state]+=std::abs(linear)*dt;}
  if(std::abs(linear)<1e-8&&std::abs(angular)>1e-8&&!freeze)turn_sum+=std::abs(angular)*dt;
  else if(turn_sum>0){turn_segments.push_back(turn_sum);turn_sum=0;}
  now+=dt;sample();for(const auto& box:boxes)ck(!body_hits(box),"full asymmetric physical body must not penetrate fixed geometric obstacle");if(!control)return;
  const auto values=nav.grid().values();const auto observed=nav.grid().observed_s();const auto revision=nav.grid().revision();
  out=nav.tick(now+.0001);
  ck(nav.grid().values()==values&&nav.grid().observed_s()==observed&&nav.grid().revision()==revision,"tick must not fabricate free cells or observation times");
  ck(std::abs(out.left)<=.600001&&std::abs(out.right)<=.600001,"reactive duty hard limit");
  if(config.velocity_control)ck(std::abs(out.linear_mps)<=.300001&&std::abs(out.angular_rps)<=1.000001,"MOT4 physical velocity hard limits");
  if(verbose&&out.state!=previous){std::cout<<"REACTIVE t="<<now<<" state="<<out.state<<" reason="<<out.reason<<" pose="<<nav.pose().x<<","<<nav.pose().y<<","<<nav.pose().yaw<<" distance030="<<distance["AVOID_ADVANCE_030"]<<" distance020="<<distance["AVOID_ADVANCE_020"]<<'\n';previous=out.state;}
 }
 void unknown(){ck(nav.grid().import_observations(std::vector<int8_t>(4096,-1),nav.grid().origin(),now),"explicit unknown fixture import");}
 void start_straight(){ck(nav.start_straight(now),"start current-range straight assist");out=nav.command();}
 void route_start(){ck(nav.grid().import_observations(std::vector<int8_t>(4096,0),nav.grid().origin(),now),"explicit previously surveyed route fixture");nav.set_goal(2.5,0);ck(nav.start(now),"ordinary goal start");out=nav.tick(now+.0001);solve();}
 void solve(){ck(out.planner_request.has_value(),"route has a real FPGA request");const auto req=*out.planner_request;astar_context astar{};std::copy(req.blocked.begin(),req.blocked.end(),astar.occupied);const int len=astar_plan(&astar,req.start%64,req.start/64,req.goal%64,req.goal/64);ck(len>1,"actual FPGA A* route exists");ck(nav.accept_plan(req.request_id,req.map_revision,{astar.path,astar.path+len},now+.0002),"production accepts actual FPGA route");out=nav.command();}
 void moving(){for(unsigned i=0;i<190;++i){step();ck(nav.enabled(),nav.reason().c_str());}ck(out.linear_mps>.10,"test reaches physical forward speed before obstacle");}
 void trigger(int side){unknown();ranges={.55,side>0?2.:1.5,side>0?1.5:2.};step();ck(out.linear_mps==0&&out.angular_rps==0&&out.state=="OBSTACLE_STOP","new obstacle must command zero before any turn");ck(!out.planner_request,"obstacle start needs no preselected return endpoint");}
};
void mirror_case(int side,bool ordinary){
 Config c=configured();if(ordinary){c.cruise_mps=c.detour_mps=.30;c.max_pwm=.60;}
 Robot r(c);r.verbose=true;if(ordinary)r.route_start();else r.start_straight();
 r.moving();r.trigger(side);bool turned=false,done=false,advanced=false;double first_angle=0,final_heading=0;
 for(unsigned i=0;i<7000&&!done;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());
  if(r.out.state=="AVOID_TURN_CLEAR"&&std::abs(r.out.angular_rps)>1e-9){
   if(!turned){first_angle=r.out.angular_rps;}turned=true;r.ranges={4.5,side>0?2.:1.5,side>0?1.5:2.};
  }
  ck(r.out.state!="AVOID_ADVANCE_030"&&r.out.state!="AVOID_ADVANCE_020"&&r.out.state!="AVOID_TURN_BACK","no legacy fixed-distance or original-heading phase");
  advanced=advanced||r.out.state=="AVOID_ADVANCE_050";
  if(!ordinary){ck(!r.out.planner_request,"straight never needs an original map route");if(turned&&r.out.state=="STRAIGHT_ADVANCE"){done=true;final_heading=r.nav.pose().yaw;}}
  else{
   if(r.out.state=="AVOID_REJOIN_WAIT")ck(r.nav.grid().import_observations(std::vector<int8_t>(4096,0),r.nav.grid().origin(),r.now),"post-clearance actual surveyed map fixture");
   if(r.out.planner_request){ck(advanced&&r.distance["AVOID_ADVANCE_050"]>=.4949,"navigation completes actual .50m before asking map rejoin");ck(r.out.planner_request->purpose=="rejoin","rejoin is a genuine FPGA map request");done=true;r.solve();}
  }
 }
 ck(done&&turned,"reactive clearance completes both mirrored cases");ck(first_angle*side>0,"turn chooses farther measured side");
 if(!ordinary){ck(std::abs(final_heading)>3*vehicle_profile::pi/180,"straight preserves newly cleared heading instead of original");ck(r.distance["AVOID_ADVANCE_050"]==0,"straight does not inherit navigation .50m leg");
  for(unsigned i=0;i<100;++i){r.step();}ck(std::abs(wrap(r.nav.pose().yaw-final_heading))<.01,"new straight heading remains held");}
 else ck(r.distance["AVOID_ADVANCE_050"]<=.5061,"map leg encoder distance tolerance");
}
void source_wait_case(unsigned bad,bool at_start){Robot r;
 if(!at_start){r.start_straight();r.moving();}
 if(bad==0)r.invalid_mask=1;else r.extra_age=.8;
 if(at_start){r.now+=.02;r.sample();r.start_straight();}
 for(unsigned i=0;i<100;++i){r.step();ck(r.nav.enabled()&&r.out.state=="SENSOR_WAIT","short sonar gap holds accepted session");ck(r.out.linear_mps==0&&r.out.angular_rps==0,"invalid range always commands zero");}
 r.invalid_mask=0;r.extra_age=0;bool resumed=false;
 for(unsigned i=0;i<200&&!resumed;++i){r.step();ck(r.nav.enabled(),"healthy post-stop source recovery keeps owner");resumed=r.out.linear_mps>.05;}
 ck(resumed,"genuine fresh sources resume after stopped observations");
}
void persistent_and_hard_faults(){
 Robot r;r.start_straight();r.moving();r.invalid_mask=7;
 for(unsigned i=0;i<270&&r.nav.enabled();++i)r.step();
 ck(!r.nav.enabled()&&r.nav.reason()=="sonar_unavailable_timeout","five-second persistent source failure terminates explicitly");
 r.invalid_mask=0;for(unsigned i=0;i<100;++i){r.step();}ck(!r.nav.enabled(),"source recovery cannot restart terminated task");
 Robot imu;imu.start_straight();imu.moving();imu.imu_valid=false;imu.step();ck(!imu.nav.enabled(),"real IMU fault is never downgraded to sonar wait");
 Robot stop;stop.start_straight();stop.invalid_mask=7;stop.step();stop.out=stop.nav.stop("operator_stop");stop.invalid_mask=0;
 for(unsigned i=0;i<100;++i){stop.step();}ck(!stop.nav.enabled(),"STOP while waiting cannot resume later");
}
void scan_wrap_and_sensor_interrupt(){
 {Robot r;r.start_straight();r.moving();r.isolated_unknown_map=true;r.trigger(-1);r.ranges={.34,1.5,2.};
  bool wrapped=false;double previous=r.yaw;
  for(unsigned i=0;i<4200&&r.nav.enabled();++i){r.step();wrapped=wrapped||std::abs(wrap(r.yaw)-wrap(previous))>3.;previous=r.yaw;}
  ck(wrapped,"continuous measured scan crosses IMU +/-pi boundary");
  ck(!r.nav.enabled()&&r.nav.reason()=="avoid_scan_exhausted","two measured full circles terminate explicit scan budget");
  ck(r.out.linear_mps==0&&r.out.angular_rps==0,"exhausted scan commands zero");
 }
 {Robot r;r.start_straight();r.moving();r.trigger(1);bool turning=false;
  for(unsigned i=0;i<100&&!turning;++i){r.step();turning=r.out.state=="AVOID_TURN_CLEAR"&&std::abs(r.out.angular_rps)>0;}
  ck(turning,"sonar dropout fixture reaches actual turn");r.invalid_mask=1;
  for(unsigned i=0;i<70;++i){r.step();ck(r.nav.enabled()&&r.out.state=="SENSOR_WAIT"&&r.out.angular_rps==0,"turn gap is zero waiting, not heading progress failure");}
  r.invalid_mask=0;r.ranges={4.5,2.,1.5};const double recovered_at=r.now;bool resumed=false;
  for(unsigned i=0;i<200&&!resumed;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());
   if(r.now-recovered_at<.30)ck(r.out.linear_mps==0&&r.out.angular_rps==0,"source recovery must stop and replace old turn observations");
   resumed=r.out.state=="STRAIGHT_ADVANCE"&&r.out.linear_mps>0;
  }
  ck(resumed,"interrupted turn can recover only with fresh stopped source observations");
 }
}
void genuine_encoder_faults(){
 for(bool turning:{false,true}){Robot r;r.start_straight();r.moving();
  if(turning){r.isolated_unknown_map=true;r.trigger(1);r.ranges={.34,2.,1.5};
   for(unsigned i=0;i<100&&r.out.state!="AVOID_TURN_CLEAR";++i){r.step();}ck(r.out.state=="AVOID_TURN_CLEAR","encoder test reaches scanning");}
  r.stuck_mask=2; // M2 stops producing counts while same-side M4 still moves.
  for(unsigned i=0;i<180&&r.nav.enabled();++i)r.step();
  ck(!r.nav.enabled()&&r.nav.reason()=="encoder_wheel_disagreement","real same-side front/rear missing counts remain latched fault");
  r.stuck_mask=0;for(unsigned i=0;i<40;++i)r.step();ck(!r.nav.enabled(),"healthy counts cannot silently clear established localization latch");
 }
}
void old_heading_and_large_scan(){Robot r;r.start_straight();r.moving();r.trigger(-1);bool turned=false,large=false;
 for(unsigned i=0;i<1100&&!large;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());
  if(r.out.state=="AVOID_TURN_CLEAR")turned=true;
  if(turned){r.ranges={.34,1.5,2.};large=std::abs(r.nav.pose().yaw)>60*vehicle_profile::pi/180;}
 }
 ck(turned&&large,"continuous scan safely accumulates over old 29-degree fatal threshold");
 r.ranges={4.5,1.5,2.};bool done=false;
 for(unsigned i=0;i<200&&!done;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());done=r.out.state=="STRAIGHT_ADVANCE";}
 ck(done,"large-turn clear direction completes without old heading-leg fatal");
}
}
int main(){try{
 mirror_case(1,false);mirror_case(-1,false);mirror_case(1,true);mirror_case(-1,true);
 for(unsigned bad=0;bad<2;++bad)for(bool start:{false,true})source_wait_case(bad,start);
 persistent_and_hard_faults();old_heading_and_large_scan();scan_wrap_and_sensor_interrupt();genuine_encoder_faults();
 std::cout<<"PASS v29 reactive-clearance "<<checks<<" checks; mirrored new heading/map .50m, fresh stopped confirmation, transient sonar wait and real faults\n";
 }catch(const std::exception&e){std::cerr<<"FAIL v29 reactive-clearance "<<checks<<" "<<e.what()<<'\n';return 1;}}
