// Production Navigator integration tests. Motion samples follow the previously
// emitted physical wheel command; obstacle policy is not reimplemented here.
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#define private public
#include "../navcore.h"
#undef private
#include "../vendor/picojson.h"
using namespace robot_nav;
namespace {
unsigned checks=0;
void check(bool value,const char* why){++checks;if(!value)throw std::runtime_error(why);}
Config configured(){Config c;c.velocity_control=true;return c;}
struct Robot{
 Config c=configured();Navigator n{c};double now=10,yaw=0;Control out;
 std::array<double,4> counts{};std::array<double,3> range{2.,2.,2.};
 Robot(){for(unsigned i=0;i<80;++i)step(false);}
 void step(bool control=true){
  constexpr double dt=.02;
  const double left=out.linear_mps-out.angular_rps*c.track_m*.5;
  const double right=out.linear_mps+out.angular_rps*c.track_m*.5;
  for(unsigned w=0;w<4;++w)counts[w]+=((c.left_wheel_mask&(1U<<w))?left:right)*dt*c.encoder_ticks_per_turn/c.wheel_circumference_m*c.encoder_signs[w];
  yaw+=out.angular_rps*dt;now+=dt;SensorFrame f;
  f.timestamp_s=now;f.device_timestamp_s=now;f.sequence=unsigned(std::llround(now*1000));
  f.imu_valid=true;f.yaw_rad=wrap(yaw);f.sonar_observation_ids_valid=true;
  const double observed=std::floor((now+1e-8)*20)/20.;
  for(unsigned w=0;w<4;++w)f.counts[w]=uint32_t(int64_t(std::llround(counts[w])));
  for(unsigned ch=0;ch<3;++ch){f.sonar_m[ch]=range[ch];f.sonar_age_s[ch]=std::max(0.,now-observed);f.sonar_observation_id[ch]=uint32_t(std::llround(observed*1000));}
  const bool first=!n.frame();const bool accepted=n.ingest(f,now);
  check(first||accepted,"physical command-generated telemetry remains valid");
  if(control){const auto cells=n.grid().values();const auto seen=n.grid().observed_s();const auto rev=n.grid().revision();out=n.tick(now+.00001);
   check(n.grid().values()==cells&&n.grid().observed_s()==seen&&n.grid().revision()==rev,"control never fabricates map observations");
   check(n.enabled(),n.reason().c_str());}
 }
 void start(bool navigation=false){if(navigation){n.explore();check(n.start(now),"fresh exploration start");}else check(n.start_straight(now),"fresh heading assist start");out=n.command();}
 void obstacle(std::array<double,3> values){range=values;step();check(out.linear_mps==0&&out.angular_rps==0&&out.state=="OBSTACLE_STOP","each beam close commands STOP before turn");}
 int turn(){for(unsigned i=0;i<150;++i){step();if(std::abs(out.angular_rps)>1e-8){check(out.linear_mps==0,"avoidance rotation has zero forward velocity");return out.angular_rps>0?1:-1;}}return 0;}
};
void three_beams(){
 for(bool navigation:{false,true})for(unsigned ch=0;ch<3;++ch){
  Robot r;r.start(navigation);auto d=r.range;d[ch]=.299;
  if(ch==0){d[1]=1.8;d[2]=.8;}r.obstacle(d);
  const int expected=ch==1?-1:1;check(r.turn()==expected,"front chooses larger opening; near flank turns away");
  check(!r.out.planner_request,"local response never requests map route before clearance");
 }
 for(auto values:{std::array<double,3>{.299,.5,1.5},std::array<double,3>{2.,.25,.29},std::array<double,3>{2.,.29,.25},std::array<double,3>{.299,.7,.7}}){
  Robot r;r.start();r.obstacle(values);const int expected=values[1]>values[2]?1:-1;
  check(r.turn()==expected,"front-only and both-near choose greater opening, tie clockwise");
 }
}
void close_side_cannot_skip_turn(){
 for(unsigned flank:{1U,2U}){
  Robot r;r.start();auto ranges=r.range;ranges[flank]=.29;r.obstacle(ranges);const int sign=flank==1?-1:1;
  check(r.turn()==sign,"clear front cannot bypass close-side turn");
  r.range[flank]=.319;
  for(unsigned i=0;i<40;++i){r.step();check(r.out.linear_mps==0,"2cm release hysteresis prevents immediate stop/resume chatter");}
  r.range[flank]=.321;bool resumed=false;
  for(unsigned i=0;i<150&&!resumed;++i){r.step();resumed=r.out.linear_mps>0;}
  check(resumed&&r.out.state=="STRAIGHT_ADVANCE","fresh front and both released flanks resume new heading");
  r.range[flank]=.31;for(unsigned i=0;i<20;++i){r.step();check(r.out.state=="STRAIGHT_ADVANCE","released .31m remains above stop threshold");}
  r.range[flank]=.299;r.step();check(r.out.state=="OBSTACLE_STOP"&&r.out.linear_mps==0,"a genuinely new flank crossing stops again");
 }
}
void new_danger_reselects(){
 for(int sign:{-1,1}){
  Robot r;r.start();r.obstacle({.29,sign>0?2.:1.,sign>0?1.:2.});check(r.turn()==sign,"initial larger opening turn starts");
  r.range={.29,sign>0?.29:2.,sign>0?2.:.29};r.step();
  check(r.out.linear_mps==0&&r.out.angular_rps==0,"new obstacle in turn direction commands zero before reversal");
  check(r.out.reason=="avoid_new_side_obstacle_stop_before_reverse","direction invalidation has explicit diagnostic");
  check(r.turn()==-sign,"stopped selection turns away from new close flank");
 }
}
void retained_physical_guards(){
 Robot r;r.start();r.obstacle({.29,.19,.19});
 for(unsigned i=0;i<120;++i){r.step();check(r.out.linear_mps==0&&r.out.angular_rps==0,"both physically obstructed sides never force a turn");}
 check(r.out.state=="AVOID_WAIT","real physical restriction remains a nonmoving wait");
 r.range={.29,.6,1.2};check(r.turn()==-1,"fresh physically open larger side recovers without task restart");
}
void turn_to_forward_has_no_zero_frame(){
 for(bool navigation:{false,true}){
  Robot r;r.start(navigation);r.obstacle({.29,1.,2.});check(r.turn()==-1,"handoff fixture starts actual right turn");
  r.range={2.,2.,2.};bool moved=false;
  for(unsigned i=0;i<150&&!moved;++i){r.step();moved=r.out.linear_mps>0;
   check(moved||std::abs(r.out.angular_rps)>1e-8,"safe confirmed turn-to-forward handoff inserts no zero command frame");
  }
  check(moved,"fresh rolling clear observations reach translation");
  check(r.out.state==(navigation?"AVOID_ADVANCE_050":"STRAIGHT_ADVANCE"),"handoff preserves straight/navigation distinction");
 }
}
void navigation_departure_side_stop(){
 for(int map_kind:{-1,1})for(unsigned flank:{1U,2U}){
  Robot r;r.start(true);r.n.bootstrap_active_=false;r.n.avoidance_active_=true;r.n.bypass_=Navigator::Bypass::MoveOut;
  r.n.bypass_target_=r.n.pose().yaw;r.n.bypass_leg_start_=r.n.total_distance_;r.n.progress_at_=r.now;r.n.progress_pose_=r.n.pose();
  check(r.n.grid().import_observations(std::vector<int8_t>(4096,map_kind),r.n.grid().origin(),r.now),"explicit map fixture imports");
  r.out=r.n.tick(r.now+.00001);check(r.out.state=="AVOID_ADVANCE_050"&&r.out.linear_mps>0,"local .50m departure ignores old/unknown map");
  r.range[flank]=.299;r.step();check(r.out.linear_mps==0&&r.out.angular_rps==0&&r.out.state=="AVOID_STOP","side close stops navigation departure too");
 }
}
void navigation_rejoin_near_stops_before_planner_or_turn(){
 for(bool pending_plan:{false,true})for(unsigned channel:{0U,1U,2U}){
  Robot r;r.start(true);r.n.bootstrap_active_=false;r.n.avoidance_active_=true;r.n.bypass_=Navigator::Bypass::None;
  if(pending_plan){PlannerRequest request;request.created_s=r.now;r.n.pending_=request;r.n.state_="WAIT_PLAN";}
  else{r.n.turn_active_=true;r.n.turn_target_yaw_=wrap(r.n.pose_.yaw+.3);r.n.state_="TURN";}
  auto values=r.range;values[channel]=.299;r.obstacle(values);
  check(!r.n.pending_&&!r.n.turn_active_,"rejoin cannot hide new near sonar behind avoidance-active/planner/turn flags");
 }
}
void real_range_regression(const std::string& path){
 std::ifstream input(path);check(bool(input),"recorded range fixture opens");
 const std::string bytes((std::istreambuf_iterator<char>(input)),{});picojson::value data;
 check(picojson::parse(data,bytes).empty(),"recorded range fixture parses");
 // Preserve the distinction: these are real range values paired with synthetic
 // healthy stationary odometry, not a claimed full reconstruction of a drive.
 for(const auto& sample:data.get("samples").get<picojson::array>())for(bool mirror:{false,true}){
  std::array<double,3> ranges{};const auto& values=sample.get("range_m").get<picojson::array>();
  for(unsigned ch=0;ch<3;++ch)ranges[ch]=values[ch].get<double>();
  if(mirror)std::swap(ranges[1],ranges[2]);
  Robot r;r.start();r.obstacle(ranges);
  check(r.out.linear_mps==0&&r.out.angular_rps==0,"recorded side close cannot emit the old forward request");
  if(std::min(ranges[1],ranges[2])<.18){for(unsigned i=0;i<50;++i){r.step();check(r.out.linear_mps==0&&r.out.angular_rps==0,"recorded .047m respects actual body-sweep guard");}}
  else check(r.turn()==(ranges[1]<.30?-1:1),"physically clear recorded side-near chooses away direction");
 }
}
}
int main(int argc,char**argv){try{three_beams();close_side_cannot_skip_turn();new_danger_reselects();retained_physical_guards();turn_to_forward_has_no_zero_frame();navigation_departure_side_stop();navigation_rejoin_near_stops_before_planner_or_turn();real_range_regression(argc>1?argv[1]:"tests/fixtures/three-sonar-real-20261028.json");
 std::cout<<"PASS v32 three-sonar avoidance "<<checks<<" checks; stop-first, away/larger side, release hysteresis, changed danger, physical guards, map-independent departure\n";
 }catch(const std::exception&e){std::cerr<<"FAIL v32 three-sonar "<<checks<<" "<<e.what()<<'\n';return 1;}}
