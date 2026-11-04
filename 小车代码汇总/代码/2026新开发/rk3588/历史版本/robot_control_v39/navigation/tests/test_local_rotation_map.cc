// Actual recorded map replay. Exposes private production guards for tests only.
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#ifndef NAVCORE_HEADER
#define NAVCORE_HEADER "../navcore.h"
#endif
#define private public
#include NAVCORE_HEADER
#undef private
#include "../vendor/picojson.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace robot_nav;
namespace {
unsigned checks=0;
void ck(bool v,const char* why){++checks;if(!v)throw std::runtime_error(why);}
struct Fixture {
 Config config;Pose pose;double now=0;Point origin;std::vector<int8_t> cells;
 std::array<double,3> ranges{},ages{};
 explicit Fixture(const std::string& path){
  std::ifstream in(path);ck(bool(in),"actual fixture opens from packaged source");
  std::string bytes((std::istreambuf_iterator<char>(in)),{});picojson::value j;
  ck(picojson::parse(j,bytes).empty(),"actual fixture JSON parses");
  const auto p=j.get("pose_m_rad").get<picojson::array>();pose={p[0].get<double>(),p[1].get<double>(),p[2].get<double>()};
  now=j.get("now_s").get<double>();const auto o=j.get("origin_m").get<picojson::array>();origin={o[0].get<double>(),o[1].get<double>()};
  config.velocity_control=true;config.resolution_m=j.get("resolution_m").get<double>();
  const auto d=j.get("sonar_m").get<picojson::array>(),a=j.get("sonar_age_s").get<picojson::array>();
  for(unsigned i=0;i<3;++i){ranges[i]=d[i].get<double>();ages[i]=a[i].get<double>();}
  const auto map=j.get("cells").get<std::string>();ck(map.size()==4096,"actual map has all4096 classified cells");
  for(char c:map){ck(c=='?'||c=='.'||c=='#',"recorded legend only");cells.push_back(c=='#'?1:c=='.'?0:-1);}
 }
 Navigator make()const{
  Navigator n(config);n.pose_=pose;SensorFrame f;f.timestamp_s=now-.10;f.device_timestamp_s=f.timestamp_s;f.imu_valid=true;f.yaw_rad=pose.yaw;
  for(unsigned i=0;i<3;++i){f.sonar_m[i]=ranges[i];f.sonar_age_s[i]=ages[i];}
  n.ingest(f,f.timestamp_s);f.timestamp_s=now;f.device_timestamp_s=now;f.sequence=1;
  ck(n.ingest(f,now),"recorded-world pose supplies real100ms stationary source history");
  ck(n.grid().import_observations(cells,origin,now),"exact classified map imported without erasing occupied cells");return n;
 }
};
void replay(const Fixture& f){
 auto n=f.make();const auto values=n.grid().values();const auto observed=n.grid().observed_s();const auto rev=n.grid().revision();
 for(double angle:{-.08,-.04,-.02,-.012,-.01,.01,.012,.02,.04,.08}){
  ck(n.bypass_rotation_clear(wrap(n.pose_.yaw+angle),f.now),"historical recorded map does not veto fresh sonar local rotation");
  const auto& d=n.local_guard_diagnostics();ck(!d.blocked&&d.pure_rotation&&d.channel==-1&&d.cell_x==-1,"local guard does not consult map cells");
  ck(d.reason=="fresh_sonar_rotation_clear_map_not_authority","fresh sonar provenance explicit");
 }
 ck(n.grid().values()==values&&n.grid().observed_s()==observed&&n.grid().revision()==rev,"entire map evidence unchanged");
 auto state=f.make();state.straight_assist_=true;state.bypass_=Navigator::Bypass::Select;
 state.state_="AVOID_WAIT";state.progress_at_=f.now;state.progress_pose_=f.pose;
 auto control=state.service_bypass(f.now,.02);
 ck(state.state_=="AVOID_TURN_CLEAR"&&std::abs(control.angular_rps)>0,"already stopped nomination starts turn without an empty control frame");
 control=state.service_bypass(f.now+.02,.02);
 const int side=f.ranges[1]>f.ranges[2]?1:-1;
 ck(state.state_=="AVOID_TURN_CLEAR"&&control.linear_mps==0&&control.angular_rps*side>0,"front-obstacle physical turn follows larger recorded sonar");
 for(double dx:{-.00005,.00005})for(double dy:{-.00005,.00005})for(double da:{-.00005,.00005})for(int direction:{-1,1}){
  n.pose_={f.pose.x+dx,f.pose.y+dy,f.pose.yaw+da};
  ck(n.bypass_rotation_clear(wrap(n.pose_.yaw+direction*.012),f.now),"rounding bounds cannot restore map veto");
 }
 n.pose_=f.pose;Pose translating=f.pose;translating.x+=.00001;translating.yaw+=.012;
 ck(!n.footprint_increment_free(f.pose,f.pose,translating,f.now,true),"ordinary planned-map translated sweep keeps original map guard");
 auto occupied=f.cells;occupied[OccupancyGrid::index({29,30})]=1;
 ck(n.grid().import_observations(occupied,f.origin,f.now),"explicit mapped blocker imports");
 for(int direction:{-1,1})ck(n.bypass_rotation_clear(wrap(n.pose_.yaw+direction*.012),f.now),"even known occupied local cells are not sonar-authority veto");
 ck(n.grid().import_observations(f.cells,f.origin,f.now),"original map restored");
 for(unsigned ch=0;ch<3;++ch){n.frame_->sonar_m[ch]=.10;ck(!n.bypass_rotation_clear(wrap(n.pose_.yaw+.012),f.now),"real measured near body radius still blocks");n.frame_->sonar_m[ch]=f.ranges[ch];}
 n.frame_->sonar_m[1]=.19;ck(!n.bypass_rotation_clear(wrap(n.pose_.yaw+.012),f.now),"left raw200mm floor preserved");n.frame_->sonar_m[1]=f.ranges[1];
 n.frame_->sonar_m[2]=.19;ck(!n.bypass_rotation_clear(wrap(n.pose_.yaw-.012),f.now),"right raw200mm floor preserved");
 auto unknown=f.make();ck(unknown.grid().import_observations(std::vector<int8_t>(4096,-1),f.origin,f.now),"unknown map imports");
 Pose target=f.pose;target.yaw+=.012;
 ck(!unknown.footprint_increment_free(f.pose,f.pose,target,f.now,false),"ordinary planned-map unknown guard preserved");
 ck(unknown.bypass_rotation_clear(target.yaw,f.now),"same unknown map cannot veto local sonar rotation");
}
void source_and_motion_guards(){
 Config c;c.velocity_control=true;Navigator n(c);SensorFrame f;f.timestamp_s=10;f.device_timestamp_s=10;f.imu_valid=true;f.sonar_m={4.5,4.5,4.5};n.ingest(f,10);
 f.timestamp_s=10.05;f.device_timestamp_s=f.timestamp_s;ck(n.ingest(f,10.05),"two real timestamps establish odometry");
 ck(n.start_straight(10.05),"unchanged physical setpoint mode starts");
 f.timestamp_s=10.10;f.device_timestamp_s=f.timestamp_s;f.sonar_m[0].reset();n.ingest(f,10.10);auto cmd=n.tick(10.1001);
 ck(n.enabled()&&cmd.state=="SENSOR_WAIT"&&cmd.linear_mps==0&&cmd.angular_rps==0,"missing source remains zero WAIT with same authorization");
 for(unsigned i=1;i<=101&&n.enabled();++i){f.timestamp_s=10.1+i*.05;f.device_timestamp_s=f.timestamp_s;n.ingest(f,f.timestamp_s);cmd=n.tick(f.timestamp_s+.0001);}
 ck(n.enabled()&&n.state()=="SENSOR_WAIT"&&cmd.linear_mps==0&&cmd.angular_rps==0,"5s source-only failure remains zero wait");
 Navigator stale(c);f=SensorFrame{};f.imu_valid=true;f.timestamp_s=15;f.device_timestamp_s=15;f.sonar_m={4.5,4.5,4.5};stale.ingest(f,15);
 f.timestamp_s=15.05;f.device_timestamp_s=15.05;stale.ingest(f,15.05);ck(stale.start_straight(15.05),"fresh sources before age-expiry test");
 f.timestamp_s=15.1;f.device_timestamp_s=15.1;f.sonar_age_s={.8,.8,.8};stale.ingest(f,15.1);cmd=stale.tick(15.1001);
 ck(stale.enabled()&&cmd.state=="SENSOR_WAIT"&&cmd.reason=="sonar_stale_zero_wait"&&cmd.linear_mps==0&&cmd.angular_rps==0,"explicit old source ages remain zero WAIT even with valid4500 payload");
 Navigator jump(c);f=SensorFrame{};f.imu_valid=true;f.timestamp_s=20;f.device_timestamp_s=f.timestamp_s;f.sonar_m={4.5,4.5,4.5};jump.ingest(f,20);
 f.timestamp_s=20.05;f.device_timestamp_s=f.timestamp_s;jump.ingest(f,20.05);ck(jump.start_straight(20.05),"physical source established before jump");
 f.timestamp_s=20.1;f.device_timestamp_s=f.timestamp_s;f.counts[0]=10000000;jump.ingest(f,20.1);cmd=jump.tick(20.1001);
 ck(jump.localization_diagnostics().latched&&jump.localization_diagnostics().first_reason=="encoder_jump"&&cmd.linear_mps==0,"real odometry jump remains latched zero");
#ifndef V29_2_EXPECT_LEGACY
 const auto d=jump.motion_gate_diagnostics(20.1001);ck(d.frame_available&&d.frame_sequence==f.sequence,"motion diagnostics refer to actual aligned frame");
 ck(d.target_linear_mps==0&&d.target_angular_rps==0,"diagnostics cannot authorize motion after real fault");
#endif
}
}
int main(int argc,char**argv){try{
 Fixture f(argc==2?argv[1]:"tests/fixtures/local-rotation-map-20261027.json");replay(f);source_and_motion_guards();
 std::cout<<"PASS v32 sonar-only local rotation with preserved planned-map guards ";
 std::cout<<checks<<" checks: original4096map/two directions/16roundedbounds/innerobstacle/translatedsweep/near/sourceexpiry/realodom; no map clearing or hardware IO\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL local rotation "<<checks<<' '<<e.what()<<'\n';return 1;}}
