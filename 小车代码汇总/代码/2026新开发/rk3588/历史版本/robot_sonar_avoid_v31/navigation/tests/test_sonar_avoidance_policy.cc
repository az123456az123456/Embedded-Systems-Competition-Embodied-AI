// Production regression: only one logged map cell is available, not a full map.
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
void ck(bool yes,const char* why){++checks;if(!yes)throw std::runtime_error(why);}
Navigator ready(double front=.266,double left=.601,double right=1.249){
 Config c;c.velocity_control=true;Navigator n(c);SensorFrame f;
 f.timestamp_s=100;f.device_timestamp_s=100;f.imu_valid=true;f.sonar_m={front,left,right};
 f.sonar_observation_ids_valid=true;f.sonar_observation_id={100000,100000,100000};
 n.ingest(f,100);f.timestamp_s=100.05;f.device_timestamp_s=100.05;f.sequence=1;
 ck(n.ingest(f,100.05),"second source establishes odometry");return n;
}
void recorded_blocker(const std::string& path){
 std::ifstream in(path);ck(bool(in),"sparse logged-cell fixture opens");
 const std::string bytes((std::istreambuf_iterator<char>(in)),{});picojson::value j;ck(picojson::parse(j,bytes).empty(),"fixture parses");
 auto n=ready();const auto p=j.get("pose_m_rad").get<picojson::array>();n.pose_={p[0].get<double>(),p[1].get<double>(),p[2].get<double>()};
 const auto range=j.get("sonar_m").get<picojson::array>(),age=j.get("sonar_age_s").get<picojson::array>();
 for(unsigned ch=0;ch<3;++ch){n.frame_->sonar_m[ch]=range[ch].get<double>();n.frame_->sonar_age_s[ch]=age[ch].get<double>();}
 const auto origin=j.get("origin_m").get<picojson::array>();const auto g=j.get("guard");
 std::vector<int8_t> cells(4096,-1);const Cell cell{int(g.get("cell_x").get<double>()),int(g.get("cell_y").get<double>())};cells[OccupancyGrid::index(cell)]=1;
 ck(n.grid().import_observations(cells,{origin[0].get<double>(),origin[1].get<double>()},100.05),"exact logged occupied cell; others unknown");
 const auto values=n.grid().values();const auto observed=n.grid().observed_s();const auto rev=n.grid().revision();
 n.straight_assist_=true;n.bypass_=Navigator::Bypass::Select;n.state_="AVOID_WAIT";n.progress_at_=100.05;n.progress_pose_=n.pose_;
 auto out=n.service_bypass(100.05,.02);
#ifdef V30_EXPECT_OLD
 ck(n.state_=="AVOID_WAIT"&&out.angular_rps==0,"v29.2 reproduces logged occupied-padding zero command");
 ck(n.local_guard_diagnostics().reason=="known_occupied_safety_padding","old blocked reason matches diagnostic");
#else
 ck(n.state_=="AVOID_TURN_CLEAR"&&out.angular_rps<0,"already stopped selection starts rotation in the same tick");
 ck(n.bypass_side_==-1,"actual right1.249m larger than left .601m chooses RIGHT");
 out=n.service_bypass(100.07,.02);ck(out.angular_rps<0&&out.linear_mps==0,"next step rotates right");
 ck(n.local_guard_diagnostics().reason=="fresh_sonar_rotation_clear_map_not_authority","map not local guard authority");
#endif
 ck(n.grid().values()==values&&n.grid().observed_s()==observed&&n.grid().revision()==rev,"map values, unknowns, times, revision unchanged");
}
#ifndef V30_EXPECT_OLD
void selected_larger_side(){
 for(int side:{-1,1}){
  auto n=ready(.25,side>0?1.5:.60,side>0?.60:1.5);n.straight_assist_=true;n.bypass_=Navigator::Bypass::Select;
  n.service_bypass(100.05,.02);ck(n.bypass_side_==side,"both mirrors choose larger opening");
  auto out=n.service_bypass(100.07,.02);ck(out.angular_rps*side>0,"selected physical rotation sign");
  n.frame_->sonar_m[1]=side>0?.5:1.7;n.frame_->sonar_m[2]=side>0?1.7:.5;
  out=n.service_bypass(100.09,.02);ck(n.bypass_side_==side&&out.angular_rps*side>0,"reading swap cannot reverse established turn");
  n.frame_->sonar_m[side>0?1:2]=.19;out=n.service_bypass(100.11,.02);
  ck(out.angular_rps==0&&out.linear_mps==0&&n.bypass_side_==-side,"new single close flank stops before turning away");
  ck(n.reason()=="avoid_new_side_obstacle_stop_before_reverse","direction invalidation exact reason");
 }
 auto tie=ready(.25,.7,.7);tie.bypass_=Navigator::Bypass::Select;tie.service_bypass(100.05,.02);ck(tie.bypass_side_==-1,"tie retains clockwise/right fallback");
}
void front_only_and_maps(){
 auto n=ready(2.,.34,.34);n.straight_assist_=true;n.bypass_=Navigator::Bypass::Select;n.stationary_since_=99.;n.frame_->sonar_observation_id[0]=100050;
 ck(n.bypass_forward_clear(100.05),"front clear and both flanks above .32m release threshold");
 auto out=n.service_bypass(100.05,.02);ck(out.angular_rps==0&&out.linear_mps==0,"one distinct front sample still stopped");
 n.frame_->sonar_observation_id[0]=100100;out=n.service_bypass(100.1,.02);
 ck(n.bypass_==Navigator::Bypass::None&&n.state_=="STRAIGHT_ADVANCE","two front samples and released flanks resume straight");
 n.enabled_=true;n.straight_origin_=n.pose_;n.progress_at_=100.1;n.last_tick_=100.1;n.frame_->timestamp_s=100.12;out=n.tick(100.1201);
 ck(out.linear_mps>0&&out.angular_rps==0,"direct straight front-only resume");
 for(int kind:{-1,1}){
  auto leg=ready(2.,.34,.34);leg.bypass_=Navigator::Bypass::MoveOut;leg.bypass_target_=leg.pose_.yaw;leg.progress_at_=100.05;leg.progress_pose_=leg.pose_;leg.bypass_leg_start_=0;
  ck(leg.grid().import_observations(std::vector<int8_t>(4096,kind),leg.grid().origin(),100.05),"unknown/occupied test map imports");
  const auto values=leg.grid().values();const auto observed=leg.grid().observed_s();const auto rev=leg.grid().revision();
  out=leg.service_bypass(100.07,.02);ck(out.state=="AVOID_ADVANCE_050"&&out.linear_mps>0,"local nav .50m leg ignores historical map veto");
  ck(values==leg.grid().values()&&observed==leg.grid().observed_s()&&rev==leg.grid().revision(),"complete map evidence retained");
  leg.frame_->sonar_m[0]=.10;out=leg.service_bypass(100.09,.02);ck(out.linear_mps==0&&out.angular_rps==0&&out.state=="AVOID_STOP","real new front near stops local departure");
 }
 auto nav=ready();Pose p=nav.pose_;p.x+=.4;ck(!nav.motion_increment_free(p,100.05,false),"ordinary planned-map unknown guard preserved");
}
void real_faults(){
 auto n=ready(4.5,4.5,4.5);ck(n.start_straight(100.05),"fresh straight start");n.frame_->sonar_m[0]=.1;auto out=n.tick(100.0501);
 ck(out.state=="OBSTACLE_STOP"&&out.linear_mps==0&&out.angular_rps==0,"new front obstacle zero before selection");
 n.frame_->sonar_m[0].reset();n.frame_->timestamp_s=100.10;out=n.tick(100.1001);ck(n.enabled()&&out.state=="SENSOR_WAIT"&&out.angular_rps==0,"missing source zero WAIT");
 for(unsigned i=1;i<=101&&n.enabled();++i){n.frame_->timestamp_s=100.1+i*.05;out=n.tick(n.frame_->timestamp_s+.0001);}
 ck(!n.enabled()&&n.reason()=="sonar_unavailable_timeout","5s persistent source remains terminal");
 auto stop=ready(4.5,4.5,4.5);ck(stop.start_straight(100.05),"stop ready");stop.stop("phone_stop");out=stop.tick(100.1);ck(!stop.enabled()&&out.linear_mps==0&&out.angular_rps==0,"STOP cannot resume on range recovery");
 auto imu=ready(4.5,4.5,4.5);ck(imu.start_straight(100.05),"IMU ready");imu.frame_->imu_valid=false;out=imu.tick(100.1);ck(!imu.enabled()&&out.linear_mps==0&&out.angular_rps==0,"real IMU fault terminal");
 auto jump=ready(4.5,4.5,4.5);ck(jump.start_straight(100.05),"encoder ready");auto f=*jump.frame_;f.timestamp_s=100.1;f.device_timestamp_s=100.1;f.counts[0]=10000000;jump.ingest(f,100.1);out=jump.tick(100.1001);ck(jump.localization_diagnostics().latched&&out.linear_mps==0&&out.angular_rps==0,"encoder jump latched zero");
}
#endif
}
int main(int argc,char**argv){try{
 recorded_blocker(argc==2?argv[1]:"tests/fixtures/sonar-policy-blocker-20210101.json");
#ifndef V30_EXPECT_OLD
 selected_larger_side();front_only_and_maps();real_faults();
#endif
 std::cout<<"PASS sonar-only local avoidance "<<checks<<" checks; sparse logged blocker (not absent full map), larger opening, flank release and real fault guards\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL sonar policy "<<checks<<' '<<e.what()<<'\n';return 1;}}
