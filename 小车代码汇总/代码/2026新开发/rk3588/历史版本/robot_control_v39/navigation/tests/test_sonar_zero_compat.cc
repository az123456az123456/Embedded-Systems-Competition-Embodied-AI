// Compatibility of the unchanged v29 production decoders/bridge/control with
// FPGA-normalized zero=>4500. This file does not implement a UART parser.
#include "../navcore.h"
#include "../sensor_bridge.h"
#include "../protocol.h"
#include "../sonar_diagnostics.h"
#include "../../dashboard/phone/follow_sensors.h"
extern "C" {
#include "../../dashboard/src/sensor_data.h"
}
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace robot_nav;
namespace {
unsigned checks=0;
void ck(bool okay,const char* why){++checks;if(!okay)throw std::runtime_error(why);}
constexpr double BASE_S=100.;
constexpr uint64_t BASE_MS=100000;
const uint8_t HOST[6]={2,3,4,5,6,7};
void put16(uint8_t*p,uint16_t n){p[0]=n>>8;p[1]=n;}
void put32(uint8_t*p,uint32_t n){p[0]=n>>24;p[1]=n>>16;p[2]=n>>8;p[3]=n;}
std::vector<uint8_t> envelope(const std::vector<uint8_t>& payload){
 ck(payload.size()==64,"actual SNS1 payload has 64 bytes");std::vector<uint8_t> bytes(80,0);
 std::fill(bytes.begin(),bytes.begin()+6,255);const uint8_t source[6]={2,0xac,0x13,0x80,0,1};std::copy(source,source+6,bytes.begin()+6);
 bytes[12]=0x88;bytes[13]=0xb8;std::copy(payload.begin(),payload.end(),bytes.begin()+14);
 put16(bytes.data()+78,navwire::crc16(payload.data(),payload.size()));return bytes;
}
Config configuration(){Config c;c.velocity_control=true;return c;}
struct Pipe {
 Config config=configuration();Navigator nav{config};NavigationInputs input{config};sensor_state ui{};
 uint32_t device=0,sequence=0,imu_sequence=0;double yaw=0;std::array<double,4> counts{};
 std::array<uint32_t,3> source_ms{};std::array<bool,3> have_source{};
 std::array<uint16_t,3> distances{{4500,4500,4500}};Control output;
 navwire::Sensors latest;uint64_t now_ms=BASE_MS;unsigned accepts=0;
 Pipe(){sensor_init(&ui);input.update_imu(imu(0),BASE_S+.008);for(unsigned i=0;i<12;++i)step(7,false);ck(nav.localization_valid(now()),"actual bridge establishes source-aligned odometry");}
 double now()const{return BASE_S+device/1000.+.008;}
 robotnav::GatewayTelemetry imu(uint32_t uptime){
  robotnav::GatewayTelemetry t;t.available=t.imu_valid=t.imu_fresh=t.imu_bias_valid=t.imu_timestamp_valid=true;
  t.imu_calibrating=t.imu_action_pending=false;t.imu_timestamp_ms=uptime;t.imu_sequence=++imu_sequence;t.yaw_deg=yaw*180/vehicle_profile::pi;t.imu_sample_age_ms=5;
  t.request_started_ms=BASE_MS+uptime+5;t.received_ms=t.request_started_ms+3;return t;
 }
 bool receive(const std::vector<uint8_t>& bytes,bool tick){
  sensor_sample display{};navwire::Sensors sensors;
  const bool a=sensor_decode(bytes.data(),bytes.size(),&display);
  const bool b=navwire::sensorFrame(bytes.data(),bytes.size(),HOST,sensors);
  ck(a==b,"display and navigation SNS1 integrity decisions agree on test transport");
  if(!a)return false;
  ck(sensor_update(&ui,&display,BASE_MS+device+1),"production UI accepts actual new packet");
  latest=sensors;now_ms=BASE_MS+device+8;input.update_imu(imu(device),now());
  input.enqueue(sensors,BASE_S+device/1000.+.001);input.drain(nav,now());
  ck(input.fault_reason().empty(),input.fault_reason().c_str());
  ck(nav.frame().has_value()&&nav.frame()->sequence==sensors.sequence,"actual NavigationInputs ingests source-aligned packet");
  if(tick){output=nav.tick(now()+.0001);}++accepts;return true;
 }
 void step(unsigned update_mask,bool tick=true){
  constexpr double dt=.05;
  const double vl=output.linear_mps-output.angular_rps*config.track_m*.5,vr=output.linear_mps+output.angular_rps*config.track_m*.5;
  const auto previous=counts;for(unsigned w=0;w<4;++w)counts[w]+=((config.left_wheel_mask&(1U<<w))?vl:vr)*dt*config.encoder_ticks_per_turn/config.wheel_circumference_m*config.encoder_signs[w];
  yaw+=(vr-vl)/config.track_m*dt;device+=50;
  std::vector<uint8_t> p(64,0);std::memcpy(p.data(),"SNS1",4);p[4]=1;put32(p.data()+8,++sequence);put32(p.data()+12,device);
  unsigned valid=0;
  for(unsigned ch=0;ch<3;++ch){if(update_mask&(1U<<ch)){source_ms[ch]=device;have_source[ch]=true;}
   const uint16_t age=have_source[ch]?uint16_t(std::min(65535u,device-source_ms[ch])):65535;
   if(have_source[ch]&&age<=1000)valid|=1U<<ch;
   put16(p.data()+48+2*ch,distances[ch]);put16(p.data()+54+2*ch,age);
  }
  p[5]=valid;p[6]=7U^valid;put16(p.data()+60,50);
  for(unsigned w=0;w<4;++w){put32(p.data()+16+4*w,uint32_t(int32_t(std::llround(counts[w]))));put32(p.data()+32+4*w,uint32_t(int32_t(std::llround(counts[w])-std::llround(previous[w]))));}
  ck(receive(envelope(p),tick),"well-formed SNS1 transport accepted");
 }
 void start(){ck(nav.start_straight(now()),"4500 valid source supports requested straight session");output=nav.command();}
};
void normalized_far_and_next_near(){
 Pipe p;p.start();
 for(unsigned i=0;i<60;++i){p.step(7);ck(p.nav.enabled(),"fresh4500 straight remains authorized");}
 ck(p.output.linear_mps>.1,"production physical velocity command advances on valid normalized far");
 for(unsigned ch=0;ch<3;++ch){double metres=0;char text[32];
  ck(sensor_distance(&p.ui,ch,p.now_ms,&metres)==SENSOR_VALID&&metres==4.5,"display preserves exact4500mm value");
  ck(sensor_distance_text(&p.ui,ch,p.now_ms,text,sizeof(text))==SENSOR_VALID&&!std::strcmp(text,">4 m"),"existing UI displays normalized far as >4m");
  ck(p.nav.frame()->sonar_m[ch]&&*p.nav.frame()->sonar_m[ch]==4.5,"actual decoder->bridge->Navigator retains4.5m");
 }
 const auto follow=robotphone::followRanges(p.ui,p.now_ms);ck(follow.valid[0]&&follow.valid[1]&&follow.valid[2],"old follow sensor helper accepts4500fresh");
 const auto diagnostic=sonar_diagnostics_json(p.latest,true,BASE_S+p.device/1000.+.001,p.now(),p.config);
 picojson::value d;ck(picojson::parse(d,diagnostic).empty(),"actual sonar diagnostic JSON parses");
 ck(d.get("channels").get<picojson::array>()[0].get("navigation_usable").get<bool>(),"phone diagnostic marks4500 usable");
 for(auto value:p.nav.grid().values())ck(value<=0,"censored4500 production stream creates no occupied endpoint");
 // A corrupt new transport packet cannot become a near OR a far observation.
 std::vector<uint8_t> raw(64,0);std::memcpy(raw.data(),"SNS1",4);raw[4]=1;raw[5]=7;put32(raw.data()+8,p.sequence+1);put32(raw.data()+12,p.device+1);
 for(unsigned ch=0;ch<3;++ch)put16(raw.data()+48+ch*2,4500);
 auto damaged=envelope(raw);damaged[14+48]^=1;
 const auto last_packet=p.nav.frame()->sequence;const auto source=p.nav.frame()->sonar_observation_id;const auto received=p.ui.received_ms;const auto count=p.accepts;
 ck(!p.receive(damaged,false),"bad SNS1 CRC rejected by both actual decoders");
 ck(p.nav.frame()->sequence==last_packet&&p.nav.frame()->sonar_observation_id==source&&p.ui.received_ms==received&&p.accepts==count,"rejected packet cannot renew observation or UI freshness");
 p.distances[0]=100;p.step(1);
 ck(p.output.linear_mps==0&&p.output.angular_rps==0&&p.nav.state()=="OBSTACLE_STOP","first genuine new near after4500 immediately commands zero");
 ck(p.nav.frame()->sonar_m[0]&&*p.nav.frame()->sonar_m[0]==.1,"new near is not smoothed back into4500 by RK");
}
void source_silence_is_not_a_new_far(){
 Pipe p;p.start();for(unsigned i=0;i<40;++i)p.step(7);
 const auto last_id=p.nav.frame()->sonar_observation_id;
 const auto pose=p.nav.pose();const auto cell=p.nav.grid().cell(pose.x+2.,pose.y);const auto index=p.nav.grid().index(cell);const auto stamp=p.nav.grid().observed_s()[index];
 bool saw_wait=false,saw_ui_stale=false,saw_follow_stale=false;const auto began=p.device;
 for(unsigned i=0;i<140;++i){p.step(0);
  ck(p.nav.frame()->sonar_observation_id==last_id,"fresh SNS1 packets with growing age retain actual UART source ID");
  ck(p.nav.grid().observed_s()[index]==stamp,"silence cannot refresh old free map evidence");
  ck(sensor_link_fresh(&p.ui,p.now_ms),"sensor transport remains live during raw source silence");
  if(p.device-began>500){const auto ranges=robotphone::followRanges(p.ui,p.now_ms);saw_follow_stale=saw_follow_stale||!ranges.valid[0];}
  if(p.device-began>1000)saw_ui_stale=saw_ui_stale||sensor_distance(&p.ui,0,p.now_ms,nullptr)==SENSOR_STALE;
  if(p.nav.enabled()&&p.nav.state()=="SENSOR_WAIT"){saw_wait=true;ck(p.output.linear_mps==0&&p.output.angular_rps==0,"expired source zero WAIT retains authorized session");}
 }
 ck(saw_wait&&saw_ui_stale&&saw_follow_stale,"independent source ages expire nav/UI/follow despite fresh transport");
 ck(p.nav.enabled()&&p.output.linear_mps==0&&p.output.angular_rps==0,"persistent source silence preserves authorized zero wait");
 for(unsigned i=0;i<30;++i)p.step(7);
 ck(p.nav.enabled(),"restored source retains same authorized task after measured stop recovery");
}
std::vector<uint8_t> hex(const std::string&s){ck(s.size()==128,"trace is one64B SNS1 payload");std::vector<uint8_t>b;for(size_t i=0;i<s.size();i+=2)b.push_back(uint8_t(std::stoul(s.substr(i,2),nullptr,16)));return b;}
void actual_rtl_trace(const std::string& file){
 std::ifstream in(file);ck(bool(in),"actual RTL trace opens");sensor_state display;sensor_init(&display);Config c=configuration();Navigator nav(c);NavigationInputs input(c);
 unsigned rows=0,far=0,near=0,source_repeats=0;bool trace_started=false,trace_near_stopped=false;std::array<uint32_t,3> last_id{};std::array<bool,3> seen{};uint32_t imu_seq=0;std::string line;
 robotnav::GatewayTelemetry seed;seed.available=seed.imu_valid=seed.imu_fresh=seed.imu_bias_valid=seed.imu_timestamp_valid=true;seed.imu_calibrating=seed.imu_action_pending=false;seed.imu_sequence=++imu_seq;seed.imu_timestamp_ms=0;seed.imu_sample_age_ms=5;seed.request_started_ms=BASE_MS+5;seed.received_ms=BASE_MS+8;input.update_imu(seed,BASE_S+.008);
 while(std::getline(in,line)){
  const auto tag=line.find("SNS1 t_ms=");if(tag==std::string::npos)continue;const auto mark=line.find("packet=",tag);ck(mark!=std::string::npos,"actual RTL trace payload marker");
  const auto data=hex(line.substr(mark+7,128));const auto bytes=envelope(data);sensor_sample ui;navwire::Sensors sensors;
  ck(sensor_decode(bytes.data(),bytes.size(),&ui),"production UI decoder accepts actual UART->capture snapshot");
  ck(navwire::sensorFrame(bytes.data(),bytes.size(),HOST,sensors),"production nav decoder accepts actual capture snapshot");
  const auto now=BASE_S+sensors.uptime_ms/1000.+.008;
  ck(sensor_update(&display,&ui,BASE_MS+sensors.uptime_ms+1),"actual trace sequence/uptime accepted without synthetic freshness");
  robotnav::GatewayTelemetry t;t.available=t.imu_valid=t.imu_fresh=t.imu_bias_valid=t.imu_timestamp_valid=true;t.imu_calibrating=t.imu_action_pending=false;t.imu_timestamp_ms=sensors.uptime_ms;t.imu_sequence=++imu_seq;t.yaw_deg=0;t.imu_sample_age_ms=5;t.request_started_ms=BASE_MS+sensors.uptime_ms+5;t.received_ms=t.request_started_ms+3;
  input.update_imu(t,now);input.enqueue(sensors,BASE_S+sensors.uptime_ms/1000.+.001);input.drain(nav,now);ck(input.fault_reason().empty(),input.fault_reason().c_str());
  ck(nav.frame()&&nav.frame()->sequence==sensors.sequence,"actual trace aligned into production navigation without stale frame substitution");
  if(!trace_started&&nav.localization_valid(now)&&sensors.valid==7&&sensors.distance_mm[0]==4500){
   ck(nav.start_straight(now),"actual initial normalized4500 snapshots permit straight session");trace_started=true;
   for(auto value:nav.grid().values())ck(value<=0,"actual zero4500 capture never paints occupied endpoint");
  }
  if(trace_started&&!trace_near_stopped&&sensors.valid==7&&sensors.distance_mm[0]<=200){
   const auto command=nav.tick(now);
   ck(command.linear_mps==0&&command.angular_rps==0&&nav.state()=="OBSTACLE_STOP","actual nextnear UART capture reaches production immediate zero stop");trace_near_stopped=true;
  }
  for(unsigned ch=0;ch<3;++ch){
   if((sensors.valid&(1U<<ch))&&!(sensors.fault&(1U<<ch))&&sensors.age_ms[ch]<500){
    double metres=0;ck(sensor_distance(&display,ch,BASE_MS+sensors.uptime_ms+8,&metres)==SENSOR_VALID,"actual normalized fresh source is UI valid");
    if(sensors.distance_mm[ch]==4500){++far;char text[32];sensor_distance_text(&display,ch,BASE_MS+sensors.uptime_ms+8,text,sizeof(text));ck(!std::strcmp(text,">4 m"),"actual zero mapping display is censored far");}
    if(sensors.distance_mm[ch]>=30&&sensors.distance_mm[ch]<=200)++near;
   }
   if(sensors.age_ms[ch]!=65535){const uint32_t id=sensors.uptime_ms-sensors.age_ms[ch];if(seen[ch]&&last_id[ch]==id)++source_repeats;last_id[ch]=id;seen[ch]=true;}
  }
  ++rows;
 }
 ck(trace_started&&trace_near_stopped,"actual UART capture exercises far session then fresh-near stop");
 ck(rows==39&&far>0&&near>0&&source_repeats>0,"actual RTL includes far, immediate near and retained-source silence snapshots");
 std::cout<<"TRACE rows="<<rows<<" fresh4500_channels="<<far<<" near_channels="<<near<<" retained_source_ids="<<source_repeats<<'\n';
}
}
int main(int argc,char**argv){try{
 normalized_far_and_next_near();source_silence_is_not_a_new_far();
 if(argc==2)actual_rtl_trace(argv[1]);
 std::cout<<"PASS unchanged-v29 sonar-zero compatibility "<<checks<<" checks; production SNS1 decoders/display/bridge/nav, censored4500, immediate new near, bad-CRC no refresh, live-transport source expiry and5sstop; no hardware IO\n";
 }catch(const std::exception&e){std::cerr<<"FAIL sonar-zero compatibility "<<checks<<" "<<e.what()<<'\n';return 1;}}
