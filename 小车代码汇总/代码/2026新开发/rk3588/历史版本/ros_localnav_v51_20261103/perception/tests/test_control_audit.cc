#include "control_audit.h"
extern "C" {
#include "wronglog_state.h"
}
#include "../../navigation/vendor/picojson.h"
#include <cassert>
#include <cerrno>
#include <fstream>
#include <iostream>
#include <iterator>
#include <new>
#include <unistd.h>
/* Only this caller thread fails allocations. The logger worker and controller
 * fixture setup remain ordinary production code. */
thread_local bool audit_fail_allocations=false;
thread_local unsigned audit_allocation_failures=0;
__attribute__((noinline)) void *operator new(std::size_t size){
 if(audit_fail_allocations){++audit_allocation_failures;throw std::bad_alloc();}
 if(void *p=std::malloc(size?size:1))return p;
 throw std::bad_alloc();
}
__attribute__((noinline)) void *operator new[](std::size_t size){return ::operator new(size);}
__attribute__((noinline)) void operator delete(void *p) noexcept {std::free(p);}
__attribute__((noinline)) void operator delete[](void *p) noexcept {std::free(p);}
__attribute__((noinline)) void operator delete(void *p,std::size_t) noexcept {std::free(p);}
__attribute__((noinline)) void operator delete[](void *p,std::size_t) noexcept {std::free(p);}
using namespace robotphone;
namespace {
int checks=0;
void check(bool v,const char *label){++checks;if(!v){std::cerr<<"FAIL "<<label<<'\n';std::exit(1);}}
Config config(){Config c;c.left_motor_mask=10;c.bearing.enabled=true;for(auto &e:c.bearing.eyes){e.calibrated=true;e.fx_px=600;e.cx_px=480;}return c;}
void ready(Controller &c,uint64_t now){c.updateMotorLink(now,true);c.updateArcProtocol(true);c.updateAngleProtocol(true);c.updateImu({now,true,0,0,0});c.updateRanges({now,{true,true,true},{2,2,2}});}
struct Fixture {
 Controller c{config()};ControlAudit audit;MotorDiagnostics d;MotorAck a;
 Fixture(){d.last_ack_ms=1000;d.tx_frames=17;d.ack_frames=17;a.sequence=17;a.left_mask=10;a.invert_mask=15;a.configured=true;ready(c,1000);audit.observe(1000,c.state(),d,a,10,15,1000,0);}
 void observe(uint64_t now){audit.observe(now,c.state(),d,a,10,15,1000,c.state().last_control_seq);}
 void arm(){check(c.arm(1,1000),"actual arm");audit.command(1000,"arm",1,200,"",false,c.state());observe(1000);}
};
}
int main(){
 char path[]="/tmp/acg138-control-audit-XXXXXX";int fd=mkstemp(path);check(fd>=0,"temp log");close(fd);setenv("ROBOT_WRONG_LOG_PATH",path,1);wrong_log_start("dashboard-test","v26.3");
 {
  Fixture f;f.arm();check(f.c.drive(2,.3,0,false,1010),"actual drive before log allocation fault");f.c.tick(1020);
  const State before=f.c.state();JsonObject input;std::string error;
  check(parseFlatJson("{\"client_reason\":\"telemetry_timeout\",\"client_diag_id\":7}",input,error),"prepare client log allocation fault");
  audit_fail_allocations=true;
  f.audit.observe(1020,f.c.state(),f.d,f.a,10,15,1000,2);
  f.audit.command(1020,"drive",2,409,"logging_allocation_failure_test",true,f.c.state());
  const auto ignored=ControlAudit::clientReported(input);
  audit_fail_allocations=false;
  check(audit_allocation_failures>=3,"all three logging entry points exercised allocation failures");
  check(!ignored.present,"failed diagnostic parse safely returns empty metadata");
  check(f.c.state().armed==before.armed&&f.c.state().left==before.left&&f.c.state().right==before.right&&f.c.state().reason==before.reason&&f.c.state().last_control_seq==before.last_control_seq,"failed logging leaves controller output and lease sequence unchanged");
  f.observe(1030);check(f.c.drive(3,0,0,false,1040)&&f.c.state().armed,"controller remains usable after logging allocation failure");
 }
 {
  Fixture f;f.arm();check(f.c.drive(2,.3,0,false,1010),"actual manual drive");f.c.tick(1020);f.observe(1020);
  ready(f.c,1400);f.c.tick(1400);f.observe(1400);check(f.c.state().reason=="motion_command_stale","actual control heartbeat timeout");
 }
 {
  Fixture f;f.arm();f.c.updateMotorLink(1020,false);f.observe(1020);check(f.c.state().reason=="motor_link_not_ready","actual ACK loss");
 }
 {
  Fixture f;f.arm();check(f.c.drive(2,.3,0,false,1010),"drive before release");f.observe(1010);
  check(f.c.drive(3,0,0,false,1020),"actual joystick release");f.audit.command(1020,"drive",3,200,"",true,f.c.state());f.observe(1020);check(f.c.state().armed,"release leaves armed");
  f.c.disarm(1030);f.audit.command(1030,"disarm",4,200,"",true,f.c.state());f.observe(1030);
  JsonObject input;std::string error;check(parseFlatJson("{\"client_reason\":\"telemetry_timeout\",\"client_mono_ms\":91,\"client_state_age_ms\":1501,\"client_lease_age_ms\":23,\"client_diag_id\":7}",input,error),"parse client diagnostics");
  auto client=ControlAudit::clientReported(input);check(client.present&&client.reason=="telemetry_timeout"&&client.id==7,"known client diagnostics preserved without RK-clock math");f.audit.command(1031,"disarm",5,200,"",false,f.c.state(),false,client);
  check(parseFlatJson("{\"client_reason\":\"Bearer-secret-must-not-appear\",\"client_mono_ms\":-1,\"client_diag_id\":1.5}",input,error),"parse malformed diagnostics");client=ControlAudit::clientReported(input);check(!client.present&&client.mono_ms==-1&&client.id==-1,"unknown reason and invalid counters sanitized");
 }
 {
  Fixture f;Detection box;box.class_id=1;box.eye=0;box.score=.9;box.x1=200;box.x2=230;box.y1=200;box.y2=300;
  f.c.updateVision({1,1000,{box}});f.arm();check(f.c.select(1,f.c.vision().boxes[0].track_id,1000),"actual target select");check(f.c.follow(2,1000),"actual follow");f.observe(1000);
  f.c.updateRanges({1020,{false,true,true},{2,2,2}});f.c.tick(1020);f.observe(1020);check(f.c.state().reason=="sonar_stale","actual invalid sonar stops follow");
 }
 {
  Fixture f;f.arm();f.c.updateMotorLink(1040,false);check(f.c.drive(2,.5,0,false,1040)&&f.c.state().armed&&f.c.state().left==0,"valid blocked drive pauses without revoking");
  check(!f.c.drive(3,2,0,false,1041)&&f.c.state().armed,"actual invalid drive rejected without disarming");
  f.audit.command(1041,"drive",3,409,"invalid_command",true,f.c.state());f.observe(1041);
 }
 {
  Fixture f;SoftcoreMonitor passive;passive.socketState(true);
  f.audit.observe(1200,f.c.state(),f.d,f.a,10,15,1000,0,&passive,false);
  check(!f.c.state().armed,"passive diagnostic cannot arm controller");
 }
 sensor_state sensors{};sensors.have_sample=1;sensors.received_ms=1000;sensors.sample.sequence=77;sensors.sample.uptime_ms=2000;sensors.sample.valid_mask=6;sensors.sample.fault_mask=1;
 sensors.sample.distance_mm[0]=1456;sensors.sample.age_ms[0]=199;sensors.sample.position[2]=-60528;sensors.sample.illegal_mask=4;
 wronglog_sensor_snapshot(&sensors,1050,-1,ENETDOWN,80,79,77);
 imu_state imu{};imu_init(&imu);imu.sample.sequence=83;imu.sample.timestamp=2010;imu.sample.ag_age=7;imu.received_ms=1050;imu.have_packet=1;imu.yaw=179.99;
 wronglog_imu_snapshot(&imu,1060,-1,ENETDOWN,83,83,80);
 wronglog_network_error("imu_network","receive_failed",ENETDOWN,-1);
 wrong_log_shutdown();
 std::ifstream log(path);std::string line,whole;bool lease=false,ack=false,sonar=false,explicit_stop=false,reject=false,raw=false,imuseq=false,network=false,zero_error=false,client_seen=false,softcore_seen=false;
 while(std::getline(log,line)){
  whole+=line; picojson::value v;check(picojson::parse(v,line).empty(),"all records valid JSON");const auto &o=v.get<picojson::object>();
  auto it=o.find("reason");std::string reason=it!=o.end()&&it->second.is<std::string>()?it->second.get<std::string>():"";
  const auto &data=o.at("data").get<picojson::object>();
  lease|=reason=="motion_command_stale";ack|=reason=="motor_link_not_ready";sonar|=reason=="sonar_stale";
  explicit_stop|=reason=="explicit_control_stop";zero_error|=reason=="joystick_released"||reason=="navigation_zero";
  if(reason=="invalid_command"&&data.count("http_status")){reject=data.at("http_status").get<double>()==409&&data.at("controller_reason").get<std::string>()=="invalid_command"&&data.at("sequence").get<double>()==3;}
  if(data.count("retained_distance_mm")){raw=true;check(data.at("retained_distance_mm").get<picojson::array>()[0].get<double>()==1456,"retained range not fabricated raw code");check(data.at("total_range_age_ms").get<picojson::array>()[0].get<double>()==249,"physical range age preserved");check(data.at("counts").get<picojson::array>()[2].get<double>()==-60528,"signed counts preserved");}
  if(data.count("source_timestamp_ms"))imuseq=data.at("sequence").get<double>()==83;
  if(data.count("softcore_diagnostics")){const auto &core=data.at("softcore_diagnostics");softcore_seen=core.get("protocol").get<std::string>()=="NAV1"&&core.get("passive").get<bool>()&&!core.get("received").get<bool>();}
  network|=reason=="receive_failed"&&data.at("errno").get<double>()==ENETDOWN;
  if(data.count("client_reported_unverified")){const auto &client=data.at("client_reported_unverified").get<picojson::object>();client_seen=client.at("reason").get<std::string>()=="telemetry_timeout"&&client.at("state_age_ms").get<double>()==1501;}
 }
 check(lease&&ack&&sonar,"all real controller causes recorded");check(explicit_stop&&!zero_error,"human stop distinguished from zero-duty normal transition");check(reject,"paused controller rejection cause retained with request seq");check(raw&&imuseq&&network,"sensor source correlation and receive errno available");
 check(whole.find("Bearer ")==std::string::npos&&whole.find("\"session\"")==std::string::npos,"no credentials/session");
 check(client_seen&&whole.find("Bearer-secret")==std::string::npos,"client stop cause present and untrusted fields safe");
 check(softcore_seen,"passive NAV1 diagnostics included in actual wrong.log motor snapshot");
 std::cout<<"PASS control audit "<<checks<<" checks; log="<<path<<'\n';
}
