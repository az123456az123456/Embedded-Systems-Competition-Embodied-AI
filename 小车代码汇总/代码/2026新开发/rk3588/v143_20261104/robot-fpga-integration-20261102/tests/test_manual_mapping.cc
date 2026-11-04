// Real Runtime/Gateway/ARB1 on loopback; synthetic backend, never hardware.
#include "gateway_fixture.h"
#define main production_gateway_main
#include "../gateway/runtime.cc"
#undef main
#include <filesystem>
int main(){
 const std::string dir="/tmp/manual-map-fixture-"+std::to_string(getpid());std::filesystem::create_directory(dir);chmod(dir.c_str(),0700);
 const auto token=dir+"/token";{std::ofstream f(token);f<<"012345\n";}
 unsigned bp=freePort(),gp=freePort();Backend backend;backend.fpga_quiet_fixture=true;backend.start(bp);std::atomic<bool>done{false};
 std::atomic<uint64_t>gen{0};std::atomic<unsigned>starts{0},stops{0},other_missions{0};
 LocalBridge bridge(dir+"/ipc",true,[&](Channel,const AcceptedMessage&m){for(const auto&b:m.message.blocks)if(const auto*q=std::get_if<Mission>(&b.payload)){
  if(q->operation==ARB1_MANUAL_MAPPING_START){assert(!q->permit_seq&&!q->auth_epoch);gen=q->gateway_generation;++starts;}
  else if(q->operation==ARB1_MAPPING_STOP)++stops;
  else ++other_missions;
 }});
 std::thread pub([&]{while(!done){if(bridge.ready(Channel::State)){
  auto now=mono_ns();Json s={{"state",gen?"MANUAL":"IDLE"},{"strategy","manual"},{"active",bool(gen)},
    {"manual_ready",!gen},{"ready",false},{"published_mono_ns",now},{"gateway_generation",std::to_string(gen)}};
  Message m;m.header.type=ARB1_MSG_SNAPSHOT;Metadata meta;meta.received_mono_ns=now;meta.source_age_ns=0;
  m.blocks={{meta,Diagnostic{Json({{"kind","mapping"},{"value",s}}).dump()}}};assert(bridge.send(Channel::State,m));
 }std::this_thread::sleep_for(std::chrono::milliseconds(25));}});
 {
  GatewayOptions o;o.bind_address="127.0.0.1";o.port=gp;o.backend_port=bp;o.token_file=token;o.fpga_navigation_source=true;
  Runtime runtime(o,dir+"/ipc","");std::string error;assert(runtime.start(error));
  auto session=text(request(gp,"POST","/api/connect","{}").value,"session");uint64_t seq=0,ns=0;unsigned checks=0;
  auto check=[&](bool ok){if(!ok)std::cerr<<"manual runtime failed "<<checks<<'\n';assert(ok);++checks;};
  auto control=[&](const std::string&op,const std::string&extra=""){return request(gp,"POST","/api/control","{\"session\":\""+session+"\",\"op\":\""+op+"\",\"seq\":"+std::to_string(++seq)+extra+"}");};
  auto nav=[&](const std::string&op){return request(gp,"POST","/api/navigation","{\"session\":\""+session+"\",\"op\":\""+op+"\",\"nav_seq\":"+std::to_string(++ns)+"}");};
  auto status=[&]{return request(gp,"GET","/api/state").value.get("navigation");};
  auto wait=[&](auto pred){auto end=ms()+2000;while(ms()<end){check(control("idle").code==200);if(pred())return true;std::this_thread::sleep_for(std::chrono::milliseconds(30));}return false;};
  check(wait([&]{return bridge.ready(Channel::Control)&&flag(status(),"supports_manual_mapping");}));
  check(control("arm").code==200);check(nav("manual-mapping-start").code==200);
  check(wait([&]{return starts==1&&text(status().get("mapping"),"state")=="MANUAL";}));
  check(!flag(status(),"active")&&!flag(status(),"sends_velocity"));
  check(nav("mapping-start").code==409);
  check(control("drive",",\"throttle\":0.2,\"steer\":0").code==200);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));check(text(status().get("mapping"),"state")=="MANUAL");
  check(control("stop").code==200);std::this_thread::sleep_for(std::chrono::milliseconds(120));check(text(status().get("mapping"),"state")=="MANUAL");
  check(starts==1&&other_missions==0&&stops==0);
  {std::lock_guard<std::mutex>l(backend.mutex);check(backend.fpga_operations.empty());}check(nav("mapping-stop").code==200);
  check(wait([&]{return stops==1;}));check(starts==1&&other_missions==0);
  {std::lock_guard<std::mutex>l(backend.mutex);for(const auto&op:backend.operations)check(op!="claim"&&op!="start"&&op!="twist");}
  std::cout<<"PASS "<<checks<<" manual mapping Runtime/Gateway/ARB1 checks\n";
 }
 done=true;pub.join();backend.server.stop();std::filesystem::remove_all(dir);
}
