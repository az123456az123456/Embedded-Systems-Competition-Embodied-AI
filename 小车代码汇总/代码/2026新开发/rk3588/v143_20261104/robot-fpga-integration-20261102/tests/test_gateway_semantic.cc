// Run in an isolated UID-0 user namespace. No hardware, real tokens or ROS.
#include "gateway_fixture.h"
#define main production_gateway_main
#include "../gateway/runtime.cc"
#undef main
#include <filesystem>

int main(int argc,char**argv) {
 assert(argc==2);std::ifstream commands_file(argv[1]);Json commands;commands_file>>commands;
 assert(commands.is_array()&&!commands.empty());
 const auto dir="/tmp/semantic-gateway-fixture-"+std::to_string(getpid());
 std::filesystem::create_directory(dir);chmod(dir.c_str(),0700);
 const auto token=dir+"/token";{std::ofstream f(token);f<<"012345\n";}
 const unsigned bp=freePort(),gp=freePort();Backend backend;backend.start(bp);
 std::mutex mutex;std::string task,generation,phase="IDLE";bool wrong_task=false,stale=false;
 std::atomic<bool> done{false};std::atomic<unsigned> cancels{0};uint64_t drive_seq=0;
 Json drive_command={{"op","drive"},{"throttle",.15},{"steer",0.},{"speed",0.},{"direction","cw"}};
 robotphone::HttpServer semantic;std::string error;
 assert(semantic.start("127.0.0.1",8088,[&](int fd,const robotphone::HttpRequest&r){
  assert(r.path=="/task"&&r.headers.at("authorization")=="Bearer 012345");
  const auto j=Json::parse(r.body);
  {std::lock_guard<std::mutex> l(mutex);task=j.at("task");generation=j.at("gateway_generation");phase="SEARCHING";wrong_task=stale=false;}
  robotphone::httpReply(fd,200,"{\"ok\":true}");
 },error));
 LocalBridge bridge(dir+"/ipc",true,[&](Channel,const AcceptedMessage&m){
  for(const auto& b:m.message.blocks)if(const auto*q=std::get_if<Mission>(&b.payload))if(q->operation==ARB1_CANCEL)++cancels;
 });
 std::thread publisher([&]{while(!done){
  if(bridge.ready(Channel::State)){
   const auto now=mono_ns();Json s,e;
   {std::lock_guard<std::mutex> l(mutex);
    s={{"task",wrong_task?"previous-task":task},{"gateway_generation",generation},{"published_mono_ns",stale?now-1000000000:now},
       {"state",phase},{"reason",phase=="FOUND"?"confirmed_and_stopped":"fixture"},{"ready",true},{"active",phase!="IDLE"&&phase!="FOUND"}};
    e={{"gateway_generation",generation},{"semantic_external",true},{"active",true},{"state","EXPLORING"},{"reason","random_walk"},
       {"drive",{{"produced_mono_ns",std::to_string(now)},{"valid_for_ms","200"},{"generation","7"},{"gateway_generation",generation},
       {"sequence",std::to_string(++drive_seq)},{"motion_requested",true},{"pose_age_ns","0"},{"scan_age_ns","0"},
       {"op","drive"},{"throttle",.15},{"steer",0.},{"speed",0.},{"direction","cw"}}}};
    for(const auto*key:{"op","throttle","steer","speed","direction"})e["drive"][key]=drive_command.value(key,e["drive"][key]);
   }
   Message m;m.header.type=ARB1_MSG_SNAPSHOT;Metadata meta;meta.received_mono_ns=now;meta.source_age_ns=0;
   Json unused_map={{"state","ERROR"},{"reason","unused_map_fixture"},{"active",false},{"gateway_generation",s.at("gateway_generation")}};
   for(const auto& item:{std::make_pair("semantic",s),std::make_pair("exploration",e),std::make_pair("mapping",unused_map)}){
    m.blocks={{meta,Diagnostic{Json({{"kind",item.first},{"value",item.second}}).dump()}}};
    assert(bridge.send(Channel::State,m));
   }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
 }});
 {
 GatewayOptions o;o.bind_address="127.0.0.1";o.port=gp;o.backend_port=bp;o.token_file=token;o.fpga_navigation_source=true;
 Runtime runtime(o,dir+"/ipc","");assert(runtime.start(error));
 auto connected=request(gp,"POST","/api/connect","{}");assert(connected.code==200);
 const auto session=text(connected.value,"session");uint64_t seq=0,navseq=0;unsigned checks=0;
 auto check_at=[&](bool ok,int line){if(!ok)std::cerr<<"failed check "<<checks<<" at line "<<line<<'\n';assert(ok);++checks;};
#define check(ok) check_at((ok),__LINE__)
 auto control=[&](const std::string&op){return request(gp,"POST","/api/control","{\"session\":\""+session+"\",\"op\":\""+op+"\",\"seq\":"+std::to_string(++seq)+"}");};
 auto nav=[&](const std::string&op){return request(gp,"POST","/api/navigation","{\"session\":\""+session+"\",\"op\":\""+op+"\",\"nav_seq\":"+std::to_string(++navseq)+",\"text\":\"找到未戴口罩的人\"}");};
 auto status=[&]{return request(gp,"GET","/api/state").value.get("navigation");};
 auto keepalive=[&]{check(control("idle").code==200);};
 auto wait=[&](auto pred,int timeout=2500){const auto until=ms()+timeout;while(ms()<until){keepalive();if(pred())return true;std::this_thread::sleep_for(std::chrono::milliseconds(40));}return false;};
 auto moving=[&]{std::lock_guard<std::mutex> l(backend.mutex);return backend.throttle!=0||backend.steer!=0;};
 auto assert_zero=[&](int duration){const auto until=ms()+duration;while(ms()<until){keepalive();check(!moving());std::this_thread::sleep_for(std::chrono::milliseconds(30));}};
 check(wait([&]{return bridge.ready(Channel::Control);}));
 std::this_thread::sleep_for(std::chrono::milliseconds(150));
 auto start=[&]{check(control("arm").code==200);keepalive();check(nav("semantic-start").code==200);check(wait(moving));};
 start();
 check(!flag(status().get("mapping"),"active"));
 // Feed commands emitted by the Python random walker through the real Runtime,
 // admission checks and backend serializer, including both turn directions.
 for(const auto&command:commands){
  const bool rotate=command.at("op")=="rotate";
  unsigned before;{std::lock_guard<std::mutex>l(backend.mutex);before=rotate?backend.rotations:backend.drives;}
  {std::lock_guard<std::mutex>l(mutex);drive_command=command;}
  check(wait([&]{std::lock_guard<std::mutex>l(backend.mutex);
    if(rotate)return backend.rotations>before&&std::abs(backend.rotation_speed-command.at("speed").get<double>())<1e-9&&backend.rotation_clockwise==(command.at("direction")=="cw");
    return backend.drives>before&&std::abs(backend.throttle-command.at("throttle").get<double>())<1e-9&&std::abs(backend.steer-command.at("steer").get<double>())<1e-9;
  }));
  check(text(status(),"state")!="CANCELLED");
 }
 {std::lock_guard<std::mutex>l(mutex);drive_command={{"op","drive"},{"throttle",.15},{"steer",0.},{"speed",0.},{"direction","cw"}};}
 // Remain in search for several seconds with fresh commands and a live phone.
 const auto until=ms()+1600;while(ms()<until){keepalive();check(text(status(),"state")!="CANCELLED");std::this_thread::sleep_for(std::chrono::milliseconds(40));}
 {std::lock_guard<std::mutex> l(mutex);phase="STOPPING";}
 check(wait([&]{return text(status(),"state")=="STOPPING";}));assert_zero(250);
 {std::lock_guard<std::mutex> l(mutex);phase="FOUND";}
 check(wait([&]{return text(status(),"state")=="FOUND";}));assert_zero(350);
 check(!flag(status(),"active")&&cancels>0);
 // The STOPPING diagnostic may be lost; direct FOUND must survive cancellation.
 {std::lock_guard<std::mutex> l(mutex);phase="IDLE";}std::this_thread::sleep_for(std::chrono::milliseconds(100));
 start();{std::lock_guard<std::mutex> l(mutex);phase="FOUND";}
 check(wait([&]{return text(status(),"state")=="FOUND";}));assert_zero(250);
 // Old-task or stale success can revoke motion, but must never report success.
 for(bool old_task:{true,false}){
  {std::lock_guard<std::mutex> l(mutex);phase="IDLE";wrong_task=stale=false;}std::this_thread::sleep_for(std::chrono::milliseconds(100));
  start();{std::lock_guard<std::mutex> l(mutex);phase="FOUND";wrong_task=old_task;stale=!old_task;}
  check(wait([&]{return text(status(),"state")=="CANCELLED";}));assert_zero(200);check(text(status(),"state")!="FOUND");
 }
 // A human stop while awaiting stationary confirmation must remain cancelled.
 {std::lock_guard<std::mutex> l(mutex);phase="IDLE";wrong_task=stale=false;}std::this_thread::sleep_for(std::chrono::milliseconds(100));
 start();{std::lock_guard<std::mutex> l(mutex);phase="STOPPING";}
 check(wait([&]{return text(status(),"state")=="STOPPING";}));check(nav("stop").code==200);
 {std::lock_guard<std::mutex> l(mutex);phase="FOUND";}
 assert_zero(300);check(text(status(),"state")=="CANCELLED");
 // The fix changes the producer; an out-of-contract turn is still rejected.
 {std::lock_guard<std::mutex>l(mutex);phase="IDLE";}std::this_thread::sleep_for(std::chrono::milliseconds(100));
 start();{std::lock_guard<std::mutex>l(mutex);drive_command={{"op","rotate"},{"throttle",0.},{"steer",0.},{"speed",.55},{"direction","cw"}};}
 check(wait([&]{return text(status(),"state")=="CANCELLED";}));assert_zero(200);
 std::cout<<"PASS "<<checks<<" production Runtime/Gateway checks; synthetic backend and ROS IPC; no real motion\n";
 }
 done=true;publisher.join();semantic.stop();backend.server.stop();
 // LocalBridge owns and removes its sockets on exit; retain its directory.
 return 0;
}
