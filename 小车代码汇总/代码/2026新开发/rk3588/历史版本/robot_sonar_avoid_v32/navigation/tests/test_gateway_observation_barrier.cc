// Real loopback HTTP regression. No mocked Gateway internals and no hardware.
#include "../gateway.h"
#include "../vendor/http_server.h"
#include "../vendor/picojson.h"
#include <arpa/inet.h>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
using namespace std::chrono;
static std::uint64_t ms(){return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();}
static unsigned port(){int f=socket(AF_INET,SOCK_STREAM,0);assert(f>=0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);assert(!bind(f,(sockaddr*)&a,sizeof(a)));socklen_t n=sizeof(a);assert(!getsockname(f,(sockaddr*)&a,&n));close(f);return ntohs(a.sin_port);}
struct Response{int code=0;picojson::value value;};
static Response request(unsigned p,const std::string& method,const std::string& path,const std::string& body=""){
 int f=socket(AF_INET,SOCK_STREAM,0);assert(f>=0);timeval timeout{2,0};setsockopt(f,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(p);assert(!connect(f,(sockaddr*)&a,sizeof(a)));
 const auto text=method+" "+path+" HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer 012345\r\nContent-Length: "+std::to_string(body.size())+"\r\nConnection: close\r\n\r\n"+body;assert(robotphone::writeAll(f,text.data(),text.size()));std::string data;char b[4096];ssize_t n;while((n=recv(f,b,sizeof(b),0))>0)data.append(b,n);close(f);auto at=data.find("\r\n\r\n");assert(at!=std::string::npos);Response r;std::string h;std::istringstream in(data);in>>h>>r.code;assert(picojson::parse(r.value,data.substr(at+4)).empty());return r;
}
static std::string str(const picojson::value& v,const char* key){return v.get(key).get<std::string>();}
struct Backend {
 robotphone::HttpServer server;std::mutex mutex;bool armed=false;std::uint64_t seq=0;double duty=0;
 std::atomic<unsigned> delay_get{0},delay_drive{0},reads{0},get_entered{0},drive_entered{0};std::vector<std::string> ops;
 bool reject_arm=false,reject_config=false,reject_wheels=false,replace_session=false; std::atomic<unsigned> delay_arm{0},arm_entered{0},delay_connect{0},connect_entered{0};
 std::string session="session";
 std::string state(bool okay=true)const{return std::string("{\"ok\":")+(okay?"true":"false")+",\"session\":\""+session+"\",\"armed\":"+(armed?"true":"false")+",\"imu_action_pending\":false,\"reason\":\"fixture\",\"motor\":{\"velocity_protocol_supported\":true,\"softcore_ready\":true,\"ack_protocol\":4,\"link_ok\":true,\"left_mask\":10,\"invert_mask\":15,\"left\":"+std::to_string(duty)+",\"right\":"+std::to_string(duty)+",\"ack_age_ms\":1},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":0,\"roll\":0,\"pitch\":0,\"timestamp_ms\":100,\"sequence\":10,\"sample_age_ms\":1}}";}
 void start(unsigned p){std::string e;assert(server.start("127.0.0.1",p,[&](int fd,const robotphone::HttpRequest& req){
   if(req.path=="/api/state"){
     ++reads;++get_entered;std::string snapshot;{std::lock_guard<std::mutex> l(mutex);snapshot=state();}
     std::this_thread::sleep_for(milliseconds(delay_get.load()));robotphone::httpReply(fd,200,snapshot);return;
   }
   if(req.path=="/api/connect"){
     ++connect_entered;std::this_thread::sleep_for(milliseconds(delay_connect.load()));
     std::lock_guard<std::mutex> l(mutex);
     if(armed){robotphone::httpReply(fd,409,"{\"ok\":false,\"error\":\"control_owner_active\"}");return;}
     if(replace_session){session="replacement-session";seq=0;}
     robotphone::httpReply(fd,200,state());return;
   }
   picojson::value v;assert(picojson::parse(v,req.body).empty());const auto op=str(v,"op");
   if(op=="arm"){++arm_entered;std::this_thread::sleep_for(milliseconds(delay_arm.load()));}
   if(op=="drive"||op=="twist"){++drive_entered;std::this_thread::sleep_for(milliseconds(delay_drive.load()));}
   std::lock_guard<std::mutex> l(mutex);int code=200;const auto s=std::uint64_t(v.get("seq").get<double>());
   if(s<=seq)code=409;else{seq=s;ops.push_back(op);
     if(op=="arm"){if(reject_arm)code=409;else armed=true;}
     else if(op=="stop"||op=="disarm"){armed=false;duty=0;}
     else if(op=="configure"){if(reject_config)code=409;else{armed=false;duty=0;}}
     else if(op=="idle")duty=0;
     else if(op=="drive"||op=="twist"){if(!armed||(op=="twist"&&reject_wheels))code=409;else duty=op=="drive"?v.get("throttle").get<double>():v.get("linear_mps").get<double>();}
   }
   auto out=state(code==200);if(op=="claim")out.insert(out.size()-1,",\"navigation_ticket\":\"private\"");robotphone::httpReply(fd,code,out);
 },e));}
};


int main(){
 unsigned checks=0;auto check=[&](bool okay){assert(okay);++checks;};Backend b;auto bp=port(),gp=port();b.start(bp);std::string token="/tmp/v262-observation-token-"+std::to_string(getpid());{std::ofstream f(token);f<<"012345\n";}
 robotnav::GatewayCallbacks callbacks;callbacks.navigation_command=[](const std::string&,double,double,std::string&){return true;};callbacks.navigation_state=[](){return "{\"can_start\":true}";};
 robotnav::GatewayOptions opts;opts.backend_port=bp;opts.port=gp;opts.bind_address="127.0.0.1";opts.token_file=token;robotnav::Gateway g(opts,callbacks);std::string error;check(g.start(error));check(request(gp,"POST","/api/connect","{}").code==200);
 std::atomic<unsigned> sequence{0};auto cmd=[&](const std::string& op){return request(gp,"POST","/api/control","{\"session\":\"session\",\"seq\":"+std::to_string(++sequence)+",\"op\":\""+op+"\"}");};
 b.delay_arm=100;Response arm;std::thread arming([&]{arm=cmd("arm");});while(!b.arm_entered)std::this_thread::yield();
 auto cache=request(gp,"GET","/api/state");check(cache.code==200);check(cache.value.get("gateway_control_pending").get<bool>());check(!cache.value.get("armed").get<bool>());arming.join();check(arm.code==200);b.delay_arm=0;
 check(g.requestNavigation("explore",0,0,error));auto generation=g.generation();b.delay_drive=100;
 auto entered=b.drive_entered.load();bool applied=false;std::thread wheels([&]{applied=g.applyNav(0,0,generation);});while(b.drive_entered==entered)std::this_thread::yield();
 cache=request(gp,"GET","/api/state");check(cache.code==200);check(!cache.value.get("gateway_control_pending").get<bool>());check(cache.value.get("armed").get<bool>());check(cache.value.get("gateway_state_age_ms").get<double>()<300);wheels.join();check(applied);
 // STOP admission remains an immediate observation barrier while the prior
 // authorized wheels IO is still outstanding; its late reply cannot rearm.
 entered=b.drive_entered;std::thread old_wheels([&]{g.applyNav(0,0,generation);});while(b.drive_entered==entered)std::this_thread::yield();
 Response stop;std::thread stopping([&]{stop=cmd("stop");});auto deadline=ms()+200;while(g.navigationActive()&&ms()<deadline)std::this_thread::sleep_for(milliseconds(1));check(!g.navigationActive());
 cache=request(gp,"GET","/api/state");check(cache.code==200);check(cache.value.get("gateway_control_pending").get<bool>());old_wheels.join();stopping.join();check(stop.code==200);check(!g.telemetry().armed);check(!g.applyNav(.2,.2,generation));b.delay_drive=0;
 check(cmd("arm").code==200);check(g.requestNavigation("explore",0,0,error));generation=g.generation();{std::lock_guard<std::mutex> lock(b.mutex);b.reject_wheels=true;}
 check(!g.applyNav(0,0,generation));check(!g.navigationActive());check(!g.telemetry().armed);
 // Exempt wheels IO never refreshes source time by merely beginning. With no
 // new confirmed backend observation, the cache honestly exceeds the 300ms
 // Android parser bound, even though it can still be delivered over HTTP.
 auto prior=request(gp,"GET","/api/state");std::this_thread::sleep_for(milliseconds(320));cache=request(gp,"GET","/api/state");
 check(cache.code==200);check(!cache.value.get("gateway_control_pending").get<bool>());check(cache.value.get("gateway_state_age_ms").get<double>()>=300);check(cache.value.get("gateway_state_age_ms").get<double>()>=prior.value.get("gateway_state_age_ms").get<double>()+300);
 // Session negotiation is a separate socket path, but remains a mutation
 // barrier. A rejected competing connect must preserve the current owner.
 check(cmd("arm").code==200);b.delay_connect=100;entered=b.connect_entered;
 Response reconnect;std::thread competing_connect([&]{reconnect=request(gp,"POST","/api/connect","{}");});while(b.connect_entered==entered)std::this_thread::yield();
 cache=request(gp,"GET","/api/state");check(cache.code==200);check(cache.value.get("gateway_control_pending").get<bool>());check(cache.value.get("armed").get<bool>());
 competing_connect.join();check(reconnect.code==409);check(g.telemetry().armed);check(cmd("idle").code==200);check(cmd("stop").code==200);
 // A successful replacement establishes a new owner and generation. A prior
 // owner's ARM admitted during that socket wait cannot apply afterwards.
 {std::lock_guard<std::mutex> lock(b.mutex);b.replace_session=true;}
 const auto prior_generation=g.generation();entered=b.connect_entered;
 std::thread replacement([&]{reconnect=request(gp,"POST","/api/connect","{}");});while(b.connect_entered==entered)std::this_thread::yield();
 cache=request(gp,"GET","/api/state");check(cache.code==200);check(cache.value.get("gateway_control_pending").get<bool>());check(!cache.value.get("armed").get<bool>());
 Response old_owner;std::thread queued_old_owner([&]{old_owner=cmd("arm");});replacement.join();queued_old_owner.join();check(reconnect.code==200);check(old_owner.code==403);check(!g.telemetry().armed);check(g.generation()>prior_generation);check(!g.applyNav(.1,.1,prior_generation));
 cache=request(gp,"GET","/api/state");check(!cache.value.get("gateway_control_pending").get<bool>());
 check(request(gp,"POST","/api/control","{\"session\":\"replacement-session\",\"seq\":1,\"op\":\"arm\"}").code==200);check(g.telemetry().armed);
 check(request(gp,"POST","/api/control","{\"session\":\"replacement-session\",\"seq\":2,\"op\":\"stop\"}").code==200);check(!g.telemetry().armed);
 g.stop();b.server.stop();unlink(token.c_str());printf("PASS %u real Gateway observation-barrier checks: ARM/STOP/session protected, competing owner rejected, replaced owner revoked, periodic wheels preserves confirmed cache, rejected wheels disarms, source age continues advancing\n",checks);
}
