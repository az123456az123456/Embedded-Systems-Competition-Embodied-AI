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
 std::atomic<bool> wheels_inside{false}; std::atomic<unsigned> delay_get{0},delay_drive{0},reads{0},get_entered{0},drive_entered{0};std::vector<std::string> ops;
 bool reject_arm=false,reject_config=false;
 std::string state(bool okay=true)const{return std::string("{\"ok\":")+(okay?"true":"false")+",\"session\":\"real-session-12345\",\"armed\":"+(armed?"true":"false")+",\"imu_action_pending\":false,\"reason\":\"fixture\",\"motor\":{\"link_ok\":true,\"left_mask\":10,\"invert_mask\":15,\"left\":"+std::to_string(duty)+",\"right\":"+std::to_string(duty)+",\"ack_age_ms\":1},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":0,\"roll\":0,\"pitch\":0,\"timestamp_ms\":100,\"sequence\":10,\"sample_age_ms\":1}}";}
 void start(unsigned p){std::string e;assert(server.start("127.0.0.1",p,[&](int fd,const robotphone::HttpRequest& req){
   if(req.path=="/api/state"){
     ++reads;++get_entered;std::string snapshot;{std::lock_guard<std::mutex> l(mutex);snapshot=state();}
     std::this_thread::sleep_for(milliseconds(delay_get.load()));robotphone::httpReply(fd,200,snapshot);return;
   }
   if(req.path=="/api/connect"){std::lock_guard<std::mutex> l(mutex);robotphone::httpReply(fd,200,state());return;}
   picojson::value v;assert(picojson::parse(v,req.body).empty());const auto op=str(v,"op");
   if(op=="wheels")wheels_inside=true; if(op=="drive"||op=="wheels"){++drive_entered;std::this_thread::sleep_for(milliseconds(delay_drive.load()));} if(op=="wheels")wheels_inside=false;
   std::lock_guard<std::mutex> l(mutex);int code=200;const auto s=std::uint64_t(v.get("seq").get<double>());
   if(s<=seq)code=409;else{seq=s;ops.push_back(op);
     if(op=="arm"){if(reject_arm)code=409;else armed=true;}
     else if(op=="stop"||op=="disarm"){armed=false;duty=0;}
     else if(op=="configure"){if(reject_config)code=409;else{armed=false;duty=0;}}
     else if(op=="idle")duty=0;
     else if(op=="drive"||op=="wheels"){if(!armed)code=409;else duty=op=="drive"?v.get("throttle").get<double>():v.get("left").get<double>();}
   }
   auto out=state(code==200);if(op=="claim")out.insert(out.size()-1,",\"navigation_ticket\":\"private\"");robotphone::httpReply(fd,code,out);
 },e));}
};


int main(int argc,char**argv){
 assert(argc==2);const std::string fixture_dir=argv[1];
 Backend b;auto bp=port(),gp=port(),pp=port();b.start(bp);std::string token=fixture_dir+"/token";{std::ofstream f(token);f<<"012345\n";}
 std::atomic<bool> core_active{false};
 robotnav::GatewayCallbacks callbacks;callbacks.navigation_command=[&](const std::string&,double,double,std::string&){core_active=true;return true;};callbacks.navigation_cancel=[&](const std::string&){core_active=false;};callbacks.navigation_state=[](){return "{\"can_start\":true,\"state\":\"BOOTSTRAP_WAIT\",\"navigation_enabled\":true}";};
 robotnav::GatewayOptions opts;opts.backend_port=bp;opts.port=gp;opts.bind_address="127.0.0.1";opts.token_file=token;robotnav::Gateway g(opts,callbacks);std::string error;assert(g.start(error));
 std::atomic<unsigned> pending_gets{0},fresh_gets{0},idle_count{0},stop_count{0},wheels_count{0};std::atomic<unsigned> max_age{0};
 robotphone::HttpServer proxy;assert(proxy.start("127.0.0.1",pp,[&](int fd,const robotphone::HttpRequest& req){
  if(req.path=="/api/video.mjpg"){robotphone::httpReply(fd,503,"{}");return;}
  if(req.path=="/api/state"&&g.navigationActive()){
   // Fault scheduling only: deliver each real GET during a periodic wheels IO.
   // No fake state, JSON, ages, or pending bits are injected.
   auto end=ms()+200;while(!b.wheels_inside&&ms()<end)std::this_thread::sleep_for(milliseconds(1));
  }
  auto out=request(gp,req.method,req.path,req.body);
  if(req.path=="/api/control") {picojson::value v;assert(picojson::parse(v,req.body).empty());auto op=str(v,"op");if(op=="idle")++idle_count;if(op=="stop"||op=="disarm")++stop_count;}
  if(req.path=="/api/state"&&g.navigationActive()&&out.code==200){auto age=out.value.get("gateway_state_age_ms").get<double>();unsigned previous=max_age.load();while(unsigned(age)>previous&&!max_age.compare_exchange_weak(previous,unsigned(age))){}if(out.value.get("gateway_control_pending").get<bool>())++pending_gets;else ++fresh_gets;}
  robotphone::httpReply(fd,out.code,out.value.serialize());
 },error));
 {std::ofstream f(fixture_dir+"/port");f<<pp<<std::endl;}
 std::atomic<bool> run{true};std::thread runtime([&]{auto last_poll=ms();while(run){if(ms()-last_poll>=50){g.pollState();last_poll=ms();}if(g.navigationActive()){
   b.delay_drive=90; if(g.applyNav(0,0,g.generation()))++wheels_count;
  }std::this_thread::sleep_for(milliseconds(10));}});
 auto end=ms()+13000;while(ms()<end){if(stop_count>0&&wheels_count>5)break;std::this_thread::sleep_for(milliseconds(10));}
 std::this_thread::sleep_for(milliseconds(100));run=false;runtime.join();printf("NATIVE real Gateway: pending_GET=%u nonpending_GET=%u max_age=%.0f idle_ACK=%u zero_wheels_ACK=%u STOP_or_DISARM=%u final_active=%d final_armed=%d\n",pending_gets.load(),fresh_gets.load(),double(max_age.load()),idle_count.load(),wheels_count.load(),stop_count.load(),g.navigationActive(),g.telemetry().armed);fflush(stdout);proxy.stop();g.stop();b.server.stop();unlink(token.c_str());
}
