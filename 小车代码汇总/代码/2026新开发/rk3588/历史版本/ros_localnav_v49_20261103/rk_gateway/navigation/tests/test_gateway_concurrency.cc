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
 std::atomic<unsigned> delay_get{0},delay_drive{0},delay_arm{0},reads{0},get_entered{0},drive_entered{0},arm_entered{0};std::vector<std::string> ops;
 bool reject_arm=false,reject_config=false;
 std::string state(bool okay=true)const{return std::string("{\"ok\":")+(okay?"true":"false")+",\"session\":\"session\",\"armed\":"+(armed?"true":"false")+",\"imu_action_pending\":false,\"reason\":\"fixture\",\"motor\":{\"velocity_protocol_supported\":true,\"softcore_ready\":true,\"ack_protocol\":4,\"link_ok\":true,\"left_mask\":10,\"invert_mask\":15,\"left\":"+std::to_string(duty)+",\"right\":"+std::to_string(duty)+",\"ack_age_ms\":1},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":0,\"roll\":0,\"pitch\":0,\"timestamp_ms\":100,\"sequence\":10,\"sample_age_ms\":1}}";}
 void start(unsigned p){std::string e;assert(server.start("127.0.0.1",p,[&](int fd,const robotphone::HttpRequest& req){
   if(req.path=="/api/state"){
     ++reads;++get_entered;std::string snapshot;{std::lock_guard<std::mutex> l(mutex);snapshot=state();}
     std::this_thread::sleep_for(milliseconds(delay_get.load()));robotphone::httpReply(fd,200,snapshot);return;
   }
   if(req.path=="/api/connect"){std::lock_guard<std::mutex> l(mutex);robotphone::httpReply(fd,200,state());return;}
   picojson::value v;assert(picojson::parse(v,req.body).empty());const auto op=str(v,"op");
   if(op=="drive"||op=="twist"){++drive_entered;std::this_thread::sleep_for(milliseconds(delay_drive.load()));}
   if(op=="arm"){++arm_entered;std::this_thread::sleep_for(milliseconds(delay_arm.load()));}
   std::lock_guard<std::mutex> l(mutex);int code=200;const auto s=std::uint64_t(v.get("seq").get<double>());
   if(s<=seq)code=409;else{seq=s;ops.push_back(op);
     if(op=="arm"){if(reject_arm)code=409;else armed=true;}
     else if(op=="stop"||op=="disarm"){armed=false;duty=0;}
     else if(op=="configure"){if(reject_config)code=409;else{armed=false;duty=0;}}
     else if(op=="idle"||op=="pause")duty=0;
     else if(op=="lease"){}
     else if(op=="drive"||op=="twist"){if(!armed)code=409;else duty=op=="drive"?v.get("throttle").get<double>():v.get("linear_mps").get<double>();}
   }
   auto out=state(code==200);if(op=="claim")out.insert(out.size()-1,",\"navigation_ticket\":\"private\"");robotphone::httpReply(fd,code,out);
 },e));}
};
int main(){
 unsigned checks=0;auto check=[&](bool ok){assert(ok);++checks;};Backend b;const auto bp=port(),gp=port();b.start(bp);const auto token="/tmp/gateway-v26-concurrency-"+std::to_string(getpid());{std::ofstream f(token);f<<"012345\n";}
 robotnav::GatewayCallbacks callbacks;callbacks.navigation_command=[](const std::string&,double,double,std::string&){return true;};callbacks.navigation_state=[](){return "{\"can_start\":true}";};callbacks.navigation_map=[](){return "{\"available\":true}";};
 robotnav::GatewayOptions opts;opts.backend_port=bp;opts.port=gp;opts.bind_address="127.0.0.1";opts.token_file=token;robotnav::Gateway g(opts,callbacks);std::string error;check(g.start(error));check(request(gp,"POST","/api/connect","{}").code==200);
 std::atomic<unsigned> sequence{0};auto cmd=[&](const std::string& op,const std::string& extra=""){return request(gp,"POST","/api/control","{\"session\":\"session\",\"seq\":"+std::to_string(++sequence)+",\"op\":\""+op+"\""+extra+"}");};
 check(cmd("arm").code==200);
 std::this_thread::sleep_for(milliseconds(30));
 auto aged=request(gp,"GET","/api/state");
 const double age=aged.value.get("gateway_state_age_ms").get<double>();
 check(age>=20);check(!aged.value.get("gateway_control_pending").get<bool>());
 check(aged.value.get("imu").get("sample_age_ms").get<double>()>=1+age);
 check(aged.value.get("motor").get("ack_age_ms").get<double>()>=1+age);
 // Four phone telemetry clients share one cached snapshot while the real 20Hz
 // runtime poll is blocked on a slow read. Heartbeats and manual DRIVE are free.
 b.delay_get=120;std::atomic<unsigned> responses{0},failures{0};
 auto before=b.reads.load();std::thread poller([&]{g.pollState();});while(b.get_entered==0)std::this_thread::yield();
 std::vector<std::thread> clients;for(int k=0;k<4;++k)clients.emplace_back([&]{for(int i=0;i<12;++i){auto r=request(gp,"GET",i%2?"/api/navigation":"/api/state");if(r.code==200)++responses;else ++failures;}});
 auto start=ms();check(cmd("idle").code==200);check(ms()-start<100);
 start=ms();check(cmd("drive",",\"throttle\":0.2,\"steer\":0").code==200);check(ms()-start<100);
 for(auto& t:clients) { t.join(); }
 poller.join();check(responses==48&&failures==0);check(b.reads==before+1);check(g.telemetry().armed);{std::lock_guard<std::mutex> l(b.mutex);check(b.armed&&b.duty==.2);}
 // An authenticated idle is admitted while a navigation start is waiting on
 // 120ms backend telemetry. The 30 second owner lease is independent of each short IO.
 check(cmd("drive",",\"throttle\":0,\"steer\":0").code==200);std::this_thread::sleep_for(milliseconds(230));
 bool started=false;std::thread nav([&]{started=g.requestNavigation("explore",0,0,error);});std::this_thread::sleep_for(milliseconds(40));
 auto idle=cmd("idle");check(idle.code==200);nav.join();check(started&&g.navigationActive());auto generation=g.generation();check(g.applyNav(.1,.1,generation));
 // Reads/maps do not renew the foreground owner. A 370ms gap does not revoke the 30 second owner; explicit STOP still wins.
 std::this_thread::sleep_for(milliseconds(370));check(g.navigationActive());{std::lock_guard<std::mutex> l(b.mutex);check(b.armed);}check(cmd("stop").code==200);{std::lock_guard<std::mutex> l(b.mutex);check(!b.armed&&b.duty==0);}
 b.delay_get=0;check(cmd("arm").code==200);
 // A delayed telemetry snapshot from before ARM is never allowed to overwrite
 // the newer command's acknowledged state.
 check(cmd("stop").code==200);b.delay_get=100;before=b.get_entered;std::thread old_get([&]{g.pollState();});while(b.get_entered==before)std::this_thread::yield();check(cmd("arm").code==200);old_get.join();check(g.telemetry().armed);b.delay_get=0;
 // STOP revokes the old route immediately, even while an old DRIVE is pending.
 // The accepted old packet must occur before STOP, never afterwards; queued
 // older DRIVE/ARM cannot be replayed after the stop barrier.
 b.delay_drive=100;before=b.drive_entered;Response moving;std::thread drive([&]{moving=cmd("drive",",\"throttle\":0.3,\"steer\":0");});while(b.drive_entered==before)std::this_thread::yield();
 auto pending_cache=request(gp,"GET","/api/state");
 check(pending_cache.code==200&&!pending_cache.value.get("gateway_control_pending").get<bool>());
 Response queued,stopped;std::thread queue([&]{queued=cmd("drive",",\"throttle\":0.4,\"steer\":0");});std::this_thread::sleep_for(milliseconds(10));std::thread stop([&]{stopped=cmd("stop");});
 start=ms();check(!g.navigationActive());check(ms()-start<50);drive.join();queue.join();stop.join();check(stopped.code==200);check(queued.code==409);check(str(queued.value,"error")=="stop_pending"||str(queued.value,"error")=="control_superseded");{std::lock_guard<std::mutex> l(b.mutex);check(!b.armed&&b.duty==0);check(b.ops.back()=="stop");}
 b.delay_drive=0;check(!g.applyNav(.2,.2,generation));
 // The local console path obeys the same pre-queue stop epoch. A CLI Start
 // queued before STOP cannot capture a newer epoch merely after getting IO.
 check(cmd("arm").code==200);b.delay_drive=100;before=b.drive_entered;
 std::thread held([&]{cmd("drive",",\"throttle\":0.2,\"steer\":0");});
 while(b.drive_entered==before)std::this_thread::yield();
 bool local_started=true;std::string local_error;
 std::thread local([&]{local_started=g.requestNavigation("explore",0,0,local_error);});
 std::this_thread::sleep_for(milliseconds(10));
 check(cmd("stop").code==200);held.join();local.join();b.delay_drive=0;
 check(!local_started);check(local_error=="stop_pending"||local_error=="navigation_cancelled");
 check(!g.navigationActive());{std::lock_guard<std::mutex> l(b.mutex);check(!b.armed&&b.duty==0);}
 // Rejected ARM/configuration remain failures. No optimistic cache can report
 // accepted settings that the dashboard rejected.
 {std::lock_guard<std::mutex> l(b.mutex);b.reject_arm=true;}check(cmd("arm").code==409);check(!g.telemetry().armed);{std::lock_guard<std::mutex> l(b.mutex);b.reject_arm=false;b.reject_config=true;}check(cmd("configure",",\"left_mask\":5,\"invert_mask\":0").code==409);check(g.telemetry().left_mask==10);{std::lock_guard<std::mutex> l(b.mutex);b.reject_config=false;}
 // Missing confirmation zeros movement while retaining logical authorization. Even if a very late
 // old DRIVE arrives after PAUSE, backend sequence ordering rejects it.
 check(cmd("arm").code==200);b.delay_drive=180;check(cmd("drive",",\"throttle\":0.5,\"steer\":0").code==503);std::this_thread::sleep_for(milliseconds(80));{std::lock_guard<std::mutex> l(b.mutex);check(b.armed&&b.duty==0);check(b.ops.back()=="pause");}b.delay_drive=0;
 check(cmd("arm").code==200);check(cmd("drive",",\"throttle\":0.2,\"steer\":0").code==200);check(cmd("stop").code==200);
 // A 180ms ARM arrives at the backend after the 150ms HTTP deadline.
 // The higher-sequence zero PAUSE must prevent that old ARM from authorizing.
 b.delay_arm=180;check(cmd("arm").code==503);std::this_thread::sleep_for(milliseconds(70));
 {std::lock_guard<std::mutex> l(b.mutex);check(!b.armed&&b.duty==0);check(b.ops.back()=="pause");}
 b.delay_arm=0;check(cmd("arm").code==200);check(cmd("stop").code==200);
 // An ARM in progress remains an observation barrier; ordinary DRIVE is not.
 b.delay_arm=100;before=b.arm_entered;Response late_arm;
 std::thread pending_arm([&]{late_arm=cmd("arm");});while(b.arm_entered==before)std::this_thread::yield();
 auto arm_cache=request(gp,"GET","/api/state");check(arm_cache.code==200&&arm_cache.value.get("gateway_control_pending").get<bool>());
 auto newer_stop=cmd("stop");pending_arm.join();check(newer_stop.code==200);
 {std::lock_guard<std::mutex> l(b.mutex);check(!b.armed&&b.duty==0);check(b.ops.back()=="stop");}
 b.delay_arm=0;
 // Even a newly received cache includes the real backend query's wait time.
 b.delay_get=120;g.pollState();auto delayed_cache=request(gp,"GET","/api/state");
 check(delayed_cache.code==200);
 check(delayed_cache.value.get("gateway_state_age_ms").get<double>()>=100);
 g.stop();b.server.stop();unlink(token.c_str());std::printf("PASS %u real-HTTP concurrency checks: cached GET fanout, 120ms telemetry isolation, 30s owner lease with short IO pauses, stale snapshot isolation, STOP barrier, rejected ARM/config and late 180ms DRIVE rejection\n",checks);
}
