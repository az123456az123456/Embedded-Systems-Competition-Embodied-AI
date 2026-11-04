// Production phone service + production Gateway, real loopback HTTP, no hardware.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#include "../../dashboard/phone/phone_service.cc"
#pragma GCC diagnostic pop
#include "../gateway.h"
#include "../vendor/picojson.h"
#include <future>
#include <iostream>
#include <stdexcept>
using namespace std::chrono;
static unsigned checks;
static void check(bool okay,const char* message){++checks;if(!okay)throw std::runtime_error(message);}
static uint64_t ms(){return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();}
static unsigned port(){int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"local socket");sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);check(bind(fd,(sockaddr*)&a,sizeof(a))==0,"local bind");socklen_t n=sizeof(a);check(getsockname(fd,(sockaddr*)&a,&n)==0,"local port");close(fd);return ntohs(a.sin_port);}
struct Response{int code=0;picojson::value value;};
static Response http(unsigned p,const char* path,const std::string& body="",bool get=false){
 int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"client socket");timeval timeout{2,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(p);check(connect(fd,(sockaddr*)&a,sizeof(a))==0,"client connect");
 const std::string data=std::string(get?"GET ":"POST ")+path+" HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer 123456\r\nContent-Length: "+std::to_string(body.size())+"\r\nConnection: close\r\n\r\n"+body;check(writeAll(fd,data.data(),data.size()),"request write");std::string reply;char block[8192];ssize_t n;while((n=recv(fd,block,sizeof(block),0))>0)reply.append(block,n);close(fd);auto split=reply.find("\r\n\r\n");check(split!=std::string::npos,"HTTP response framing");Response r;std::istringstream header(reply);std::string version;header>>version>>r.code;check(picojson::parse(r.value,reply.substr(split+4)).empty(),"JSON response");return r;
}
static bool flag(const Response&r,const char*k){const auto& v=r.value.get(k);return v.is<bool>()&&v.get<bool>();}
int main(){try{
 char temp[]="/tmp/control-v36-http-XXXXXX";check(mkdtemp(temp)!=nullptr,"temporary fixture");const std::string token=std::string(temp)+"/token";{std::ofstream f(token);f<<"123456\n";}
 phone_options o{};o.demo=1;o.bind_address="127.0.0.1";o.port=port();o.pair_token_file=token.c_str();o.left_mask=10;o.invert_mask=15;o.camera_width=1920;o.camera_height=1080;o.camera_view="both";
 phone_service service(o);std::string error;check(service.start(error),"production backend start");
 std::atomic<bool> done{false},polling{true};
 std::thread sensor([&]{while(!done){{std::lock_guard<std::mutex> guard(service.state_mutex);const auto now=imu_clock_ms();service.imu.initialized=service.imu.have_packet=service.imu.bias_valid=1;service.imu.calibrating=0;service.imu.received_ms=now;service.imu.sample.flags=15;service.imu.sample.ag_age=0;service.imu.sample.sequence++;service.imu.sample.timestamp=now;}std::this_thread::sleep_for(milliseconds(5));}});
 robotnav::GatewayOptions go;go.bind_address="127.0.0.1";go.port=port();go.backend_port=o.port;go.token_file=token;robotnav::GatewayCallbacks cb;std::atomic<bool> nav{false};cb.navigation_command=[&](const std::string&,double,double,std::string&){nav=true;return true;};cb.navigation_cancel=[&](const std::string&){nav=false;};cb.navigation_state=[](){return "{\"can_start\":true,\"state\":\"CRUISE\"}";};cb.navigation_map=[](){return "{\"available\":true}";};robotnav::Gateway gateway(go,cb);check(gateway.start(error),"production gateway start");
 struct Cleanup {std::atomic<bool>&done;std::thread&sensor;robotnav::Gateway&g;phone_service&s;~Cleanup(){g.stop();done=true;if(sensor.joinable())sensor.join();s.stop();}}cleanup{done,sensor,gateway,service};
 auto connected=http(go.port,"/api/connect","{}");check(connected.code==200,"connect accepted");const auto session=connected.value.get("session").get<std::string>();std::atomic<uint64_t> seq{0};
 auto command=[&](const char* op,const std::string& fields=""){return http(go.port,"/api/control","{\"session\":\""+session+"\",\"seq\":"+std::to_string(++seq)+",\"op\":\""+op+"\""+fields+"}");};
 std::this_thread::sleep_for(milliseconds(40));auto armed=command("arm");check(armed.code==200&&flag(armed,"armed"),"explicit arm is authoritative");check(armed.value.get("control_lease_ms").get<double>()==30000,"30 second public contract");check(armed.value.get("accepted_seq").get<double>()==1,"accepted seq echoes phone seq");
 auto moving=command("drive",",\"throttle\":0.5,\"steer\":0");check(moving.code==200,"real service drive accepted");std::this_thread::sleep_for(milliseconds(400));gateway.pollState();auto paused=http(go.port,"/api/state","",true);check(paused.code==200&&flag(paused,"armed")&&flag(paused,"motion_paused"),"350ms expires action only through HTTP");
 auto idle=command("idle");check(idle.code==200&&flag(idle,"armed"),"idle keeps logical arm");
 uint64_t stamp;{std::lock_guard<std::mutex>guard(service.state_mutex);stamp=service.controller.state().authorization_ms;}
 auto invalid=command("drive",",\"throttle\":2,\"steer\":0");check(invalid.code==409,"invalid parameters rejected");{std::lock_guard<std::mutex>guard(service.state_mutex);check(service.controller.state().armed&&service.controller.state().authorization_ms==stamp,"invalid request neither disarms nor renews");}
 auto fresh=command("idle");check(fresh.code==200,"valid command after 409 remains connected");
 // Continual non-authority-changing requests cannot starve source telemetry.
 std::atomic<unsigned> fresh_reads{0};std::thread poller([&]{while(polling){gateway.pollState();std::this_thread::sleep_for(milliseconds(10));}});
 struct PollGuard{std::atomic<bool>&on;std::thread&t;~PollGuard(){on=false;if(t.joinable())t.join();}}poll_guard{polling,poller};
 uint64_t prior=0;for(unsigned i=0;i<35;i++){check(command("drive",",\"throttle\":0,\"steer\":0").code==200,"20Hz command while telemetry polls");auto r=http(go.port,"/api/state","",true);check(r.code==200,"cached state remains available");auto source=uint64_t(r.value.get("gateway_state_age_ms").get<double>());if(source<150)++fresh_reads;std::this_thread::sleep_for(milliseconds(40));}
 check(fresh_reads>=30,"GET cache continuously refreshed under 20Hz ordinary commands");(void)prior;
 // Begin actual private capability / physical-velocity path.
 check(command("drive",",\"throttle\":0,\"steer\":0").code==200,"zero intent accepted");check(gateway.requestNavigation("straight",0,0,error),"navigation preparation accepts real backend");check(gateway.navigationActive(),"navigation active under simulation ACK4");
 const auto generation=gateway.generation();for(unsigned i=0;i<20;i++){check(command("idle").code==200,"phone idle renews private owner lease without cancelling nav");check(gateway.applyNav(.1,.1,generation),"MOT4 apply during owner heartbeat");std::this_thread::sleep_for(milliseconds(40));}
 {std::lock_guard<std::mutex>guard(service.state_mutex);stamp=service.controller.state().authorization_ms;}
 for(unsigned i=0;i<20;i++){check(gateway.applyNav(.1,.1,generation),"internal authorized navigation continues");std::this_thread::sleep_for(milliseconds(30));}
 {std::lock_guard<std::mutex>guard(service.state_mutex);check(service.controller.state().authorization_ms==stamp,"internal twist does not extend real phone authority");}
 gateway.cancelNavigation("encoder_wheel_disagreement",generation);check(!gateway.navigationActive(),"algorithm fault cancels autonomous intent");{std::lock_guard<std::mutex>guard(service.state_mutex);check(service.controller.state().armed&&service.controller.state().left==0&&service.controller.state().linear_mps==0,"ordinary navigation fault leaves armed with exact zero");}
 check(command("idle").code==200,"idle after algorithm fault");polling=false;poller.join();
 // STOP beats delayed/replayed motion. Strict owner seq is checked by both hops.
 check(command("stop").code==200,"explicit stop");auto replay=http(go.port,"/api/control","{\"session\":\""+session+"\",\"seq\":1,\"op\":\"arm\"}");check(replay.code==409,"old ARM cannot undo stop");gateway.pollState();check(!flag(http(go.port,"/api/state","",true),"armed"),"stopped cache remains disarmed");
 check(command("arm").code==200,"new explicit arm after stop");auto final_idle=command("idle");check(final_idle.code==200,"start real 30 second idle window");
 const auto start=ms();std::cout<<"WAIT actual wall-clock authorization expiry 30 seconds\n"<<std::flush;
 while(ms()-start<30100){gateway.pollState();auto r=http(go.port,"/api/state","",true);if(ms()-start<29500)check(flag(r,"armed"),"GET alone retains pre-expiry authority but never extends it");std::this_thread::sleep_for(milliseconds(100));}
 gateway.pollState();auto expired=http(go.port,"/api/state","",true);check(!flag(expired,"armed"),"actual 30 second no-command expiry despite continual GETs");check(command("drive",",\"throttle\":0.5,\"steer\":0").code==409,"drive after real expiry cannot rearm");
 check(command("arm").code==200,"new explicit arm recovers from true expiry");check(command("disconnect").code==200,"explicit disconnect accepted");gateway.pollState();check(!flag(http(go.port,"/api/state","",true),"armed"),"disconnect revokes real service authority");check(command("arm").code==403,"disconnected session cannot rearm without a new connect");
 std::cout<<"PASS v36 production HTTP/gateway lifecycle "<<checks<<" checks; actual 30s timer, 20Hz cache freshness, private owner sync, nav-fault pause, stop/replay/disconnect\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
