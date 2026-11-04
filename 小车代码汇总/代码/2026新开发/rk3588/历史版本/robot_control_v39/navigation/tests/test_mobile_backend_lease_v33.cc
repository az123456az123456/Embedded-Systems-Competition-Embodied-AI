// Production Gateway and production dashboard Controller execute in this test.
// Loopback HTTP is real; IMU/FPGA-ACK observations and the phone owner are models.
// No AF_PACKET socket, camera, FPGA or physical motor is opened.
#include "../gateway.h"
#include "../vendor/http_server.h"
#include "../vendor/picojson.h"
#include "../../dashboard/phone/controller.h"
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <sys/socket.h>
#include <unistd.h>

namespace {
using namespace std::chrono;
std::atomic<unsigned> checks{0};
uint64_t now(){return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();}
void check(bool value,const char* why){++checks;if(!value)throw std::runtime_error(why);}
unsigned free_port(){int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"socket");sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);check(!bind(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)),"bind");socklen_t n=sizeof(a);check(!getsockname(fd,reinterpret_cast<sockaddr*>(&a),&n),"getsockname");close(fd);return ntohs(a.sin_port);}
struct Response{int status=0;picojson::value json;};
Response post(unsigned port,const std::string& path,const std::string& body){
 int fd=socket(AF_INET,SOCK_STREAM,0);check(fd>=0,"HTTP socket");timeval timeout{2,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
 sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);check(!connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)),"HTTP connect");
 auto request="POST "+path+" HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer 012345\r\nContent-Length: "+std::to_string(body.size())+"\r\nConnection: close\r\n\r\n"+body;
 check(robotphone::writeAll(fd,request.data(),request.size()),"HTTP write");std::string bytes;char block[4096];ssize_t n;while((n=recv(fd,block,sizeof(block),0))>0)bytes.append(block,n);close(fd);
 const auto split=bytes.find("\r\n\r\n");check(split!=std::string::npos,"HTTP response");Response result;std::string protocol;std::istringstream header(bytes);header>>protocol>>result.status;check(picojson::parse(result.json,bytes.substr(split+4)).empty(),"HTTP JSON");return result;
}
struct Backend {
 robotphone::Controller controller;std::mutex mutex;robotphone::HttpServer server;
 std::atomic<bool> running{true};std::thread tick_thread;uint64_t sequence=0;
 unsigned protocol=1;std::string capability;
 static robotphone::Config config(){robotphone::Config c;c.left_motor_mask=10;return c;}
 Backend():controller(config()){}
 ~Backend(){running=false;if(tick_thread.joinable())tick_thread.join();server.stop();}
 std::string state(bool okay=true){const auto& s=controller.state();return std::string("{\"ok\":")+(okay?"true":"false")+",\"session\":\"session\",\"armed\":"+(s.armed?"true":"false")+",\"reason\":\""+s.reason+"\",\"imu_action_pending\":false,\"motor\":{\"link_ok\":true,\"left_mask\":10,\"invert_mask\":15,\"left\":"+std::to_string(s.left)+",\"right\":"+std::to_string(s.right)+",\"ack_left\":0,\"ack_right\":0,\"ack_sequence\":1,\"ack_age_ms\":0,\"ack_protocol\":"+std::to_string(protocol)+",\"velocity_protocol_supported\":true,\"softcore_ready\":true,\"softcore_reason\":\"ready\"},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":0,\"roll\":0,\"pitch\":0,\"sequence\":1,\"timestamp_ms\":1,\"sample_age_ms\":0}}";}
 void start(unsigned port){
  std::string error;check(server.start("127.0.0.1",port,[&](int fd,const robotphone::HttpRequest& request){
   std::lock_guard<std::mutex> lock(mutex);const auto time=now();
   if(request.path=="/api/connect"||request.path=="/api/state"){robotphone::httpReply(fd,200,state());return;}
   picojson::value input;check(picojson::parse(input,request.body).empty(),"backend parse");const auto op=input.get("op").get<std::string>();const auto seq=uint64_t(input.get("seq").get<double>());bool okay=true;
   if(seq<=sequence&&op!="stop"&&op!="disarm")okay=false;else{
    sequence=std::max(sequence,seq);
    if(op=="arm")okay=controller.arm(seq,time);
    else if(op=="drive")okay=controller.drive(seq,input.get("throttle").get<double>(),input.get("steer").get<double>(),false,time);
    else if(op=="idle"){if(controller.state().mode!=robotphone::Mode::Disarmed&&controller.state().mode!=robotphone::Mode::Idle)okay=controller.drive(seq,0,0,false,time);}
    else if(op=="claim"){okay=controller.state().armed&&controller.state().mode==robotphone::Mode::Idle&&controller.driveVelocity(seq,0,0,time);if(okay){capability="private-loopback-ticket";protocol=4;}}
    else if(op=="twist"){okay=!capability.empty()&&input.get("navigation_ticket").is<std::string>()&&input.get("navigation_ticket").get<std::string>()==capability;if(okay)okay=controller.driveVelocity(seq,input.get("linear_mps").get<double>(),input.get("angular_rps").get<double>(),time);}
    else if(op=="stop")controller.stop(time);
    else if(op=="disarm")controller.disarm(time);
    else okay=false;
   }
   auto output=state(okay);if(op=="claim"&&okay)output.insert(output.size()-1,",\"navigation_ticket\":\"private-loopback-ticket\"");robotphone::httpReply(fd,okay?200:409,output);
  },error),"backend start");
  tick_thread=std::thread([&]{while(running){{std::lock_guard<std::mutex> lock(mutex);const auto time=now();controller.updateMotorLink(time,true);controller.updateArcProtocol(true);controller.updateVelocityProtocol(true,true);controller.updateImu({time,true,0,0,0});controller.tick(time);}std::this_thread::sleep_for(milliseconds(5));}});
  std::this_thread::sleep_for(milliseconds(15));
 }
 robotphone::State snapshot(){std::lock_guard<std::mutex> lock(mutex);return controller.state();}
};
void run(){
 Backend backend;const auto backend_port=free_port(),public_port=free_port();backend.start(backend_port);
 const auto token="/tmp/mobile-backend-lease-v33-"+std::to_string(getpid());{std::ofstream file(token);file<<"012345\n";}
 std::atomic<unsigned> cancels{0};std::string cancel_reason;
 robotnav::GatewayCallbacks cb;cb.navigation_command=[](const std::string&,double,double,std::string&){return true;};cb.navigation_cancel=[&](const std::string& why){++cancels;cancel_reason=why;};cb.navigation_state=[](){return "{\"can_start\":true}";};
 robotnav::GatewayOptions options;options.bind_address="127.0.0.1";options.port=public_port;options.backend_port=backend_port;options.token_file=token;
 robotnav::Gateway gateway(options,cb);std::string error;check(gateway.start(error),"gateway start");check(post(public_port,"/api/connect","{}").status==200,"connect");uint64_t seq=0;
 auto control=[&](const std::string& op,const std::string& extra=""){return post(public_port,"/api/control","{\"session\":\"session\",\"seq\":"+std::to_string(++seq)+",\"op\":\""+op+"\""+extra+"}");};
 // Manual motion has its own actual Controller350ms source-command lease,
 // even though the gateway's navigation-owner watchdog is not active.
 check(control("arm").status==200,"manual ARM");check(control("drive",",\"throttle\":0.25,\"steer\":0").status==200,"manual motion accepted");const auto motion_started=now();
 while(backend.snapshot().armed&&now()-motion_started<600)std::this_thread::sleep_for(milliseconds(5));
 const auto stopped=backend.snapshot();check(!stopped.armed&&stopped.reason=="lease_expired"&&stopped.left==0&&stopped.right==0,"actual Controller independently removes expired movement authority");check(now()-motion_started>=330&&now()-motion_started<550,"Controller350ms lease timing");
 check(control("drive",",\"throttle\":0.25,\"steer\":0").status==409,"late manual command cannot re-arm expired Controller");check(!backend.snapshot().armed,"late DRIVE remains zero/disarmed");
 std::cout<<"SCENARIO actual_Controller_motion_lease350ms stopped_after_ms="<<now()-motion_started<<" reason="<<stopped.reason<<'\n';
 // For navigation the main controller keeps producing fresh twists while
 // actual authenticated foreground owner arrivals are190/195ms apart.
 check(control("arm").status==200,"new explicit ARM");check(control("drive",",\"throttle\":0,\"steer\":0").status==200,"navigation zero intent");check(gateway.beginNavigation(error),"navigation capability");
 std::atomic<bool> running{true};std::thread navigator([&]{while(running){gateway.pollState();if(gateway.navigationActive())gateway.applyNav(.1,0,gateway.generation());std::this_thread::sleep_for(milliseconds(15));}});
 struct Join{std::atomic<bool>& run;std::thread& thread;~Join(){run=false;if(thread.joinable())thread.join();}}join{running,navigator};
 for(unsigned interval:{190U,195U}){
  const auto began=now();unsigned renewed=0;
  while(now()-began<2200){check(control("idle").status==200,"authenticated owner renewal");check(backend.snapshot().armed,"fresh190/195ms owner does not disarm actual Controller");std::this_thread::sleep_for(milliseconds(interval));++renewed;}
  check(gateway.navigationActive()&&cancels==0,"actual Gateway stays authorized with continuous190/195ms owner arrivals");std::cout<<"SCENARIO actual_Gateway_owner_interval_ms="<<interval<<" renewed="<<renewed<<" remained_armed=yes\n";
 }
 check(control("idle").status==200,"final current owner");const auto owner_stopped=now();
 while((gateway.navigationActive()||backend.snapshot().armed)&&now()-owner_stopped<750)std::this_thread::sleep_for(milliseconds(5));
 const auto owner_stop_latency=now()-owner_stopped;
 check(!gateway.navigationActive()&&!backend.snapshot().armed,"true missing foreground owner stops actual Gateway and Controller");check(now()-owner_stopped>=330&&now()-owner_stopped<700,"actual Gateway350ms owner timeout timing");
 check(cancels>0&&cancel_reason=="phone_lease_expired","owner expiry keeps original concrete cause");
 const auto epoch=gateway.generation();check(!gateway.applyNav(.2,0,epoch),"old route cannot issue motion after owner expiry");check(control("idle").status==200,"readable live session can receive idle after motor stop");std::this_thread::sleep_for(milliseconds(100));check(!backend.snapshot().armed&&!gateway.navigationActive(),"returning owner heartbeat cannot rearm or restart route");
 check(control("stop").status==200,"explicit STOP remains admitted while disarmed");
 std::cout<<"SCENARIO actual_Gateway_owner_lease350ms stopped_after_ms="<<owner_stop_latency<<" reason="<<cancel_reason<<'\n';
 running=false;navigator.join();gateway.stop();unlink(token.c_str());
}
}
int main(){try{run();std::cout<<"PASS "<<checks<<" actual Gateway/Controller backend authority checks; no physical IO\n";return 0;}catch(const std::exception& e){std::cerr<<"FAIL mobile/backend lease: "<<e.what()<<'\n';return 1;}}
