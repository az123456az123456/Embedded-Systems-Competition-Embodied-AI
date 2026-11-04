// Production Gateway + actual loopback HTTP; no hardware or mocked internals.
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
namespace {
unsigned checks=0;
void check(bool value) { ++checks; assert(value); }
std::uint64_t ms() { return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count(); }
unsigned freePort() {
  int fd=socket(AF_INET,SOCK_STREAM,0);assert(fd>=0);sockaddr_in address{};
  address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  assert(!bind(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address)));
  socklen_t size=sizeof(address);assert(!getsockname(fd,reinterpret_cast<sockaddr*>(&address),&size));
  close(fd);return ntohs(address.sin_port);
}
struct Reply { int status=0; picojson::value json; };
Reply request(unsigned port,const std::string& path,const std::string& body="",bool get=false) {
  int fd=socket(AF_INET,SOCK_STREAM,0);assert(fd>=0);timeval timeout{2,0};
  setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));sockaddr_in address{};
  address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=htons(port);
  assert(!connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address)));
  const auto bytes=std::string(get?"GET ":"POST ")+path+" HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer 012345\r\nContent-Length: "+std::to_string(body.size())+"\r\nConnection: close\r\n\r\n"+body;
  assert(robotphone::writeAll(fd,bytes.data(),bytes.size()));std::string all;char buffer[4096];ssize_t size;
  while((size=recv(fd,buffer,sizeof(buffer),0))>0)all.append(buffer,size);
  close(fd);const auto at=all.find("\r\n\r\n");assert(at!=std::string::npos);
  Reply result;std::istringstream header(all);std::string protocol;header>>protocol>>result.status;
  assert(picojson::parse(result.json,all.substr(at+4)).empty());return result;
}
std::string error(const Reply& reply) { return reply.json.get("error").get<std::string>(); }
bool flag(const Reply& reply,const char* key) { return reply.json.get("navigation").get(key).get<bool>(); }
template<class Predicate> void waitFor(Predicate ready) {
  const auto until=ms()+1500;while(!ready()&&ms()<until)std::this_thread::sleep_for(milliseconds(1));check(ready());
}
struct Backend {
  robotphone::HttpServer server;std::mutex mutex;bool armed=false;std::uint64_t sequence=0;
  double left=0,right=0;std::vector<std::string> operations;std::function<void(int)> video_handler;
  std::atomic<unsigned> delay_zero{0},delay_claim{0},zero_entered{0},claim_entered{0};
  std::atomic<bool> reject_zero{false},reject_claim{false};
  std::atomic<unsigned> ack_protocol{4};std::atomic<bool> softcore_ready{true};std::string softcore_reason="ready";
  std::string state(bool okay=true) const {
    return std::string("{\"ok\":")+(okay?"true":"false")+",\"session\":\"session\",\"armed\":"+(armed?"true":"false")+
      ",\"imu_action_pending\":false,\"reason\":\"fixture\",\"motor\":{\"velocity_protocol_supported\":true,\"softcore_ready\":"+(softcore_ready?"true":"false")+",\"softcore_reason\":\""+softcore_reason+"\",\"ack_protocol\":"+std::to_string(ack_protocol.load())+",\"link_ok\":true,\"left_mask\":10,\"invert_mask\":15,\"left\":"+std::to_string(left)+",\"right\":"+std::to_string(right)+
      ",\"ack_age_ms\":1},\"imu\":{\"valid\":true,\"fresh\":true,\"bias_valid\":true,\"calibrating\":false,\"yaw\":0,\"roll\":0,\"pitch\":0,\"timestamp_ms\":100,\"sequence\":10,\"sample_age_ms\":1}}";
  }
  void start(unsigned port) {
    std::string problem;check(server.start("127.0.0.1",port,[&](int fd,const robotphone::HttpRequest& req) {
      if(req.path=="/api/video.mjpg"){
        std::function<void(int)> callback;{std::lock_guard<std::mutex> lock(mutex);callback=video_handler;}
        if(callback){callback(fd);return;}
      }
      if(req.path=="/api/state"||req.path=="/api/connect") {
        std::lock_guard<std::mutex> lock(mutex);robotphone::httpReply(fd,200,state());return;
      }
      picojson::value value;assert(picojson::parse(value,req.body).empty());const auto op=value.get("op").get<std::string>();
      if(op=="drive") { ++zero_entered;std::this_thread::sleep_for(milliseconds(delay_zero.load())); }
      if(op=="claim") { ++claim_entered;std::this_thread::sleep_for(milliseconds(delay_claim.load())); }
      std::lock_guard<std::mutex> lock(mutex);int code=200;const auto incoming=std::uint64_t(value.get("seq").get<double>());
      if(incoming<=sequence)code=409;
      else {
        sequence=incoming;operations.push_back(op);
        if(op=="arm")armed=true;
        else if(op=="stop"||op=="disarm") { armed=false;left=right=0; }
        else if((op=="drive"&&reject_zero)||(op=="claim"&&reject_claim))code=409;
        else if((op=="drive"||op=="claim"||op=="twist")&&!armed)code=409;
        else if(op=="drive") { left=right=value.get("throttle").get<double>(); }
        else if(op=="pause") { left=right=0; }
        else if(op=="twist") {
          assert(req.path=="/api/navigation/twist"&&value.get("navigation_ticket").get<std::string>()=="private-capability");
          left=value.get("linear_mps").get<double>();right=value.get("angular_rps").get<double>();
        }
      }
      auto output=state(code==200);if(op=="claim"&&code==200)output.insert(output.size()-1,",\"navigation_ticket\":\"private-capability\"");
      robotphone::httpReply(fd,code,output);
    },problem));
  }
  bool stopped() { std::lock_guard<std::mutex> lock(mutex);return !armed&&left==0&&right==0; }
  bool paused() { std::lock_guard<std::mutex> lock(mutex);return armed&&left==0&&right==0; }
};
struct Fixture {
  Backend backend;unsigned bp=freePort(),gp=freePort();std::string token;
  std::unique_ptr<robotnav::Gateway> gateway;
  std::atomic<bool> core_straight{false},report_straight{false},reject_core{false};
  std::atomic<unsigned> callbacks{0},cancellations{0},unsafe_callbacks{0};
  std::uint64_t mobile=0,nav=0;std::string last_operation;
  explicit Fixture(bool complete=true,unsigned lease_ms=30000) {
    backend.start(bp);token="/tmp/gateway-straight-"+std::to_string(getpid())+"-"+std::to_string(gp);{std::ofstream file(token);file<<"012345\n";}
    robotnav::GatewayCallbacks attached;
    attached.navigation_command=[&](const std::string& op,double x,double y,std::string& problem) {
      ++callbacks;last_operation=op;
      if(op=="straight"||op=="start"||op=="explore") {
        std::lock_guard<std::mutex> lock(backend.mutex);
        if(!backend.armed||backend.left!=0||backend.right!=0||backend.operations.empty()||backend.operations.back()!="claim")++unsafe_callbacks;
        check(x==0&&y==0);
        if(reject_core) { problem="sonar_invalid";return false; }
        core_straight=op=="straight";return true;
      }
      return false;
    };
    if(complete)attached.navigation_cancel=[&](const std::string&) { core_straight=false;++cancellations; };
    attached.navigation_state=[&] { return std::string("{\"can_start\":true,\"straight_assist_active\":")+((core_straight||report_straight)?"true":"false")+"}"; };
    robotnav::GatewayOptions options;options.backend_port=bp;options.port=gp;options.bind_address="127.0.0.1";options.token_file=token;options.owner_lease_ms=lease_ms;
    gateway=std::make_unique<robotnav::Gateway>(options,attached);std::string problem;check(gateway->start(problem));
    check(request(gp,"/api/connect","{}").status==200);
  }
  ~Fixture() { gateway->stop();backend.server.stop();unlink(token.c_str()); }
  Reply control(const std::string& op,const std::string& fields="") {
    return request(gp,"/api/control","{\"session\":\"session\",\"seq\":"+std::to_string(++mobile)+",\"op\":\""+op+"\""+fields+"}");
  }
  Reply start(const std::string& op="straight",std::uint64_t intent=0,std::uint64_t navigation=0) {
    if(!intent)intent=mobile;
    if(!navigation)navigation=++nav;else nav=std::max(nav,navigation);
    return request(gp,"/api/navigation","{\"session\":\"session\",\"op\":\""+op+"\",\"nav_seq\":"+std::to_string(navigation)+",\"intent_seq\":"+std::to_string(intent)+"}");
  }
  Reply status() { return request(gp,"/api/state","",true); }
  void prepare() {
    backend.delay_zero=backend.delay_claim=0;
    check(control("arm").status==200);check(control("drive",",\"throttle\":0,\"steer\":0").status==200);
  }
};
}
int main() {
  {
    Fixture fixture(false);fixture.prepare();auto state=fixture.status();
    check(!flag(state,"supports_straight_assist"));check(!flag(state,"straight_assist_active"));
    auto reply=fixture.start();check(reply.status==409&&error(reply)=="navigation_not_attached");
    check(fixture.callbacks==0&&!fixture.gateway->navigationActive());
  }
  {
    Fixture fixture;auto state=fixture.status();check(flag(state,"supports_straight_assist"));
    check(flag(state,"supports_start_stop"));check(!flag(state,"straight_assist_active"));
    auto missing=request(fixture.gp,"/api/navigation","{\"session\":\"session\",\"op\":\"straight\"}");
    check(missing.status==400&&error(missing)=="invalid_navigation_request");
    auto other=request(fixture.gp,"/api/navigation","{\"session\":\"other\",\"op\":\"straight\",\"nav_seq\":1,\"intent_seq\":1}");
    check(other.status==403);check(!fixture.gateway->navigationActive()&&fixture.backend.stopped());
    check(fixture.control("arm").status==200);auto no_zero=fixture.start();
    check(no_zero.status==409&&error(no_zero)=="navigation_zero_intent_required");check(fixture.callbacks==0);
    fixture.prepare();{std::lock_guard<std::mutex> lock(fixture.backend.mutex);fixture.backend.armed=false;}
    fixture.gateway->pollState();auto no_arm=fixture.start();
    check(no_arm.status==409&&error(no_arm)=="arm_from_phone_first");check(fixture.callbacks==0&&fixture.backend.stopped());
  }
  {
    Fixture fixture;fixture.prepare();const auto intent=fixture.mobile;auto reply=fixture.start();
    check(reply.status==200&&fixture.gateway->navigationActive());check(fixture.last_operation=="straight");
    check(fixture.callbacks==1&&fixture.unsafe_callbacks==0);check(flag(reply,"straight_assist_active"));
    const auto generation=fixture.gateway->generation();check(fixture.gateway->applyNav(.12,.11,generation));
    check(flag(fixture.status(),"straight_assist_active"));
    auto replay=fixture.start("straight",intent,fixture.nav);check(replay.status==409&&error(replay)=="stale_navigation_sequence");
    auto reused=fixture.start("straight",intent);check(reused.status==409&&error(reused)=="navigation_zero_intent_required");
    check(fixture.callbacks==1&&fixture.gateway->navigationActive());
    check(fixture.control("drive",",\"throttle\":0.1,\"steer\":0.1").status==200);
    check(!fixture.gateway->navigationActive()&&!fixture.core_straight);check(!fixture.gateway->applyNav(.2,.2,generation));
    check(!flag(fixture.status(),"straight_assist_active"));
    // Core-requested mode alone must not advertise running while the gateway is inactive.
    fixture.report_straight=true;check(!flag(fixture.status(),"straight_assist_active"));
    check(fixture.control("stop").status==200);check(fixture.backend.stopped());
    fixture.report_straight=false;fixture.prepare();reply=fixture.start("start");
    check(reply.status==200&&fixture.last_operation=="start"&&!flag(reply,"straight_assist_active"));
    check(fixture.control("stop").status==200);fixture.prepare();reply=fixture.start("explore");
    check(reply.status==200&&fixture.last_operation=="explore"&&!flag(reply,"straight_assist_active"));
  }
  {
    Fixture fixture(true,350);fixture.prepare();const auto expired=fixture.mobile;std::this_thread::sleep_for(milliseconds(370));
    check(fixture.control("idle").status==200);auto reply=fixture.start("straight",expired);
    check(reply.status==409&&error(reply)=="navigation_zero_intent_required");check(fixture.callbacks==0);
    fixture.prepare();const auto old_zero=fixture.mobile;
    check(fixture.control("drive",",\"throttle\":0.2,\"steer\":0").status==200);
    reply=fixture.start("straight",old_zero);check(reply.status==409&&error(reply)=="stale_navigation_intent");
    check(fixture.callbacks==0&&!fixture.gateway->navigationActive());
  }
  for(bool during_zero:{true,false}) {
    Fixture fixture;fixture.prepare();fixture.backend.delay_zero=100;fixture.backend.delay_claim=100;
    const auto zero=fixture.backend.zero_entered.load(),claim=fixture.backend.claim_entered.load();Reply start,stop;
    std::thread starter([&] { start=fixture.start(); });
    waitFor([&] { return during_zero?fixture.backend.zero_entered>zero:fixture.backend.claim_entered>claim; });
    check(!fixture.gateway->navigationActive()&&!fixture.core_straight&&fixture.callbacks==0);
    std::thread stopper([&] { stop=fixture.control("stop"); });starter.join();stopper.join();
    check(start.status==409&&error(start)=="navigation_cancelled");check(stop.status==200);
    check(fixture.callbacks==0&&!fixture.gateway->navigationActive()&&!fixture.core_straight&&fixture.backend.stopped());
    check(!flag(fixture.status(),"straight_assist_active"));
  }
  for(bool reject_zero:{true,false}) {
    Fixture fixture;fixture.prepare();fixture.backend.reject_zero=reject_zero;fixture.backend.reject_claim=!reject_zero;
    auto reply=fixture.start();check(reply.status==409);check(error(reply)==(reject_zero?"navigation_zero_rejected":"navigation_capability_rejected"));
    check(fixture.callbacks==0&&!fixture.gateway->navigationActive()&&fixture.backend.paused());
  }
  {
    Fixture fixture;fixture.prepare();fixture.reject_core=true;auto reply=fixture.start();
    check(reply.status==409&&error(reply)=="sonar_invalid");check(fixture.callbacks==1&&fixture.unsafe_callbacks==0);
    check(!fixture.gateway->navigationActive()&&!fixture.core_straight&&fixture.backend.paused());
  }
  {
    Fixture fixture(true,350);fixture.prepare();check(fixture.start().status==200);const auto generation=fixture.gateway->generation();
    check(fixture.gateway->applyNav(.1,.1,generation));std::this_thread::sleep_for(milliseconds(390));
    check(!fixture.gateway->navigationActive()&&!fixture.core_straight&&fixture.backend.stopped());
    check(!fixture.gateway->applyNav(.1,.1,generation));check(!flag(fixture.status(),"straight_assist_active"));
  }
  std::printf("PASS %u real-HTTP straight-assist checks: complete attachment, arm/zero/one-use sequences, expired/replayed intent, begin zero+claim order, manual/STOP races, backend/core rejection, explicit shortened350ms expiry fixture; default30000ms, rejection pauses without disarm, capability and start/explore compatibility\n",checks);
  return 0;
}
