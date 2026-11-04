#include "../gateway.h"
#include "../vendor/http_server.h"
#include "../vendor/picojson.h"
#include <arpa/inet.h>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
unsigned freePort() {
  int fd = socket(AF_INET, SOCK_STREAM, 0); assert(fd >= 0);
  sockaddr_in address{}; address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  assert(bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0);
  socklen_t length = sizeof(address); assert(getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length) == 0);
  unsigned port = ntohs(address.sin_port); close(fd); return port;
}
struct Response { int code = 0; picojson::value value; };
Response request(unsigned port, const std::string &method, const std::string &path,
                 const std::string &body = "", const std::string &token = "012345") {
  int fd = socket(AF_INET, SOCK_STREAM, 0); assert(fd >= 0);
  timeval timeout{2,0}; setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  sockaddr_in address{}; address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(port);
  assert(connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0);
  std::string message = method + " " + path + " HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer " + token +
      "\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
  assert(robotphone::writeAll(fd, message.data(), message.size()));
  std::string bytes; char buffer[4096]; ssize_t size;
  while ((size = recv(fd, buffer, sizeof(buffer), 0)) > 0) bytes.append(buffer, size_t(size));
  close(fd); assert(bytes.find("\r\n\r\n") != std::string::npos);
  Response result; std::string protocol; std::istringstream status(bytes); status >> protocol >> result.code;
  assert(picojson::parse(result.value, bytes.substr(bytes.find("\r\n\r\n") + 4)).empty()); return result;
}
bool flag(const picojson::value &value, const char *key) { return value.get(key).get<bool>(); }
std::string text(const picojson::value &value, const char *key) { return value.get(key).get<std::string>(); }
double number(const picojson::value &value, const char *key) { return value.get(key).get<double>(); }

struct Backend {
  robotphone::HttpServer server; std::mutex mutex; std::string session;
  unsigned connections = 0; bool armed = false, imu_fresh = true, calibrating = false, fail_drive = false;
  unsigned left_mask = 10, invert_mask = 15;
  bool missing_yaw = false, wrong_mapping = false;std::atomic<bool> reject_disarm{false};
  bool velocity_supported=true,softcore_ready=true;unsigned ack_protocol=4;
  bool imu_pending = false;
  double throttle = 0, steer = 0; std::uint64_t last_sequence = 0;
  std::vector<std::string> operations;
  std::atomic<bool> delay_drive{false}, entered_drive{false};
  std::string state(bool okay = true, bool include_session = false) const {
    std::ostringstream out;
    out << "{\"ok\":" << (okay ? "true" : "false") << ",\"imu_action_pending\":" << (imu_pending ? "true" : "false") << ",\"armed\":" << (armed ? "true" : "false")
        << ",\"reason\":\"fixture\",\"motor\":{\"velocity_protocol_supported\":"<<(velocity_supported?"true":"false")<<",\"softcore_ready\":"<<(softcore_ready?"true":"false")<<",\"ack_protocol\":"<<ack_protocol<<",\"link_ok\":true,\"left_mask\":" << (wrong_mapping ? 7 : left_mask) << ",\"invert_mask\":" << invert_mask << ",\"left\":" << .6 * (throttle + steer)
        << ",\"right\":" << .6 * (throttle - steer) << ",\"ack_age_ms\":8},\"imu\":{\"valid\":true,\"fresh\":"
        << (imu_fresh ? "true" : "false") << ",\"calibrating\":" << (calibrating ? "true" : "false")
        << ",\"bias_valid\":true," << (missing_yaw ? "" : "\"yaw\":12.5,") << "\"roll\":1,\"pitch\":2,\"sequence\":42,\"sample_age_ms\":11}";
    if (include_session) out << ",\"session\":\"" << session << '"';
    return out.str() + '}';
  }
  void start(unsigned port) {
    std::string error;
    assert(server.start("127.0.0.1", port, [this](int client, const robotphone::HttpRequest &request) {
      std::lock_guard<std::mutex> lock(mutex); int code = 200; std::string answer;
      assert(request.headers.at("authorization") == "Bearer 012345");
      if (request.path == "/api/state") answer = state();
      else if (request.path == "/api/connect") {
        if (armed) { code = 409; answer = state(false); }
        else { session = "session-" + std::to_string(++connections); last_sequence = 0; answer = state(true, true); }
      } else if (request.path == "/api/control" || request.path == "/api/navigation/claim" || request.path == "/api/navigation/twist") {
        picojson::value v; assert(picojson::parse(v, request.body).empty());
        auto op = text(v, "op"); auto sequence = std::uint64_t(number(v, "seq"));
        if (text(v, "session") != session) { code = 403; answer = state(false); }
        else if (sequence <= last_sequence) { code = 409; answer = state(false); }
        else {
          last_sequence = sequence; operations.push_back(op);
          if (op == "disarm" && reject_disarm) { code=503; }
          else if (op == "claim") { assert(request.path=="/api/navigation/claim"); }
          else if (op == "arm") armed = true;
          else if (op == "disarm" || op == "stop" || op == "configure" || op == "zero" || op == "calibrate") {
            armed = false; throttle = steer = 0;
            if (op == "configure") { left_mask = unsigned(number(v, "left_mask")); invert_mask = unsigned(number(v, "invert_mask")); }
          }
          else if (op == "idle") throttle = steer = 0;
          else if (op == "drive" || op=="twist") {
            if (delay_drive) { entered_drive = true; std::this_thread::sleep_for(std::chrono::milliseconds(80)); }
            if (fail_drive || !armed) code = 409;
            else if(op=="twist") { assert(request.path=="/api/navigation/twist"&&text(v,"navigation_ticket")=="fixture-private-ticket");throttle=(number(v,"linear_mps")+number(v,"angular_rps"))/1.2;steer=(number(v,"linear_mps")-number(v,"angular_rps"))/1.2; }
            else { throttle = number(v, "throttle"); steer = number(v, "steer"); }
          }
          answer = state(code == 200);
          if(op=="claim")answer.insert(answer.size()-1,",\"navigation_ticket\":\"fixture-private-ticket\"");
        }
      } else if (request.path == "/api/video.mjpg") {
        robotphone::httpReply(client, 200, "{\"passthrough\":true}"); return;
      } else { code = 404; answer = "{}"; }
      robotphone::httpReply(client, code, answer);
    }, error));
  }
};
}  // namespace

int main(){
 const unsigned bp=freePort(),gp=freePort();Backend b;b.start(bp);
 const std::string token="/tmp/fpga-gateway-test-"+std::to_string(getpid());{std::ofstream out(token);out<<"012345\n";}
 std::vector<robotnav::FpgaNavigationIntent> intents;std::mutex m;std::atomic<bool> ready{true},mapping_ready{false},manual_ready{false},execution_pending{true},semantic_ready{false};unsigned cancellations=0;
 robotnav::GatewayCallbacks cb;
 cb.fpga_navigation_ready=[&](std::string&e){if(!ready)e="NVA1_not_ready";return ready.load();};
 cb.fpga_navigation_command=[&](const robotnav::FpgaNavigationIntent&i,std::string&){std::lock_guard<std::mutex>l(m);intents.push_back(i);return true;};
 cb.exploration_mapping_ready=[&](std::string&e){if(!mapping_ready)e="exploration_adapter_not_attached";return mapping_ready.load();};
 cb.manual_mapping_ready=[&](std::string&e){if(!manual_ready)e="manual_mapping_not_ready";return manual_ready.load();};
 cb.semantic_search_ready=[&](std::string&e){if(!semantic_ready)e="semantic_service_not_ready";return semantic_ready.load();};
 cb.fpga_execution_pending=[&]{return execution_pending.load();};
 std::mutex display_mutex;std::string display_snapshot="{}";
 cb.navigation_state=[&]{std::lock_guard<std::mutex>l(display_mutex);return display_snapshot;};
 cb.navigation_cancel=[&](const std::string&){++cancellations;};
 robotnav::GatewayOptions o;o.bind_address="127.0.0.1";o.port=gp;o.backend_port=bp;o.token_file=token;o.fpga_navigation_source=true;
 robotnav::Gateway gateway(o,cb);std::string error;assert(gateway.start(error));
 auto connection=request(gp,"POST","/api/connect","{}");assert(connection.code==200);auto session=text(connection.value,"session");uint64_t seq=0,navseq=0;
 auto control=[&](const std::string&op,const std::string&extra=""){return request(gp,"POST","/api/control","{\"session\":\""+session+"\",\"op\":\""+op+"\",\"seq\":"+std::to_string(++seq)+extra+"}");};
 auto nav=[&](const std::string&op,const std::string&extra=""){return request(gp,"POST","/api/navigation","{\"session\":\""+session+"\",\"op\":\""+op+"\",\"nav_seq\":"+std::to_string(++navseq)+extra+"}");};
 unsigned checks=0;auto check=[&](bool pass){if(!pass)std::cerr<<"FAIL_CHECK_INDEX "<<checks<<"\n";assert(pass);++checks;};
 check(nav("start").code==409);check(control("arm").code==200);
 auto n=b.operations.size();check(nav("goto",",\"x\":1.05,\"y\":0.05,\"map_session\":\"ros-1\",\"map_revision\":1").code==200);
 check(!gateway.navigationActive()&&intents.back().operation=="goal"&&b.operations.size()==n);
 check(nav("start").code==200);check(gateway.navigationActive()&&b.operations.size()==n);
 check(!gateway.applyNav(.2,0,gateway.generation()));
 check(nav("goal",",\"x\":1.05,\"y\":0.05,\"map_session\":\"ros-1\",\"map_revision\":1").code==409);
 check(nav("stop").code==200);check(!gateway.navigationActive()&&cancellations>0&&b.operations.size()==n);
 check(nav("goto",",\"x\":1.05,\"y\":0.05,\"map_session\":\"ros-1\",\"map_revision\":1").code==200);check(!gateway.navigationActive());
 check(nav("mapping-start").code==409); // Generic FPGA adapter alone is insufficient.
 check(!flag(request(gp,"GET","/api/state").value.get("navigation"),"supports_exploration_mapping"));
 mapping_ready=true;
 check(flag(request(gp,"GET","/api/state").value.get("navigation"),"supports_exploration_mapping"));
 check(nav("mapping-start").code==200);check(intents.back().operation=="mapping-start");
 const auto exploration_gen=gateway.generation();uint64_t drive_seq=0;
 auto drive=[&](double throttle,double steer=0.,uint64_t generation=0,unsigned ttl=200,int age=0){
   return gateway.applyExplorationDrive("drive",throttle,steer,0.,false,generation?generation:exploration_gen,
      ++drive_seq,std::uint64_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count())-age,ttl,error);
 };
 check(drive(-.18));check(gateway.navigationActive()&&b.throttle==-.18&&b.steer==0.);
 check(drive(0.));check(b.throttle==0.&&gateway.navigationActive());
 check(drive(.30));check(drive(0.));
 for(double throttle:{-.180001,.300001,std::nan(""),double(INFINITY)}){
   const auto writes=b.operations.size();check(!drive(throttle)&&error=="invalid_exploration_command"&&b.operations.size()==writes);
 }
 check(!drive(-.18,.01)&&error=="invalid_exploration_command");
 check(!drive(-.18,0.,exploration_gen-1)&&error=="exploration_authority_revoked");
 check(!drive(-.18,0.,0,200,200)&&error=="stale_exploration_command");
 check(!drive(-.18,0.,0,501)&&error=="stale_exploration_command");
 check(drive(0.));

 // Delayed ROS/runtime status still belongs to the previous lifecycle. An
 // accepted new mapping request must not expose its IDLE/ERROR/LOADED as final.
 for(const char* old_phase:{"IDLE","LOADED","ERROR"}){
  {std::lock_guard<std::mutex>l(display_mutex);display_snapshot=std::string("{\"mapping\":{\"state\":\"")+old_phase+"\",\"active\":false,\"gateway_generation\":\"0\",\"map_commit_confirmed\":true}}";}
  auto status=request(gp,"GET","/api/state");auto mapping=status.value.get("navigation").get("mapping");
  check(text(mapping,"state")=="STARTING"&&flag(mapping,"active")&&!flag(mapping,"map_commit_confirmed"));
 }
 // A current-generation error is real and must remain visible; admission
 // projection must not hide failures or claim that a map was committed.
 for(const char* phase:{"WAITING","ERROR"}){
  {std::lock_guard<std::mutex>l(display_mutex);display_snapshot=std::string("{\"mapping\":{\"state\":\"")+phase+"\",\"active\":false,\"gateway_generation\":\""+std::to_string(exploration_gen)+"\"}}";}
  auto status=request(gp,"GET","/api/state");
  check(text(status.value.get("navigation").get("mapping"),"state")==phase);
 }
 {std::lock_guard<std::mutex>l(display_mutex);display_snapshot="{}";}
 const auto starts=intents.size();check(nav("mapping-start").code==409&&intents.size()==starts);
 check(nav("mapping-stop").code==200);check(intents.back().operation=="mapping-stop"&&!gateway.navigationActive());
 check(!drive(-.18)&&error=="exploration_authority_revoked");
 check(!flag(request(gp,"GET","/api/state").value.get("navigation"),"active"));
 check(!b.armed&&b.operations.back()=="disarm");
 const auto stopped_intents=intents.size();b.reject_disarm=true;
 check(nav("mapping-stop").code==409&&intents.size()==stopped_intents);b.reject_disarm=false;
 check(nav("mapping-start").code==409);check(control("arm").code==200);
 check(nav("mapping-start").code==200);
 execution_pending=false;ready=false;gateway.pollState();
 check(gateway.navigationActive());
 check(!flag(request(gp,"GET","/api/state").value.get("navigation"),"motion_active"));
 ready=true;execution_pending=true;gateway.pollState();
 check(control("drive",",\"throttle\":0.1,\"steer\":0").code==200);check(!gateway.navigationActive());
 check(nav("goal",",\"x\":1.05,\"y\":0.05,\"map_session\":\"ros-1\",\"map_revision\":1").code==200);
 check(nav("start").code==200);const auto running_generation=gateway.generation();
 check(!gateway.finishFpgaNavigation("old_terminal",running_generation-1)&&gateway.navigationActive());
 n=b.operations.size();check(gateway.finishFpgaNavigation("FPGA_arrived",running_generation));
 check(!gateway.navigationActive()&&b.operations.size()==n);
 ready=false;check(nav("start").code==409);ready=true;
 check(nav("mapping-start").code==200);execution_pending=true;ready=false;
 gateway.pollState();check(gateway.navigationActive());ready=true;
 check(nav("straight").code==400);check(!gateway.beginNavigation(error));
 check(intents.front().phone_sequence>0&&intents.front().owner_source_ms>0);
 check(nav("stop").code==200);check(control("drive",",\"throttle\":0,\"steer\":0").code==200);
 check(nav("semantic-start",",\"text\":\"找到未戴口罩的人\"").code==409);
 semantic_ready=true;
 const auto before_bad=intents.size();
 check(nav("semantic-start",",\"text\":\"不要找到未戴口罩的人\"").code==400&&intents.size()==before_bad);
 check(nav("semantic-start",",\"text\":\"找到红色水瓶\"").code==400&&intents.size()==before_bad);
 check(nav("semantic-start",",\"text\":\"找到未戴口罩的人\"").code==200);
 check(intents.back().operation=="semantic-start"&&intents.back().semantic_target=="NO-Mask"&&gateway.navigationActive());
 const auto sg=gateway.generation();
 const auto current_ms=std::uint64_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
 check(gateway.applyExplorationDrive("drive",.3,0.,0.,false,sg,1,current_ms,200,error));
 check(nav("stop").code==200);
 check(!gateway.applyExplorationDrive("drive",.3,0.,0.,false,sg,2,current_ms,200,error));
 check(control("arm").code==200);check(control("drive",",\"throttle\":0,\"steer\":0").code==200);
 check(nav("manual-mapping-start").code==409);manual_ready=true;
 const auto writes=b.operations.size();check(nav("manual-mapping-start").code==200);
 check(intents.back().operation=="manual-mapping-start"&&!gateway.navigationActive()&&b.operations.size()==writes);
 const auto mg=gateway.generation();
 check(!gateway.applyExplorationDrive("drive",.3,0.,0.,false,mg,1,current_ms,200,error));
 check(control("drive",",\"throttle\":0.2,\"steer\":0").code==200);
 check(!gateway.navigationActive());check(control("stop").code==200);
 check(nav("mapping-stop").code==200&&intents.back().operation=="mapping-stop");
 for(auto& op:b.operations)check(op!="twist"&&op!="claim");
 gateway.stop();b.server.stop();unlink(token.c_str());std::cout<<"{\"cases\":"<<checks<<",\"failed\":0,\"real_gateway_HTTP\":true,\"backend\":\"test transport fixture, not real dashboard\"}\n";
}
