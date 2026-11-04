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
  bool missing_yaw = false, wrong_mapping = false;
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
          if (op == "claim") { assert(request.path=="/api/navigation/claim"); }
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

int main() {
  const unsigned backend_port = freePort(), gateway_port = freePort();
  Backend backend; backend.start(backend_port);
  const std::string token = "/tmp/nav-gateway-test-" + std::to_string(getpid());
  { std::ofstream file(token); file << "012345\n"; }
  std::atomic<unsigned> cancellations{0}, accepted{0}; std::string last_cancel; std::mutex callback_mutex;
  robotnav::GatewayCallbacks callbacks;
  std::atomic<unsigned> synchronized_layout{0}, command_layout{0};
  callbacks.navigation_layout = [&](const robotnav::GatewayTelemetry& t) {
    if(t.available && t.link_ok && t.left_mask<16 && __builtin_popcount(t.left_mask)==2)
      synchronized_layout=t.left_mask;
  };
  callbacks.navigation_command = [&](const std::string &op, double, double, std::string &) { command_layout=synchronized_layout.load(); ++accepted; return op == "goal" || op == "start" || op == "explore" || op == "reset-map"; };
  callbacks.navigation_cancel = [&](const std::string &reason) { ++cancellations; std::lock_guard<std::mutex> lock(callback_mutex); last_cancel = reason; };
  callbacks.navigation_state = [] { return "{\"can_start\":true,\"readiness_reason\":\"\",\"start_mode\":\"explore\",\"phase\":\"fixture\"}"; };
  callbacks.navigation_map = [] { return "{\"schema\":1,\"available\":true,\"cells\":\"fixture\",\"snapshot_age_ms\":0}"; };
  unsigned goto_calls=0;
  callbacks.navigation_goto = [&](double x,double y,const std::string& map_session,std::uint32_t revision,std::string& e){
    ++goto_calls;
    if(map_session!="boot-0"||revision!=42){e="map_snapshot_expired";return false;}
    if(x!=1.05||y!=.05){e="goal_not_verified_free";return false;}
    return true;
  };
  robotnav::GatewayOptions options; options.bind_address = "127.0.0.1"; options.port = gateway_port;
  options.backend_port = backend_port; options.token_file = token;
  robotnav::Gateway gateway(options, callbacks); std::string error;
  assert(gateway.start(error));
  unsigned checks = 0;
  auto check = [&](bool okay) { assert(okay); ++checks; };
  check(request(gateway_port, "GET", "/api/state", "", "wrong").code == 401);
  check(request(gateway_port, "GET", "/api/navigation/map", "", "wrong").code == 401);
  auto map = request(gateway_port, "GET", "/api/navigation/map");
  check(map.code == 200 && flag(map.value, "available") && number(map.value, "schema") == 1);
  check(backend.connections == 0 && backend.operations.empty());
  check(!gateway.requestNavigation("start", 0, 0, error));
  check(gateway.requestNavigation("reset-map", 0, 0, error));
  auto connected = request(gateway_port, "POST", "/api/connect", "{}");
  check(connected.code == 200); auto session = text(connected.value, "session");
  std::uint64_t sequence = 0;
  auto command = [&](const std::string &op, const std::string &extra = "") {
    return request(gateway_port, "POST", "/api/control", "{\"session\":\"" + session + "\",\"seq\":" + std::to_string(++sequence) + ",\"op\":\"" + op + '"' + extra + '}');
  };
  check(command("arm").code == 200);
  check(gateway.requestNavigation("goal", 1, 2, error));
  check(gateway.requestNavigation("start", 0, 0, error));
  auto generation = gateway.generation();
  auto telem = gateway.pollState();
  check(telem.available && telem.armed && telem.imu_fresh && telem.imu_sequence == 42 && telem.yaw_deg == 12.5);
  check(telem.left_mask == 10 && telem.invert_mask == 15 && telem.imu_sample_age_ms == 11);
  check(gateway.applyNav(.2, .1, generation));
  {
    std::lock_guard<std::mutex> lock(backend.mutex);
    check(std::abs(.6 * (backend.throttle + backend.steer) - .2) < 1e-10);
    check(std::abs(.6 * (backend.throttle - backend.steer) - .1) < 1e-10);
  }
  auto before_idle = backend.last_sequence;
  check(command("idle").code == 200); check(backend.last_sequence == before_idle);
  check(gateway.navigationActive());
  check(request(gateway_port, "POST", "/api/connect", "{}").code == 409);
  check(gateway.navigationActive());
  check(request(gateway_port, "POST", "/api/control", "{\"session\":\"forged\",\"seq\":999,\"op\":\"stop\"}").code == 403);
  check(gateway.navigationActive());
  check(request(gateway_port, "POST", "/api/control", "{\"session\":\"" + session + "\",\"seq\":1,\"op\":\"stop\"}").code == 200);
  check(!gateway.navigationActive()); check(!gateway.applyNav(.2, .2, generation));
  check(!backend.armed);
  check(command("arm").code == 200); check(gateway.requestNavigation("explore", 0, 0, error));
  generation = gateway.generation(); check(gateway.applyNav(.2, .2, generation));
  check(command("drive", ",\"throttle\":0.3,\"steer\":-0.1").code == 200);
  check(!gateway.navigationActive()); check(!gateway.applyNav(.2, .2, generation));
  check(backend.throttle == .3 && backend.steer == -.1);
  check(command("idle").code == 200);
  check(gateway.requestNavigation("start", 0, 0, error));
  generation = gateway.generation(); check(gateway.applyNav(.15, .15, generation));
  // Map viewing must never renew the foreground driver's lease.
  for(int i=0;i<5;++i){check(request(gateway_port,"GET","/api/navigation/map").code==200);std::this_thread::sleep_for(std::chrono::milliseconds(90));}
  check(!gateway.navigationActive()); check(!backend.armed); check(!gateway.applyNav(.2, .2, generation));
  check(command("arm").code == 200);
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.imu_fresh = false; }
  check(!gateway.requestNavigation("start", 0, 0, error)); check(error == "imu_not_ready");
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.imu_fresh = true; backend.calibrating = true; }
  check(!gateway.requestNavigation("start", 0, 0, error));
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.calibrating = false; }
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.missing_yaw = true; }
  check(!gateway.requestNavigation("start", 0, 0, error)); check(error == "imu_not_ready");
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.missing_yaw = false; backend.wrong_mapping = true; }
  check(!gateway.requestNavigation("start", 0, 0, error)); check(error == "wheel_mapping_mismatch");
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.wrong_mapping = false; }
  // Wheel topology comes from the currently verified phone/FPGA layout. A
  // rear-wheel remap is not rejected merely because it differs from old mask10.
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.left_mask=12; }
  check(gateway.requestNavigation("start",0,0,error));
  check(command_layout==12 && synchronized_layout==12);
  bool visited_current_layout=false;
  gateway.visitNavigation([&](bool active,std::uint64_t){visited_current_layout=active&&synchronized_layout==12;});
  check(visited_current_layout);
  generation=gateway.generation();check(gateway.applyNav(.20,.10,generation));
  check(gateway.telemetry().left_mask==12);
  check(command("stop").code==200);
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.left_mask=10; }
  check(command("arm").code==200);

  { std::lock_guard<std::mutex> lock(backend.mutex); backend.imu_pending = true; }
  check(!gateway.requestNavigation("start", 0, 0, error)); check(error == "imu_not_ready");
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.imu_pending = false; }
  check(command("idle").code == 200); check(gateway.requestNavigation("start", 0, 0, error));
  generation = gateway.generation(); backend.delay_drive = true;
  std::thread pending([&] { gateway.applyNav(.2, .2, generation); });
  for (int i = 0; i < 100 && !backend.entered_drive; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  check(backend.entered_drive);
  check(command("stop").code == 200); pending.join(); backend.delay_drive = false;
  check(!backend.armed && backend.operations.back() == "stop");
  check(!gateway.applyNav(.2, .2, generation));
  check(command("arm").code == 200); check(gateway.requestNavigation("start", 0, 0, error));
  generation = gateway.generation();
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.fail_drive = true; }
  check(!gateway.applyNav(.2, .2, generation)); check(!gateway.navigationActive() && !backend.armed);
  { std::lock_guard<std::mutex> lock(backend.mutex); backend.fail_drive = false; }
  check(command("arm").code == 200);
  check(request(gateway_port,"POST","/api/navigation","{\"session\":\""+session+"\",\"op\":\"start\"}").code==400);
  check(command("drive",",\"throttle\":0,\"steer\":0").code==200);
  auto start_request=[&](unsigned navseq,std::uint64_t intent,const std::string& op="start"){
    return request(gateway_port,"POST","/api/navigation","{\"session\":\""+session+"\",\"op\":\""+op+
      "\",\"nav_seq\":"+std::to_string(navseq)+",\"intent_seq\":"+std::to_string(intent)+"}");
  };
  auto navigation = start_request(1,sequence);
  check(navigation.code == 200 && flag(navigation.value.get("navigation"), "active"));
  check(flag(navigation.value.get("navigation"),"supports_start_stop"));
  check(number(navigation.value.get("navigation"),"start_protocol_version")==2);
  check(start_request(1,sequence).code==409); // duplicate action cannot restart
  check(start_request(2,sequence).code==409); // accepted zero intent is one-use
  check(request(gateway_port, "GET", "/api/video.mjpg").code == 200);
  check(command("zero").code == 200); check(!gateway.navigationActive() && !backend.armed);
  unsigned cancellations_before_zero = cancellations;
  check(command("zero").code == 200); check(cancellations == cancellations_before_zero + 1);
  unsigned cancellations_before_configure = cancellations;
  check(command("configure", ",\"left_mask\":10,\"invert_mask\":15").code == 200);
  check(cancellations == cancellations_before_configure);
  check(command("configure", ",\"left_mask\":5,\"invert_mask\":15").code == 200);
  check(cancellations == cancellations_before_configure + 1);
  check(command("configure", ",\"left_mask\":10,\"invert_mask\":15").code == 200);
  check(gateway.requestNavigation("reset-map", 0, 0, error));
  auto go=[&](unsigned seq,std::uint64_t intent,const std::string& map_session="boot-0",unsigned revision=42){
    return request(gateway_port,"POST","/api/navigation","{\"session\":\""+session+
        "\",\"op\":\"goto\",\"x\":1.05,\"y\":0.05,\"map_session\":\""+map_session+
        "\",\"map_revision\":"+std::to_string(revision)+",\"nav_seq\":"+std::to_string(seq+100)+
        ",\"intent_seq\":"+std::to_string(intent)+"}");
  };
  check(command("arm").code==200);
  check(command("drive",",\"throttle\":0,\"steer\":0").code==200);auto intent=sequence;
  check(command("idle").code==200);check(go(1,intent).code==200&&gateway.navigationActive()&&goto_calls==1);
  check(go(1,intent).code==409&&goto_calls==1); // duplicate never restarts route
  check(command("drive",",\"throttle\":0.2,\"steer\":0").code==200);
  check(go(2,intent).code==409&&!gateway.navigationActive()&&goto_calls==1); // overtaken by manual
  check(backend.throttle==.2&&backend.armed);
  intent=sequence;check(go(3,intent+100).code==409); // future intent cannot pre-authorize commands
  check(go(4,intent,"old-boot").code==409&&!gateway.navigationActive()&&!backend.armed);
  check(command("arm").code==200);intent=sequence;
  check(command("stop").code==200);check(go(5,intent).code==409&&!backend.armed);
  check(command("arm").code==200);intent=sequence;check(go(6,intent).code==200);
  generation=gateway.generation();check(go(7,intent).code==200);
  check(!gateway.applyNav(.2,.2,generation)); // new tap invalidates older route output
  gateway.cancelNavigation("old_failed_plan",generation);check(gateway.navigationActive());
  bool visited=false;gateway.visitNavigation([&](bool a,std::uint64_t g){visited=a&&g!=generation;});check(visited);
  std::atomic<bool> entered{false},released{false};
  std::thread cycle([&]{gateway.visitNavigation([&](bool a,std::uint64_t){assert(a);entered=true;
    std::this_thread::sleep_for(std::chrono::milliseconds(40));released=true;});});
  while(!entered)std::this_thread::yield();
  check(command("stop").code==200);cycle.join();check(released&&!gateway.navigationActive());
  check(command("stop").code==200);
  // New phone Start/Explore uses the same intent barrier as map goto, plus a
  // confirmed zero DRIVE. Delayed requests never restart after manual takeover.
  check(command("arm").code==200);auto armed_intent=sequence;
  check(start_request(201,armed_intent).code==409); // arm is not a zero-drive handshake
  check(command("drive",",\"throttle\":0,\"steer\":0").code==200);auto zero_intent=sequence;
  check(command("idle").code==200);
  check(start_request(202,zero_intent).code==200&&gateway.navigationActive());
  check(command("stop").code==200);check(!gateway.navigationActive()&&!backend.armed);
  check(command("arm").code==200);
  check(start_request(203,zero_intent).code==409&&!gateway.navigationActive());
  check(command("drive",",\"throttle\":0,\"steer\":0").code==200);zero_intent=sequence;
  check(command("drive",",\"throttle\":0.1,\"steer\":0").code==200);
  check(start_request(204,zero_intent,"explore").code==409&&!gateway.navigationActive());
  check(command("drive",",\"throttle\":0,\"steer\":0").code==200);zero_intent=sequence;
  check(start_request(205,zero_intent+1).code==409); // future cannot preauthorize
  check(start_request(206,zero_intent,"explore").code==200&&gateway.navigationActive());
  auto stop_api=request(gateway_port,"POST","/api/navigation","{\"session\":\""+session+"\",\"op\":\"stop\"}");
  check(stop_api.code==200&&!gateway.navigationActive()&&!backend.armed);
  check(command("arm").code==200);check(start_request(207,zero_intent).code==409);
  check(command("drive",",\"throttle\":0,\"steer\":0").code==200);zero_intent=sequence;
  std::this_thread::sleep_for(std::chrono::milliseconds(370));
  check(start_request(208,zero_intent).code==409); // watching status cannot renew owner lease
  check(command("idle").code==200);
  check(start_request(209,zero_intent).code==409); // a renewed idle lease cannot revive an expired zero intent
  check(command("drive",",\"throttle\":0,\"steer\":0").code==200);
  check(start_request(210,sequence).code==200);
  check(command("stop").code==200);check(command("arm").code==200);check(command("idle").code==200);
  check(gateway.requestNavigation("start", 0, 0, error)); // local console compatibility
  check(command("stop").code==200);
  backend.velocity_supported=false;backend.ack_protocol=1;check(command("arm").code==200);
  check(!gateway.requestNavigation("start",0,0,error)&&error=="fpga_v28_velocity_required");
  check(!gateway.navigationActive()); // Explicit old-FPGA rejection, no duty fallback.
  backend.velocity_supported=true;backend.ack_protocol=4;backend.softcore_ready=false;
  check(command("stop").code==200);check(command("arm").code==200);
  check(gateway.requestNavigation("start",0,0,error));
  check(gateway.navigationPreparing()&&!gateway.navigationActive()&&backend.armed);
  backend.softcore_ready=true;gateway.pollState();
  check(gateway.navigationActive()&&!gateway.navigationPreparing());
  check(command("stop").code==200);check(command("arm").code==200);
  check(gateway.requestNavigation("start",0,0,error));generation=gateway.generation();
  check(!gateway.applyNav(.301,0,generation)&&!gateway.navigationActive()&&!backend.armed);
  gateway.stop(); check(!backend.armed); check(cancellations >= 7 && accepted >= 8);
  backend.server.stop(); unlink(token.c_str());
  std::printf("PASS %u gateway assertions: shared owner/sequence, actual-duty mapping, phone takeover, stale generation, 350ms foreground lease, calibrated IMU gating, serialized stop, backend rejection, MJPEG passthrough and shutdown\n", checks);
}
