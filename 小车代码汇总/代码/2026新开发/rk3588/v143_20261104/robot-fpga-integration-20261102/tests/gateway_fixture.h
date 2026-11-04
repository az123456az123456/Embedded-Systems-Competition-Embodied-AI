// Synthetic loopback backend shared with the Gateway regression fixture.
#pragma once
#include "../rk_gateway/navigation/gateway.h"
#include "../rk_gateway/navigation/vendor/http_server.h"
#include "../rk_gateway/navigation/vendor/picojson.h"
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
  bool fpga_quiet_fixture=false;std::vector<std::string> fpga_operations;
  unsigned connections = 0; bool armed = false, imu_fresh = true, calibrating = false, fail_drive = false;
  unsigned left_mask = 10, invert_mask = 15;
  bool missing_yaw = false, wrong_mapping = false;std::atomic<bool> reject_disarm{false};
  bool velocity_supported=true,softcore_ready=true;unsigned ack_protocol=4;
  bool imu_pending = false;
  double throttle = 0, steer = 0; std::uint64_t last_sequence = 0;
  double rotation_speed=0;bool rotation_clockwise=false;unsigned rotations=0,drives=0;
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
      if(fpga_quiet_fixture&&request.path.compare(0,21,"/api/navigation/fpga/")==0){
        fpga_operations.push_back(request.path);answer="{\"ok\":true,\"generation\":\"1\",\"quiet\":true,\"enabled\":false}";
      }
      else if (request.path == "/api/state") answer = state();
      else if (request.path == "/api/connect") {
        if (armed) { code = 409; answer = state(false); }
        else { session = "session-" + std::to_string(++connections); last_sequence = 0; answer = state(true, true); }
      } else if (request.path == "/api/control" || request.path == "/api/navigation/claim" || request.path == "/api/navigation/twist" || request.path == "/api/navigation/lease" || request.path == "/api/navigation/pause") {
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
          else if (request.path == "/api/navigation/lease") { assert(op=="lease"); }
          else if (op == "idle" || op=="pause") throttle = steer = 0;
          else if (op == "drive" || op=="twist") {
            if (delay_drive) { entered_drive = true; std::this_thread::sleep_for(std::chrono::milliseconds(80)); }
            if (fail_drive || !armed) code = 409;
            else if(op=="twist") { assert(request.path=="/api/navigation/twist"&&text(v,"navigation_ticket")=="fixture-private-ticket");throttle=(number(v,"linear_mps")+number(v,"angular_rps"))/1.2;steer=(number(v,"linear_mps")-number(v,"angular_rps"))/1.2; }
            else { throttle = number(v, "throttle"); steer = number(v, "steer");++drives; }
          }
          else if(op=="rotate") {
            if(!armed)code=409;
            else{rotation_speed=number(v,"speed");rotation_clockwise=text(v,"direction")=="cw";++rotations;throttle=0;steer=rotation_clockwise?rotation_speed:-rotation_speed;}
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
