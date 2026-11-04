#include "gateway.h"
#include "vendor/http_server.h"
#include "vendor/json_flat.h"
#include "vendor/picojson.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robotnav {
namespace {
using robotphone::HttpRequest;
using robotphone::JsonObject;
using robotphone::jsonBool;
using robotphone::jsonNumber;
using robotphone::jsonString;
using robotphone::quoteJson;
using Clock = std::chrono::steady_clock;
std::uint64_t nowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}
bool validWheelLayout(unsigned left, unsigned invert) {
  if(left==0 || left>=15 || invert>15)return false;
  unsigned bits=0;for(unsigned value=left;value;value>>=1)bits+=value&1U;
  return bits==2;
}
bool equalSecret(const std::string &a, const std::string &b) {
  unsigned difference = unsigned(a.size() ^ b.size());
  for (size_t i = 0; i < std::max(a.size(), b.size()); ++i)
    difference |= unsigned(i < a.size() ? static_cast<unsigned char>(a[i]) : 0) ^
                  unsigned(i < b.size() ? static_cast<unsigned char>(b[i]) : 0);
  return difference == 0;
}
bool parseObject(const std::string &bytes, picojson::value &value) {
  auto at = bytes.begin();
  std::string error = picojson::parse(value, at, bytes.end());
  while (at != bytes.end() && std::isspace(static_cast<unsigned char>(*at))) ++at;
  return error.empty() && at == bytes.end() && value.is<picojson::object>();
}
const picojson::value &member(const picojson::value &v, const char *name) {
  static const picojson::value empty;
  if (!v.is<picojson::object>()) return empty;
  const auto &object = v.get<picojson::object>();
  const auto found = object.find(name);
  return found == object.end() ? empty : found->second;
}
bool boolean(const picojson::value &v, const char *key, bool fallback = false) {
  const auto &x = member(v, key); return x.is<bool>() ? x.get<bool>() : fallback;
}
double number(const picojson::value &v, const char *key, double fallback = 0) {
  const auto &x = member(v, key); return x.is<double>() ? x.get<double>() : fallback;
}
std::string string(const picojson::value &v, const char *key) {
  const auto &x = member(v, key); return x.is<std::string>() ? x.get<std::string>() : "";
}
std::string flatJson(const JsonObject &object) {
  std::string result = "{"; bool first = true;
  for (const auto &entry : object) {
    if (!first) result += ',';
    first = false; result += quoteJson(entry.first) + ':';
    result += entry.second.type == robotphone::JsonValue::String ? quoteJson(entry.second.text) : entry.second.text;
  }
  return result + '}';
}
void setNumber(JsonObject &object, const char *key, double value) {
  std::ostringstream output; output.precision(17); output << value;
  object[key] = {robotphone::JsonValue::Number, output.str()};
}
void setString(JsonObject &object, const char *key, const std::string &value) {
  object[key] = {robotphone::JsonValue::String, value};
}
struct Socket {
  int fd = -1;
  explicit Socket(int value = -1) : fd(value) {}
  ~Socket() { if (fd >= 0) ::close(fd); }
  Socket(const Socket &) = delete;
};
int connectLocal(unsigned port, unsigned timeout_ms) {
  Socket result(socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
  if (result.fd < 0) return -1;
  sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (connect(result.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    if (errno != EINPROGRESS) return -1;
    pollfd event{result.fd, POLLOUT, 0};
    if (poll(&event, 1, int(timeout_ms)) <= 0) return -1;
    int status = 0; socklen_t size = sizeof(status);
    if (getsockopt(result.fd, SOL_SOCKET, SO_ERROR, &status, &size) || status) return -1;
  }
  int flags = fcntl(result.fd, F_GETFL, 0);
  if (flags < 0 || fcntl(result.fd, F_SETFL, flags & ~O_NONBLOCK)) return -1;
  timeval timeout{int(timeout_ms / 1000), int((timeout_ms % 1000) * 1000)};
  setsockopt(result.fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  setsockopt(result.fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  int fd = result.fd; result.fd = -1; return fd;
}
std::string httpHead(const std::string &method, const std::string &path,
                     const std::string &body, const std::string &token, unsigned port) {
  return method + " " + path + " HTTP/1.1\r\nHost: 127.0.0.1:" + std::to_string(port) +
      "\r\nAuthorization: Bearer " + token + "\r\nContent-Type: application/json\r\nContent-Length: " +
      std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
}
struct Reply { int status = 503; std::string body = "{\"ok\":false,\"error\":\"dashboard_unavailable\"}"; bool transport_ok = false; std::uint64_t request_started_ms=0; };
Reply backendRequest(unsigned port, unsigned timeout_ms, const std::string &token,
                     const std::string &method, const std::string &path, const std::string &body) {
  Reply result; result.request_started_ms=nowMs(); Socket socket(connectLocal(port, timeout_ms));
  if (socket.fd < 0) return result;
  const auto request = httpHead(method, path, body, token, port);
  if (!robotphone::writeAll(socket.fd, request.data(), request.size())) return result;
  std::string bytes; size_t header_end = std::string::npos, expected = 0;
  const auto deadline = nowMs() + timeout_ms;
  while (bytes.size() <= 262144 && nowMs() <= deadline) {
    char block[4096]; ssize_t count = recv(socket.fd, block, sizeof(block), 0);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return result;
    bytes.append(block, size_t(count));
    if (header_end == std::string::npos) {
      header_end = bytes.find("\r\n\r\n");
      if (header_end == std::string::npos) continue;
      std::istringstream header(bytes.substr(0, header_end));
      std::string version; int status = 0; header >> version >> status;
      if (version != "HTTP/1.1" && version != "HTTP/1.0") return result;
      if (status < 100 || status > 599) return result;
      result.status = status;
      std::string line; bool have_length = false;
      while (std::getline(header, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto colon = line.find(':'); if (colon == std::string::npos) continue;
        std::string name = line.substr(0, colon);
        for (char &c : name) c = char(std::tolower(static_cast<unsigned char>(c)));
        if (name == "transfer-encoding") return Reply{};
        if (name == "content-length") {
          if (have_length) return Reply{};
          std::istringstream length(line.substr(colon + 1)); unsigned long long size = 0;
          if (!(length >> size) || size > 262144) return Reply{};
          std::string extra; if (length >> extra) return Reply{};
          expected = size_t(size); have_length = true;
        }
      }
      if (!have_length) return Reply{};
      header_end += 4;
    }
    if (header_end != std::string::npos && bytes.size() >= header_end + expected) {
      if (bytes.size() != header_end + expected) return Reply{};
      result.body = bytes.substr(header_end, expected); result.transport_ok = true; return result;
    }
  }
  return Reply{};
}
Reply errorReply(int status, const std::string &error) {
  Reply result; result.status = status;
  result.body = "{\"ok\":false,\"error\":" + quoteJson(error) + '}'; return result;
}
}  // namespace

struct Gateway::Impl {
  GatewayOptions options; GatewayCallbacks callbacks; std::string token;
  robotphone::HttpServer server;
  mutable std::mutex mutex;
  // State is never held across socket IO or while waiting for another IO lane.
  std::mutex control_serial, telemetry_serial;
  unsigned commands_inflight = 0, pending_stops = 0;
  std::uint64_t control_epoch = 1;
  std::mutex auth_mutex;
  std::uint64_t auth_window_ms = 0, auth_block_until_ms = 0;
  unsigned auth_failures = 0;
  std::atomic<bool> stopped{true}; std::thread watchdog;
  bool active = false;
  std::uint64_t generation_id = 1, backend_sequence = 0, mobile_sequence = 0;
  std::uint64_t navigation_sequence = 0, cancellation_sequence = 0;
  std::uint64_t last_owner_ms = 0;
  std::string session, cancel_reason = "not_started";
  std::uint64_t prepared_zero_sequence = 0, prepared_zero_ms=0;
  GatewayTelemetry state;
  picojson::value last_json;
  std::string last_event;
  void event(const std::string& reason,const std::string& op="",int http_status=0) {
    // Deliberately excludes session, pairing token, request bodies and targets.
    auto safe=[](const std::string& value) {
      std::string result;
      for(unsigned char c:value) {
        if(result.size()==64)break;
        result+=std::isalnum(c)||c=='_'||c=='-'?char(c):'_';
      }
      return result.empty()?std::string("none"):result;
    };
    const auto key=safe(reason)+"/"+safe(op)+"/"+std::to_string(http_status);
    if(key==last_event)return;
    last_event=key;
    const auto now=nowMs();
    const long long age=last_owner_ms&&now>=last_owner_ms?static_cast<long long>(now-last_owner_ms):-1;
    std::fprintf(stderr,"GATEWAY_CTRL reason=%s op=%s http_status=%d owner_age_ms=%lld generation=%llu\n",
      safe(reason).c_str(),safe(op).c_str(),http_status,age,static_cast<unsigned long long>(generation_id));
  }
  Impl(GatewayOptions o, GatewayCallbacks c) : options(std::move(o)), callbacks(std::move(c)) {}

  void invalidate(const std::string &reason, bool notify_when_inactive = false) {
    const bool had_navigation = active;
    if((had_navigation && reason.compare(0,6,"phone_")==0 && reason!="phone_command_failed" && reason!="phone_lease_expired") || reason=="phone_stop" || reason=="phone_disarm")event(reason,reason.substr(6));
    active = false; prepared_zero_sequence=prepared_zero_ms=0; navigation_ticket.clear(); ++generation_id; cancel_reason = reason;
    if ((had_navigation || notify_when_inactive) && callbacks.navigation_cancel) callbacks.navigation_cancel(reason);
  }
  void update(const Reply &reply) {
    picojson::value parsed;
    if (!reply.transport_ok || !parseObject(reply.body, parsed)) {
      state.available = false; state.reason = "dashboard_unavailable"; return;
    }
    last_json = parsed;
    state.received_ms = nowMs(); state.request_started_ms=reply.request_started_ms;
    state.available = boolean(parsed, "ok");
    state.armed = boolean(parsed, "armed"); state.reason = string(parsed, "reason");
    state.imu_action_pending = boolean(parsed, "imu_action_pending", true);
    const auto &motor = member(parsed, "motor"), &imu = member(parsed, "imu");
    state.link_ok = boolean(motor, "link_ok");
    const double left_mask = number(motor, "left_mask", -1), invert_mask = number(motor, "invert_mask", -1);
    state.left_mask = left_mask >= 0 && left_mask <= 15 && std::floor(left_mask) == left_mask ? unsigned(left_mask) : 16;
    state.invert_mask = invert_mask >= 0 && invert_mask <= 15 && std::floor(invert_mask) == invert_mask ? unsigned(invert_mask) : 16;
    state.left_duty = number(motor, "left"); state.right_duty = number(motor, "right");
    state.ack_left_duty = number(motor, "ack_left"); state.ack_right_duty = number(motor, "ack_right");
    state.motor_ack_age_ms = number(motor, "ack_age_ms", -1);
    state.imu_valid = boolean(imu, "valid"); state.imu_fresh = boolean(imu, "fresh");
    state.imu_bias_valid = boolean(imu, "bias_valid"); state.imu_calibrating = boolean(imu, "calibrating", true);
    state.imu_sample_age_ms = number(imu, "sample_age_ms", -1);
    state.yaw_deg = number(imu, "yaw"); state.roll_deg = number(imu, "roll"); state.pitch_deg = number(imu, "pitch");
    double sequence = number(imu, "sequence");
    state.imu_sequence = sequence >= 0 && sequence <= 4294967295.0 ? std::uint64_t(sequence) : 0;
    const double source_time=number(imu,"timestamp_ms",-1);
    state.imu_timestamp_valid=member(imu,"timestamp_ms").is<double>() && std::isfinite(source_time) &&
      source_time>=0 && source_time<=4294967295.0 && std::floor(source_time)==source_time;
    state.imu_timestamp_ms=state.imu_timestamp_valid?std::uint32_t(source_time):0;
    if (!member(imu, "yaw").is<double>() || !member(imu, "roll").is<double>() || !member(imu, "pitch").is<double>() ||
        !member(imu, "sequence").is<double>() || sequence < 0 || sequence > 4294967295.0 || std::floor(sequence) != sequence ||
        !std::isfinite(state.yaw_deg) || !std::isfinite(state.roll_deg) || !std::isfinite(state.pitch_deg)) state.imu_valid = false;
    if (callbacks.navigation_layout) callbacks.navigation_layout(state);
  }
  std::string navigation_ticket;
  // The caller holds control_serial and the short-lived state lock. A STOP may
  // revoke the generation while this socket is pending; backend sequence order
  // then ensures no earlier packet can resurrect motion after the STOP.
  Reply command(JsonObject input, std::unique_lock<std::mutex>& lock,
                const std::string &path="/api/control") {
    if (session.empty()) return errorReply(409, "no_phone_owner");
    setString(input, "session", session); setNumber(input, "seq", double(++backend_sequence));
    ++commands_inflight;
    lock.unlock();
    Reply result = backendRequest(options.backend_port, options.backend_timeout_ms, token, "POST", path, flatJson(input));
    lock.lock();
    --commands_inflight;
    update(result); return result;
  }
  void refresh(std::unique_lock<std::mutex>& lock) {
    const auto sequence = backend_sequence;
    const auto epoch = control_epoch;
    const bool busy = commands_inflight != 0;
    lock.unlock();
    Reply reply = backendRequest(options.backend_port, options.backend_timeout_ms, token, "GET", "/api/state", "");
    lock.lock();
    // A GET that overlaps any control mutation is not a safe snapshot of the
    // acknowledged arm/layout state. The command's actual reply owns that state.
    if (!busy && !commands_inflight && sequence == backend_sequence && epoch == control_epoch &&
        reply.request_started_ms>=state.request_started_ms) {
      if(!reply.transport_ok && state.available)event("dashboard_unavailable","telemetry",reply.status);
      update(reply);
    }
  }
  Reply cached() const {
    if (!state.available || !last_json.is<picojson::object>()) return errorReply(503, "dashboard_unavailable");
    picojson::value snapshot=last_json;
    auto& root=snapshot.get<picojson::object>();
    const auto now=nowMs();
    // The backend may have taken 120 ms to answer. Use the request's start as
    // a conservative source bound; a slow read must not give an old cache a
    // new lifetime just because its reply has only just arrived.
    const auto source=state.request_started_ms?std::min(state.request_started_ms,state.received_ms):state.received_ms;
    const auto age=now>=source?now-source:0;
    root["gateway_state_age_ms"]=picojson::value(double(age));
    root["gateway_control_pending"]=picojson::value(commands_inflight!=0 || pending_stops!=0);
    // Do not turn cached data into a fresh sample just because HTTP GET was
    // answered now. Older clients also see increasing sensor/ACK ages.
    for(const auto& field:{std::pair<const char*,const char*>{"imu","sample_age_ms"},
                          std::pair<const char*,const char*>{"motor","ack_age_ms"}}) {
      auto& section=root[field.first];
      if(!section.is<picojson::object>())continue;
      auto& value=section.get<picojson::object>()[field.second];
      if(value.is<double>() && std::isfinite(value.get<double>()) && value.get<double>()>=0)
        value=picojson::value(value.get<double>()+double(age));
    }
    Reply r; r.status=200; r.transport_ok=true; r.body=snapshot.serialize(); return r;
  }
  void disarm(std::unique_lock<std::mutex>& lock) {
    if (session.empty()) return;
    JsonObject input; setString(input, "op", "disarm"); const auto reply=command(input,lock);
    if(!reply.transport_ok || reply.status!=200 || !state.available || state.armed)
      event("disarm_not_confirmed","disarm",reply.status);
  }
  void fail(const std::string &reason, std::unique_lock<std::mutex>& lock,
            const std::string& op="", int http_status=0) {
    event(reason,op,http_status);
    ++control_epoch; invalidate(reason); disarm(lock);
  }
  bool ownerFresh() const { return last_owner_ms && nowMs() >= last_owner_ms && nowMs() - last_owner_ms <= options.owner_lease_ms; }
  bool motionReady(std::string &error) const {
    if (session.empty() || !ownerFresh()) error = "phone_foreground_lease_required";
    else if (!state.available || nowMs() < state.received_ms || nowMs() - state.received_ms > 300) error = "dashboard_state_stale";
    else if (!state.armed) error = "arm_from_phone_first";
    else if (!state.link_ok || !std::isfinite(state.motor_ack_age_ms) || state.motor_ack_age_ms < 0 || state.motor_ack_age_ms > 300) error = "motor_link_not_ready";
    else if (!validWheelLayout(state.left_mask,state.invert_mask)) error = "wheel_mapping_mismatch";
    else if (!state.imu_valid || !state.imu_fresh || state.imu_calibrating || !state.imu_bias_valid || state.imu_action_pending ||
             !std::isfinite(state.imu_sample_age_ms) || state.imu_sample_age_ms < 0 || state.imu_sample_age_ms > 300) error = "imu_not_ready";
    else return true;
    return false;
  }
  bool begin(std::string &error, std::unique_lock<std::mutex>& lock, bool refresh_backend=true) {
    const auto expected = generation_id;
    if(refresh_backend)refresh(lock);
    if (stopped || generation_id != expected) { error="navigation_cancelled"; return false; }
    if (!motionReady(error)) {
      if (active) fail(error,lock);
      return false;
    }
    // First command zeros any previous manual intent. Neither this nor start arms.
    JsonObject idle; setString(idle, "op", "drive"); setNumber(idle, "throttle", 0); setNumber(idle, "steer", 0);
    auto zero = command(idle,lock);
    if (stopped || generation_id != expected) { error="navigation_cancelled"; return false; }
    if (!zero.transport_ok || zero.status != 200 || !state.available || !state.armed) { error = "navigation_zero_rejected"; fail(error,lock); return false; }
    JsonObject claim;setString(claim,"op","claim");
    auto capability=command(claim,lock,"/api/navigation/claim");picojson::value claimed;
    if (stopped || generation_id != expected) { error="navigation_cancelled"; return false; }
    if(!capability.transport_ok||capability.status!=200||!parseObject(capability.body,claimed)||string(claimed,"navigation_ticket").empty()){
      error="navigation_capability_rejected";fail(error,lock);return false;
    }
    if (!motionReady(error)) { fail(error,lock); return false; }
    navigation_ticket=string(claimed,"navigation_ticket");
    // The ticket is only for this loopback gateway; never return it to a phone.
    if(last_json.is<picojson::object>())last_json.get<picojson::object>().erase("navigation_ticket");
    ++generation_id; active = true; cancel_reason.clear(); return true;
  }
  bool navigation(const std::string &op, double x, double y, std::string &error, std::unique_lock<std::mutex>& lock) {
    if (op == "stop") { fail("navigation_stop",lock); return true; }
    if (op == "reset-map") {
      fail("navigation_reset",lock);
      if (!callbacks.navigation_command) { error = "navigation_not_attached"; return false; }
      return callbacks.navigation_command(op, 0, 0, error);
    }
    if (op != "goal" && op != "start" && op != "explore") { error = "unknown_navigation_operation"; return false; }
    if (!std::isfinite(x) || !std::isfinite(y)) { error = "invalid_goal"; return false; }
    if (!ownerFresh()) { error = "phone_foreground_lease_required"; return false; }
    if (!callbacks.navigation_command) { error = "navigation_not_attached"; return false; }
    // Configuration and reset/start can arrive between the main loop's polls.
    // Refresh and synchronize the actual ACK layout before touching the core.
    const auto expected=generation_id;
    refresh(lock);
    if (stopped || generation_id!=expected) { error="navigation_cancelled"; return false; }
    if (op == "goal" && active) fail("goal_changed",lock);
    const bool starting = op == "start" || op == "explore";
    // Complete every socket wait before enabling the navigation core. begin()
    // drops the control mutex for zero DRIVE and capability IO, so enabling the
    // core first exposes (inactive gateway, enabled core) to visitNavigation().
    // The runtime correctly treats that pair as a manual cancellation. Commit
    // the gateway and core together instead: begin's final activation and the
    // callback below both execute under this same mutex, without intervening IO.
    if (starting && !begin(error,lock,false)) return false;
    if (!callbacks.navigation_command(op, x, y, error)) {
      if (error.empty()) error = "navigation_rejected";
      // A sensor may become stale while zero/claim is pending. The core's final
      // readiness decision wins; never leave a capability or armed navigation
      // session behind after its rejection.
      if (starting) fail(error,lock);
      return false;
    }
    return true;
  }
  std::string decorate(const std::string &body) {
    picojson::value root; if (!parseObject(body, root)) return body;
    picojson::object nav;
    if (callbacks.navigation_state) {
      picojson::value extra; if (parseObject(callbacks.navigation_state(), extra)) nav = extra.get<picojson::object>();
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      nav["active"] = picojson::value(active); nav["gateway"] = picojson::value(true);
      nav["phone_lease_ms"] = picojson::value(double(options.owner_lease_ms));
      nav["phone_fresh"] = picojson::value(ownerFresh());
      nav["cancel_reason"] = picojson::value(cancel_reason);
      nav["generation"] = picojson::value(double(generation_id));
      const bool core_ready=boolean(picojson::value(nav),"can_start");
      std::string start_error;
      const bool backend_ready=motionReady(start_error);
      nav["can_start"]=picojson::value(core_ready&&backend_ready);
      if(core_ready&&!backend_ready)nav["readiness_reason"]=picojson::value(start_error);
      nav["supports_start_stop"]=picojson::value(true);
      nav["start_protocol_version"]=picojson::value(2.);
      nav["requires_zero_intent"]=picojson::value(true);
    }
    root.get<picojson::object>()["navigation"] = picojson::value(nav); return root.serialize();
  }
  void stream(int client, const HttpRequest &request) {
    Socket upstream(connectLocal(options.backend_port, options.backend_timeout_ms));
    if (upstream.fd < 0) { robotphone::httpReply(client, 503, errorReply(503, "dashboard_unavailable").body); return; }
    auto bytes = httpHead("GET", request.path, "", token, options.backend_port);
    if (!robotphone::writeAll(upstream.fd, bytes.data(), bytes.size())) return;
    while (!stopped) {
      pollfd events[2] = {{upstream.fd, POLLIN, 0}, {client, POLLIN, 0}};
      int ready = poll(events, 2, 100);
      if (ready < 0 && errno == EINTR) continue;
      if (ready < 0) return;
      if (events[1].revents & (POLLHUP | POLLERR | POLLNVAL)) return;
      if (events[1].revents & POLLIN) { char c; if (recv(client, &c, 1, MSG_PEEK | MSG_DONTWAIT) <= 0) return; }
      if (!(events[0].revents & (POLLIN | POLLHUP))) continue;
      char block[32768]; ssize_t n = recv(upstream.fd, block, sizeof(block), 0);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0 || !robotphone::writeAll(client, block, size_t(n))) return;
    }
  }
  Reply handleControl(const HttpRequest &request) {
    JsonObject input; std::string error, claimed, op; double number_seq = 0;
    if (!robotphone::parseFlatJson(request.body, input, error) || !jsonString(input, "session", claimed) ||
        !jsonString(input, "op", op) || !jsonNumber(input, "seq", number_seq) || number_seq < 1 ||
        number_seq > 9007199254740991.0 || std::floor(number_seq) != number_seq) return errorReply(400, "invalid_control_request");
    const bool stop_op = op == "stop" || op == "disarm";
    std::uint64_t epoch;
    {
      std::lock_guard<std::mutex> admission(mutex);
      if (stopped) return errorReply(503, "gateway_stopped");
      if (session.empty() || !equalSecret(session, claimed)) return errorReply(403, "not_control_owner");
      if (std::uint64_t(number_seq) <= mobile_sequence && !stop_op) return errorReply(409, "stale_sequence");
      mobile_sequence = std::max(mobile_sequence, std::uint64_t(number_seq));
      // Record the authenticated foreground arrival, never the end of a queued
      // telemetry read / control request. Reading state/maps does not renew it.
      last_owner_ms = nowMs();
      if (active && op == "idle") return cached();
      if (stop_op) { ++control_epoch; ++pending_stops; }
      if (op != "idle") {
        cancellation_sequence=mobile_sequence;
        invalidate(op == "configure" ? "phone_configuration_check" : "phone_" + op,
                   op == "zero" || op == "calibrate");
      }
      epoch=control_epoch;
    }
    std::lock_guard<std::mutex> serial(control_serial);
    std::unique_lock<std::mutex> lock(mutex);
    if (stopped) { if(stop_op)--pending_stops; return errorReply(503,"gateway_stopped"); }
    if (!equalSecret(session,claimed)) { if(stop_op)--pending_stops; return errorReply(403,"not_control_owner"); }
    if (!stop_op && pending_stops) return errorReply(409,"stop_pending");
    if (op=="idle" && active) return cached();
    if (!stop_op && (epoch!=control_epoch || std::uint64_t(number_seq)<cancellation_sequence))
      return errorReply(409,"control_superseded");
    // A command that spent its entire foreground lease queued cannot move a car
    // after the sender has gone away, even if it was originally well formed.
    if (!stop_op && !ownerFresh()) return errorReply(409,"phone_foreground_lease_required");
    const unsigned prior_left_mask=state.left_mask,prior_invert_mask=state.invert_mask;
    auto result=command(input,lock);
    if(op=="arm" && result.transport_ok && result.status==200 && state.available && state.armed)
      event("phone_arm","arm",200);
    if (op=="configure" && result.transport_ok && result.status==200 && state.available &&
        (state.left_mask!=prior_left_mask || state.invert_mask!=prior_invert_mask) && callbacks.navigation_cancel)
      callbacks.navigation_cancel("phone_configure");
    if (!result.transport_ok || result.status!=200 || !state.available) fail("phone_command_failed",lock,op,result.status);
    else if (op=="drive" && epoch==control_epoch && std::uint64_t(number_seq)>=cancellation_sequence) {
      double throttle=0,steer=0;
      if(jsonNumber(input,"throttle",throttle)&&jsonNumber(input,"steer",steer)&&throttle==0&&steer==0) {
        prepared_zero_sequence=std::uint64_t(number_seq); prepared_zero_ms=nowMs();
      }
    }
    if(stop_op)--pending_stops;
    return result;
  }
  Reply handleNavigation(const HttpRequest &request) {
    JsonObject input; std::string error, claimed, op;
    if (!robotphone::parseFlatJson(request.body, input, error) || !jsonString(input, "session", claimed) || !jsonString(input, "op", op))
      return errorReply(400, "invalid_navigation_request");
    double x = 0, y = 0;
    if ((op == "goal" || op == "goto") && (!jsonNumber(input, "x", x) || !jsonNumber(input, "y", y) || !std::isfinite(x) || !std::isfinite(y)))
      return errorReply(400, "invalid_goal");
    std::uint64_t epoch;
    {
      std::lock_guard<std::mutex> admission(mutex);
      if(stopped)return errorReply(503,"gateway_stopped");
      if(session.empty()||!equalSecret(session,claimed))return errorReply(403,"not_control_owner");
      if(op=="stop"||op=="reset-map") { ++control_epoch; ++pending_stops; invalidate("navigation_"+op); }
      epoch=control_epoch;
    }
    std::lock_guard<std::mutex> serial(control_serial);
    std::unique_lock<std::mutex> lock(mutex);
    const bool stop_op=op=="stop"||op=="reset-map";
    if(stopped) { if(stop_op)--pending_stops; return errorReply(503,"gateway_stopped"); }
    if(!equalSecret(session,claimed)) { if(stop_op)--pending_stops; return errorReply(403,"not_control_owner"); }
    if(!stop_op && pending_stops)return errorReply(409,"stop_pending");
    if(op!="stop" && op!="reset-map" && epoch!=control_epoch)return errorReply(409,"navigation_cancelled");
    if(op=="goto"||op=="start"||op=="explore") {
      double nav_seq=0,intent_seq=0;
      if(!jsonNumber(input,"nav_seq",nav_seq)||!jsonNumber(input,"intent_seq",intent_seq)||
         !std::isfinite(nav_seq)||nav_seq<1||nav_seq>9007199254740991.0||std::floor(nav_seq)!=nav_seq||
         !std::isfinite(intent_seq)||intent_seq<1||intent_seq>9007199254740991.0||std::floor(intent_seq)!=intent_seq)
        return errorReply(400,"invalid_navigation_request");
      if(std::uint64_t(nav_seq)<=navigation_sequence)return errorReply(409,"stale_navigation_sequence");
      navigation_sequence=std::uint64_t(nav_seq);
      if(std::uint64_t(intent_seq)<cancellation_sequence||std::uint64_t(intent_seq)>mobile_sequence)
        return errorReply(409,"stale_navigation_intent");
      if(op!="goto"&&(!prepared_zero_sequence||std::uint64_t(intent_seq)!=prepared_zero_sequence||
         nowMs()<prepared_zero_ms||nowMs()-prepared_zero_ms>options.owner_lease_ms))
        return errorReply(409,"navigation_zero_intent_required");
      if(op=="goto"){
        std::string map_session;double revision=0;
        if(!jsonString(input,"map_session",map_session)||map_session.empty()||map_session.size()>128||
           !jsonNumber(input,"map_revision",revision)||!std::isfinite(revision)||revision<0||
           revision>4294967295.0||std::floor(revision)!=revision)
          return errorReply(400,"invalid_navigation_request");
        if(!callbacks.navigation_goto)return errorReply(409,"navigation_not_attached");
        // Preserve map-bound goto compatibility. A later manual command still
        // invalidates this intent and the generation of every old wheel output.
        if(!begin(error,lock))return errorReply(409,error);
        if(!callbacks.navigation_goto(x,y,map_session,std::uint32_t(revision),error)){
          if(error.empty())error="navigation_rejected";
          fail(error,lock);return errorReply(409,error);
        }
      }else{
        // HTTP start/explore requires a confirmed zero DRIVE from THIS owner,
        // with a one-use intent. Idle heartbeats may intervene; manual input,
        // STOP, mapping changes and session changes revoke it.
        if(!navigation(op,0,0,error,lock))return errorReply(409,error);
        prepared_zero_sequence=prepared_zero_ms=0;
      }
    } else if (!navigation(op, x, y, error,lock)) { if(stop_op)--pending_stops; return errorReply(409, error); }
    if(stop_op)--pending_stops;
    Reply r; r.status = 200; r.transport_ok = true; r.body = last_json.serialize(); return r;
  }
  void request(int client, const HttpRequest &request) {
    auto auth = request.headers.find("authorization");
    int gate = 0;
    {
      std::lock_guard<std::mutex> lock(auth_mutex); const auto now = nowMs();
      if (now < auth_block_until_ms) gate = 429;
      else if (auth == request.headers.end() || !equalSecret(auth->second, "Bearer " + token)) {
        if (now - auth_window_ms > 60000) { auth_window_ms = now; auth_failures = 0; }
        if (++auth_failures >= 10) { auth_block_until_ms = now + 30000; auth_failures = 0; }
        gate = 401;
      }
    }
    if (gate) {
      robotphone::httpReply(client, gate, errorReply(gate, "pairing_token_required").body); return;
    }
    if (request.method == "GET" && request.path == "/api/video.mjpg") { stream(client, request); return; }
    if (request.method == "GET" && request.path == "/api/navigation/map") {
      robotphone::httpReply(client, callbacks.navigation_map ? 200 : 503,
          callbacks.navigation_map ? callbacks.navigation_map() :
          "{\"schema\":1,\"available\":false,\"reason\":\"navigation_map_unavailable\"}");
      return;
    }
    Reply result;
    if (request.method == "POST" && request.path == "/api/control") result = handleControl(request);
    else if (request.method == "POST" && request.path == "/api/navigation") result = handleNavigation(request);
    else if (request.method == "GET" && (request.path == "/api/state" || request.path == "/api/navigation")) {
      std::lock_guard<std::mutex> lock(mutex);
      result = cached();
    } else if (request.method == "POST" && request.path == "/api/connect") {
      std::lock_guard<std::mutex> serial(control_serial);
      std::unique_lock<std::mutex> lock(mutex);
      if (stopped) { lock.unlock(); robotphone::httpReply(client, 503, errorReply(503, "gateway_stopped").body); return; }
      if(pending_stops) { lock.unlock(); robotphone::httpReply(client,409,errorReply(409,"stop_pending").body);return; }
      ++commands_inflight;
      lock.unlock();
      // A connection request must never steal a live owner's control session.
      result = backendRequest(options.backend_port, options.backend_timeout_ms, token, "POST", "/api/connect", request.body);
      lock.lock();
      --commands_inflight;
      picojson::value value;
      if (result.transport_ok && result.status == 200 && parseObject(result.body, value) && boolean(value, "ok")) {
        ++control_epoch; invalidate("session_changed"); session = string(value, "session");
        backend_sequence = mobile_sequence = navigation_sequence = cancellation_sequence = 0; last_owner_ms = 0; update(result);
        if (session.empty()) result = errorReply(503, "dashboard_session_missing");
      } else if (!result.transport_ok) fail("connect_failed",lock,"connect",result.status);
    } else result = errorReply(404, "not_found");
    robotphone::httpReply(client, result.status, decorate(result.body));
  }
};

Gateway::Gateway(GatewayOptions options, GatewayCallbacks callbacks)
    : impl_(new Impl(std::move(options), std::move(callbacks))) {}
Gateway::~Gateway() { stop(); }
bool Gateway::start(std::string &error) {
  auto &i = *impl_;
  if (!i.stopped) { error = "gateway_already_started"; return false; }
  if (i.options.port < 1024 || i.options.port > 65535 || i.options.backend_port < 1024 ||
      i.options.backend_port > 65535 || i.options.port == i.options.backend_port ||
      i.options.owner_lease_ms < 100 || i.options.owner_lease_ms > 350 ||
      i.options.backend_timeout_ms < 10 || i.options.backend_timeout_ms > 200) { error = "invalid_gateway_options"; return false; }
  std::ifstream file(i.options.token_file); std::getline(file, i.token);
  if (!i.token.empty() && i.token.back() == '\r') i.token.pop_back();
  if (!file || i.token.size() < 6 || i.token.size() > 128 || i.token.find_first_of(" \t\r\n") != std::string::npos) { error = "cannot_read_pairing_token"; return false; }
  i.stopped = false;
  if (!i.server.start(i.options.bind_address, i.options.port,
        [&i](int client, const HttpRequest &request) { i.request(client, request); }, error)) { i.stopped = true; return false; }
  i.watchdog = std::thread([&i] {
    while (!i.stopped) {
      bool expired=false;
      {
        std::lock_guard<std::mutex> lock(i.mutex);
        if (i.active && !i.ownerFresh()) {
          i.event("phone_lease_expired","watchdog"); ++i.control_epoch; ++i.pending_stops; i.invalidate("phone_lease_expired"); expired=true;
        }
      }
      if(expired) {
        std::lock_guard<std::mutex> serial(i.control_serial);
        std::unique_lock<std::mutex> lock(i.mutex);
        i.disarm(lock); --i.pending_stops;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  return true;
}
void Gateway::stop() {
  auto &i=*impl_; if(i.stopped.exchange(true))return;
  {std::lock_guard<std::mutex> lock(i.mutex); ++i.control_epoch; i.invalidate("gateway_shutdown");}
  {std::lock_guard<std::mutex> serial(i.control_serial);std::unique_lock<std::mutex> lock(i.mutex);i.disarm(lock);}
  i.server.stop(); if(i.watchdog.joinable())i.watchdog.join();
}
GatewayTelemetry Gateway::pollState() {
  auto &i=*impl_;
  std::unique_lock<std::mutex> sample(i.telemetry_serial,std::try_to_lock);
  if(!sample.owns_lock())return telemetry();
  std::unique_lock<std::mutex> lock(i.mutex);
  if(!i.stopped)i.refresh(lock);
  std::string error;
  const bool failed=i.active&&!i.motionReady(error);
  const auto expected=i.generation_id;
  auto result=i.state;
  lock.unlock();sample.unlock();
  if(failed)cancelNavigation(error,expected);
  return result;
}
GatewayTelemetry Gateway::telemetry() const { std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->state; }
bool Gateway::navigationActive() const { std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->active; }
std::uint64_t Gateway::generation() const { std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->generation_id; }
bool Gateway::beginNavigation(std::string &error) {
  std::uint64_t epoch;
  {std::lock_guard<std::mutex> admission(impl_->mutex);epoch=impl_->control_epoch;}
  std::lock_guard<std::mutex> serial(impl_->control_serial);
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if(impl_->stopped){error="gateway_stopped";return false;}
  if(impl_->pending_stops){error="stop_pending";return false;}
  if(epoch!=impl_->control_epoch){error="navigation_cancelled";return false;}
  return impl_->begin(error,lock);
}
bool Gateway::requestNavigation(const std::string &op,double x,double y,std::string &error) {
  const bool stop_op=op=="stop"||op=="reset-map";
  std::uint64_t epoch;
  {
    std::lock_guard<std::mutex> admission(impl_->mutex);
    if(stop_op){++impl_->control_epoch;++impl_->pending_stops;impl_->invalidate("navigation_"+op);}
    epoch=impl_->control_epoch;
  }
  std::lock_guard<std::mutex> serial(impl_->control_serial);
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if(impl_->stopped){if(stop_op)--impl_->pending_stops;error="gateway_stopped";return false;}
  if(!stop_op&&impl_->pending_stops){error="stop_pending";return false;}
  if(!stop_op&&epoch!=impl_->control_epoch){error="navigation_cancelled";return false;}
  const bool result=impl_->navigation(op,x,y,error,lock);
  if(stop_op)--impl_->pending_stops;
  return result;
}
void Gateway::cancelNavigation(const std::string &reason,std::uint64_t expected) {
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if(expected&&expected!=impl_->generation_id)return;
    if(!impl_->active)return;
    impl_->event(reason,"navigation_cancel");++impl_->control_epoch;++impl_->pending_stops;impl_->invalidate(reason);
  }
  std::lock_guard<std::mutex> serial(impl_->control_serial);
  std::unique_lock<std::mutex> lock(impl_->mutex);impl_->disarm(lock);--impl_->pending_stops;
}
void Gateway::visitNavigation(const std::function<void(bool,std::uint64_t)>& visit) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if(impl_->callbacks.navigation_layout)impl_->callbacks.navigation_layout(impl_->state);
  visit(!impl_->stopped&&impl_->active,impl_->generation_id);
}
bool Gateway::applyNav(double left,double right,std::uint64_t generation) {
  auto &i=*impl_;
  std::lock_guard<std::mutex> serial(i.control_serial);
  std::unique_lock<std::mutex> lock(i.mutex);
  if(i.stopped||!i.active||generation!=i.generation_id||i.pending_stops)return false;
  std::string error;
  if(!i.motionReady(error)){i.fail(error,lock);return false;}
  if(!std::isfinite(left)||!std::isfinite(right)||std::abs(left)>.60||std::abs(right)>.60){i.fail("invalid_navigation_duty",lock);return false;}
  JsonObject input;setString(input,"op","wheels");
  setNumber(input,"left",left);setNumber(input,"right",right);setString(input,"navigation_ticket",i.navigation_ticket);
  auto response=i.command(input,lock,"/api/navigation/wheels");
  if(!response.transport_ok||response.status!=200||!i.state.available||!i.state.armed){i.fail("navigation_command_failed",lock,"wheels",response.status);return false;}
  return !i.stopped&&i.active&&generation==i.generation_id;
}
}  // namespace robotnav
