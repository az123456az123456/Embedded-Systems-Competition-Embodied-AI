#ifndef ROBOT_NAVIGATION_GATEWAY_H
#define ROBOT_NAVIGATION_GATEWAY_H

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace robotnav {

struct GatewayOptions {
  std::string bind_address = "0.0.0.0";
  unsigned port = 8080;
  unsigned backend_port = 8082;  // Always 127.0.0.1, never an arbitrary host.
  std::string token_file = "/root/robot-perception/pair-token.txt";
  unsigned owner_lease_ms = 30000;
  unsigned backend_timeout_ms = 150;
};

struct GatewayTelemetry {
  std::uint64_t received_ms = 0, request_started_ms = 0, imu_sequence = 0, motor_ack_sequence = 0;
  // FPGA sample clock, shared with SNS1 uptime. Never use HTTP receipt time
  // as the IMU sample time when aligning the independently delivered streams.
  std::uint32_t imu_timestamp_ms = 0;
  bool imu_timestamp_valid = false;
  bool available = false, armed = false, link_ok = false;
  bool velocity_protocol_supported = false, softcore_ready = false;
  // Read-only FPGA NAV1 status, never an ACK or an authority/lease source.
  std::string softcore_reason, softcore_diagnostics_json = "{}";
  unsigned motor_ack_protocol = 0;
  bool navigation_preparing = false;
  std::uint64_t preparation_started_ms = 0;
  std::string preparation_operation;
  bool imu_valid = false, imu_fresh = false, imu_calibrating = true;
  bool imu_bias_valid = false, imu_action_pending = true;
  double yaw_deg = 0, roll_deg = 0, pitch_deg = 0;
  double imu_sample_age_ms = -1, motor_ack_age_ms = -1;
  unsigned left_mask = 0, invert_mask = 0;
  double left_duty = 0, right_duty = 0, ack_left_duty = 0, ack_right_duty = 0;
  std::string reason;
};

struct GatewayCallbacks {
  // Invoked while the control mutex is held. Callbacks MUST NOT call Gateway
  // methods or take a mutex held by a thread calling Gateway. Queue requests or
  // update atomic state instead. Returning true means accepted, not arrived.
  // For start/explore/straight this callback runs only after zero DRIVE and the backend
  // capability claim succeed. Core activation and gateway activation are then
  // observed together by visitNavigation(); callback rejection cancels both.
  std::function<bool(const std::string &op, double x_m, double y_m,
                     std::string &error)> navigation_command;
  std::function<void(const std::string &reason)> navigation_cancel;
  // Synchronize encoder pairing from the same fresh, acknowledged motor state
  // before a navigation operation/tick. Called under the control mutex.
  std::function<void(const GatewayTelemetry&)> navigation_layout;
  // The status callback is called without the control mutex; return a JSON object.
  std::function<std::string()> navigation_state;
  // Cached map only; no backend access, control lease renewal, or motor effects.
  std::function<std::string()> navigation_map;
  // Atomic map-bound goal selection and start, with live occupancy revalidation.
  std::function<bool(double x, double y, const std::string& map_session,
                     std::uint32_t revision, std::string& error)> navigation_goto;
};

// A single command arbiter in front of the unchanged v21/v21.1 dashboard. The
// dashboard remains the sole sender of motor packets. Phone API/video continue on :8080;
// launch the dashboard with --phone-bind 127.0.0.1 --phone-port 8082.
class Gateway {
 public:
  explicit Gateway(GatewayOptions, GatewayCallbacks = {});
  ~Gateway();
  Gateway(const Gateway &) = delete;
  Gateway &operator=(const Gateway &) = delete;
  bool start(std::string &error);
  void stop();
  // The runtime polls the loopback backend here. Socket waits do not hold the
  // control-state mutex. Public GETs reuse this cache and never refresh a lease.
  GatewayTelemetry pollState();
  GatewayTelemetry telemetry() const;
  bool navigationActive() const;
  bool navigationPreparing() const;
  std::uint64_t generation() const;
  // Physical velocity command: linear +/-0.30 m/s, CCW yaw rate +/-1 rad/s.
  // Requires matching ACK4 plus softcore_ready; transient ACK1/readiness gaps
  // preserve the session at zero for at most2s; never falls back to PWM.
  // Pass the generation obtained before computing this command; takeover or a
  // new navigation start invalidates commands computed against an old route.
  bool applyNav(double linear_mps, double angular_rps, std::uint64_t generation);
  // Local terminal start is allowed only while the phone owns a fresh foreground
  // control lease and has explicitly armed. Same checks as /api/navigation.
  bool beginNavigation(std::string &error);
  // Local foreground console equivalent of POST /api/navigation. Goal/start/straight
  // require a live phone owner; stop always cancels and disarms that owner.
  bool requestNavigation(const std::string &op, double x_m, double y_m,
                         std::string &error);
  void cancelNavigation(const std::string &reason, std::uint64_t expected_generation = 0);
  // Serialize a short navigation-core tick / planner submission with phone
  // takeover. Callback may take the core mutex, but MUST NOT call Gateway.
  void visitNavigation(const std::function<void(bool active, std::uint64_t generation)>&);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robotnav
#endif
