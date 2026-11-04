#ifndef ROBOT_PHONE_CONTROLLER_H
#define ROBOT_PHONE_CONTROLLER_H

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include "follow_geometry.h"

namespace robotphone {

// Identity memory only. Retained boxes never become control observations.
constexpr std::uint64_t kTargetIdentityHoldMs = 800;

// All times use the same monotonic clock, in milliseconds. The caller authenticates
// the owner before invoking commands; sensor polling must never invoke drive/follow.
struct Config {
  unsigned left_motor_mask = 0;  // M1..M4 are bits 0..3; right is its complement.
  double max_duty = 0.60;
  double slew_per_second = 1.20;
  std::uint64_t lease_ms = 30000, motion_ttl_ms = 350, motor_ack_ms = 300;
  std::uint64_t imu_fresh_ms = 300, vision_fresh_ms = 500;
  double heading_kp = 0.025, heading_max_steer = 0.35;
  double manual_steer_deadband = 0.04;
  double straight_cone_deg = 15.0;
  double turn_inner_ratio = 0.25;
  double turn_full_deg = 45.0;
  double follow_max_forward = 0.22;
  double stop_height_fraction = 0.55, stop_area_fraction = 0.25;
  double minimum_target_score = 0.30;
  double display_width = 864, display_height = 486;
  BearingConfig bearing;
  double follow_window_deg = 25.0;
  double follow_stop_front_m = 0.20, follow_stop_side_m = 0.22;
  bool follow_stereo_view = true;
  double follow_distance_m = .30, follow_distance_deadband_m = .05;
  bool follow_front_sonar_distance = true;
};

struct Detection {
  std::uint64_t track_id = 0;  // Assigned by Controller, never an array index.
  int class_id = -1;
  std::string name;
  double score = 0;
  int eye = -1;  // -1: one camera; 0/1: left/right half of the displayed image.
  double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  // Controller recomputes these from the physical camera's display region.
  double center_x = 0, height_fraction = 0, area_fraction = 0;
  // Internal source-array association for preview alignment; never a track ID.
  int source_index = -1;
};

struct VisionFrame {
  std::uint64_t frame_id = 0, time_ms = 0;
  std::vector<Detection> boxes;
  ImageGeometry geometry{};
  unsigned complete_mask=3,requested_mask=3;
  bool control_eligible=true;
};

struct ImuSample {
  std::uint64_t time_ms = 0;
  bool calibrated = false;
  double roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
};
struct FollowRanges {
  std::uint64_t time_ms = 0;
  bool valid[3] = {false,false,false};
  double metres[3] = {0,0,0};  // front, left, right; front also ranges the frontal target
};

// Original scan/pose times, never HTTP arrival time. Observations cannot grant
// motion authority or renew the phone owner's command lease.
struct FollowObservation {
  std::uint64_t target_id=0, selection_ms=0, vision_ms=0, scan_ms=0, pose_ms=0;
  bool matched=false, forward_clear=false, reverse_clear=false, turn_clear=false;
  double distance_m=0, bearing_right_deg=0, x_body_m=0, y_body_m=0;
};

enum class Mode { Disarmed, Idle, Manual, HeadingHold, Follow, RotateCw, RotateCcw, Navigation };
const char* modeName(Mode mode);

struct State {
  bool armed = false, estop = false, foreground = true;
  bool motion_paused = false;
  std::string pause_reason;
  std::uint64_t authorization_ms = 0;
  bool motor_link_fresh = false, imu_fresh = false, vision_fresh = false;
  Mode mode = Mode::Disarmed;
  std::string reason = "disarmed";
  std::uint64_t target_id = 0, last_control_seq = 0;
  double left = 0, right = 0, heading_error_deg = 0;
  bool twist_mode = false;
  double linear_mps = 0, angular_rps = 0;
  bool arc_mode = false;
  int arc_outer_permille = 0, arc_curve_q8 = 0;
  bool angle_mode = false;
  double turn_angle_deg = 0, follow_forward = 0;
  std::uint64_t selection_ms=0;
};

class Controller {
 public:
  explicit Controller(Config config = {});
  // Configuration clears current motion but retains an existing logical authorization.
  bool configure(const Config& config);
  const Config& config() const { return config_; }
  void updateMotorLink(std::uint64_t time_ms, bool ready);
  void updateImu(const ImuSample& sample);
  void resetYawReference(std::uint64_t time_ms);
  void updateRanges(const FollowRanges& sample) { ranges_ = sample; }
  void updateAngleProtocol(bool ready) { angle_ready_ = ready; }
  void updateArcProtocol(bool ready);
  void updateVelocityProtocol(bool supported,bool ready);
  BearingResult selectedBearing(std::uint64_t now_ms) const;
  bool selectedRays(std::uint64_t now_ms,double& left_deg,double& right_deg) const;
  // Dimensionless image error, not a fabricated physical bearing.
  bool selectedImage(std::uint64_t now_ms,double& error,double& yaw_since_capture) const;
  bool updateFollowObservation(const FollowObservation&,std::uint64_t now_ms);
  const FollowObservation& followObservation() const { return follow_observation_; }
  bool followObservationFresh(std::uint64_t now_ms) const;
  bool followDistanceValid(std::uint64_t now_ms) const;
  double followDistanceMetres() const;
  std::uint64_t followDistanceTimeMs() const;
  bool followPositionValid(std::uint64_t now_ms) const;
  const Detection* selectedTarget() const { return currentTarget(); }
  VisionFrame updateVision(VisionFrame frame);
  const VisionFrame& vision() const { return vision_; }
  const ImuSample& imu() const { return imu_; }
  void setForeground(bool foreground, std::uint64_t now_ms);
  bool arm(std::uint64_t seq, std::uint64_t now_ms);
  bool drive(std::uint64_t seq, double throttle, double steer, bool hold,
             std::uint64_t now_ms);
  bool driveVelocity(std::uint64_t seq,double linear_mps,double angular_rps,std::uint64_t now_ms,
                     std::uint64_t owner_time_ms = UINT64_MAX);
  bool idle(std::uint64_t seq, std::uint64_t now_ms, std::uint64_t owner_time_ms = UINT64_MAX);
  bool renewAuthorization(std::uint64_t owner_time_ms, std::uint64_t now_ms);
  bool lease(std::uint64_t seq, std::uint64_t now_ms, std::uint64_t owner_time_ms);
  void commandSourceTime(std::uint64_t value = UINT64_MAX) { command_source_ms_=value; }
  void pause(const std::string& reason);
  bool driveWheels(std::uint64_t seq, double left, double right, std::uint64_t now_ms);
  bool rotate(std::uint64_t seq, bool clockwise, double speed, std::uint64_t now_ms);
  bool select(std::uint64_t frame_id, std::uint64_t track_id,
              std::uint64_t now_ms);
  bool canSelect(std::uint64_t frame_id, std::uint64_t track_id,
                 std::uint64_t now_ms) const;
  bool follow(std::uint64_t seq, std::uint64_t now_ms);
  void stop(std::uint64_t now_ms, bool emergency = false);
  void disarm(std::uint64_t now_ms);
  void ownerDisconnected(std::uint64_t now_ms);
  const State& tick(std::uint64_t now_ms);
  const State& state() const { return state_; }

 private:
  Config config_;
  bool configured_ = false, have_motor_ = false, motor_ready_ = false;
  bool have_imu_ = false, have_vision_ = false, have_tick_ = false;
  bool have_lease_ = false, hold_requested_ = false, recapture_heading_ = false;
  std::uint64_t motor_time_ = 0, lease_time_ = 0, last_tick_ = 0;
  std::uint64_t next_track_id_ = 1, command_source_ms_ = UINT64_MAX;
  double throttle_ = 0, steer_ = 0, heading_target_ = 0;
  ImuSample imu_;
  FollowRanges ranges_;
  FollowObservation follow_observation_;
  struct RememberedTarget { Detection box; std::uint64_t time_ms; };
  std::vector<RememberedTarget> tracking_;
  bool angle_ready_ = false, arc_ready_ = false, velocity_supported_ = false, velocity_ready_ = false, raw_wheels_ = false;
  int travel_sign_ = 1;
  double raw_left_ = 0, raw_right_ = 0;
  std::deque<ImuSample> yaw_history_;
  std::uint64_t yaw_epoch_ms_ = 0;
  VisionFrame vision_;
  std::deque<VisionFrame> history_;
  State state_;
  void zero(const std::string& reason, bool disarm);
  void waitFollow(const std::string& reason);
  void refresh(std::uint64_t now_ms);
  bool motionReady(std::uint64_t now_ms);
  bool acceptSequence(std::uint64_t seq);
  const Detection* currentTarget() const;
  void mix(double forward, double steer, bool joystick_scale,
           double& left, double& right) const;
  bool yawAt(std::uint64_t time_ms, double& yaw) const;
  void frontSonarFollow(bool image_mode,double image_error,
                        double image_delta,double camera_angle);
};

}  // namespace robotphone
#endif
