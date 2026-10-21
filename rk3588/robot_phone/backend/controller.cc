#include "controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robotphone {
namespace {
double clip(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
bool fresh(std::uint64_t now, std::uint64_t then, std::uint64_t limit) {
  return now >= then && now - then <= limit;
}
double wrap(double degrees) {
  return std::remainder(degrees, 360.0);
}
bool finite(double v) { return std::isfinite(v); }
bool validConfig(const Config& c) {
  return c.left_motor_mask > 0 && c.left_motor_mask < 15 &&
      finite(c.max_duty) && c.max_duty > 0 && c.max_duty <= .60 &&
      finite(c.slew_per_second) && c.slew_per_second > 0 &&
      c.lease_ms > 0 && c.lease_ms <= 350 &&
      c.motor_ack_ms > 0 && c.motor_ack_ms <= 300 &&
      c.imu_fresh_ms > 0 && c.imu_fresh_ms <= 500 &&
      c.vision_fresh_ms > 0 && c.vision_fresh_ms <= 500 &&
      finite(c.heading_kp) && c.heading_kp >= 0 &&
      finite(c.heading_max_steer) && c.heading_max_steer >= 0 && c.heading_max_steer <= .5 &&
      finite(c.manual_steer_deadband) && c.manual_steer_deadband >= 0 && c.manual_steer_deadband < .5 &&
      finite(c.follow_max_forward) && c.follow_max_forward > 0 && c.follow_max_forward <= .25 &&
      finite(c.follow_kp) && c.follow_kp >= 0 && c.follow_kp <= 1 &&
      finite(c.follow_deadband) && c.follow_deadband >= 0 &&
      finite(c.follow_forward_window) && c.follow_forward_window > c.follow_deadband && c.follow_forward_window <= 1 &&
      finite(c.stop_height_fraction) && c.stop_height_fraction > 0 && c.stop_height_fraction <= 1 &&
      finite(c.stop_area_fraction) && c.stop_area_fraction > 0 && c.stop_area_fraction <= 1 &&
      finite(c.minimum_target_score) && c.minimum_target_score >= 0 && c.minimum_target_score <= 1 &&
      finite(c.display_width) && finite(c.display_height) && c.display_width > 1 && c.display_height > 1;
}
double overlap(const Detection& a, const Detection& b) {
  const double w = std::max(0.0, std::min(a.x2,b.x2)-std::max(a.x1,b.x1));
  const double h = std::max(0.0, std::min(a.y2,b.y2)-std::max(a.y1,b.y1));
  const double intersection = w*h;
  const double area = (a.x2-a.x1)*(a.y2-a.y1)+(b.x2-b.x1)*(b.y2-b.y1)-intersection;
  return area > 0 ? intersection/area : 0;
}
// No appearance re-identification: a missing/ambiguous object gets a new ID and
// requires another tap. This deliberately prefers a stop over switching objects.
double affinity(const Detection& a, const Detection& b, const Config& c) {
  if (a.eye != b.eye || a.class_id != b.class_id) return -1;
  const double wr = (b.x2-b.x1)/(a.x2-a.x1), hr = (b.y2-b.y1)/(a.y2-a.y1);
  if (wr < .55 || wr > 1.8 || hr < .55 || hr > 1.8) return -1;
  const double width = c.display_width / (a.eye == -1 ? 1 : 2);
  const double dx = ((b.x1+b.x2)-(a.x1+a.x2))*.5/width;
  const double dy = ((b.y1+b.y2)-(a.y1+a.y2))*.5/c.display_height;
  const double distance = std::hypot(dx,dy), iou = overlap(a,b);
  if (distance > .16 || (iou < .18 && distance > .055)) return -1;
  return .65*iou + .35*(1-distance/.16);
}
struct Best {
  int index = -1;
  double first = -1, second = -1;
  void add(int i, double value) {
    if (value < 0) return;
    if (value > first) { second = first; first = value; index = i; }
    else if (value > second) second = value;
  }
  bool unique() const { return index >= 0 && (second < 0 || first-second >= .15); }
};
}  // namespace

const char* modeName(Mode mode) {
  switch (mode) {
    case Mode::Disarmed: return "disarmed";
    case Mode::Idle: return "idle";
    case Mode::Manual: return "manual";
    case Mode::HeadingHold: return "heading_hold";
    case Mode::Follow: return "follow";
  }
  return "disarmed";
}

Controller::Controller(Config config) { configure(config); state_.reason = "disarmed"; }

bool Controller::configure(const Config& config) {
  zero("configuration_changed", true);
  config_ = config;
  configured_ = validConfig(config_);
  have_vision_ = false;
  vision_ = {};
  history_.clear();
  if (!configured_) state_.reason = "invalid_configuration";
  return configured_;
}

void Controller::zero(const std::string& reason, bool disarm_flag) {
  state_.left = state_.right = state_.heading_error_deg = 0;
  state_.target_id = 0;
  throttle_ = steer_ = 0;
  hold_requested_ = recapture_heading_ = false;
  have_lease_ = false;
  if (disarm_flag) state_.armed = false;
  state_.mode = state_.armed ? Mode::Idle : Mode::Disarmed;
  state_.reason = reason;
}

void Controller::refresh(std::uint64_t now) {
  state_.motor_link_fresh = have_motor_ && motor_ready_ && fresh(now,motor_time_,config_.motor_ack_ms);
  state_.imu_fresh = have_imu_ && imu_.calibrated && fresh(now,imu_.time_ms,config_.imu_fresh_ms);
  state_.vision_fresh = have_vision_ && fresh(now,vision_.time_ms,config_.vision_fresh_ms);
}

void Controller::updateMotorLink(std::uint64_t time, bool ready) {
  if (have_motor_ && time < motor_time_) return;
  have_motor_ = true; motor_ready_ = ready; motor_time_ = time;
  if (!ready && state_.armed) zero("motor_link_not_ready",true);
}

void Controller::updateImu(const ImuSample& sample) {
  if (have_imu_ && sample.time_ms < imu_.time_ms) return;
  imu_ = sample;
  have_imu_ = finite(sample.roll_deg) && finite(sample.pitch_deg) && finite(sample.yaw_deg);
  if ((!have_imu_ || !sample.calibrated) &&
      (state_.mode == Mode::Follow || hold_requested_)) zero("imu_not_calibrated",false);
}

VisionFrame Controller::updateVision(VisionFrame frame) {
  if (!finite(config_.display_width) || !finite(config_.display_height) ||
      config_.display_width <= 1 || config_.display_height <= 1) return vision_;
  if (have_vision_ && (frame.frame_id <= vision_.frame_id || frame.time_ms < vision_.time_ms)) return vision_;
  std::vector<Detection> valid;
  for (auto box : frame.boxes) {
    if (box.eye < -1 || box.eye > 1 || box.class_id < 0 || !finite(box.score) ||
        box.score < 0 || box.score > 1 || !finite(box.x1) || !finite(box.x2) ||
        !finite(box.y1) || !finite(box.y2)) continue;
    const double width = config_.display_width / (box.eye == -1 ? 1 : 2);
    const double origin = box.eye == 1 ? width : 0;
    box.x1 = clip(box.x1,origin,origin+width); box.x2 = clip(box.x2,origin,origin+width);
    box.y1 = clip(box.y1,0,config_.display_height); box.y2 = clip(box.y2,0,config_.display_height);
    if (box.x2 <= box.x1 || box.y2 <= box.y1) continue;
    box.center_x = clip(((box.x1+box.x2)*.5-origin)/width*2-1,-1,1);
    box.height_fraction = (box.y2-box.y1)/config_.display_height;
    box.area_fraction = (box.x2-box.x1)*(box.y2-box.y1)/(width*config_.display_height);
    box.track_id = 0;
    valid.push_back(std::move(box));
  }
  frame.boxes = std::move(valid);
  if (have_vision_ && fresh(frame.time_ms,vision_.time_ms,std::min<std::uint64_t>(250,config_.vision_fresh_ms))) {
    std::vector<Best> old(vision_.boxes.size()), current(frame.boxes.size());
    for (std::size_t i=0;i<old.size();++i) for (std::size_t j=0;j<current.size();++j) {
      double score = affinity(vision_.boxes[i],frame.boxes[j],config_);
      old[i].add(static_cast<int>(j),score); current[j].add(static_cast<int>(i),score);
    }
    for (std::size_t j=0;j<current.size();++j) {
      const auto& best = current[j];
      if (best.unique() && old[best.index].unique() && old[best.index].index == static_cast<int>(j))
        frame.boxes[j].track_id = vision_.boxes[best.index].track_id;
    }
  }
  for (auto& box: frame.boxes) if (!box.track_id) box.track_id = next_track_id_++;
  vision_ = std::move(frame); have_vision_ = true;
  history_.push_back(vision_);
  while (!history_.empty() && (history_.size()>32 ||
         !fresh(vision_.time_ms,history_.front().time_ms,500))) history_.pop_front();
  if (state_.target_id && !currentTarget()) zero("target_lost",false);
  return vision_;
}

void Controller::setForeground(bool foreground, std::uint64_t) {
  state_.foreground = foreground;
  if (!foreground) zero("background",true);
}

bool Controller::acceptSequence(std::uint64_t seq) {
  if (seq <= state_.last_control_seq) return false;
  state_.last_control_seq = seq;
  return true;
}

bool Controller::arm(std::uint64_t seq, std::uint64_t now) {
  refresh(now);
  if (!configured_ || !state_.foreground || !state_.motor_link_fresh ||
      !acceptSequence(seq)) return false;
  zero("ready",true);
  state_.estop = false; state_.armed = true; state_.mode = Mode::Idle;
  last_tick_ = now; have_tick_ = true;
  return true;
}

bool Controller::motionReady(std::uint64_t now) {
  refresh(now);
  if (!state_.armed || state_.estop) return false;
  if (!state_.foreground) { zero("background",true); return false; }
  if (!state_.motor_link_fresh) { zero("motor_link_stale",true); return false; }
  if (have_lease_ && !fresh(now,lease_time_,config_.lease_ms)) {
    zero("lease_expired",true); return false;
  }
  return true;
}

bool Controller::drive(std::uint64_t seq, double throttle, double steer, bool hold,
                       std::uint64_t now) {
  if (!motionReady(now) || seq <= state_.last_control_seq) return false;
  if (!finite(throttle) || !finite(steer) || std::abs(throttle)>1 || std::abs(steer)>1) {
    zero("invalid_command",true); return false;
  }
  if (throttle == 0 && steer == 0) {
    if (!acceptSequence(seq)) return false;
    zero("joystick_released",false);
    return true;
  }
  if (hold && !state_.imu_fresh) { zero("imu_stale",false); return false; }
  if (!acceptSequence(seq)) return false;
  state_.target_id = 0;
  if (hold && !hold_requested_) { heading_target_ = imu_.yaw_deg; recapture_heading_ = false; }
  hold_requested_ = hold;
  throttle_ = throttle; steer_ = steer;
  state_.mode = hold ? Mode::HeadingHold : Mode::Manual;
  state_.reason = hold ? "heading_hold" : "manual";
  lease_time_ = now; have_lease_ = true;
  return true;
}

const Detection* Controller::currentTarget() const {
  if (!state_.target_id) return nullptr;
  for (const auto& box: vision_.boxes)
    if (box.track_id == state_.target_id && box.score >= config_.minimum_target_score) return &box;
  return nullptr;
}

bool Controller::select(std::uint64_t frame_id, std::uint64_t track_id, std::uint64_t now) {
  refresh(now);
  if (!track_id || !state_.vision_fresh) return false;
  bool found = false;
  for (const auto& frame: history_) if (frame.frame_id == frame_id && fresh(now,frame.time_ms,500)) {
    for (const auto& box: frame.boxes) if (box.track_id == track_id && box.score >= config_.minimum_target_score) found = true;
  }
  if (!found) return false;
  const auto old_id = state_.target_id;
  state_.target_id = track_id;
  if (!currentTarget()) { state_.target_id = old_id; return false; }
  // A new tap never starts/resumes movement by itself. Cancel any previous
  // joystick/heading/follow command before installing the chosen target.
  zero("target_selected",false); state_.target_id = track_id;
  return true;
}

bool Controller::follow(std::uint64_t seq, std::uint64_t now) {
  if (!motionReady(now) || seq <= state_.last_control_seq) return false;
  if (!state_.imu_fresh) { zero("imu_stale",false); return false; }
  if (!state_.vision_fresh || !currentTarget()) { zero("target_lost",false); return false; }
  if (!acceptSequence(seq)) return false;
  hold_requested_ = recapture_heading_ = false;
  throttle_ = steer_ = 0;
  state_.mode = Mode::Follow; state_.reason = "following";
  lease_time_ = now; have_lease_ = true;
  return true;
}

void Controller::stop(std::uint64_t, bool emergency) {
  zero(emergency ? "emergency_stop" : "stopped",true);
  if (emergency) state_.estop = true;
}
void Controller::disarm(std::uint64_t) { zero("disarmed",true); }
void Controller::ownerDisconnected(std::uint64_t) { zero("owner_disconnected",true); }

void Controller::mix(double forward, double steer, bool joystick_scale,
                     double& left, double& right) const {
  left = forward+steer; right = forward-steer;
  const double magnitude = std::max({std::abs(left),std::abs(right),joystick_scale ? 1.0 : config_.max_duty});
  const double scale = config_.max_duty/magnitude;
  left *= scale; right *= scale;
}

const State& Controller::tick(std::uint64_t now) {
  refresh(now);
  if (have_tick_ && now < last_tick_) { zero("clock_reversed",true); return state_; }
  const double dt = have_tick_ ? std::min<std::uint64_t>(now-last_tick_,100)/1000.0 : 0;
  last_tick_ = now; have_tick_ = true;
  if (!state_.armed) return state_;
  if (!motionReady(now)) return state_;
  if (state_.mode == Mode::Idle) return state_;
  if (!have_lease_ || !fresh(now,lease_time_,config_.lease_ms)) {
    zero("lease_expired",true); return state_;
  }
  double forward = throttle_, steer = steer_;
  bool joystick = true;
  if (state_.mode == Mode::Follow) {
    if (!state_.imu_fresh) { zero("imu_stale",false); return state_; }
    const auto* target = currentTarget();
    if (!state_.vision_fresh || !target) { zero("target_lost",false); return state_; }
    if (target->height_fraction >= config_.stop_height_fraction ||
        target->area_fraction >= config_.stop_area_fraction) {
      zero("target_reached",false); return state_;
    }
    const double error = std::abs(target->center_x) <= config_.follow_deadband ? 0 : target->center_x;
    steer = clip(config_.follow_kp*error,-.35,.35);
    forward = std::abs(error) >= config_.follow_forward_window ? 0 :
        config_.follow_max_forward*(1-std::abs(error)/config_.follow_forward_window);
    joystick = false;
  } else if (hold_requested_) {
    if (!state_.imu_fresh) { zero("imu_stale",false); return state_; }
    if (std::abs(steer_) > config_.manual_steer_deadband) {
      recapture_heading_ = true; state_.heading_error_deg = 0;
    } else if (forward != 0) {
      if (recapture_heading_) { heading_target_ = imu_.yaw_deg; recapture_heading_ = false; }
      state_.heading_error_deg = wrap(imu_.yaw_deg-heading_target_);
      // Positive yaw means turned left. A positive steer drives the left wheels
      // faster, correcting right. This sign must not reverse when backing up.
      steer = clip(config_.heading_kp*state_.heading_error_deg,-config_.heading_max_steer,config_.heading_max_steer);
    } else { steer = 0; state_.heading_error_deg = 0; }
  } else state_.heading_error_deg = 0;
  double left=0,right=0;
  mix(forward,steer,joystick,left,right);
  if (left == 0 && right == 0) { state_.left = state_.right = 0; return state_; }
  const double delta = std::max(std::abs(left-state_.left),std::abs(right-state_.right));
  const double fraction = delta > 0 ? std::min(1.0,config_.slew_per_second*dt/delta) : 1;
  state_.left += (left-state_.left)*fraction;
  state_.right += (right-state_.right)*fraction;
  return state_;
}
}  // namespace robotphone
