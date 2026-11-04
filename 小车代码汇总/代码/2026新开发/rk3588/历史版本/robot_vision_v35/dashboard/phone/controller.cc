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
      finite(c.straight_cone_deg) && c.straight_cone_deg >= 0 && c.straight_cone_deg < 80 &&
      finite(c.turn_inner_ratio) && c.turn_inner_ratio >= .10 && c.turn_inner_ratio <= .80 &&
      finite(c.turn_full_deg) && c.turn_full_deg > c.straight_cone_deg && c.turn_full_deg > 15 && c.turn_full_deg <= 90 &&
      finite(c.manual_steer_deadband) && c.manual_steer_deadband >= 0 && c.manual_steer_deadband < .5 &&
      finite(c.follow_max_forward) && c.follow_max_forward > 0 && c.follow_max_forward <= .22 &&
      finite(c.stop_height_fraction) && c.stop_height_fraction > 0 && c.stop_height_fraction <= 1 &&
      finite(c.stop_area_fraction) && c.stop_area_fraction > 0 && c.stop_area_fraction <= 1 &&
      finite(c.minimum_target_score) && c.minimum_target_score >= 0 && c.minimum_target_score <= 1 &&
      finite(c.display_width) && finite(c.display_height) && c.display_width > 1 && c.display_height > 1 &&
      finite(c.follow_window_deg) && c.follow_window_deg > 2 && c.follow_window_deg <= 25 &&
      finite(c.follow_stop_front_m) && c.follow_stop_front_m >= .50 &&
      finite(c.follow_stop_side_m) && c.follow_stop_side_m >= .22;
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
    case Mode::RotateCw: return "rotate_cw";
    case Mode::RotateCcw: return "rotate_ccw";
    case Mode::Navigation: return "navigation_velocity";
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
  state_.twist_mode=false;state_.linear_mps=state_.angular_rps=0;
  state_.arc_mode = false; state_.arc_outer_permille = state_.arc_curve_q8 = 0;
  raw_wheels_ = false; travel_sign_ = 1;
  state_.angle_mode = false; state_.turn_angle_deg = state_.follow_forward = 0;
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

void Controller::updateArcProtocol(bool ready) {
  arc_ready_ = ready;
  if (!ready && !raw_wheels_ &&
      (state_.mode == Mode::Manual || state_.mode == Mode::HeadingHold))
    zero("arc_protocol_lost",true);
}

void Controller::updateVelocityProtocol(bool supported,bool ready){
  velocity_supported_=supported;velocity_ready_=ready;
  if(state_.twist_mode&&(!supported||(!ready&&(state_.linear_mps!=0||state_.angular_rps!=0))))
    zero(supported?"softcore_not_ready":"velocity_protocol_lost",true);
}

void Controller::updateImu(const ImuSample& sample) {
  if (have_imu_ && sample.time_ms < imu_.time_ms) return;
  imu_ = sample;
  have_imu_ = finite(sample.roll_deg) && finite(sample.pitch_deg) && finite(sample.yaw_deg);
  if (!have_imu_ || !sample.calibrated) yaw_history_.clear();
  else if(sample.time_ms>yaw_epoch_ms_) {
    // Same sensor sample may be published by several UI ticks. It must not
    // become an artificial new IMU observation or extend history freshness.
    if (!yaw_history_.empty() && yaw_history_.back().time_ms == sample.time_ms)
      yaw_history_.back() = sample;
    else yaw_history_.push_back(sample);
    while (!yaw_history_.empty() && (yaw_history_.size()>256 ||
           sample.time_ms-yaw_history_.front().time_ms>2000)) yaw_history_.pop_front();
  }
  if ((!have_imu_ || !sample.calibrated) &&
      (state_.mode == Mode::Follow || hold_requested_)) zero("imu_not_calibrated",false);
}

void Controller::resetYawReference(std::uint64_t time) {
  zero("imu_reference_changed",true);
  yaw_history_.clear();yaw_epoch_ms_=time;
}

bool Controller::yawAt(std::uint64_t time,double& yaw) const {
  if(yaw_history_.empty())return false;
  for(const auto &sample:yaw_history_)if(sample.time_ms==time){yaw=sample.yaw_deg;return true;}
  if(time<yaw_history_.front().time_ms)return false;
  for(size_t i=1;i<yaw_history_.size();++i){
    const auto &a=yaw_history_[i-1],&b=yaw_history_[i];
    if(time<=b.time_ms){
      if(b.time_ms-a.time_ms>100)return false;
      yaw=a.yaw_deg+wrap(b.yaw_deg-a.yaw_deg)*double(time-a.time_ms)/double(b.time_ms-a.time_ms);
      return true;
    }
  }
  if(time-yaw_history_.back().time_ms>50)return false;
  yaw=yaw_history_.back().yaw_deg;return true;
}

BearingResult Controller::selectedBearing(std::uint64_t now) const {
  BearingResult result;
  const auto *target=currentTarget();
  if(!target){result.reason="target_lost";return result;}
  const double normalized_u=((target->x1+target->x2)*.5-(target->eye==1?config_.display_width/2:0))/(config_.display_width/2);
  const double normalized_v=(target->y1+target->y2)*.5/config_.display_height;
  double u=0,v=0;std::string geometry_error;
  if(!cameraPixel(vision_.geometry,normalized_u,normalized_v,u,v,geometry_error)){result.reason=geometry_error;return result;}
  result.u_px=u;result.v_px=v;
  if(!config_.follow_stereo_view || target->eye<0){result.reason="view_not_supported";return result;}
  if(vision_.time_ms<=yaw_epoch_ms_){result.reason="yaw_history_missing";return result;}
  if(!fresh(now,vision_.time_ms,config_.vision_fresh_ms)){result.reason="vision_stale";return result;}
  // Calibrating pixels is allowed while disarmed. Missing intrinsics are
  // reported before IMU history to avoid hiding the actual setup requirement.
  result=targetBearing(config_.bearing,target->eye,result.u_px,0);
  result.v_px=v;
  if(!result.valid)return result;
  double capture_yaw=0;
  if(!have_imu_ || !imu_.calibrated || !fresh(now,imu_.time_ms,config_.imu_fresh_ms) ||
     !yawAt(vision_.time_ms,capture_yaw)){result.valid=false;result.reason="yaw_history_missing";return result;}
  result=targetBearing(config_.bearing,target->eye,result.u_px,wrap(imu_.yaw_deg-capture_yaw));result.v_px=v;return result;
}

VisionFrame Controller::updateVision(VisionFrame frame) {
  // Partial/late pairs are display observations only and never renew control.
  if(!frame.control_eligible || !frame.requested_mask || frame.complete_mask!=frame.requested_mask)return vision_;
  if (!finite(config_.display_width) || !finite(config_.display_height) ||
      config_.display_width <= 1 || config_.display_height <= 1) return vision_;
  if (have_vision_ && (frame.frame_id <= vision_.frame_id || frame.time_ms < vision_.time_ms)) return vision_;
  std::vector<Detection> valid;
  double content_left=0,content_top=0,content_right=1,content_bottom=1;std::string geometry_error;
  const bool usable_geometry=imageContent(frame.geometry,content_left,content_top,content_right,content_bottom,geometry_error);
  for (auto box : frame.boxes) {
    if(!usable_geometry)continue;
    if (box.eye < -1 || box.eye > 1 || box.class_id < 0 || !finite(box.score) ||
        box.score < 0 || box.score > 1 || !finite(box.x1) || !finite(box.x2) ||
        !finite(box.y1) || !finite(box.y2)) continue;
    const double width = config_.display_width / (box.eye == -1 ? 1 : 2);
    const double origin = box.eye == 1 ? width : 0;
    const double left=origin+width*content_left,right=origin+width*content_right;
    const double top=config_.display_height*content_top,bottom=config_.display_height*content_bottom;
    box.x1 = clip(box.x1,left,right); box.x2 = clip(box.x2,left,right);
    box.y1 = clip(box.y1,top,bottom); box.y2 = clip(box.y2,top,bottom);
    if (box.x2 <= box.x1 || box.y2 <= box.y1) continue;
    box.center_x = clip(((box.x1+box.x2)*.5-left)/(right-left)*2-1,-1,1);
    box.height_fraction = (box.y2-box.y1)/(bottom-top);
    box.area_fraction = (box.x2-box.x1)*(box.y2-box.y1)/((right-left)*(bottom-top));
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
  if (!arc_ready_) { zero("arc_protocol_waiting",true); return false; }
  if (hold && !state_.imu_fresh) { zero("imu_stale",false); return false; }
  if (!acceptSequence(seq)) return false;
  state_.twist_mode=false;state_.linear_mps=state_.angular_rps=0;
  state_.target_id = 0;
  if (hold && !hold_requested_) { heading_target_ = imu_.yaw_deg; recapture_heading_ = false; }
  hold_requested_ = hold;
  raw_wheels_ = false;
  if (std::abs(throttle) >= .12*std::hypot(throttle,steer)) travel_sign_ = throttle > 0 ? 1 : -1;
  throttle_ = throttle; steer_ = steer;
  state_.mode = hold ? Mode::HeadingHold : Mode::Manual;
  state_.reason = hold ? "heading_hold" : "manual";
  lease_time_ = now; have_lease_ = true;
  return true;
}

bool Controller::driveVelocity(std::uint64_t seq,double linear,double angular,std::uint64_t now){
  if(!motionReady(now)||seq<=state_.last_control_seq)return false;
  if(!finite(linear)||!finite(angular)||std::abs(linear)>.30||std::abs(angular)>1.){
    zero("invalid_navigation_velocity",true);return false;
  }
  if(!velocity_supported_){zero("fpga_v28_velocity_required",true);return false;}
  if((linear!=0||angular!=0)&&!velocity_ready_){zero("softcore_not_ready",true);return false;}
  if(!acceptSequence(seq))return false;
  state_.left=state_.right=0;state_.heading_error_deg=0;state_.target_id=0;
  state_.arc_mode=state_.angle_mode=false;state_.arc_outer_permille=state_.arc_curve_q8=0;
  state_.turn_angle_deg=state_.follow_forward=0;
  raw_wheels_=false;hold_requested_=recapture_heading_=false;
  state_.twist_mode=true;state_.linear_mps=linear;state_.angular_rps=angular;
  state_.mode=Mode::Navigation;state_.reason="navigation_velocity";
  lease_time_=now;have_lease_=true;return true;
}

bool Controller::driveWheels(std::uint64_t seq,double left,double right,std::uint64_t now) {
  if (!motionReady(now) || seq <= state_.last_control_seq) return false;
  if (!finite(left) || !finite(right) || std::abs(left)>config_.max_duty || std::abs(right)>config_.max_duty) {
    zero("invalid_navigation_duty",true); return false;
  }
  if (!acceptSequence(seq)) return false;
  if (left==0 && right==0) { zero("navigation_zero",false); return true; }
  state_.twist_mode=false;state_.linear_mps=state_.angular_rps=0;
  state_.target_id=0; hold_requested_=recapture_heading_=false;
  raw_wheels_=true; raw_left_=left; raw_right_=right;
  state_.mode=Mode::Manual; state_.reason="navigation_wheels";
  lease_time_=now; have_lease_=true; return true;
}

const Detection* Controller::currentTarget() const {
  if (!state_.target_id) return nullptr;
  for (const auto& box: vision_.boxes)
    if (box.track_id == state_.target_id && box.score >= config_.minimum_target_score) return &box;
  return nullptr;
}

bool Controller::rotate(std::uint64_t seq, bool clockwise, double speed, std::uint64_t now) {
  if (!finite(speed) || speed < 0 || speed > 1) { zero("invalid_command",true); return false; }
  const double duty = std::min({speed,.35,config_.max_duty});
  if (!driveWheels(seq,(clockwise?1:-1)*duty,(clockwise?-1:1)*duty,now)) return false;
  if (speed > 0) {
    state_.mode = clockwise ? Mode::RotateCw : Mode::RotateCcw;
    state_.reason = modeName(state_.mode);
  }
  return true;
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
  if (!angle_ready_) { zero("angle_protocol_waiting",false); return false; }
  const auto bearing=selectedBearing(now);
  if (!bearing.valid) { zero(bearing.reason,false); return false; }
  if (!acceptSequence(seq)) return false;
  hold_requested_ = recapture_heading_ = false;
  state_.twist_mode=false;state_.linear_mps=state_.angular_rps=0;
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
  if(state_.twist_mode){
    if(!velocity_supported_||(!velocity_ready_&&(state_.linear_mps!=0||state_.angular_rps!=0)))
      zero(velocity_supported_?"softcore_not_ready":"velocity_protocol_lost",true);
    return state_;
  }
  double forward = throttle_, steer = steer_;
  state_.arc_mode=false; state_.arc_outer_permille=state_.arc_curve_q8=0;
  state_.angle_mode = false; state_.turn_angle_deg = state_.follow_forward = 0;
  bool manual_arc = !raw_wheels_ && !hold_requested_;
  bool joystick = true;
  if (state_.mode == Mode::Follow) {
    if (!state_.imu_fresh) { zero("imu_stale",false); return state_; }
    const auto* target = currentTarget();
    if (!state_.vision_fresh || !target) { zero("target_lost",false); return state_; }
    if (target->height_fraction >= config_.stop_height_fraction ||
        target->area_fraction >= config_.stop_area_fraction) {
      zero("target_reached",false); return state_;
    }
    if(!angle_ready_){zero("angle_protocol_waiting",false);return state_;}
    const auto bearing=selectedBearing(now);
    if(!bearing.valid){zero(bearing.reason,false);return state_;}
    if(!fresh(now,ranges_.time_ms,500) || !ranges_.valid[0] || !ranges_.valid[1] || !ranges_.valid[2]){
      zero("sonar_stale",false);return state_;
    }
    for(int i=0;i<3;i++)if(!finite(ranges_.metres[i]) || ranges_.metres[i]<.03 || ranges_.metres[i]>4.5){
      zero("sonar_stale",false);return state_;
    }
    if(ranges_.metres[0]<=config_.follow_stop_front_m){zero("obstacle_front",false);return state_;}
    if((bearing.turn_deg < -2 && ranges_.metres[1]<=config_.follow_stop_side_m) ||
       (bearing.turn_deg > 2 && ranges_.metres[2]<=config_.follow_stop_side_m)){
      zero("obstacle_side",false);return state_;
    }
    // MOT2 transmits the angle itself. FPGA owns angle->differential PWM,
    // output slew, reversal dead-time and its independent command watchdog.
    state_.angle_mode=true;
    state_.turn_angle_deg=std::round(clip(bearing.turn_deg,-90.,90.)*100)/100;
    const double a=std::abs(state_.turn_angle_deg);
    forward=std::min(.22,config_.follow_max_forward)*(a<=2?1:std::max(0.,1-(a-2)/(config_.follow_window_deg-2)));
    forward*=clip((ranges_.metres[0]-config_.follow_stop_front_m)/.50,0.,1.);
    state_.follow_forward=std::round(forward*1000)/1000;
    // Diagnostic requested outputs, not an applied-output/angle measurement.
    const int cdeg=std::lround(state_.turn_angle_deg*100);
    const int correction=std::abs(cdeg)<=200?0:std::max(-220,std::min(220,cdeg/25));
    state_.left=state_.follow_forward+correction/1000.;
    state_.right=state_.follow_forward-correction/1000.;
    return state_;
  } else if (hold_requested_) {
    if (!state_.imu_fresh) { zero("imu_stale",false); return state_; }
    const double stick_angle = std::atan2(std::abs(steer_),std::abs(forward))*180./3.14159265358979323846;
    if (stick_angle > config_.straight_cone_deg) {
      manual_arc = true;
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
  if (raw_wheels_) { left=raw_left_; right=raw_right_; }
  else if (manual_arc) {
    if (!arc_ready_) { zero("arc_protocol_lost",true); return state_; }
    const double radius=std::min(1.,std::hypot(forward,steer));
    const double degrees=std::atan2(std::abs(steer),std::abs(forward))*180./3.14159265358979323846;
    const double turn=clip((degrees-config_.straight_cone_deg)/
                           (config_.turn_full_deg-config_.straight_cone_deg),0.,1.);
    const int outer=std::lround(config_.max_duty*radius*1000);
    const int inner=std::lround(outer*(1.-(1.-config_.turn_inner_ratio)*turn));
    left=travel_sign_*(steer>=0?outer:inner)/1000.;
    right=travel_sign_*(steer>=0?inner:outer)/1000.;
  } else mix(forward,steer,joystick,left,right);
  if (left == 0 && right == 0) { state_.left = state_.right = 0; return state_; }
  const double delta = std::max(std::abs(left-state_.left),std::abs(right-state_.right));
  const double fraction = delta > 0 ? std::min(1.0,config_.slew_per_second*dt/delta) : 1;
  const double previous_left=state_.left,previous_right=state_.right;
  state_.left += (left-state_.left)*fraction;
  state_.right += (right-state_.right)*fraction;
  if (manual_arc) {
    // MOT1 carries physical wheel duties for sharp arcs and sign transitions.
    // MOT3 has a fixed 50% inner-wheel floor: select it only when the ACTUAL
    // slewed and permille-quantized pair is representable, never from stick angle.
    const int l=std::lround(state_.left*1000),r=std::lround(state_.right*1000);
    state_.left=l/1000.; state_.right=r/1000.;
    const int outer=std::max(std::abs(l),std::abs(r));
    const int inner=std::min(std::abs(l),std::abs(r));
    if (l*r>=0 && inner*2>=outer) {
      const int sign=(l+r)<0?-1:1;
      const bool left_outer=std::abs(l)>=std::abs(r);
      int best_curve=-1,best_error=std::numeric_limits<int>::max(),best_slow=0;
      for(int curve=0;curve<=256;++curve) {
        const int slow=outer-((outer*curve)>>9);
        const int error=std::abs(slow-inner);
        const double candidate_left=sign*(left_outer?outer:slow)/1000.;
        const double candidate_right=sign*(left_outer?slow:outer)/1000.;
        // Do not let representation changes add an unbounded jump around the
        // 50% threshold. At most one permille of integer wire quantization.
        const double max_step=config_.slew_per_second*dt+.001000001;
        if(std::abs(candidate_left-previous_left)>max_step ||
           std::abs(candidate_right-previous_right)>max_step)continue;
        if(error<best_error) { best_curve=curve;best_error=error;best_slow=slow; }
      }
      if(best_curve>=0) {
        state_.arc_mode=true;state_.arc_outer_permille=sign*outer;
        state_.arc_curve_q8=left_outer?best_curve:-best_curve;
        state_.left=sign*(left_outer?outer:best_slow)/1000.;
        state_.right=sign*(left_outer?best_slow:outer)/1000.;
      }
    }
  }
  return state_;
}
}  // namespace robotphone
