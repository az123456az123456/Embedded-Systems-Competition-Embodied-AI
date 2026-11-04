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
      c.lease_ms > 0 && c.lease_ms <= 30000 &&
      c.motion_ttl_ms > 0 && c.motion_ttl_ms <= 350 &&
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
      finite(c.follow_stop_side_m) && c.follow_stop_side_m >= .22 &&
      finite(c.follow_distance_m) && c.follow_distance_m >= .8 && c.follow_distance_m <= 3 &&
      finite(c.follow_distance_deadband_m) && c.follow_distance_deadband_m >= .05 && c.follow_distance_deadband_m <= .2;
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
  pause("configuration_changed");
  if (!validConfig(config)) { state_.reason = "invalid_configuration"; state_.pause_reason=state_.reason; return false; }
  config_ = config; configured_ = true;
  have_vision_ = false; vision_ = {}; tracking_={}; history_.clear();follow_observation_={};
  return true;
}

void Controller::zero(const std::string& reason, bool disarm_flag) {
  state_.left = state_.right = state_.heading_error_deg = 0;
  state_.twist_mode=false;state_.linear_mps=state_.angular_rps=0;
  state_.arc_mode = false; state_.arc_outer_permille = state_.arc_curve_q8 = 0;
  raw_wheels_ = false; travel_sign_ = 1;
  state_.angle_mode = false; state_.turn_angle_deg = state_.follow_forward = 0;
  if(disarm_flag||reason=="target_lost"||reason=="target_reached")state_.target_id = 0;
  throttle_ = steer_ = 0;
  hold_requested_ = recapture_heading_ = false;
  have_lease_ = false;
  if (disarm_flag) { state_.armed = false; state_.authorization_ms = 0; }
  state_.motion_paused=false; state_.pause_reason.clear();
  state_.mode = state_.armed ? Mode::Idle : Mode::Disarmed;
  state_.reason = reason;
}

void Controller::waitFollow(const std::string& reason) {
  state_.left=state_.right=state_.follow_forward=state_.turn_angle_deg=0;
  state_.angle_mode=state_.arc_mode=state_.twist_mode=false;
  state_.linear_mps=state_.angular_rps=0;
  state_.reason=reason;
}

void Controller::pause(const std::string& reason) {
  zero(reason,false); state_.motion_paused=state_.armed; state_.pause_reason=reason;
}

bool Controller::renewAuthorization(std::uint64_t owner_time, std::uint64_t now) {
  if(owner_time==now && command_source_ms_!=UINT64_MAX)owner_time=command_source_ms_;
  if (!state_.armed || owner_time>now || now-owner_time>=config_.lease_ms) return false;
  if (state_.authorization_ms && (now<state_.authorization_ms || now-state_.authorization_ms>=config_.lease_ms)) {
    zero("command_timeout_30s",true);return false;
  }
  if (owner_time>state_.authorization_ms) state_.authorization_ms=owner_time;
  return true;
}

bool Controller::idle(std::uint64_t seq, std::uint64_t now, std::uint64_t owner_time) {
  if (!lease(seq,now,owner_time==UINT64_MAX?now:owner_time)) return false;
  zero("idle",false);return true;
}

bool Controller::lease(std::uint64_t seq,std::uint64_t now,std::uint64_t owner_time) {
  if(!state_.armed || seq<=state_.last_control_seq)return false;
  if(!renewAuthorization(owner_time,now))return false;
  return acceptSequence(seq);
}

void Controller::refresh(std::uint64_t now) {
  state_.motor_link_fresh = have_motor_ && motor_ready_ && fresh(now,motor_time_,config_.motor_ack_ms);
  state_.imu_fresh = have_imu_ && imu_.calibrated && fresh(now,imu_.time_ms,config_.imu_fresh_ms);
  state_.vision_fresh = have_vision_ && fresh(now,vision_.time_ms,config_.vision_fresh_ms);
}

void Controller::updateMotorLink(std::uint64_t time, bool ready) {
  if (have_motor_ && time < motor_time_) return;
  have_motor_ = true; motor_ready_ = ready; motor_time_ = time;
  if (!ready && state_.armed) pause("motor_link_not_ready");
}

void Controller::updateArcProtocol(bool ready) {
  arc_ready_ = ready;
  if (!ready && !raw_wheels_ &&
      (state_.mode == Mode::Manual || state_.mode == Mode::HeadingHold))
    pause("arc_protocol_lost");
}

void Controller::updateVelocityProtocol(bool supported,bool ready){
  velocity_supported_=supported;velocity_ready_=ready;
  if(state_.twist_mode&&(!supported||(!ready&&(state_.linear_mps!=0||state_.angular_rps!=0)))){
    if(state_.mode==Mode::Follow&&supported)waitFollow("follow_wait_motor");
    else pause(supported?"softcore_not_ready":"velocity_protocol_lost");
  }
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
  pause("imu_reference_changed");
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

bool Controller::selectedImage(std::uint64_t now,double& error,double& delta) const {
  error=delta=0;const auto*t=currentTarget();
  if(!t||!config_.follow_stereo_view||t->eye<0||t->eye>1||
      !fresh(now,vision_.time_ms,250)||!have_imu_||!imu_.calibrated||
      !fresh(now,imu_.time_ms,config_.imu_fresh_ms))return false;
  const double origin=t->eye==1?config_.display_width/2:0;
  double u=0,v=0,capture=0;std::string reason;
  if(!cameraPixel(vision_.geometry,((t->x1+t->x2)*.5-origin)/(config_.display_width/2),
      (t->y1+t->y2)*.5/config_.display_height,u,v,reason)||!yawAt(vision_.time_ms,capture))return false;
  error=(u-480.)/480.;delta=wrap(imu_.yaw_deg-capture);
  return finite(error)&&std::abs(error)<=1&&finite(delta)&&std::abs(delta)<=8;
}

bool Controller::selectedRays(std::uint64_t now,double& left,double& right) const {
  const auto b=selectedBearing(now);const auto*t=currentTarget();
  if(!b.valid||!t)return false;
  const double origin=t->eye==1?config_.display_width/2:0;
  double ul=0,ur=0,v=0;std::string error;
  const double nv=(t->y1+t->y2)*.5/config_.display_height;
  if(!cameraPixel(vision_.geometry,(t->x1-origin)/(config_.display_width/2),nv,ul,v,error)||
     !cameraPixel(vision_.geometry,(t->x2-origin)/(config_.display_width/2),nv,ur,v,error))return false;
  const auto&e=config_.bearing.eyes[t->eye];double capture=0;
  if(!yawAt(vision_.time_ms,capture))return false;
  const double delta=wrap(imu_.yaw_deg-capture);
  left=std::atan((ul-e.cx_px)/e.fx_px)*180/M_PI+e.yaw_right_deg+delta;
  right=std::atan((ur-e.cx_px)/e.fx_px)*180/M_PI+e.yaw_right_deg+delta;
  return finite(left)&&finite(right)&&left<right&&left>-85&&right<85;
}

bool Controller::updateFollowObservation(const FollowObservation&o,std::uint64_t now) {
  if(!state_.target_id||o.target_id!=state_.target_id||o.selection_ms!=state_.selection_ms||
     !fresh(now,o.scan_ms,250)||!fresh(now,o.pose_ms,250)||!fresh(now,o.vision_ms,250)||
     o.scan_ms<follow_observation_.scan_ms||o.vision_ms>vision_.time_ms||
     !fresh(vision_.time_ms,o.vision_ms,250)||
     !finite(o.distance_m)||!finite(o.bearing_right_deg)||!finite(o.x_body_m)||!finite(o.y_body_m)||
     (o.matched&&(o.distance_m<.1||o.distance_m>12||std::abs(o.bearing_right_deg)>=85||
                  std::abs(std::hypot(o.x_body_m,o.y_body_m)-o.distance_m)>.05)))return false;
  follow_observation_=o;return true;
}

bool Controller::followObservationFresh(std::uint64_t now) const {
  const auto&o=follow_observation_;
  return state_.target_id&&o.target_id==state_.target_id&&o.selection_ms==state_.selection_ms&&
      fresh(now,o.scan_ms,250)&&fresh(now,o.pose_ms,250)&&fresh(now,o.vision_ms,250)&&
      vision_.time_ms>=o.vision_ms&&vision_.time_ms-o.vision_ms<=250;
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
  if (have_vision_ && (fresh(frame.time_ms,vision_.time_ms,250)||
      (vision_.boxes.empty()&&fresh(frame.time_ms,tracking_.time_ms,600)))) {
    std::vector<Best> old(tracking_.boxes.size()), current(frame.boxes.size());
    for (std::size_t i=0;i<old.size();++i) for (std::size_t j=0;j<current.size();++j) {
      double score = affinity(tracking_.boxes[i],frame.boxes[j],config_);
      old[i].add(static_cast<int>(j),score); current[j].add(static_cast<int>(i),score);
    }
    for (std::size_t j=0;j<current.size();++j) {
      const auto& best = current[j];
      if (best.unique() && old[best.index].unique() && old[best.index].index == static_cast<int>(j))
        frame.boxes[j].track_id = tracking_.boxes[best.index].track_id;
    }
  }
  for (auto& box: frame.boxes) if (!box.track_id) box.track_id = next_track_id_++;
  // Only bridge brief empty-frame dropouts; ambiguous visible objects get new IDs.
  if(!frame.boxes.empty())tracking_=frame;
  vision_ = std::move(frame); have_vision_ = true;
  history_.push_back(vision_);
  while (!history_.empty() && (history_.size()>32 ||
         !fresh(vision_.time_ms,history_.front().time_ms,500))) history_.pop_front();
  if (state_.target_id && !currentTarget()) {
    if(state_.mode==Mode::Follow)waitFollow("follow_wait_target");
    else zero("target_lost",false);
  }
  return vision_;
}

void Controller::setForeground(bool foreground, std::uint64_t) {
  state_.foreground = foreground;
  if (!foreground) pause("background");
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
  last_tick_ = now; have_tick_ = true; state_.authorization_ms=command_source_ms_==UINT64_MAX?now:command_source_ms_;
  return true;
}

bool Controller::motionReady(std::uint64_t now) {
  refresh(now);
  if (!state_.armed || state_.estop) return false;
  if (now<state_.authorization_ms || now-state_.authorization_ms>=config_.lease_ms) {
    zero("command_timeout_30s",true);return false;
  }
  if (!state_.foreground) { pause("background"); return false; }
  if (!state_.motor_link_fresh) { pause("motor_link_stale"); return false; }
  return true;
}

bool Controller::drive(std::uint64_t seq, double throttle, double steer, bool hold,
                       std::uint64_t now) {
  if (!state_.armed || seq <= state_.last_control_seq) return false;
  if (!finite(throttle) || !finite(steer) || std::abs(throttle)>1 || std::abs(steer)>1) {
    pause("invalid_command"); return false;
  }
  if (!motionReady(now)) {if(state_.armed&&acceptSequence(seq)&&renewAuthorization(now,now))return true;return false;}
  if (throttle == 0 && steer == 0) {
    if (!acceptSequence(seq)) return false;
    zero("joystick_released",false);
    renewAuthorization(now,now);return true;
  }
  if (!arc_ready_) {pause("arc_protocol_waiting");return acceptSequence(seq)&&renewAuthorization(now,now);}
  if (hold && !state_.imu_fresh) {pause("imu_stale");return acceptSequence(seq)&&renewAuthorization(now,now);}
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
  renewAuthorization(now,now);state_.motion_paused=false;state_.pause_reason.clear();
  return true;
}

bool Controller::driveVelocity(std::uint64_t seq,double linear,double angular,std::uint64_t now,std::uint64_t owner_time){
  if(!motionReady(now)||seq<=state_.last_control_seq)return false;
  if(!finite(linear)||!finite(angular)||std::abs(linear)>.30||std::abs(angular)>1.){
    pause("invalid_navigation_velocity");return false;
  }
  if(!velocity_supported_){pause("fpga_v28_velocity_required");return false;}
  if((linear!=0||angular!=0)&&!velocity_ready_){pause("softcore_not_ready");return false;}
  if(!acceptSequence(seq))return false;
  state_.left=state_.right=0;state_.heading_error_deg=0;state_.target_id=0;
  state_.arc_mode=state_.angle_mode=false;state_.arc_outer_permille=state_.arc_curve_q8=0;
  state_.turn_angle_deg=state_.follow_forward=0;
  raw_wheels_=false;hold_requested_=recapture_heading_=false;
  state_.twist_mode=true;state_.linear_mps=linear;state_.angular_rps=angular;
  state_.mode=Mode::Navigation;state_.reason="navigation_velocity";
  lease_time_=now;have_lease_=true;
  if(!renewAuthorization(owner_time==UINT64_MAX?now:owner_time,now)){pause("owner_authorization_expired");return false;}
  state_.motion_paused=false;state_.pause_reason.clear();return true;
}

bool Controller::driveWheels(std::uint64_t seq,double left,double right,std::uint64_t now) {
  if (!state_.armed || seq <= state_.last_control_seq) return false;
  if (!finite(left) || !finite(right) || std::abs(left)>config_.max_duty || std::abs(right)>config_.max_duty) {
    pause("invalid_navigation_duty"); return false;
  }
  if(!motionReady(now)){if(state_.armed&&acceptSequence(seq)&&renewAuthorization(now,now))return true;return false;}
  if (!acceptSequence(seq)) return false;
  if (left==0 && right==0) { zero("navigation_zero",false); renewAuthorization(now,now); return true; }
  state_.twist_mode=false;state_.linear_mps=state_.angular_rps=0;
  state_.target_id=0; hold_requested_=recapture_heading_=false;
  raw_wheels_=true; raw_left_=left; raw_right_=right;
  state_.mode=Mode::Manual; state_.reason="navigation_wheels";
  lease_time_=now; have_lease_=true; renewAuthorization(now,now);state_.motion_paused=false;state_.pause_reason.clear();return true;
}

const Detection* Controller::currentTarget() const {
  if (!state_.target_id) return nullptr;
  for (const auto& box: vision_.boxes)
    if (box.track_id == state_.target_id && box.score >= config_.minimum_target_score) return &box;
  return nullptr;
}

bool Controller::rotate(std::uint64_t seq, bool clockwise, double speed, std::uint64_t now) {
  if (!finite(speed) || speed < 0 || speed > 1) { pause("invalid_command"); return false; }
  const double duty = std::min({speed,.35,config_.max_duty});
  if (!driveWheels(seq,(clockwise?1:-1)*duty,(clockwise?-1:1)*duty,now)) return false;
  if (speed > 0 && !state_.motion_paused) {
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
  state_.selection_ms=std::max(now,state_.selection_ms+1);follow_observation_={};
  return true;
}

bool Controller::follow(std::uint64_t seq, std::uint64_t now) {
  if (!motionReady(now) || seq <= state_.last_control_seq) return false;
  if (!state_.imu_fresh) { zero("imu_stale",false); return false; }
  if (state_.mode!=Mode::Follow&&(!state_.vision_fresh || !currentTarget())) { zero("target_lost",false); return false; }
  if (!angle_ready_&&!velocity_supported_) { zero("angle_protocol_waiting",false); return false; }
  const auto bearing=selectedBearing(now);
  double image_error=0,image_delta=0;
  const bool image_mode=!config_.bearing.enabled&&velocity_supported_;
  if (state_.mode!=Mode::Follow&&!(image_mode?selectedImage(now,image_error,image_delta):bearing.valid)) {
    zero(image_mode?"follow_wait_image":bearing.reason,false);return false;
  }
  if (!acceptSequence(seq)) return false;
  hold_requested_ = recapture_heading_ = false;
  state_.twist_mode=false;state_.linear_mps=state_.angular_rps=0;
  throttle_ = steer_ = 0;
  state_.mode = Mode::Follow; state_.reason = "following";
  lease_time_ = now; have_lease_ = true;
  renewAuthorization(now,now);state_.motion_paused=false;state_.pause_reason.clear();
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
  if (have_tick_ && now < last_tick_) { pause("clock_reversed"); return state_; }
  const double dt = have_tick_ ? std::min<std::uint64_t>(now-last_tick_,100)/1000.0 : 0;
  last_tick_ = now; have_tick_ = true;
  if (!state_.armed) return state_;
  if (now<state_.authorization_ms || now-state_.authorization_ms>=config_.lease_ms) {zero("command_timeout_30s",true);return state_;}
  if (!motionReady(now)) return state_;
  if (state_.mode == Mode::Idle) return state_;
  if (!have_lease_ || !fresh(now,lease_time_,config_.motion_ttl_ms)) {
    pause("motion_command_stale"); return state_;
  }
  if(state_.twist_mode&&state_.mode!=Mode::Follow){
    if(!velocity_supported_||(!velocity_ready_&&(state_.linear_mps!=0||state_.angular_rps!=0)))
      pause(velocity_supported_?"softcore_not_ready":"velocity_protocol_lost");
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
    if (!state_.vision_fresh || !target) { waitFollow("follow_wait_target"); return state_; }
    if(!angle_ready_&&!velocity_supported_){zero("angle_protocol_waiting",false);return state_;}
    const auto bearing=selectedBearing(now);
    const bool image_mode=!config_.bearing.enabled;
    double image_error=0,image_delta=0;
    if(image_mode){
      if(!velocity_supported_){zero("velocity_protocol_lost",false);return state_;}
      if(!selectedImage(now,image_error,image_delta)){waitFollow("follow_wait_image");return state_;}
    }else if(!bearing.valid){zero(bearing.reason,false);return state_;}
    if(!fresh(now,ranges_.time_ms,500) || !ranges_.valid[0] || !ranges_.valid[1] || !ranges_.valid[2]){
      waitFollow("follow_wait_sonar");return state_;
    }
    for(int i=0;i<3;i++)if(!finite(ranges_.metres[i]) || ranges_.metres[i]<.03 || ranges_.metres[i]>4.5){
      waitFollow("follow_wait_sonar");return state_;
    }
    const auto& sensed=follow_observation_;
    const bool image_centered=!image_mode||(std::abs(image_error)<=.06&&std::abs(image_delta)<=2);
    const bool can_retreat=image_centered&&velocity_supported_&&followObservationFresh(now)&&sensed.matched&&sensed.reverse_clear&&
        sensed.distance_m<config_.follow_distance_m-config_.follow_distance_deadband_m&&std::abs(sensed.bearing_right_deg)<8;
    if(ranges_.metres[0]<=config_.follow_stop_front_m&&!can_retreat){waitFollow("follow_obstacle_front");return state_;}
    const double steer_right=image_mode?image_error:bearing.turn_deg;
    const double steer_deadband=image_mode?.025:2.;
    if((steer_right < -steer_deadband && ranges_.metres[1]<=config_.follow_stop_side_m) ||
       (steer_right > steer_deadband && ranges_.metres[2]<=config_.follow_stop_side_m)){
      waitFollow("follow_obstacle_side");return state_;
    }
    if(!followObservationFresh(now)){waitFollow("follow_wait_lidar");return state_;}
    const auto&o=follow_observation_;
    if(image_mode){
      // Pixel servo needs no focal length. The lidar observer only ranges a
      // centered target against a unique narrow frontal cluster; no pixel ray
      // or stereo depth is invented. Translation stays zero while centering.
      if(!velocity_ready_){waitFollow("follow_wait_motor");return state_;}
      if(std::abs(image_error)>.025&&!o.turn_clear){waitFollow("follow_obstacle_side");return state_;}
      state_.twist_mode=true;state_.left=state_.right=0;
      state_.angular_rps=std::abs(image_error)<=.025?0:clip(-image_error*.8,-.35,.35);
      state_.linear_mps=0;state_.reason=image_centered?"follow_wait_range":"follow_aligning";
      if(image_centered&&o.matched){
        double capture=0;
        if(!yawAt(o.pose_ms,capture)){waitFollow("follow_wait_lidar");return state_;}
        const double lidar_angle=wrap(o.bearing_right_deg+wrap(imu_.yaw_deg-capture));
        // Require both camera centering and a frontal lidar hypothesis before
        // approaching. A shifted/clipped target cannot keep old range authority.
        if(std::abs(lidar_angle)>8){state_.reason="follow_wait_range";return state_;}
        const double error=o.distance_m-config_.follow_distance_m;
        if(std::abs(image_error)<=.025&&o.turn_clear&&std::abs(lidar_angle)>2)
          state_.angular_rps=clip(-lidar_angle*.02,-.16,.16);
        if(error>config_.follow_distance_deadband_m){
          state_.linear_mps=o.forward_clear?std::min(.16,(error-config_.follow_distance_deadband_m)*.25):0;
          state_.linear_mps*=clip((ranges_.metres[0]-config_.follow_stop_front_m)/.50,0.,1.);
          state_.reason=o.forward_clear?"following":"follow_obstacle_front";
        }else if(error < -config_.follow_distance_deadband_m){
          state_.linear_mps=o.reverse_clear?-std::min(.08,(-error-config_.follow_distance_deadband_m)*.25):0;
          state_.reason=o.reverse_clear?"follow_backing":"follow_obstacle_rear";
        }else state_.reason="follow_holding_distance";
      }
      return state_;
    }
    // Vision can center before unique ranging, but needs fresh scan clearance.
    double observation_yaw=0;
    if(o.matched&&!yawAt(o.pose_ms,observation_yaw)){waitFollow("follow_wait_lidar");return state_;}
    const double angle=o.matched?wrap(o.bearing_right_deg+wrap(imu_.yaw_deg-observation_yaw)):bearing.turn_deg;
    if(std::abs(angle)>2&&!o.turn_clear){waitFollow("follow_obstacle_side");return state_;}
    // MOT2 transmits the angle itself. FPGA owns angle->differential PWM,
    // output slew, reversal dead-time and its independent command watchdog.
    state_.angle_mode=true;
    state_.turn_angle_deg=std::round(clip(angle,-90.,90.)*100)/100;
    const double a=std::abs(state_.turn_angle_deg);
    forward=std::min(.22,config_.follow_max_forward)*(a<=2?1:std::max(0.,1-(a-2)/(config_.follow_window_deg-2)));
    const double distance_error=o.distance_m-config_.follow_distance_m;
    forward*=o.matched&&o.forward_clear&&distance_error>config_.follow_distance_deadband_m?
        clip((distance_error-config_.follow_distance_deadband_m)/.70,0.,1.):0;
    forward*=clip((ranges_.metres[0]-config_.follow_stop_front_m)/.50,0.,1.);
    state_.reason=!o.matched?"follow_wait_range":!o.forward_clear?"follow_obstacle_front":
        distance_error<=config_.follow_distance_deadband_m?"follow_holding_distance":
        a>=config_.follow_window_deg?"follow_aligning":"following";
    state_.follow_forward=std::round(forward*1000)/1000;
    // Diagnostic requested outputs, not an applied-output/angle measurement.
    const int cdeg=std::lround(state_.turn_angle_deg*100);
    const int correction=std::abs(cdeg)<=200?0:std::max(-220,std::min(220,cdeg/25));
    state_.left=state_.follow_forward+correction/1000.;
    state_.right=state_.follow_forward-correction/1000.;
    if(velocity_supported_){
      if(!velocity_ready_){waitFollow("follow_wait_motor");return state_;}
      state_.angle_mode=false;state_.twist_mode=true;
      state_.angular_rps=std::abs(angle)<=2||!o.turn_clear?0:clip(-angle*.025,-.55,.55);
      state_.linear_mps=state_.follow_forward;
      if(o.matched&&distance_error < -config_.follow_distance_deadband_m){
        state_.linear_mps=o.reverse_clear&&a<8?
            -std::min(.10,(-distance_error-config_.follow_distance_deadband_m)*.25):0;
        state_.reason=state_.linear_mps<0?"follow_backing":o.reverse_clear?
            "follow_aligning":"follow_obstacle_rear";
      }
      // These fields describe legacy wheel duty only; physical-unit output is
      // reported by linear_mps/angular_rps and confirmed by the FPGA ACK.
      state_.left=state_.right=0;
    }
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
    if (!arc_ready_) { pause("arc_protocol_lost"); return state_; }
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
