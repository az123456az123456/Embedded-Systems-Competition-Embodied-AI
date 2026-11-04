#ifndef ROBOT_PHONE_CONTROL_AUDIT_H
#define ROBOT_PHONE_CONTROL_AUDIT_H
#include "controller.h"
#include "motor_link.h"
#include "json_flat.h"
#include "../../common/wrong_log.h"
#include <cmath>
#include <sstream>
#include <string_view>
namespace robotphone {
struct ClientReportedDiagnostic {
 bool present=false;
 std::string reason;
 int64_t mono_ms=-1,state_age_ms=-1,lease_age_ms=-1,id=-1;
};
/* This observer never changes controller state, performs socket/file I/O, or
 * records request bodies, bearer credentials, session IDs or claim tickets. */
class ControlAudit {
 bool seen_=false,armed_=false;
 std::string reason_;
 uint64_t snapshot_ms_=0;
 uint64_t command_snapshot_ms_=0;
 std::string command_op_;
 static double finite(double n){return std::isfinite(n)?n:0.;}
 static bool expected(const std::string &r){
  return r=="idle"||r=="disarmed"||r=="ready"||r=="manual"||r=="navigation_wheels"||
   r=="navigation_velocity"||r=="navigation_zero"||r=="joystick_released"||r=="target_selected"||
   r=="following"||r=="heading_hold"||r=="rotate_cw"||r=="rotate_ccw"||
   r=="target_reached"||r=="stopped"||r=="emergency_stop"||r=="configuration_changed"||
   r=="follow_holding_distance"||r=="follow_aligning"||r=="follow_backing"||r=="follow_wait_range"||r=="follow_wait_image";
 }
public:
 static ClientReportedDiagnostic clientReported(const JsonObject &input) noexcept {
  try {
  ClientReportedDiagnostic d;std::string reason;
  if(jsonString(input,"client_reason",reason)){
   static const char *const allowed[]={"explicit_stop","navigation_end","navigation_rejected","imu_calibrate","imu_zero","app_background","window_focus_lost","activity_pause","connection_closed","touch_cancel","rotation_cancel","joystick_release","rotation_release","arm_intent_invalidated","control_rejected","arm_configuration_failed","telemetry_timeout","session_rejected","control_ack_timeout","connection_failure","stop_request","settings_open","capture_folder_picker","arm_toggle_off","imu_calibration_dialog"};
   for(const auto *v:allowed)if(reason==v){d.reason=v;d.present=true;}
  }
  if(!d.present)return d;
  const char *const keys[]={"client_mono_ms","client_state_age_ms","client_lease_age_ms","client_diag_id"};
  int64_t *const outputs[]={&d.mono_ms,&d.state_age_ms,&d.lease_age_ms,&d.id};
  for(unsigned i=0;i<4;i++){double n=0;if(jsonNumber(input,keys[i],n)&&std::isfinite(n)&&(n>=0||((i==1||i==2)&&n==-1))&&n<=9007199254740991.&&std::floor(n)==n)*outputs[i]=static_cast<int64_t>(n);}
  return d;
  } catch(...) { return {}; }
 }
 static const char *operation(std::string_view op) noexcept {
  static const char *const allowed[]={"arm","drive","rotate","idle","select","follow","stop","disarm","zero","calibrate","configure","claim","twist","wheels","connect","disconnect","pause","lease"};
  for(const auto *name:allowed)if(op==name)return name;
  return "unknown";
 }
private:
 static std::string stateJson(uint64_t now,const State &s,const MotorDiagnostics &d,const MotorAck &a,
                             unsigned left_mask,unsigned invert_mask,uint64_t owner_ms,uint64_t request_seq,
                             const SoftcoreMonitor *softcore,bool softcore_ready){
  std::ostringstream o;o.precision(10);
  o<<"{\"sample_mono_ms\":"<<now<<",\"armed\":"<<(s.armed?"true":"false")
   <<",\"logical_armed\":"<<(s.armed?"true":"false")<<",\"motion_paused\":"<<(s.motion_paused?"true":"false")<<",\"pause_reason\":"<<quoteJson(s.pause_reason)<<",\"authorization_mono_ms\":"<<s.authorization_ms<<",\"authorization_age_ms\":"<<(s.armed&&now>=s.authorization_ms?int64_t(now-s.authorization_ms):-1)<<",\"mode\":"<<quoteJson(modeName(s.mode))<<",\"reason\":"<<quoteJson(s.reason)
   <<",\"foreground\":"<<(s.foreground?"true":"false")<<",\"estop\":"<<(s.estop?"true":"false")
   <<",\"link_fresh\":"<<(s.motor_link_fresh?"true":"false")<<",\"imu_fresh\":"<<(s.imu_fresh?"true":"false")
   <<",\"vision_fresh\":"<<(s.vision_fresh?"true":"false")<<",\"left\":"<<finite(s.left)<<",\"right\":"<<finite(s.right)
   <<",\"left_mask\":"<<left_mask<<",\"invert_mask\":"<<invert_mask<<",\"last_control_seq\":"<<s.last_control_seq
   <<",\"request_seq\":"<<request_seq<<",\"owner_age_ms\":"<<(owner_ms&&now>=owner_ms?int64_t(now-owner_ms):-1)
   <<",\"target_id\":"<<s.target_id<<",\"angle_mode\":"<<(s.angle_mode?"true":"false")
   <<",\"turn_angle_deg\":"<<finite(s.turn_angle_deg)<<",\"forward_duty\":"<<finite(s.follow_forward)
   <<",\"twist_mode\":"<<(s.twist_mode?"true":"false")<<",\"linear_mps\":"<<finite(s.linear_mps)<<",\"angular_rps\":"<<finite(s.angular_rps)
   <<",\"arc_mode\":"<<(s.arc_mode?"true":"false")<<",\"arc_outer_permille\":"<<s.arc_outer_permille<<",\"arc_curve_q8\":"<<s.arc_curve_q8
   <<",\"ack\":{\"sequence\":"<<a.sequence<<",\"age_ms\":"<<(d.last_ack_ms&&now>=d.last_ack_ms?int64_t(now-d.last_ack_ms):-1)
   <<",\"armed\":"<<(a.armed?"true":"false")<<",\"configured\":"<<(a.configured?"true":"false")
   <<",\"watchdog\":"<<(a.watchdog?"true":"false")<<",\"left\":"<<a.left<<",\"right\":"<<a.right
   <<",\"left_mask\":"<<a.left_mask<<",\"invert_mask\":"<<a.invert_mask<<",\"protocol\":"<<a.protocol<<",\"softcore_ready\":"<<(a.softcore_ready?"true":"false")<<"}"
   <<",\"tx\":"<<d.tx_frames<<",\"tx_errors\":"<<d.tx_errors<<",\"rx\":"<<d.rx_frames<<",\"ack_ok\":"<<d.ack_frames
   <<",\"rx_rejected\":"<<d.rx_rejected<<",\"errno\":"<<d.last_errno;
  if(softcore)o<<",\"softcore_diagnostics\":"<<softcore->json(now,softcore_ready,s.armed&&s.twist_mode);
  o<<"}";
  return o.str();
 }
public:
 void observe(uint64_t now,const State &s,const MotorDiagnostics &d,const MotorAck &a,
              unsigned left_mask,unsigned invert_mask,uint64_t owner_ms,uint64_t request_seq,
              const SoftcoreMonitor *softcore=nullptr,bool softcore_ready=false) noexcept {
  try {
  const bool changed=!seen_||armed_!=s.armed||reason_!=s.reason;
  if(changed||!snapshot_ms_||now<snapshot_ms_||now-snapshot_ms_>=100){
   auto json=stateJson(now,s,d,a,left_mask,invert_mask,owner_ms,request_seq,softcore,softcore_ready);
   wrong_log_snapshot("motor_control",json.c_str());snapshot_ms_=now;
   if(seen_&&changed&&!expected(s.reason))wrong_log_event("motor_control",s.reason.c_str(),json.c_str());
  }
  seen_=true;armed_=s.armed;reason_=s.reason;
  } catch(...) { /* Diagnostics must never interrupt the motor control loop. */ }
 }
 void command(uint64_t now,std::string_view op,uint64_t sequence,int status,
              std::string_view cause,bool before_armed,const State &after,bool internal=false,const ClientReportedDiagnostic &client={}) noexcept {
  try {
  const char *safe_op=operation(op);
  const std::string cause_text(cause);
  std::ostringstream o;
  o<<"{\"sample_mono_ms\":"<<now<<",\"op\":"<<quoteJson(safe_op)<<",\"sequence\":"<<sequence
   <<",\"http_status\":"<<status<<",\"origin\":\""<<(internal?"navigation_loopback":"phone_gateway")<<"\""
   <<",\"last_control_seq\":"<<after.last_control_seq
   <<",\"before_armed\":"<<(before_armed?"true":"false")<<",\"after_armed\":"<<(after.armed?"true":"false")
   <<",\"controller_reason\":"<<quoteJson(after.reason)<<",\"cause\":"<<quoteJson(cause_text)
   <<",\"category\":\""<<(status>=400?"request_rejected":op=="stop"||op=="disarm"?"explicit_stop":"control_request")<<"\"";
  if(client.present)o<<",\"client_reported_unverified\":{\"reason\":"<<quoteJson(client.reason)<<",\"mono_ms\":"<<(client.mono_ms>=0?std::to_string(client.mono_ms):"null")<<",\"state_age_ms\":"<<(client.state_age_ms>=0?std::to_string(client.state_age_ms):"null")<<",\"lease_age_ms\":"<<(client.lease_age_ms>=0?std::to_string(client.lease_age_ms):"null")<<",\"diag_id\":"<<(client.id>=0?std::to_string(client.id):"null")<<"}";
  o<<"}";
  auto json=o.str();
  if(status>=400||command_op_!=safe_op||before_armed!=after.armed||!command_snapshot_ms_||now<command_snapshot_ms_||now-command_snapshot_ms_>=100){wrong_log_snapshot("control_request",json.c_str());command_snapshot_ms_=now;command_op_=safe_op;}
  if(status>=400)wrong_log_event("control_request",cause_text.empty()?"request_rejected":cause_text.c_str(),json.c_str());
  else if(op=="stop"||op=="disarm"||op=="zero"||op=="calibrate")wrong_log_event("control_request","explicit_control_stop",json.c_str());
  else if(op=="arm"||op=="claim"||op=="connect")wrong_log_event("control_request","control_transition",json.c_str());
  } catch(...) { /* Keep the original command result when logging fails. */ }
 }
};
}
#endif
