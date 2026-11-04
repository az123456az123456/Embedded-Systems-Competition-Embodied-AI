#include "navcore.h"
#include "vehicle_geometry.h"
#include "protocol.h"
#include "gateway.h"
#include "map_snapshot.h"
#include "sensor_bridge.h"
#include "sonar_diagnostics.h"
#include "vendor/picojson.h"
#include "../common/wrong_log.h"
#include "../common/sonar_range.h"
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <thread>
#include <unistd.h>

using robot_nav::Navigator;
using robot_nav::Config;
static volatile sig_atomic_t interrupted=0;
static void signalHandler(int signal){interrupted=signal;}
static constexpr double PI=3.14159265358979323846;
static std::string quoted(const std::string&s){return picojson::value(s).serialize();}
static picojson::value number_or_null(double n){return std::isfinite(n)?picojson::value(n):picojson::value();}
static void navigation_log_format_error() noexcept {wrong_log_event("logger","navigation_format_error","{}");}
static void navigation_log_exit(int signal) noexcept {
    try {picojson::object event;event["signal_or_console_quit"]=picojson::value(double(signal));wrong_log_event("navigation","process_exit",picojson::value(event).serialize().c_str());}
    catch(...){navigation_log_format_error();}
}
static void navigation_log_exception(const char* message) noexcept {
    try {picojson::object failure;failure["message"]=picojson::value(std::string(message));wrong_log_event("navigation","process_exception",picojson::value(failure).serialize().c_str());}
    catch(...){navigation_log_format_error();}
}
static std::string localization_json(const robot_nav::LocalizationDiagnostics& d){
    picojson::object o;
    o["latched"]=picojson::value(d.latched);o["first_reason"]=picojson::value(d.first_reason);
    o["detected_at_s"]=number_or_null(d.timestamp_s);o["sample_timestamp_s"]=number_or_null(d.sample_timestamp_s);
    o["previous_timestamp_s"]=number_or_null(d.previous_timestamp_s);o["dt_s"]=number_or_null(d.dt_s);
    o["host_dt_s"]=number_or_null(d.host_dt_s);o["using_device_time"]=picojson::value(d.using_device_time);
    o["device_timestamp_s"]=d.sample_has_device_time?number_or_null(d.device_timestamp_s):picojson::value();
    o["previous_device_timestamp_s"]=d.previous_has_device_time?number_or_null(d.previous_device_timestamp_s):picojson::value();
    o["dyaw_rad"]=number_or_null(d.dyaw_rad);o["yaw_limit_rad"]=number_or_null(d.yaw_limit_rad);
    o["wheel_limit_m"]=number_or_null(d.wheel_limit_m);
    picojson::array wheels;for(double n:d.wheel_delta_m)wheels.push_back(number_or_null(n));o["wheel_delta_m"]=picojson::value(wheels);
    o["wheel_comparison_window_s"]=number_or_null(d.wheel_comparison_window_s);
    picojson::array speeds;for(double n:d.window_wheel_mps)speeds.push_back(number_or_null(n));
    o["window_wheel_mps"]=picojson::value(speeds);
    o["previous_sequence"]=picojson::value(double(d.previous_sequence));o["sequence"]=picojson::value(double(d.sequence));
    o["encoder_invalid_mask"]=picojson::value(double(d.encoder_invalid_mask));o["jump_wheel_mask"]=picojson::value(double(d.jump_wheel_mask));
    o["imu_valid"]=picojson::value(d.imu_valid);o["yaw_jump"]=picojson::value(d.yaw_jump);
    o["motion_limits_evaluated"]=picojson::value(d.motion_limits_evaluated);o["external_invalidation"]=picojson::value(d.external_invalidation);
    return picojson::value(o).serialize();
}

static std::string local_avoidance_json(const robot_nav::LocalAvoidanceDiagnostics& d){
    picojson::object o;o["active"]=picojson::value(d.active);
    o["original_heading_rad"]=number_or_null(d.original_heading_rad);o["turn_target_rad"]=number_or_null(d.turn_target_rad);
    o["turn_travel_rad"]=number_or_null(d.turn_travel_rad);o["scan_travel_rad"]=number_or_null(d.scan_travel_rad);
    o["departure_distance_m"]=number_or_null(d.departure_distance_m);o["parallel_distance_m"]=number_or_null(d.parallel_distance_m);
    o["departure_target_m"]=picojson::value(.50);o["parallel_target_m"]=picojson::value(0.);
    o["turn_selection"]=picojson::value(std::string("away_from_close_side_else_larger_sonar"));
    o["turn_direction_locked"]=picojson::value(d.departure_side!=0);
    o["forward_clearance_source"]=picojson::value(std::string("fresh_front_dynamic_and_both_flanks_above_032m"));
    o["restart_attempts"]=picojson::value(double(d.restart_attempts));o["departure_side"]=picojson::value(double(d.departure_side));
    return picojson::value(o).serialize();
}
static std::string local_guard_json(const robot_nav::LocalGuardDiagnostics& d){
    picojson::object o;o["evaluated"]=picojson::value(d.evaluated);o["blocked"]=picojson::value(d.blocked);
    o["scope"]=picojson::value(std::string("last_local_sonar_rotation_guard_map_not_authority"));
    o["reason"]=picojson::value(d.reason);o["pure_rotation"]=picojson::value(d.pure_rotation);
    o["sample_mono_ms"]=number_or_null(d.evaluated_s*1000.);o["target_yaw_rad"]=number_or_null(d.target_yaw_rad);
    o["required_radius_m"]=number_or_null(d.required_radius_m);
    o["cell_min_center_distance_m"]=number_or_null(d.cell_min_center_distance_m);
    o["padding_outside_circle_skipped"]=picojson::value(double(d.padding_outside_circle_skipped));
    o["cell_x"]=picojson::value(double(d.cell_x));o["cell_y"]=picojson::value(double(d.cell_y));o["cell_state"]=picojson::value(double(d.cell_state));
    picojson::array low,high;low.emplace_back(d.cell_min_world.first);low.emplace_back(d.cell_min_world.second);
    high.emplace_back(d.cell_max_world.first);high.emplace_back(d.cell_max_world.second);
    o["cell_min_world_m"]=picojson::value(low);o["cell_max_world_m"]=picojson::value(high);
    o["channel"]=picojson::value(double(d.channel));o["range_m"]=number_or_null(d.range_m);
    o["range_required_m"]=number_or_null(d.range_required_m);
    return picojson::value(o).serialize();
}
static std::string motion_gate_json(const robot_nav::MotionGateDiagnostics& d){
    picojson::object o;o["frame_available"]=picojson::value(d.frame_available);o["source_usable"]=picojson::value(d.source_usable);
    o["frame_sequence"]=picojson::value(double(d.frame_sequence));o["frame_age_s"]=number_or_null(d.frame_age_s);
    o["using_live_sonar"]=picojson::value(d.using_live_sonar);
    o["range_sequence"]=picojson::value(double(d.range_sequence));o["range_packet_age_s"]=number_or_null(d.range_packet_age_s);
    o["range_source"]=picojson::value(std::string(d.using_live_sonar?"latest_SNS1_range_before_IMU_alignment":"aligned_range_fallback"));
    picojson::array wheels,ranges,ages;for(double v:d.measured_wheel_mps)wheels.push_back(number_or_null(v));
    for(unsigned i=0;i<3;++i){ranges.push_back(d.sonar_m[i]?number_or_null(*d.sonar_m[i]):picojson::value());ages.push_back(number_or_null(d.total_sonar_age_s[i]));}
    o["measured_wheel_mps"]=picojson::value(wheels);o["sonar_m"]=picojson::value(ranges);o["total_sonar_age_s"]=picojson::value(ages);
    o["wheel_speed_semantics"]=picojson::value(std::string("instantaneous_SNS1_device_delta_over_sample_dt_not_250ms_disagreement_window"));
    o["measured_sample_dt_s"]=number_or_null(d.measured_sample_dt_s);o["measured_yaw_rps"]=number_or_null(d.measured_yaw_rps);
    o["safety_speed_mps"]=number_or_null(d.safety_speed_mps);o["reaction_delay_s"]=number_or_null(d.reaction_delay_s);
    o["front_stop_distance_m"]=number_or_null(d.front_stop_distance_m);o["front_resume_distance_m"]=number_or_null(d.front_resume_distance_m);
    o["front_blocked"]=picojson::value(d.front_blocked);o["forward_clear"]=picojson::value(d.forward_clear);
    o["target_linear_mps"]=number_or_null(d.target_linear_mps);o["target_angular_rps"]=number_or_null(d.target_angular_rps);
    o["stop_requested_mono_s"]=d.stop_requested_s?number_or_null(*d.stop_requested_s):picojson::value();
    o["stationary_since_mono_s"]=d.stationary_since_s?number_or_null(*d.stationary_since_s):picojson::value();
    o["stop_wait_elapsed_s"]=number_or_null(d.stop_wait_elapsed_s);o["stationary_elapsed_s"]=number_or_null(d.stationary_elapsed_s);
    o["stop_evidence_window_s"]=number_or_null(d.stop_evidence_window_s);
    o["stop_confirm_window_s"]=number_or_null(d.stop_confirm_window_s);
    o["stop_evidence_samples"]=picojson::value(double(d.stop_evidence_samples));
    o["stop_evidence_reason"]=picojson::value(d.stop_evidence_reason);
    picojson::array stop_wheels;for(double n:d.stop_window_wheel_mps)stop_wheels.push_back(number_or_null(n));
    o["stop_window_wheel_mps"]=picojson::value(stop_wheels);
    o["stop_window_yaw_rps"]=number_or_null(d.stop_window_yaw_rps);
    return picojson::value(o).serialize();
}

// Compiled profile deliberately wins over legacy config geometry. Dynamic,
// measured control limits and the operator calibration flag remain configurable.
static void applyFixedProfile(Config& c){
    c.wheel_circumference_m=vehicle_profile::wheel_circumference_m;
    c.encoder_ticks_per_turn=vehicle_profile::encoder_ticks_per_turn;
    c.encoder_signs=vehicle_profile::encoder_signs;
    c.track_m=vehicle_profile::track_m;
    c.robot_radius_m=vehicle_profile::robot_radius_m;
    c.body_front_m=vehicle_profile::body_front_m;c.body_rear_m=vehicle_profile::body_rear_m;
    c.body_left_m=vehicle_profile::body_left_m;c.body_right_m=vehicle_profile::body_right_m;
    c.wheelbase_m=vehicle_profile::wheelbase_m;
    c.imu_mount_x_m=vehicle_profile::imu_mount_x_m;c.imu_mount_y_m=vehicle_profile::imu_mount_y_m;
    c.sonar_mounts=vehicle_profile::sonar_mounts;c.sonar_half_angle_rad=vehicle_profile::sonar_half_angle_rad;
}
static picojson::value profileValue(const Config& c,int yaw_sign){
    picojson::object o;
    auto number=[&](const std::string& key,double value){o[key]=picojson::value(value);};
    o["fixed_profile"]=picojson::value(std::string(vehicle_profile::id));
    o["geometry_source"]=picojson::value(std::string("user_CAD_nominal_plus_22mm_left_extension"));
    o["geometry_reference"]=picojson::value(std::string("wheel_rectangle_center_x_forward_y_left_yaw_CCW"));
    o["encoder_source"]=picojson::value(std::string("user_fixed_60527.7_x4_counts_per_revolution"));
    o["encoder_signal_order"]=picojson::value(std::string("M1,M2,M3,M4"));
    o["wheel_mapping"]=picojson::value(std::string("phone_verified_live_FPGA_ack"));
    o["sonar_origins_measured"]=picojson::value(false);
    o["effective_turn_track_measured"]=picojson::value(false);
    number("wheel_diameter_m",vehicle_profile::wheel_diameter_m);
    number("wheel_circumference_m",c.wheel_circumference_m);
    number("encoder_ticks_per_turn",c.encoder_ticks_per_turn);
    number("track_m",c.track_m);number("wheelbase_m",c.wheelbase_m);
    number("robot_radius_m",c.robot_radius_m);
    number("body_front_m",c.body_front_m);number("body_rear_m",c.body_rear_m);
    number("body_left_m",c.body_left_m);number("body_right_m",c.body_right_m);
    number("body_width_m",c.body_left_m+c.body_right_m);
    number("body_length_m",c.body_front_m+c.body_rear_m);
    number("body_height_m",vehicle_profile::body_height_m);
    number("imu_mount_x_m",c.imu_mount_x_m);number("imu_mount_y_m",c.imu_mount_y_m);
    number("sonar_half_angle_deg",c.sonar_half_angle_rad*180/PI);
    picojson::array signs,mounts;
    for(int sign:c.encoder_signs)signs.emplace_back(double(sign));
    for(const auto& mount:c.sonar_mounts){picojson::array m;for(int i=0;i<3;++i)m.emplace_back(mount[i]*(i==2?180/PI:1));mounts.emplace_back(m);}
    o["encoder_signs"]=picojson::value(signs);o["sonar_mounts"]=picojson::value(mounts);
    picojson::object dynamic;
    dynamic["calibration_confirmed"]=picojson::value(c.configured);
    dynamic["drive_mps_per_full_pwm"]=picojson::value(c.drive_mps_per_full_pwm);
    dynamic["imu_yaw_sign"]=picojson::value(double(yaw_sign));
    dynamic["brake_mps2"]=picojson::value(c.brake_mps2);
    dynamic["reaction_s"]=picojson::value(c.reaction_s);
    dynamic["sonar_latency_s"]=picojson::value(c.sonar_latency_s);
    dynamic["stop_clearance_m"]=picojson::value(c.stop_clearance_m);
    dynamic["preserved_config_stop_clearance_m"]=picojson::value(c.preserved_config_stop_clearance_m);
    dynamic["side_clearance_m"]=picojson::value(c.side_clearance_m);
    dynamic["avoidance_step_deg"]=picojson::value(c.avoidance_step_rad*180/PI);
    o["dynamic_control"]=picojson::value(dynamic);
    return picojson::value(o);
}
static std::string runtime_log_config(const Config& c,int yaw_sign,bool drive) noexcept {
    try {
    auto value=profileValue(c,yaw_sign);auto& root=value.get<picojson::object>();
    root["drive_allowed"]=picojson::value(drive);picojson::object controls;
    const std::pair<const char*,double> fields[]={
        {"stop_base_runtime_override_m",c.stop_clearance_m},{"sonar_wait_timeout_s",c.sonar_wait_timeout_s},{"avoid_clear_front_m",c.avoid_clear_front_m},{"avoid_rejoin_distance_m",c.avoid_rejoin_distance_m},{"max_pwm",c.max_pwm},{"cruise_mps",c.cruise_mps},{"detour_mps",c.detour_mps},{"max_wheel_mps",c.max_wheel_mps},{"max_turn_rps",c.max_turn_rps},
        {"straight_speed_mps",c.straight_speed_mps},{"straight_max_pwm",c.straight_max_pwm},{"straight_replan_max_s",c.straight_replan_max_s},
        {"sensor_timeout_s",c.sensor_timeout_s},{"sonar_timeout_s",c.sonar_timeout_s},{"planner_timeout_s",c.planner_timeout_s},{"map_ttl_s",c.map_ttl_s},
        {"resolution_m",c.resolution_m},{"safety_margin_m",c.safety_margin_m},{"sonar_min_m",c.sonar_min_m},{"sonar_max_m",c.sonar_max_m},
        {"accel_mps2",c.accel_mps2},{"brake_mps2",c.brake_mps2},{"reaction_s",c.reaction_s},{"sonar_latency_s",c.sonar_latency_s},
        {"stop_dwell_s",c.stop_dwell_s},{"scan_observation_dwell_s",c.scan_observation_dwell_s},{"scan_max_s",c.scan_max_s},
        {"encoder_mismatch_ratio",c.encoder_mismatch_ratio},{"encoder_fault_after_s",c.encoder_fault_after_s},{"encoder_compare_window_s",c.encoder_compare_window_s},{"encoder_min_compare_mps",c.encoder_min_compare_mps},
        {"bootstrap_speed_mps",c.bootstrap_speed_mps},{"bootstrap_step_m",c.bootstrap_step_m},{"bootstrap_max_distance_m",c.bootstrap_max_distance_m},{"bootstrap_max_s",c.bootstrap_max_s},
        {"max_measured_wheel_mps",c.max_measured_wheel_mps},{"max_measured_turn_rps",c.max_measured_turn_rps},{"no_progress_s",c.no_progress_s}};
    for(const auto& field:fields)controls[field.first]=number_or_null(field.second);
    root["runtime_limits"]=picojson::value(controls);return value.serialize();
    }catch(...){navigation_log_format_error();return {};}
}

struct Settings {Config config;int yaw_sign=1;};
static Settings readSettings(const std::string&file){
    std::ifstream in(file);if(!in)throw std::runtime_error("cannot read config: "+file);
    std::string bytes((std::istreambuf_iterator<char>(in)),{});picojson::value root;
    auto it=bytes.begin();auto error=picojson::parse(root,it,bytes.end());
    while(it!=bytes.end()&&std::isspace(static_cast<unsigned char>(*it)))++it;
    if(!error.empty()||it!=bytes.end()||!root.is<picojson::object>())throw std::runtime_error("invalid config JSON");
    Settings s;auto&o=root.get<picojson::object>();
    auto num=[&](const std::string&k,double&v){auto f=o.find(k);if(f==o.end())return;if(!f->second.is<double>()||!std::isfinite(f->second.get<double>()))throw std::runtime_error("invalid config: "+k);v=f->second.get<double>();};
    s.config.velocity_control=true; // v28 runtime requires ACK4; do not rewrite operator calibration.
    auto flag=o.find("calibration_confirmed");if(flag==o.end()||!flag->second.is<bool>())throw std::runtime_error("calibration_confirmed must be boolean");s.config.configured=flag->second.get<bool>();
    auto&c=s.config;
    applyFixedProfile(c);
    num("drive_mps_per_full_pwm",c.drive_mps_per_full_pwm);
    double sign=1;num("imu_yaw_sign",sign);if(sign!=1&&sign!=-1)throw std::runtime_error("imu_yaw_sign must be 1 or -1");s.yaw_sign=int(sign);
    num("resolution_m",c.resolution_m);num("safety_margin_m",c.safety_margin_m);num("max_pwm",c.max_pwm);num("cruise_mps",c.cruise_mps);num("detour_mps",c.detour_mps);num("max_wheel_mps",c.max_wheel_mps);
    num("straight_speed_mps",c.straight_speed_mps);num("straight_max_pwm",c.straight_max_pwm);num("straight_replan_max_s",c.straight_replan_max_s);
    double saved_stop_clearance=c.stop_clearance_m;num("stop_clearance_m",saved_stop_clearance);
    if(saved_stop_clearance<0||saved_stop_clearance>5)throw std::runtime_error("invalid stop_clearance_m");
    c.preserved_config_stop_clearance_m=saved_stop_clearance;
    c.stop_clearance_m=.30; // Three-beam stop base; retain saved config for rollback.
    num("side_clearance_m",c.side_clearance_m);c.side_clearance_m=std::max(.30,c.side_clearance_m);
    num("detour_accel_distance_m",c.detour_accel_distance_m);num("sonar_timeout_s",c.sonar_timeout_s);num("planner_timeout_s",c.planner_timeout_s);num("map_ttl_s",c.map_ttl_s);
    num("sonar_latency_s",c.sonar_latency_s);
    double verify_turn_deg=c.avoid_verify_turn_min_rad*180./3.14159265358979323846;
    num("avoid_verify_turn_min_deg",verify_turn_deg);c.avoid_verify_turn_min_rad=verify_turn_deg*3.14159265358979323846/180.;
    num("brake_mps2",c.brake_mps2);num("accel_mps2",c.accel_mps2);num("reaction_s",c.reaction_s);num("max_turn_rps",c.max_turn_rps);
    num("sensor_timeout_s",c.sensor_timeout_s);num("rejoin_ahead_m",c.rejoin_ahead_m);
    num("stop_dwell_s",c.stop_dwell_s);num("scan_observation_dwell_s",c.scan_observation_dwell_s);
    num("scan_max_s",c.scan_max_s);num("stopped_wheel_mps",c.stopped_wheel_mps);
    double step=c.avoidance_step_rad*180/PI;num("avoidance_step_deg",step);c.avoidance_step_rad=step*PI/180;
    // Physical MOT4 setpoints do not require PWM-to-speed calibration. The
    // persisted calibration flag/ratio remain unchanged; live ACK4 + phone arm
    // still authorize motion and all geometry/encoder/braking guards remain.
    if(c.configured&&!c.errors().empty())throw std::runtime_error("invalid calibrated config: "+c.errors().front());
    return s;
}

static void migrateConfig(const std::string& input,const std::string& output){
    (void)readSettings(input);
    struct stat target_status{};if(!lstat(output.c_str(),&target_status)||errno!=ENOENT)throw std::runtime_error("migration output must be a new staging path");
    if(input==output)throw std::runtime_error("migration output must be a new staging path");
    std::ifstream in(input);if(!in)throw std::runtime_error("cannot read migration input: "+input);
    std::string bytes((std::istreambuf_iterator<char>(in)),{});picojson::value root;
    auto it=bytes.begin();const auto error=picojson::parse(root,it,bytes.end());
    while(it!=bytes.end()&&std::isspace(static_cast<unsigned char>(*it)))++it;
    if(!error.empty()||it!=bytes.end()||!root.is<picojson::object>())throw std::runtime_error("invalid migration input JSON");
    auto& object=root.get<picojson::object>();auto flag=object.find("calibration_confirmed");
    if(flag==object.end()||!flag->second.is<bool>())throw std::runtime_error("calibration_confirmed must be boolean");
    Config c;applyFixedProfile(c);
    const auto profile=profileValue(c,1).get<picojson::object>();
    for(const auto& item:profile)if(item.first!="dynamic_control"&&item.first!="wheel_mapping")object[item.first]=item.second;
    object["stop_clearance_m"]=picojson::value(.30);
    object["side_clearance_m"]=picojson::value(.30);
    object["avoidance_step_deg"]=picojson::value(25.);
    const auto text=root.serialize(true)+"\n";
    const auto tmp=output+".tmp."+std::to_string(getpid());
    const int fd=open(tmp.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd<0)throw std::runtime_error("cannot stage migrated configuration");
    bool okay=true;size_t written=0;
    while(written<text.size()){const auto n=write(fd,text.data()+written,text.size()-written);if(n<0&&errno==EINTR)continue;if(n<=0){okay=false;break;}written+=size_t(n);}
    if(okay&&fsync(fd))okay=false;
    if(close(fd))okay=false;
    if(!okay||rename(tmp.c_str(),output.c_str())){unlink(tmp.c_str());throw std::runtime_error("cannot commit migrated configuration");}
    std::cout<<"Migrated fixed vehicle profile to "<<output<<"; dynamic calibration and unknown fields preserved.\n";
}

static int checkFixedProfile(const std::string& input){
    (void)readSettings(input);
    std::ifstream in(input);std::string text((std::istreambuf_iterator<char>(in)),{});
    picojson::value root;const auto error=picojson::parse(root,text);
    if(!error.empty()||!root.is<picojson::object>())throw std::runtime_error("invalid profile JSON");
    Config c;applyFixedProfile(c);auto expected=profileValue(c,1).get<picojson::object>();
    expected.erase("dynamic_control");expected.erase("wheel_mapping");
    expected["stop_clearance_m"]=picojson::value(.30);
    expected["side_clearance_m"]=picojson::value(.30);
    expected["avoidance_step_deg"]=picojson::value(25.);
    const auto& actual=root.get<picojson::object>();
    for(const auto& item:expected){auto it=actual.find(item.first);if(it==actual.end()||it->second!=item.second){std::cout<<"Profile mismatch: "<<item.first<<'\n';return 2;}}
    std::cout<<"Fixed vehicle profile verified; dynamic calibration left unchanged.\n";return 0;
}

struct Shared {
    std::mutex mutex;
    Navigator nav;
    robot_nav::NavigationInputs inputs;
    navwire::Sensors sensors;
    bool have_sensors=false, drive=false, reset_required=false, have_wheel_layout=false;
    bool velocity_supported=false,softcore_ready=false;unsigned ack_protocol=0;
    std::string softcore_reason,softcore_diagnostics_json="{}";
    bool preparing=false;uint64_t preparation_started_ms=0;
    uint64_t softcore_state_ms=0;
    uint64_t input_request_barrier_ms=uint64_t(std::ceil(navwire::monotonicSeconds()*1000.));
    unsigned origin_epoch=0;
    double received_s=0;
    std::atomic<bool> cancel_plan{false};
    robot_nav::MapSnapshotCache map_cache;
    const std::string map_session=std::to_string(getpid())+"-"+std::to_string(navwire::monotonicSeconds());
    std::string diagnostic_state,diagnostic_reason;
    bool diagnostic_enabled=false,diagnostic_initialized=false;
    // One bounded first-fault record survives periodic log rotation. Telemetry
    // is explicitly labelled as a cached observation, not an instantaneous command.
    picojson::value localization_fault_context;
    bool needs_localization_reset() const {
        return reset_required||nav.localization_diagnostics().latched;
    }
    void retain_fault_context_locked(const robotnav::GatewayTelemetry& t,
                                    const robot_nav::Control& prior,bool was_enabled,
                                    double now) noexcept {
        try {
            if(!nav.localization_diagnostics().latched){localization_fault_context=picojson::value();return;}
            if(!localization_fault_context.is<picojson::null>())return;
            picojson::object o;
            o["captured_at_s"]=number_or_null(now);
            o["fault_detected_at_s"]=number_or_null(nav.localization_diagnostics().timestamp_s);
            o["navigation_was_enabled"]=picojson::value(was_enabled);
            o["prior_core_state"]=picojson::value(prior.state);
            o["prior_core_reason"]=picojson::value(prior.reason);
            o["prior_core_left_duty"]=number_or_null(prior.left);
            o["prior_core_right_duty"]=number_or_null(prior.right);
            o["backend_semantics"]=picojson::value(std::string("cached_dashboard_state_not_synchronous_motor_measurement"));
            o["backend_received_ms"]=picojson::value(double(t.received_ms));
            o["backend_age_ms"]=t.received_ms?number_or_null(now*1000.-double(t.received_ms)):picojson::value();
            o["backend_available"]=picojson::value(t.available);
            o["backend_armed"]=picojson::value(t.armed);
            o["backend_reason"]=picojson::value(t.reason.substr(0,160));
            o["backend_left_duty"]=number_or_null(t.left_duty);
            o["backend_right_duty"]=number_or_null(t.right_duty);
            o["ack_left_duty"]=number_or_null(t.ack_left_duty);
            o["ack_right_duty"]=number_or_null(t.ack_right_duty);
            o["ack_age_ms"]=number_or_null(t.motor_ack_age_ms);
            o["ack_sequence"]=picojson::value(double(t.motor_ack_sequence));
            o["left_mask"]=picojson::value(double(t.left_mask));
            o["invert_mask"]=picojson::value(double(t.invert_mask));
            localization_fault_context=picojson::value(o);
        }catch(...){navigation_log_format_error();}
    }
    explicit Shared(Config c,bool d,double yaw_sign=1):nav(c),inputs(c,yaw_sign),drive(d){}
    bool input_request_allowed(uint64_t started_ms) const {return started_ms>=input_request_barrier_ms;}
    bool input_sensor_allowed(double received_s) const {
        return std::isfinite(received_s)&&received_s*1000.>=double(input_request_barrier_ms);
    }
    bool synchronize_wheel_layout(const robotnav::GatewayTelemetry& telemetry){
        std::lock_guard<std::mutex> guard(mutex);
        velocity_supported=telemetry.available&&telemetry.link_ok&&telemetry.velocity_protocol_supported;
        softcore_ready=velocity_supported&&telemetry.softcore_ready;ack_protocol=telemetry.motor_ack_protocol;
        softcore_reason=telemetry.softcore_reason;softcore_diagnostics_json=telemetry.softcore_diagnostics_json;
        preparing=telemetry.navigation_preparing;preparation_started_ms=telemetry.preparation_started_ms;
        softcore_state_ms=telemetry.received_ms;
        if(!telemetry.available||!telemetry.link_ok||!std::isfinite(telemetry.motor_ack_age_ms)||
           telemetry.motor_ack_age_ms<0||telemetry.motor_ack_age_ms>300||telemetry.left_mask>15||
           __builtin_popcount(telemetry.left_mask)!=2)return false;
        const bool had_layout=have_wheel_layout;
        const bool changed=nav.config().left_wheel_mask!=telemetry.left_mask;
        if(!nav.set_wheel_layout(telemetry.left_mask))return false;
        have_wheel_layout=true;
        if(!changed&&had_layout)return false;
        cancel_plan=true;have_sensors=false;inputs.reset();localization_fault_context=picojson::value();
        input_request_barrier_ms=uint64_t(std::ceil(navwire::monotonicSeconds()*1000.));++origin_epoch;
        // A new mapping invalidates every old pose/map/route; the first verified
        // layout only establishes a new baseline and does not authorize motion.
        if(changed&&had_layout)reset_required=true;
        std::cout<<"NAV wheel_layout="<<telemetry.left_mask<<" map_reset="<<(had_layout?"required":"initial")<<'\n';
        return true;
    }
    bool go_to(double x,double y,const std::string& session,uint32_t revision,std::string& error){
        std::lock_guard<std::mutex> lock(mutex);
        const auto now=navwire::monotonicSeconds();
        if(!drive){error="observe_only";return false;}
        if(reset_required){error="reset-map_required_after_fpga_reset_or_imu_zero_or_wheel_mapping_change";return false;}
        if(!have_wheel_layout){error="verified_wheel_layout_required";return false;}
        if(session!=map_session+"-"+std::to_string(origin_epoch)){error="map_session_changed";return false;}
        if(!map_cache.validate_goal(session,revision,x,y,now,error))return false;
        if(!nav.go_to(x,y,now,error))return false;
        cancel_plan=true;return true;
    }
    std::string start_problem(double now) const{
        const auto errors=nav.config().errors();
        if(!errors.empty())return errors.front();
        if(!drive)return "observe_only";
        if(reset_required)return "reset-map_required_after_fpga_reset_or_imu_zero_or_wheel_mapping_change";
        if(!have_wheel_layout)return "verified_wheel_layout_required";
        if(!inputs.fault_reason().empty())return inputs.fault_reason();
        if(!nav.frame())return inputs.wait_reason().empty()?"odometry_not_ready":inputs.wait_reason();
        // Starting an exploration session does not command a turn. The core
        // first confirms a stop, then checks each proposed motion. In
        // particular, a known-free disk behind the car is not a prerequisite
        // for a short, forward, range-gated bootstrap step.
        return nav.start_problem(now);
    }
    // Caller owns mutex. This also lets fault records retain the original core
    // reason before a subsequent gateway cancellation changes it to STOPPED.
    std::string softcore_diagnostics_locked(double now)const {
        picojson::value value;if(!picojson::parse(value,softcore_diagnostics_json).empty()||!value.is<picojson::object>())return "{}";
        auto &o=value.get<picojson::object>();
        const auto age=[&](const char*key){auto it=o.find(key);if(it==o.end()||!it->second.is<double>())return -1.;const double stamp=it->second.get<double>();return stamp>0&&now*1000>=stamp?now*1000-stamp:-1.;};
        const double packet_age=age("sample_mono_ms"),beat_age=age("heartbeat_advance_mono_ms");
        o["packet_age_ms"]=packet_age>=0?picojson::value(packet_age):picojson::value();
        o["packet_fresh"]=picojson::value(packet_age>=0&&packet_age<=500);
        o["heartbeat_age_ms"]=beat_age>=0?picojson::value(beat_age):picojson::value();
        o["heartbeat_live"]=picojson::value(beat_age>=0&&beat_age<=3500);
        return value.serialize();
    }
    std::string status_locked(double status_now){
        const auto start_error=start_problem(status_now);
        std::ostringstream out;auto p=nav.pose();out<<std::fixed<<std::setprecision(4);
        out<<"{\"can_start\":"<<(!preparing&&start_error.empty()?"true":"false")<<",\"readiness_reason\":"<<quoted(preparing?"waiting_softcore_calibration":start_error)<<",\"start_mode\":"<<quoted(nav.straight_assist_active()?"straight":nav.goal()?"goal":"explore")<<',';
        out<<"\"straight_assist_active\":"<<(nav.straight_assist_active()?"true":"false")<<',';
        out<<"\"straight_requested_speed_mps\":"<<nav.config().straight_speed_mps
           <<",\"straight_available_speed_mps\":"<<std::min({nav.config().straight_speed_mps,nav.config().max_wheel_mps,(nav.config().velocity_control?nav.config().max_wheel_mps:nav.config().straight_max_pwm*nav.config().drive_mps_per_full_pwm)})<<',';
        out<<"\"avoidance_policy\":{\"version\":\"v33-scan-lease\",\"stop_base_m\":"<<nav.config().stop_clearance_m<<",\"preserved_config_stop_base_m\":"<<nav.config().preserved_config_stop_clearance_m<<",\"side_clearance_m\":"<<nav.config().side_clearance_m<<",\"front_clear_hysteresis_floor_m\":"<<nav.config().avoid_clear_front_m<<",\"navigation_after_clear_distance_m\":"<<nav.config().avoid_rejoin_distance_m<<",\"source_wait_timeout_s\":"<<nav.config().sonar_wait_timeout_s<<",\"turn_selection\":\"away_from_close_side_else_larger_sonar\",\"turn_direction_locked\":true,\"forward_clearance_source\":\"fresh_front_dynamic_and_both_flanks_above_032m\",\"three_beam_stop_m\":0.30,\"side_release_m\":0.32,\"stop_confirm_window_s\":0.08,\"turn_forward_handoff\":\"net_turn_then_two_new_clear_observations_no_stop_dwell\",\"minimum_verified_turn_deg\":"<<nav.config().avoid_verify_turn_min_rad*180./3.14159265358979323846<<",\"latest_sonar_guard\":\"reduce_only_before_imu_alignment\",\"scan_limit_deg\":720,\"scan_motion_budget_s\":120,\"scan_ends_when_front_clear\":true,\"selected_direction_floor_m\":0.20,\"rotation_policy\":\"v32_distance_protection_preserved\",\"local_map_veto\":false,\"distance_source\":\"ultrasonic_only\",\"route_map\":\"only_rejoin_after_050m_during_avoidance\",\"straight_after_clear\":\"continue_new_heading\",\"stop_distance_semantics\":\"base_plus_speed_source_age_reaction_and_braking\"},";
        out<<"\"softcore_control\":{\"protocol\":\"MOT4\",\"supported\":"<<(velocity_supported?"true":"false")<<",\"ready\":"<<(softcore_ready&&status_now*1000>=softcore_state_ms&&status_now*1000-softcore_state_ms<=300?"true":"false")<<",\"ack_protocol\":"<<ack_protocol<<",\"sample_mono_ms\":"<<softcore_state_ms<<",\"reason\":"<<quoted(softcore_reason)<<",\"diagnostics\":"<<softcore_diagnostics_locked(status_now)<<"},";
        out<<"\"preparing\":"<<(preparing?"true":"false")<<",\"preparation_started_ms\":"<<preparation_started_ms<<",\"core_state\":"<<quoted(nav.state())<<",\"state\":"<<quoted(preparing?"PREPARING":nav.state())<<",\"reason\":"<<quoted(preparing?"waiting_softcore_calibration":nav.reason())<<",\"navigation_enabled\":"<<(nav.enabled()?"true":"false")<<",\"drive_allowed\":"<<(drive?"true":"false")<<",\"calibrated\":"<<(nav.config().configured?"true":"false")<<",\"velocity_control\":"<<(nav.config().velocity_control?"true":"false")<<",\"reset_required\":"<<(needs_localization_reset()?"true":"false")<<",\"pose\":["<<p.x<<','<<p.y<<','<<p.yaw<<"],\"sensor_age_s\":"<<(have_sensors?navwire::monotonicSeconds()-received_s:-1)<<",\"counts\":[";
        for(int i=0;i<4;++i){if(i)out<<',';out<<sensors.counts[i];}out<<"],\"sonar_m\":[";
        for(int i=0;i<3;++i){if(i)out<<',';if((sensors.valid&(1<<i))&&!(sensors.fault&(1<<i)))out<<sensors.distance_mm[i]/1000.;else out<<"null";}
        out<<"],\"sonar_age_s\":[";
        for(int i=0;i<3;++i){
            if(i)out<<',';
            if(have_sensors&&sensors.age_ms[i]!=UINT16_MAX)out<<std::max(0.,navwire::monotonicSeconds()-received_s)+sensors.age_ms[i]/1000.;
            else out<<"null";
        }
        out<<"],\"sonar_health\":"<<robot_nav::sonar_diagnostics_json(sensors,have_sensors,received_s,status_now,nav.config())<<",\"sensor_packet\":{\"sequence\":"<<sensors.sequence<<",\"uptime_ms\":"<<sensors.uptime_ms<<",\"valid\":"<<unsigned(sensors.valid)<<",\"fault\":"<<unsigned(sensors.fault)<<",\"encoder_invert\":"<<unsigned(sensors.invert)<<",\"encoder_illegal\":"<<unsigned(sensors.illegal)<<",\"window_ms\":"<<sensors.window_ms<<",\"delta\":[";
        for(unsigned i=0;i<4;++i){if(i)out<<',';out<<sensors.delta[i];}out<<"],\"distance_mm\":[";
        for(unsigned i=0;i<3;++i){if(i)out<<',';out<<sensors.distance_mm[i];}out<<"],\"source_age_ms\":[";
        for(unsigned i=0;i<3;++i){if(i)out<<',';out<<sensors.age_ms[i];}out<<"],\"range_semantics\":\"SNS1_last_accepted_not_raw_UART\"}";
        const auto& command=nav.command();out<<",\"control\":{\"left_duty\":"<<command.left<<",\"right_duty\":"<<command.right<<",\"linear_mps\":"<<command.linear_mps<<",\"angular_rps\":"<<command.angular_rps<<'}';
        if(nav.frame()){
            const auto& frame=*nav.frame();out<<",\"aligned_sample\":{\"sequence\":"<<frame.sequence<<",\"host_timestamp_s\":"<<frame.timestamp_s<<",\"device_timestamp_s\":";
            if(frame.device_timestamp_s)out<<*frame.device_timestamp_s;else out<<"null";
            out<<",\"yaw_rad\":"<<frame.yaw_rad<<",\"imu_valid\":"<<(frame.imu_valid?"true":"false")<<",\"imu_age_s\":"<<frame.imu_age_s<<'}';
        }
        out<<",\"bootstrap_motion_elapsed_s\":"<<nav.bootstrap_motion_elapsed_s()<<",\"bootstrap_motion_budget_s\":"<<nav.config().bootstrap_max_s<<",\"vehicle_profile\":"<<quoted(vehicle_profile::id)<<",\"wheel_layout_verified\":"<<(have_wheel_layout?"true":"false")<<",\"left_wheel_mask\":"<<nav.config().left_wheel_mask<<",\"encoder_ticks_per_turn\":"<<nav.config().encoder_ticks_per_turn<<",\"encoder_signs\":[-1,1,-1,1],\"body_extents_m\":["<<nav.config().body_front_m<<','<<nav.config().body_rear_m<<','<<nav.config().body_left_m<<','<<nav.config().body_right_m<<"],\"map_revision\":"<<nav.grid().revision()<<",\"path_points\":"<<nav.path().size()<<",\"localization\":"<<localization_json(nav.localization_diagnostics())<<",\"local_avoidance\":"<<local_avoidance_json(nav.local_avoidance_diagnostics())<<",\"local_guard\":"<<local_guard_json(nav.local_guard_diagnostics())<<",\"motion_gate\":"<<motion_gate_json(nav.motion_gate_diagnostics(status_now))<<",\"localization_fault_context\":"<<localization_fault_context.serialize()<<",\"recovery_action\":"<<quoted(needs_localization_reset()?"stop_inspect_localization_then_reset_map":"none")<<",\"input_alignment\":"<<inputs.diagnostics_json()<<",\"units\":\"metres,radians\"}";return out.str();
    }
    std::string status(){
        std::lock_guard<std::mutex> lock(mutex);return status_locked(navwire::monotonicSeconds());
    }
    void log_event_locked(const char* event,const char* operation,const char* kind="diagnostic") noexcept {
        try {
        picojson::value value;
        if(!picojson::parse(value,status_locked(navwire::monotonicSeconds())).empty()||!value.is<picojson::object>())return;
        auto& object=value.get<picojson::object>();object["operation"]=picojson::value(std::string(operation));object["classification"]=picojson::value(std::string(kind));
        if(std::strcmp(kind,"fault")==0||std::strcmp(kind,"waiting_not_fault")==0){
            // A compact actual observation grid explains footprint clearance
            // failures without inventing a free map or serializing image data.
            // Do this on a state edge only, never in the 100 ms control path.
            const auto& grid=nav.grid();const auto origin=grid.origin();const double now=navwire::monotonicSeconds();
            picojson::object map;map["width"]=picojson::value(64.);map["height"]=picojson::value(64.);map["resolution_m"]=picojson::value(grid.resolution());map["map_revision"]=picojson::value(double(grid.revision()));
            picojson::array xy;xy.emplace_back(origin.first);xy.emplace_back(origin.second);map["origin_m"]=picojson::value(xy);
            map["computed_at_mono_ms"]=picojson::value(now*1000.);map["semantics"]=picojson::value(std::string("observed_occupancy_with_current_TTL_not_raw_sensor_map"));
            map["cell_order"]=picojson::value(std::string("row_major_y_increasing"));map["legend"]=picojson::value(std::string("?=unknown,.=observed_free,#=occupied"));
            std::string cells;cells.reserve(4096);for(int y=0;y<64;++y)for(int x=0;x<64;++x){const auto cell=grid.state({x,y},now);cells+=cell<0?'?':cell==0?'.':'#';}map["cells"]=picojson::value(cells);object["map_evidence"]=picojson::value(map);
        }
        std::string bytes=value.serialize();
        if(bytes.size()>16384){object.erase("map_evidence");object["map_evidence_omitted_payload_budget"]=picojson::value(true);bytes=value.serialize();}
        if(bytes.size()>16384){picojson::object fallback;fallback["classification"]=picojson::value(std::string(kind));fallback["operation"]=picojson::value(std::string(operation));fallback["state"]=picojson::value(nav.state());fallback["reason"]=picojson::value(nav.reason());fallback["diagnostics_omitted_payload_budget"]=picojson::value(true);bytes=picojson::value(fallback).serialize();}
        wrong_log_event("navigation",event,bytes.c_str());
        }catch(...){navigation_log_format_error();}
    }
    void log_snapshot() noexcept {
        try {wrong_log_snapshot("navigation",status().c_str());}
        catch(...){navigation_log_format_error();}
    }
    void observe_diagnostics_locked() noexcept {
        try {
        const bool changed=!diagnostic_initialized||diagnostic_state!=nav.state()||diagnostic_reason!=nav.reason()||diagnostic_enabled!=nav.enabled();
        if(!changed)return;
        if(nav.state()=="FAULT")log_event_locked(nav.reason().c_str(),"core_transition","fault");
        else if(nav.state()=="BOOTSTRAP_WAIT")log_event_locked(nav.reason().c_str(),"bootstrap_wait","waiting_not_fault");
        else if(nav.state()=="SENSOR_WAIT"||nav.state()=="SENSOR_RECOVERY")log_event_locked(nav.reason().c_str(),"range_source_wait","waiting_not_fault");
        else if(nav.state()=="AVOID_WAIT"||nav.state()=="AVOID_REJOIN_WAIT")log_event_locked(nav.reason().c_str(),"local_avoidance_wait","waiting_not_fault");
        else if(nav.state()=="STRAIGHT_WAIT")log_event_locked(nav.reason().c_str(),"straight_wait","waiting_not_fault");
        else if(diagnostic_initialized&&diagnostic_enabled&&!nav.enabled())log_event_locked(nav.reason().c_str(),"core_disabled","control_transition");
        diagnostic_state=nav.state();diagnostic_reason=nav.reason();diagnostic_enabled=nav.enabled();diagnostic_initialized=true;
        }catch(...){navigation_log_format_error();}
    }
    bool command(const std::string&op,double x,double y,std::string&error){
        std::lock_guard<std::mutex> lock(mutex);
        if(op=="stop"){nav.stop("operator_stop");cancel_plan=true;return true;}
        if(op=="reset-map"){
            auto config=nav.config();nav=Navigator(config);cancel_plan=true;
            reset_required=false;have_sensors=false;inputs.reset();localization_fault_context=picojson::value();
            input_request_barrier_ms=uint64_t(std::ceil(navwire::monotonicSeconds()*1000.));++origin_epoch;return true;
        }
        if(op=="goal"){
            if(!std::isfinite(x)||!std::isfinite(y)||std::abs(x)>10000||std::abs(y)>10000){error="invalid_goal";return false;}
            nav.set_goal(x,y);cancel_plan=true;return true;
        }
        if(op=="start"||op=="explore"||op=="straight"){
            const auto now=navwire::monotonicSeconds();error=start_problem(now);
            if(!error.empty()){log_event_locked(error.c_str(),op.c_str(),"request_rejected");return false;}
            // A phone Start works without a preselected map point. Existing
            // goals retain their meaning; otherwise this is autonomous explore.
            bool started=false;
            if(op=="straight")started=nav.start_straight(now);
            else {
                if(op=="explore"||!nav.goal())nav.explore();
                started=nav.start(now);
            }
            if(!started){error=nav.reason();log_event_locked(error.c_str(),op.c_str(),"request_rejected");return false;}
            cancel_plan=true;log_event_locked(op=="straight"?"straight_start_accepted":"navigation_start_accepted",op.c_str());observe_diagnostics_locked();return true;
        }
        error="unknown_navigation_operation";return false;
    }
    void cancel(const std::string&reason){
        std::lock_guard<std::mutex> lock(mutex);observe_diagnostics_locked();
        log_event_locked(reason.c_str(),"cancel_before_core_stop","control_transition");nav.stop(reason);cancel_plan=true;
        if(reason.find("zero")!=std::string::npos||reason.find("calibrate")!=std::string::npos||reason.find("configure")!=std::string::npos)reset_required=true;
        observe_diagnostics_locked();
    }
};

static void atomicText(const std::string&path,const std::string&bytes){
    const auto tmp=path+".tmp";std::ofstream out(tmp,std::ios::binary|std::ios::trunc);out<<bytes;out.close();if(!out||rename(tmp.c_str(),path.c_str()))throw std::runtime_error("cannot save navigation map/status: "+path);
}
static void exportMap(Shared&s,const std::string&dir,double now){
    std::string svg,pgm,json=s.status();
    {
        std::lock_guard<std::mutex> lock(s.mutex);auto&g=s.nav.grid();auto origin=g.origin();double resolution=g.resolution();
        std::ostringstream image;image<<"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"760\" height=\"760\" viewBox=\"0 0 760 760\"><rect width=\"760\" height=\"760\" fill=\"white\"/><text x=\"28\" y=\"32\" font-size=\"18\">RK map / FPGA A* - "<<s.nav.state()<<"</text><g transform=\"translate(28 56)\">";
        std::ostringstream gray;gray<<"P2\n64 64\n255\n";
        for(int y=63;y>=0;--y){for(int x=0;x<64;++x){auto v=g.state({x,y},now);const char*color=v<0?"#d9dde2":v==1?"#282e35":"#fafafa";image<<"<rect x=\""<<x*11<<"\" y=\""<<(63-y)*11<<"\" width=\"11\" height=\"11\" fill=\""<<color<<"\"/>";gray<<(v<0?127:v==1?0:254)<<' ';}gray<<'\n';}
        auto xy=[&](double x,double y){return std::make_pair((x-origin.first)/resolution*11,(64-(y-origin.second)/resolution)*11);};
        auto draw=[&](auto&points,const char*color,auto coordinates){if(points.empty())return;image<<"<polyline fill=\"none\" stroke=\""<<color<<"\" stroke-width=\"2\" points=\"";for(auto&p:points){auto q=coordinates(p);auto v=xy(q.first,q.second);image<<v.first<<','<<v.second<<' ';}image<<"\"/>";};
        draw(s.nav.trajectory(),"#26936b",[](const robot_nav::Pose&p){return std::make_pair(p.x,p.y);});
        draw(s.nav.path(),"#d17100",[](const robot_nav::Point&p){return p;});
        auto pose=s.nav.pose();auto p=xy(pose.x,pose.y);image<<"<g transform=\"translate("<<p.first<<' '<<p.second<<") rotate("<<-pose.yaw*180/PI<<")\"><path d=\"M 12 0 L -7 -7 L -4 0 L -7 7 Z\" fill=\"#1373a5\" stroke=\"white\"/></g></g></svg>";
        svg=image.str();pgm=gray.str();
        std::ostringstream yaml;yaml<<"image: map.pgm\nresolution: "<<resolution<<"\norigin: ["<<origin.first<<", "<<origin.second<<", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";atomicText(dir+"/map.yaml",yaml.str());
    }
    atomicText(dir+"/map.svg",svg);atomicText(dir+"/map.pgm",pgm);atomicText(dir+"/status.json",json+"\n");
}

struct PlanResult {uint32_t id=0,revision=0;unsigned origin_epoch=0;std::vector<uint16_t>cells;std::string error;};
static void help(){std::cout<<"robot-navigation v27: automatic observation / bounded exploration / FPGA A*\n"
    "  --config FILE [--interface ethX] [--drive] [--check-config] [--dump-profile]\n"
    "  --token-file FILE --backend-port 8082 --port 8080 --output DIR\n"
    "  --migrate-config INPUT --output STAGED (read/write config only; no hardware)\n"
    "  --check-profile FILE (verify fixed keys without changing calibration)\n"
    "Console: goal X Y | start | explore | stop | reset-map | status | quit\n"
    "Default is observation only. Real navigation also requires measured control limits,\n"
    "compatible FPGA, fresh sensors, foreground phone and explicit motor arming.\n";}

int main(int argc,char**argv){
    struct LogLifetime {
        bool started=false;
        void start() noexcept {wrong_log_start("navigation","robot-navigation-v33-scan-lease");started=true;}
        ~LogLifetime() noexcept {if(started)wrong_log_shutdown();}
    } log;
    try{
        std::string config="config.json",interface,output="/tmp/robot-navigation",migration_input,check_profile;bool drive=false,check=false,dump_profile=false,config_given=false,output_given=false;
        robotnav::GatewayOptions options;
        for(int i=1;i<argc;++i){std::string arg=argv[i];if(arg=="--help"){help();return 0;}if(arg=="--version"){std::cout<<"robot-navigation-v33-scan-lease\n";return 0;}if(arg=="--dump-profile"){dump_profile=true;continue;}if(arg=="--drive"){drive=true;continue;}if(arg=="--check-config"){check=true;continue;}if(i+1==argc)throw std::runtime_error("missing option value: "+arg);std::string value=argv[++i];
            if(arg=="--config"){config=value;config_given=true;}else if(arg=="--interface")interface=value;else if(arg=="--output"){output=value;output_given=true;}else if(arg=="--migrate-config")migration_input=value;else if(arg=="--check-profile")check_profile=value;else if(arg=="--token-file")options.token_file=value;else if(arg=="--port"||arg=="--backend-port"){size_t at=0;auto p=std::stoul(value,&at);if(at!=value.size()||p<1024||p>65535)throw std::runtime_error("invalid port");if(arg=="--port")options.port=p;else options.backend_port=p;}else throw std::runtime_error("unknown option "+arg);
        }
        if(!check_profile.empty()){
            if(!migration_input.empty()||drive||check||dump_profile||config_given||output_given||!interface.empty())throw std::runtime_error("--check-profile requires only a config file");
            return checkFixedProfile(check_profile);
        }
        if(!migration_input.empty()){
            if(!output_given||drive||check||dump_profile||config_given||!interface.empty())throw std::runtime_error("migration requires only --migrate-config INPUT --output STAGED");
            migrateConfig(migration_input,output);return 0;
        }
        if(dump_profile&&!config_given){Settings s;applyFixedProfile(s.config);std::cout<<profileValue(s.config,s.yaw_sign).serialize()<<'\n';return 0;}
        // Inspection, profile migration and installer checks never create logs.
        // Start before runtime initialization so failures opening hardware or
        // loading a real runtime configuration retain their cause as well.
        if(!check&&!dump_profile)log.start();
        // Calibration's explicit apply uses this same lock. Acquire it before
        // loading settings so a simultaneous startup cannot keep old CPR/signs.
        if(!check&&!dump_profile){int lockfd=open("/tmp/robot-navigation-v16.lock",O_CREAT|O_RDWR|O_CLOEXEC,0600);if(lockfd<0||flock(lockfd,LOCK_EX|LOCK_NB))throw std::runtime_error("another navigation service or calibration update is running");}
        auto settings=readSettings(config);auto errors=settings.config.errors();
        if(dump_profile){std::cout<<profileValue(settings.config,settings.yaw_sign).serialize()<<'\n';return 0;}
        if(check){std::cout<<"PROFILE "<<profileValue(settings.config,settings.yaw_sign).serialize()<<'\n';for(const auto&e:errors)std::cout<<e<<'\n';if(errors.empty())std::cout<<"Configuration structurally valid; physical calibration still requires measurement.\n";return errors.empty()?0:2;}
        const auto log_config=runtime_log_config(settings.config,settings.yaw_sign,drive);wrong_log_snapshot("navigation_config",log_config.c_str());
        if(drive&&!errors.empty())throw std::runtime_error("motion disabled: "+errors.front());
        std::cout<<"NAV_PROFILE "<<profileValue(settings.config,settings.yaw_sign).serialize()<<'\n';
        // flock lasts for this process, even if the shell disappears. The
        // dashboard itself has a separate one-instance lock.
        if(mkdir(output.c_str(),0700)&&errno!=EEXIST)throw std::runtime_error("cannot create output directory");
        if(interface.empty())interface=navwire::findInterface();
        Shared shared(settings.config,drive,settings.yaw_sign);robotnav::GatewayCallbacks callbacks;
        // Gateway calls this while its control lock is held, before callbacks
        // or control ticks, so a cached polling snapshot cannot revert a newer
        // acknowledged phone wheel grouping.
        callbacks.navigation_layout=[&](const robotnav::GatewayTelemetry& state){shared.synchronize_wheel_layout(state);};
        callbacks.navigation_command=[&](const std::string&op,double x,double y,std::string&err){return shared.command(op,x,y,err);};
        callbacks.navigation_cancel=[&](const std::string&why){shared.cancel(why);};callbacks.navigation_state=[&]{return shared.status();};
        callbacks.navigation_map=[&]{return shared.map_cache.read(navwire::monotonicSeconds());};
        callbacks.navigation_goto=[&](double x,double y,const std::string& session,uint32_t revision,std::string& err){return shared.go_to(x,y,session,revision,err);};
        navwire::RawSocket sensors;sensors.open(interface,navwire::SENSOR_TYPE);
        navwire::Planner planner;planner.open(interface);
        robotnav::Gateway gateway(options,callbacks);std::string error;if(!gateway.start(error))throw std::runtime_error(error);
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);std::signal(SIGHUP,signalHandler);std::signal(SIGPIPE,SIG_IGN);
        std::cout<<"NAV v33-scan-lease interface="<<interface<<" mode="<<(drive?"explicit-drive":"observation-only")<<" map="<<output<<"/map.json\n";help();std::cout.flush();
        std::mutex result_mutex;std::optional<PlanResult> result;std::thread worker;std::atomic<bool> running{false};uint32_t submitted=0;
        struct Cleanup {Shared&s;std::thread&w;robotnav::Gateway&g;~Cleanup(){s.cancel_plan=true;g.cancelNavigation("navigation_process_exit");if(w.joinable())w.join();g.stop();}} cleanup{shared,worker,gateway};
        double last_export=0,last_map=0,last_poll=0,last_log=0,last_log_config=0;robotnav::GatewayTelemetry telemetry;std::string input;bool console_open=true;
        unsigned origin_epoch=0;
        while(!interrupted){
            double now=navwire::monotonicSeconds();
            if(now-last_poll>=.05){telemetry=gateway.pollState();last_poll=navwire::monotonicSeconds();}
            // Wheel grouping is synchronized by the gateway-locked callback.
            // origin_epoch below resets this thread's yaw/count baseline.
            std::vector<std::pair<navwire::Sensors,double>> packets;
            std::vector<uint8_t> bytes;double packet_stamp=0;
            // Retain every SNS sample, including 1 ms sonar-event snapshots.
            // Device time measures motion; kernel receipt time measures freshness.
            for(int i=0;i<128&&sensors.receive(bytes,0,&packet_stamp);++i){
                navwire::Sensors packet;
                if(navwire::sensorFrame(bytes.data(),bytes.size(),sensors.mac(),packet))packets.emplace_back(packet,packet_stamp);
            }
            now=navwire::monotonicSeconds();
            // Near echoes are reduce-only safety events. Process the newest
            // SNS1 before IMU interpolation, mapping, exports or queued pose
            // updates; this path can stop but cannot start the vehicle.
            bool urgent_zero=false;uint64_t urgent_ticket=0;
            gateway.visitNavigation([&](bool current_active,uint64_t generation){
                urgent_ticket=generation;std::lock_guard<std::mutex> guard(shared.mutex);
                for(const auto& incoming:packets){
                    const auto& raw=incoming.first;robot_nav::LiveSonarSample live;
                    live.timestamp_s=incoming.second;live.sequence=raw.sequence;
                    for(unsigned i=0;i<3;++i){
                        live.age_s[i]=raw.age_ms[i]/1000.;live.observation_id[i]=raw.uptime_ms-raw.age_ms[i];
                        if((raw.valid&(1U<<i))&&!(raw.fault&(1U<<i))&&robot_sonar_range_supported(raw.distance_mm[i]))
                            live.metres[i]=robot_sonar_range_censored(raw.distance_mm[i])/1000.;
                    }
                    const bool interrupted_motion=shared.nav.observe_latest_sonar(live,now);
                    urgent_zero=urgent_zero||(current_active&&interrupted_motion);
                }
            });
            if(urgent_zero)gateway.applyNav(0,0,urgent_ticket);
            {
                std::lock_guard<std::mutex> guard(shared.mutex);
                if(origin_epoch!=shared.origin_epoch){origin_epoch=shared.origin_epoch;submitted=0;}
                const auto prior_command=shared.nav.command();const bool was_enabled=shared.nav.enabled();
                // A zero/map/layout reset may have happened after this cached
                // HTTP request started. Wait for a new request in this epoch.
                if(shared.input_request_allowed(telemetry.request_started_ms))shared.inputs.update_imu(telemetry,now);
                for(const auto& incoming:packets){
                    const auto& newest=incoming.first;const double newest_stamp=incoming.second;
                    const bool restart=shared.have_sensors&&int32_t(newest.uptime_ms-shared.sensors.uptime_ms)<0;
                    const bool forward=!shared.have_sensors||int32_t(newest.sequence-shared.sensors.sequence)>0;
                    if(restart){
                        shared.reset_required=true;shared.nav.invalidate_localization("FPGA_restarted_reset_map_required",now);
                        shared.cancel_plan=true;
                    }
                    if(forward||restart){
                        shared.sensors=newest;shared.have_sensors=true;shared.received_s=newest_stamp;
                        if(shared.have_wheel_layout&&!shared.reset_required&&shared.input_sensor_allowed(newest_stamp))shared.inputs.enqueue(newest,newest_stamp);
                    }
                }
                if(shared.have_wheel_layout&&!shared.reset_required)shared.inputs.drain(shared.nav,now);
                shared.retain_fault_context_locked(telemetry,prior_command,was_enabled,now);
                shared.observe_diagnostics_locked();
            }
            {std::optional<PlanResult> ready;{std::lock_guard<std::mutex> g(result_mutex);ready=std::move(result);result.reset();}if(ready){std::lock_guard<std::mutex> g(shared.mutex);if(ready->origin_epoch==shared.origin_epoch){if(ready->error.empty())shared.nav.accept_plan(ready->id,ready->revision,ready->cells,now);else shared.nav.reject_plan(ready->id,ready->error);}}}
            uint64_t ticket=0;bool active=false,core_enabled=false;robot_nav::Control command;
            // Same gateway -> core lock order as goto/takeover. A new goal must
            // not be stopped by a stale inactive snapshot, nor may an old job
            // clear cancel_plan after a concurrent manual stop/new destination.
            gateway.visitNavigation([&](bool current_active,uint64_t generation){
                ticket=generation;active=current_active;
                std::lock_guard<std::mutex>g(shared.mutex);
                now=navwire::monotonicSeconds();
                command=shared.nav.tick(now);
                shared.observe_diagnostics_locked();
                if(!active&&shared.nav.enabled()){shared.nav.stop("phone_or_manual_cancel");shared.cancel_plan=true;}
                shared.observe_diagnostics_locked();
                core_enabled=shared.nav.enabled();if(!core_enabled)shared.cancel_plan=true;
                // Obstacle stop/scan may replace a route request while the
                // navigator remains enabled. Retire that stale FPGA job now,
                // rather than letting its full timeout block the new detour.
                if(running.load()&&(!command.planner_request||command.planner_request->request_id!=submitted))shared.cancel_plan=true;
                if(active&&core_enabled&&command.planner_request&&command.planner_request->request_id!=submitted&&!running.load()){
                    if(worker.joinable())worker.join();
                    auto request=*command.planner_request;submitted=request.request_id;shared.cancel_plan=false;running=true;
                    const unsigned job_epoch=shared.origin_epoch;
                    worker=std::thread([&,request,job_epoch]{PlanResult r;r.id=request.request_id;r.revision=request.map_revision;r.origin_epoch=job_epoch;try{r.cells=planner.plan(request.blocked,request.start,request.goal,&shared.cancel_plan);}catch(const std::exception&e){r.error=e.what();}{std::lock_guard<std::mutex>g(result_mutex);result=std::move(r);}running=false;});
                }
            });
            if(active){if(!core_enabled)gateway.cancelNavigation(command.reason,ticket);else gateway.applyNav(command.linear_mps,command.angular_rps,ticket);}
            if(now-last_log>=.1){shared.log_snapshot();last_log=now;}
            if(now-last_log_config>=5.){wrong_log_snapshot("navigation_config",log_config.c_str());last_log_config=now;}
            pollfd stdin_poll{STDIN_FILENO,POLLIN,0};if(console_open&&poll(&stdin_poll,1,0)>0&&(stdin_poll.revents&(POLLIN|POLLHUP))){char b[1024];auto n=read(STDIN_FILENO,b,sizeof(b));if(n<=0){console_open=false;}else{input.append(b,n);size_t end;while((end=input.find('\n'))!=std::string::npos){auto line=input.substr(0,end);input.erase(0,end+1);std::istringstream fields(line);std::string op,extra;double x=0,y=0;fields>>op;if(op=="quit"){interrupted=1;break;}if(op=="status"){std::cout<<shared.status()<<'\n';continue;}if(op=="help"){help();continue;}if(op=="goal"&&(!(fields>>x>>y)||!std::isfinite(x)||!std::isfinite(y))){std::cout<<"Use: goal X Y (metres from starting pose)\n";continue;}if(fields>>extra){std::cout<<"unexpected argument\n";continue;}if(op.empty())continue;std::string e;bool okay=gateway.requestNavigation(op,x,y,e);std::cout<<(okay?"accepted":e)<<'\n';}std::cout.flush();}}
            if(now-last_map>=.5){
                std::string snapshot;
                {std::lock_guard<std::mutex> lock(shared.mutex);
                    snapshot=robot_nav::map_snapshot_json(shared.nav,now,shared.have_sensors?now-shared.received_s:-1,
                        shared.needs_localization_reset(),shared.map_session+"-"+std::to_string(shared.origin_epoch));}
                shared.map_cache.publish(std::move(snapshot),now);
                atomicText(output+"/map.json",shared.map_cache.read(now));last_map=now;
            }
            if(now-last_export>=1){exportMap(shared,output,now);last_export=now;std::cout<<"NAV "<<shared.status()<<'\n';std::cout.flush();}
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        navigation_log_exit(interrupted);
        return 0;
    }catch(const std::exception&e){
        navigation_log_exception(e.what());
        std::cerr<<"Navigation stopped: "<<e.what()<<'\n';return 1;
    }
}
