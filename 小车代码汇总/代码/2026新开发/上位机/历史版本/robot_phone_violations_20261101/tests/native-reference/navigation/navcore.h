#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "vehicle_geometry.h"

// Metres, seconds, radians; +x forward at origin, +y left, yaw CCW.
// Signal order: M1, M2, M3, M4. Runtime ACK supplies wheel-side grouping.
// No host-side A* implementation.
namespace robot_nav {
constexpr unsigned MAP_SIZE = 64;
// Above this range the UI and map deliberately retain only a lower bound.
constexpr double SONAR_CENSORED_RANGE_M = 4.0;
using Point = std::pair<double, double>;
using Cell = std::pair<int, int>;
using Bitmap = std::array<uint8_t, 512>; // bit y*64+x, LSB first; 1 blocked
struct Pose { double x=0, y=0, yaw=0; };
double wrap(double angle);
int32_t count_delta(uint32_t newer, uint32_t older);

struct Config {
    bool configured = false;
    // Runtime MOT4 capability, never persisted as operator calibration.
    bool velocity_control = false;
    double wheel_circumference_m=vehicle_profile::wheel_circumference_m;
    double track_m=vehicle_profile::track_m, robot_radius_m=vehicle_profile::robot_radius_m;
    double encoder_ticks_per_turn=vehicle_profile::encoder_ticks_per_turn; // FPGA x4 counts / wheel revolution
    double drive_mps_per_full_pwm=0; // measured conversion, not nameplate RPM
    std::array<int,4> encoder_signs=vehicle_profile::encoder_signs;
    unsigned left_wheel_mask=10; // two bits, synchronized from verified motor ACK
    double body_front_m=vehicle_profile::body_front_m, body_rear_m=vehicle_profile::body_rear_m;
    double body_left_m=vehicle_profile::body_left_m, body_right_m=vehicle_profile::body_right_m;
    double wheelbase_m=vehicle_profile::wheelbase_m;
    double imu_mount_x_m=vehicle_profile::imu_mount_x_m, imu_mount_y_m=vehicle_profile::imu_mount_y_m;
    double resolution_m=.10, safety_margin_m=.08;
    // x, y and angle of front / left / right sensor relative to axle centre.
    std::array<std::array<double,3>,3> sonar_mounts=vehicle_profile::sonar_mounts;
    double sonar_half_angle_rad=vehicle_profile::sonar_half_angle_rad;
    double sonar_min_m=.03, sonar_max_m=4.5;
    double sensor_timeout_s=.25, sonar_timeout_s=.75, sonar_latency_s=.30;
    // Total age since a genuine accepted observation, never time since the
    // latest SNS1 packet. Only a previously moving straight command may coast
    // through this bounded source gap; it cannot authorize a new manoeuvre.
    double sonar_straight_grace_s=1.0;
    double planner_timeout_s=40.0, map_ttl_s=60;
    double cruise_mps=.15, detour_mps=.22, max_wheel_mps=.30, max_pwm=.45;
    // Independent assist target/cap; ordinary navigation keeps max_pwm and
    // its existing cruise/detour targets unchanged.
    double straight_speed_mps=.30, straight_max_pwm=.60, straight_replan_max_s=120;
    double max_turn_rps=.6, accel_mps2=.15, brake_mps2=.4, reaction_s=.85;
    double stop_clearance_m=.30, side_clearance_m=.30;
    double preserved_config_stop_clearance_m=.20; // diagnostic only; never authorizes motion
    double sonar_wait_timeout_s=5., avoid_clear_front_m=.35, avoid_rejoin_distance_m=.50;
    double avoid_verify_turn_min_rad=8*vehicle_profile::pi/180.;
    double avoidance_step_rad=25*vehicle_profile::pi/180.;
    double stop_dwell_s=.25, scan_observation_dwell_s=.35, scan_max_s=40.;
    double stopped_wheel_mps=.015;
    double detour_accel_distance_m=.70, rejoin_ahead_m=.60;
    double waypoint_tolerance_m=.08, heading_tolerance_rad=.13962634015954636;
    double encoder_mismatch_ratio=.60, encoder_fault_after_s=.60;
    double encoder_compare_window_s=.25, encoder_min_compare_mps=.01;
    double bootstrap_speed_mps=.06, bootstrap_step_m=.10;
    // Limit physical distance and cumulative forward-command time separately.
    // Safe obstacle/observation waits do not consume the movement budget.
    double bootstrap_max_distance_m=1.20, bootstrap_max_s=60;
    double max_measured_wheel_mps=1.5, max_measured_turn_rps=5, no_progress_s=4;
    std::vector<std::string> errors() const;
};

struct SensorFrame {
    double timestamp_s=0; // host kernel receipt time: freshness and pose history
    // Optional unwrapped device sample time, seconds. When present on two
    // consecutive frames it drives motion rates; never mix clock domains.
    std::optional<double> device_timestamp_s;
    std::array<uint32_t,4> counts{};
    double yaw_rad=0;
    std::array<std::optional<double>,3> sonar_m{};
    std::array<double,3> sonar_age_s{};
    // Device observation milliseconds (uptime - channel age), not packet seq.
    // Optional for replay/simulator callers that have no device clock.
    std::array<uint32_t,3> sonar_observation_id{};
    bool sonar_observation_ids_valid=false;
    std::array<bool,4> encoder_ok{{true,true,true,true}};
    bool imu_valid=false;
    double imu_age_s=0;
    uint32_t sequence=0;
};

// Immediate range-only observation. It never changes pose, IMU alignment or
// encoder time. Host receive time and source age retain their original clocks.
struct LiveSonarSample {
    double timestamp_s=0;
    uint32_t sequence=0;
    std::array<std::optional<double>,3> metres{};
    std::array<double,3> age_s{};
    std::array<uint32_t,3> observation_id{};
};

struct LocalizationDiagnostics {
    bool latched=false;
    std::string first_reason;
    double timestamp_s=0, sample_timestamp_s=0, previous_timestamp_s=0;
    double dt_s=0, host_dt_s=0;
    bool using_device_time=false, sample_has_device_time=false, previous_has_device_time=false;
    double device_timestamp_s=0, previous_device_timestamp_s=0;
    double dyaw_rad=0, yaw_limit_rad=0, wheel_limit_m=0;
    std::array<double,4> wheel_delta_m{};
    uint32_t previous_sequence=0, sequence=0;
    uint8_t encoder_invalid_mask=0, jump_wheel_mask=0;
    bool imu_valid=false, yaw_jump=false, motion_limits_evaluated=false, external_invalidation=false;
    double wheel_comparison_window_s=0;
    std::array<double,4> window_wheel_mps{};
};

struct LocalAvoidanceDiagnostics {
    bool active=false;
    double original_heading_rad=0,turn_target_rad=0,turn_travel_rad=0,scan_travel_rad=0;
    double departure_distance_m=0,parallel_distance_m=0;
    unsigned restart_attempts=0;
    int departure_side=0;
    unsigned attempted_side_mask=0,blocked_side_mask=0,direction_switches=0;
    double motion_elapsed_s=0;
};
struct LocalGuardDiagnostics {
    bool evaluated=false,blocked=false,pure_rotation=false;
    std::string reason="not_evaluated";
    double evaluated_s=0,target_yaw_rad=0,cell_min_center_distance_m=0,required_radius_m=0;
    double range_m=0,range_required_m=0;
    int channel=-1,cell_x=-1,cell_y=-1,cell_state=-1;
    Point cell_min_world{},cell_max_world{};
    unsigned padding_outside_circle_skipped=0;
};
struct MotionGateDiagnostics {
    bool frame_available=false,source_usable=false,front_blocked=false,forward_clear=false;
    bool straight_source_grace=false;
    double source_grace_limit_s=1.;
    std::array<double,3> last_genuine_source_age_s{};
    std::array<double,4> measured_wheel_mps{};
    double measured_sample_dt_s=0,measured_yaw_rps=0,safety_speed_mps=0,reaction_delay_s=0;
    double front_stop_distance_m=0,front_resume_distance_m=0;
    double target_linear_mps=0,target_angular_rps=0;
    uint32_t frame_sequence=0;
    double frame_age_s=0;
    std::array<std::optional<double>,3> sonar_m{};
    std::array<double,3> total_sonar_age_s{};
    std::optional<double> stop_requested_s,stationary_since_s;
    double stop_wait_elapsed_s=0,stationary_elapsed_s=0;
    double stop_evidence_window_s=0,stop_confirm_window_s=.080,stop_window_yaw_rps=0;
    std::array<double,4> stop_window_wheel_mps{};
    unsigned stop_evidence_samples=0;
    std::string stop_evidence_reason="not_checked";
    bool using_live_sonar=false;
    uint32_t range_sequence=0;
    double range_packet_age_s=0;
};
struct PlannerRequest {
    uint32_t request_id=0, map_revision=0;
    Bitmap blocked{};
    uint16_t start=0, goal=0;
    Point origin{};
    double resolution=.1, created_s=0;
    std::string purpose="route";
    int side=0; // departure relative to ORIGINAL route: +left / -right / 0 axial
    uint16_t width=64, height=64;
};

struct Control {
    double left=0, right=0, linear_mps=0, angular_rps=0;
    std::string state="STOPPED", reason="not_started";
    std::optional<PlannerRequest> planner_request;
};

class OccupancyGrid {
public:
    explicit OccupancyGrid(const Config& config);
    Cell cell(double x, double y) const;
    Point center(Cell c) const;
    static bool in_bounds(Cell c);
    static uint16_t index(Cell c);
    static bool blocked(const Bitmap& bits, unsigned id);
    int state(Cell c, double now) const; // 0 free, 1 occupied, -1 unknown
    Bitmap planning_bitmap(double now) const;
    void recenter(const Pose& pose);
    void mark_body(const Pose& pose, double stamp);
    void update_sonar(const Pose& pose, unsigned channel, double distance, double stamp);
    const std::array<float,4096>& values() const { return values_; }
    const std::array<double,4096>& observed_s() const { return observed_; }
    Point origin() const { return origin_; }
    double resolution() const { return config_.resolution_m; }
    uint32_t revision() const { return revision_; }
    // Explicit operator-confirmed / simulator map import. Not used by live sonar
    // adapter. Unknown=-1, free=0, occupied=1, row-major 4096 cells.
    bool import_observations(const std::vector<int8_t>& cells, Point origin, double stamp);
private:
    Config config_;
    Point origin_;
    std::array<float,4096> values_{};
    std::array<double,4096> observed_{};
    uint32_t revision_=0;
    void add(int x, int y, double evidence, double stamp);
};

class Navigator {
public:
    explicit Navigator(Config config);
    bool ingest(const SensorFrame& frame, double now_s);
    bool ingest(const SensorFrame& frame) { return ingest(frame, frame.timestamp_s); }
    void set_goal(double x, double y);
    bool go_to(double x, double y, double now, std::string& error);
    void explore();
    bool set_wheel_layout(unsigned left_mask); // changes disarm and clear localization/map
    void reset_localization(); // explicit reset: clears old pose/map/goal, disarms
    bool start(double now);
    // Heading assist captures the current IMU/world heading. Current valid,
    // fresh ranges gate forward motion and small positive-wheel corrections;
    // no post-stop observation or historical-map gate precedes that motion.
    // Local avoidance uses fresh sonar only; FPGA route planning/rejoin
    // retains the original observed-map requirements.
    bool start_straight(double now);
    bool straight_assist_active() const { return straight_assist_; }
    // Start checks real sensor/calibration readiness, not a surrounding free
    // disk. Exploration may enter bounded, stopped bootstrap observation.
    std::string start_problem(double now) const;
    Control stop(const std::string& reason="operator_stop");
    Control tick(double now);
    bool accept_plan(uint32_t request_id, uint32_t map_revision,
                     const std::vector<uint16_t>& cell_ids, double now);
    Control reject_plan(uint32_t request_id, const std::string& reason);
    LocalAvoidanceDiagnostics local_avoidance_diagnostics() const;
    const LocalGuardDiagnostics& local_guard_diagnostics() const { return local_guard_; }
    MotionGateDiagnostics motion_gate_diagnostics(double now) const;
    const Pose& pose() const { return pose_; }
    const OccupancyGrid& grid() const { return grid_; }
    OccupancyGrid& grid() { return grid_; }
    bool enabled() const { return enabled_; }
    double bootstrap_motion_elapsed_s() const { return bootstrap_forward_s_; }
    const std::string& state() const { return state_; }
    const std::string& reason() const { return reason_; }
    const std::vector<Point>& path() const { return path_; }
    const std::vector<Pose>& trajectory() const { return trajectory_; }
    const std::optional<Point>& goal() const { return goal_; }
    const std::optional<PlannerRequest>& pending_request() const { return pending_; }
    const Control& command() const { return command_; }
    const Config& config() const { return config_; }
    const std::optional<SensorFrame>& frame() const { return frame_; }
    // Returns true only when a previously moving command was reduced to zero.
    bool observe_latest_sonar(const LiveSonarSample& sample,double now);
    bool localization_valid(double now) const;
    std::string localization_problem(double now) const;
    const LocalizationDiagnostics& localization_diagnostics() const { return localization_diagnostics_; }
    // Stop immediately. A real established reference becomes latched-invalid;
    // an unavailable startup source does not fabricate an established pose.
    Control invalidate_localization(const std::string& reason,double now_s);
    // Geometry diagnostic used by the same production motion guard. Existing
    // body coverage is exempt; newly swept space must be free unless this is a
    // sensor-gated straight bootstrap step. No occupancy evidence is written.
    bool motion_increment_free(const Pose& target,double now,bool allow_unknown=false) const;
private:
    Config config_;
    OccupancyGrid grid_;
    Pose pose_, progress_pose_;
    bool enabled_=false, valid_odometry_=false, exploration_=false, localization_lost_=false;
    std::string state_="STOPPED", reason_="not_started";
    Control command_;
    LocalizationDiagnostics localization_diagnostics_;
    std::optional<SensorFrame> frame_;
    std::optional<SensorFrame> live_sonar_;
    struct GenuineRange {
        bool have=false,have_id=false;
        double metres=0,source_host_s=0;
        uint32_t observation_id=0;
    };
    std::array<GenuineRange,3> genuine_ranges_{};
    void record_genuine_ranges(const SensorFrame& ranges);
    bool straight_source_grace(double now) const;
    bool fresh_range_requests_stop(double now) const;
    const SensorFrame& control_ranges() const { return live_sonar_?*live_sonar_:*frame_; }
    std::optional<Point> goal_, rejoin_point_;
    std::optional<PlannerRequest> pending_;
    std::optional<Pose> detour_origin_;
    std::optional<double> detour_departure_heading_;
    Pose detour_departure_origin_;
    std::optional<double> last_tick_;
    std::vector<Point> path_, original_route_;
    std::vector<Pose> trajectory_;
    std::vector<std::pair<double,Pose>> history_;
    std::array<double,3> sonar_stamp_;
    std::array<uint32_t,3> sonar_id_{};
    std::array<bool,3> have_sonar_id_{};
    std::array<double,4> wheel_bad_s_{}, measured_wheel_mps_{};
    std::array<double,4> wheel_window_distance_{};
    double wheel_window_s_=0;
    double measured_yaw_rps_=0, last_motion_mps_=0;
    struct MotionObservation {
        double host_s=0,source_s=0,yaw=0;
        std::array<uint32_t,4> counts{};
    };
    std::vector<MotionObservation> motion_observations_;
    double stop_evidence_window_s_=0,stop_window_yaw_rps_=0;
    std::array<double,4> stop_window_wheel_mps_{};
    unsigned stop_evidence_samples_=0;
    std::string stop_evidence_reason_="not_checked";
    std::optional<double> sonar_wait_started_;
    bool sonar_recovery_wait_=false;
    std::string sonar_wait_resume_state_;
    bool avoidance_active_=false, obstacle_stopping_=false;
    bool turn_active_=false, turn_settling_=false, turn_preparing_=false, turn_is_scan_=false;
    double stopped_requested_s_=0, scan_started_s_=0, scan_travel_rad_=0;
    std::optional<double> stationary_since_;
    double turn_start_yaw_=0, turn_target_yaw_=0, turn_last_yaw_=0, turn_travel_rad_=0;
    std::string turn_resume_state_;
    bool bootstrap_active_=false;
    bool straight_assist_=false;
    Pose straight_origin_;
    double straight_handoff_until_s_=0;
    std::string straight_turn_wait_resume_;
    std::optional<double> straight_replan_started_s_;
    unsigned straight_replans_=0;
    Pose bootstrap_origin_,bootstrap_step_origin_,bootstrap_step_target_;
    double bootstrap_forward_s_=0,bootstrap_distance_start_=0;
    void initialize_bootstrap(double now);
    Control service_bootstrap(double now,double dt);
    Control service_straight(double now,double dt);
    // Local bypass is source-sonar gated; historical occupancy is retained
    // for planning only and unknown cells are never asserted to be free.
    enum class Bypass { None, Stop, Select, TurnOut, MoveOut, TurnBack, MoveParallel, Rejoin };
    Bypass bypass_=Bypass::None,bypass_after_stop_=Bypass::None;
    bool bypass_turning_=false,bypass_rotated_=false;
    double bypass_heading_=0,bypass_target_=0,bypass_turn_start_=0,bypass_turn_last_=0,bypass_turn_travel_=0;
    double bypass_started_=0,bypass_scan_=0,bypass_leg_start_=0;
    double bypass_motion_s_=0,bypass_turn_started_s_=0;
    double measured_sample_dt_s_=0;
    std::optional<uint32_t> bypass_clear_id_;
    std::optional<double> bypass_min_turn_reached_s_;
    unsigned bypass_clear_samples_=0;
    double bypass_departure_distance_=0,bypass_parallel_distance_=0;
    unsigned bypass_attempts_=0;
    int bypass_side_=0;
    // A blocked direction is not retried on each 20ms control tick. It needs
    // two new source observations above its actual blocking threshold +2cm.
    // These belong to the whole encounter, including stopped/source waits.
    struct BypassDirectionBlock {
        bool blocked=false;
        int channel=-1;
        double release_m=0;
        uint32_t last_observation=0;
        unsigned release_samples=0;
    };
    std::array<BypassDirectionBlock,2> bypass_direction_block_{};
    unsigned bypass_attempted_sides_=0,bypass_direction_switches_=0;
    int bypass_last_started_side_=0;
    mutable LocalGuardDiagnostics local_guard_;
    void bypass_stop(Bypass next,double now,const std::string& why);
    bool bypass_stopped(double now);
    bool bypass_rotation_clear(double target,double now) const;
    void account_bypass_turn();
    void block_bypass_direction(int side,int channel,double required);
    bool bypass_direction_clear(int side,double target,double now);
    void interrupt_bypass_direction(double now,const std::string& reason);
    bool bypass_forward_clear(double now) const;
    double bypass_front_clear_distance(double now) const;
    void bypass_turn(double goal,Bypass phase,double now);
    Control service_bypass(double now,double dt);
    Control request_bypass_rejoin(double now);
    bool footprint_increment_free(const Pose& current,const Pose& from,const Pose& target,double now,bool allow_unknown,LocalGuardDiagnostics* diagnostic=nullptr) const;
    bool rotation_free(double target_yaw, double now) const;
    bool footprint_free(const Pose& pose, double now) const;
    double safety_speed() const;
    double reaction_delay(double now) const;
    double front_stop_distance(double now) const;
    double recent_yaw_rate() const;
    bool stopped_and_observed(double now);
    bool motion_stopped(double now);
    void obstacle_stop(double now);
    bool begin_turn(double heading, bool scanning, double now);
    Control service_turn(double now,double dt);
    bool fresh_observations_after(double stamp) const;
    uint32_t request_id_=0;
    unsigned rejoin_index_=0;
    int side_=0;
    double rejoin_heading_=0, total_distance_=0, detour_start_distance_=0, progress_at_=0;
    std::string sensor_problem(double now) const;
    LocalizationDiagnostics localization_evidence(const SensorFrame& frame,double now) const;
    void latch_localization(const std::string& reason,const LocalizationDiagnostics& evidence);
    Control fail(const std::string& reason);
    std::optional<Pose> pose_at(double stamp) const;
    bool free_world(Point p, const Bitmap& bitmap) const;
    bool segment_free(Point a, Point b, const Bitmap& bitmap) const;
    void request(Point goal, double now, const Bitmap& bitmap,
                 const std::string& purpose="route", int side=0);
    std::optional<Point> choose_frontier(const Bitmap& bitmap, double now) const;
    Control emit(double linear, double angular, double dt);
    Control waiting_control();
};
} // namespace robot_nav
