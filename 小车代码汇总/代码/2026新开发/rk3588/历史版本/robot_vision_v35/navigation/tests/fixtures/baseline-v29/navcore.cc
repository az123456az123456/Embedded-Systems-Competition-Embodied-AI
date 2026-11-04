#include "navcore.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace robot_nav {
namespace {
constexpr double PI=3.14159265358979323846;
double bounded(double v,double lo,double hi){return std::max(lo,std::min(hi,v));}
bool finite(double v){return std::isfinite(v);}
double distance(Point a,Point b){return std::hypot(a.first-b.first,a.second-b.second);}
Point xy(const Pose& p){return {p.x,p.y};}
bool positive(double v){return finite(v)&&v>0;}
using Polygon=std::vector<Point>;
Polygon clip_plane(const Polygon& input,double nx,double ny,double limit){
    Polygon out;if(input.empty())return out;
    Point a=input.back();double da=a.first*nx+a.second*ny-limit;
    for(Point b:input){
        double db=b.first*nx+b.second*ny-limit;
        if((da<=1e-10)!=(db<=1e-10)){
            const double t=da/(da-db);out.emplace_back(a.first+t*(b.first-a.first),a.second+t*(b.second-a.second));
        }
        if(db<=1e-10)out.push_back(b);
        a=b;da=db;
    }
    return out;
}
Polygon rectangle_corners(const Pose& p,const Config& c,double margin){
    Polygon out;const double co=std::cos(p.yaw),si=std::sin(p.yaw);
    for(Point q:{Point{c.body_front_m+margin,c.body_left_m+margin},Point{-c.body_rear_m-margin,c.body_left_m+margin},
                 Point{-c.body_rear_m-margin,-c.body_right_m-margin},Point{c.body_front_m+margin,-c.body_right_m-margin}})
        out.emplace_back(p.x+q.first*co-q.second*si,p.y+q.first*si+q.second*co);
    return out;
}
double cross(Point a,Point b,Point c){return (b.first-a.first)*(c.second-a.second)-(b.second-a.second)*(c.first-a.first);}
Polygon convex_hull(Polygon p){
    std::sort(p.begin(),p.end());p.erase(std::unique(p.begin(),p.end()),p.end());if(p.size()<3)return p;
    Polygon out(p.size()*2);size_t n=0;
    for(Point v:p){while(n>=2&&cross(out[n-2],out[n-1],v)<=0)--n;out[n++]=v;}
    const size_t lower=n;
    for(size_t i=p.size()-1;i-->0;){Point v=p[i];while(n>lower&&cross(out[n-2],out[n-1],v)<=0)--n;out[n++]=v;}
    out.resize(n-1);return out;
}
Polygon sweep_envelope(const Pose& from,const Pose& to,const Config& c,double margin){
    Polygon points=rectangle_corners(from,c,margin),end=rectangle_corners(to,c,margin);
    points.insert(points.end(),end.begin(),end.end());
    // End-point hull alone misses the corner arcs BETWEEN sampled angles.
    // Every point's arc deviates from its end-point chord by at most this
    // sagitta. A square dilation conservatively contains the circular bound.
    const double radius=std::hypot(std::max(c.body_front_m,c.body_rear_m)+margin,std::max(c.body_left_m,c.body_right_m)+margin);
    const double sagitta=radius*(1-std::cos(std::abs(wrap(to.yaw-from.yaw))*.5));
    if(sagitta>0){Polygon expanded;for(Point p:points)for(double x:{-sagitta,sagitta})for(double y:{-sagitta,sagitta})expanded.emplace_back(p.first+x,p.second+y);points=std::move(expanded);}
    return convex_hull(std::move(points));
}
Polygon clip_convex(Polygon p,const Polygon& hull){
    for(size_t i=0;i<hull.size()&&!p.empty();++i){const Point a=hull[i],b=hull[(i+1)%hull.size()];
        const double nx=b.second-a.second,ny=a.first-b.first;
        p=clip_plane(p,nx,ny,nx*a.first+ny*a.second);
    }return p;
}
bool has_area(const Polygon& p){
    double area=0;for(size_t i=0;i<p.size();++i){const auto& a=p[i];const auto& b=p[(i+1)%p.size()];area+=a.first*b.second-a.second*b.first;}
    return std::abs(area)>1e-12;
}
bool in_rectangle(Point p,const Pose& pose,const Config& c,double margin){
    const double dx=p.first-pose.x,dy=p.second-pose.y,co=std::cos(pose.yaw),si=std::sin(pose.yaw);
    const double x=dx*co+dy*si,y=-dx*si+dy*co;
    return x>=-c.body_rear_m-margin-1e-9&&x<=c.body_front_m+margin+1e-9&&
           y>=-c.body_right_m-margin-1e-9&&y<=c.body_left_m+margin+1e-9;
}
bool newly_swept(const Polygon& p,const Pose& current,const Config& c,double margin){
    if(!has_area(p))return false;
    for(Point q:p)if(!in_rectangle(q,current,c,margin))return true;
    return false;
}
}
double wrap(double v){ return std::atan2(std::sin(v),std::cos(v)); }
int32_t count_delta(uint32_t n,uint32_t o){
    uint32_t d=n-o;
    return d<=0x7fffffffU ? static_cast<int32_t>(d) : -static_cast<int32_t>(0xffffffffU-d)-1;
}
std::vector<std::string> Config::errors()const{
    std::vector<std::string> e;
    if(!configured&&!velocity_control)e.emplace_back("motion_calibration_not_confirmed");
    if(!(wheel_circumference_m>=.01&&wheel_circumference_m<=5))e.emplace_back("invalid_wheel_circumference_m");
    if(!finite(encoder_ticks_per_turn)||encoder_ticks_per_turn<1||encoder_ticks_per_turn>10000000)e.emplace_back("invalid_encoder_ticks_per_turn");
    if(!(track_m>=.03&&track_m<=2))e.emplace_back("invalid_track_m");
    if(!(robot_radius_m>=.01&&robot_radius_m<=2))e.emplace_back("invalid_robot_radius_m");
    if(!velocity_control&&!(drive_mps_per_full_pwm>=.01&&drive_mps_per_full_pwm<=5))e.emplace_back("invalid_drive_mps_per_full_pwm");
    if(velocity_control&&(!finite(drive_mps_per_full_pwm)||drive_mps_per_full_pwm<0||drive_mps_per_full_pwm>5))e.emplace_back("invalid_legacy_pwm_calibration");
    if(velocity_control&&(max_wheel_mps>.30||max_turn_rps>1.))e.emplace_back("velocity_exceeds_MOT4_limits");
    if(!positive(resolution_m)||resolution_m<.02||resolution_m>.5)e.emplace_back("invalid_resolution_m");
    unsigned left_count=0;for(unsigned i=0;i<4;++i)left_count+=(left_wheel_mask>>i)&1U;
    if(left_wheel_mask>15||left_count!=2)e.emplace_back("invalid_left_wheel_mask");
    for(double v:{body_front_m,body_rear_m,body_left_m,body_right_m})
        if(!positive(v)||v>2)e.emplace_back("invalid_body_rectangle");
    if(!(avoidance_step_rad>=5*PI/180&&avoidance_step_rad<=29*PI/180))e.emplace_back("invalid_avoidance_step");
    if(!(stop_dwell_s>=.1&&stop_dwell_s<=5&&scan_observation_dwell_s>=.1&&scan_observation_dwell_s<=5&&scan_max_s>=5&&scan_max_s<=120&&stopped_wheel_mps>0&&stopped_wheel_mps<=.03))e.emplace_back("invalid_scan_stop_limits");
    for(int s:encoder_signs)if(s!=-1&&s!=1)e.emplace_back("invalid_encoder_sign");
    for(const auto& m:sonar_mounts)for(double v:m)if(!finite(v))e.emplace_back("invalid_sonar_mount");
    if(!(positive(cruise_mps)&&cruise_mps<=detour_mps&&detour_mps<=max_wheel_mps))e.emplace_back("invalid_speed_limits");
    if(!(max_pwm>0&&max_pwm<=.60&&max_turn_rps>0&&max_turn_rps<=1.5))e.emplace_back("invalid_command_limits");
    if(!(finite(straight_speed_mps)&&straight_speed_mps>0&&straight_speed_mps<=.30&&
         finite(straight_max_pwm)&&straight_max_pwm>0&&straight_max_pwm<=.60&&
         finite(straight_replan_max_s)&&straight_replan_max_s>0&&straight_replan_max_s<=120))e.emplace_back("invalid_straight_assist_limits");
    if(!(positive(sensor_timeout_s)&&positive(sonar_timeout_s)&&positive(planner_timeout_s)&&positive(map_ttl_s)&&positive(sonar_wait_timeout_s)&&positive(avoid_clear_front_m)&&positive(avoid_rejoin_distance_m)))e.emplace_back("invalid_timeout");
    if(!(positive(accel_mps2)&&positive(brake_mps2)&&finite(sonar_latency_s)&&sonar_latency_s>=0&&finite(reaction_s)&&reaction_s>=0))e.emplace_back("invalid_motion_limits");
    if(!(finite(safety_margin_m)&&safety_margin_m>=0&&safety_margin_m<=1&&positive(stop_clearance_m)&&positive(side_clearance_m)))e.emplace_back("invalid_clearance");
    if(!(positive(sonar_min_m)&&sonar_max_m>sonar_min_m&&sonar_half_angle_rad>0&&sonar_half_angle_rad<PI/2))e.emplace_back("invalid_sonar_limits");
    if(!(positive(waypoint_tolerance_m)&&positive(heading_tolerance_rad)&&positive(no_progress_s)))e.emplace_back("invalid_tracking_limits");
    if(!(encoder_compare_window_s>=.10&&encoder_compare_window_s<=1&&encoder_min_compare_mps>=.005&&encoder_min_compare_mps<=.05))e.emplace_back("invalid_encoder_comparison_limits");
    if(!(bootstrap_speed_mps>=.01&&bootstrap_speed_mps<=.06&&bootstrap_step_m>=.04&&bootstrap_step_m<=.15&&bootstrap_max_distance_m>=bootstrap_step_m&&bootstrap_max_distance_m<=2&&bootstrap_max_s>=10&&bootstrap_max_s<=120))e.emplace_back("invalid_bootstrap_limits");
    return e;
}

OccupancyGrid::OccupancyGrid(const Config& c):config_(c),origin_(-32*c.resolution_m,-32*c.resolution_m){
    if(!positive(config_.resolution_m)||config_.resolution_m<.02||config_.resolution_m>.5)config_.resolution_m=.1;
    if(!finite(config_.robot_radius_m)||config_.robot_radius_m<0||config_.robot_radius_m>2)config_.robot_radius_m=0;
    if(!finite(config_.safety_margin_m)||config_.safety_margin_m<0||config_.safety_margin_m>1)config_.safety_margin_m=0;
    origin_={-32*config_.resolution_m,-32*config_.resolution_m};
    observed_.fill(-std::numeric_limits<double>::infinity());
}
Cell OccupancyGrid::cell(double x,double y)const{
    return {static_cast<int>(std::floor((x-origin_.first)/config_.resolution_m)),static_cast<int>(std::floor((y-origin_.second)/config_.resolution_m))};
}
Point OccupancyGrid::center(Cell c)const{return {origin_.first+(c.first+.5)*config_.resolution_m,origin_.second+(c.second+.5)*config_.resolution_m};}
bool OccupancyGrid::in_bounds(Cell c){return c.first>=0&&c.first<64&&c.second>=0&&c.second<64;}
uint16_t OccupancyGrid::index(Cell c){return static_cast<uint16_t>(c.second*64+c.first);}
bool OccupancyGrid::blocked(const Bitmap& b,unsigned i){return i>=4096||(b[i>>3]&(1U<<(i&7)));}
int OccupancyGrid::state(Cell c,double now)const{
    if(!in_bounds(c))return 1;
    unsigned i=index(c);
    if(now-observed_[i]>config_.map_ttl_s||now<observed_[i])return -1;
    return values_[i]<=-.5f?0:(values_[i]>=.7f?1:-1);
}
void OccupancyGrid::add(int x,int y,double e,double stamp){
    if(!in_bounds({x,y}))return;
    unsigned i=index({x,y});
    if(stamp-observed_[i]>config_.map_ttl_s)values_[i]=0;
    // A new echo immediately removes an old free-space claim. Otherwise a
    // saturated -4 free cell would hide a newly arrived obstacle for seconds.
    if(e>0&&values_[i]<0)values_[i]=0;
    values_[i]=static_cast<float>(bounded(values_[i]+e,-4,4));
    observed_[i]=std::max(observed_[i],stamp);
}
void OccupancyGrid::recenter(const Pose& p){
    Cell c=cell(p.x,p.y);
    if(c.first>=14&&c.first<50&&c.second>=14&&c.second<50)return;
    int sx=c.first-32,sy=c.second-32;
    auto old=values_;auto seen=observed_;
    values_.fill(0);observed_.fill(-std::numeric_limits<double>::infinity());
    for(int y=0;y<64;++y)for(int x=0;x<64;++x){
        Cell src{x+sx,y+sy};
        if(in_bounds(src)){values_[index({x,y})]=old[index(src)];observed_[index({x,y})]=seen[index(src)];}
    }
    origin_.first+=sx*config_.resolution_m;origin_.second+=sy*config_.resolution_m;++revision_;
}
void OccupancyGrid::mark_body(const Pose& p,double stamp){
    // Only cells wholly contained in the actual asymmetric rectangle are a
    // free prior. In particular, a circumcircle must not erase unknown corners
    // or grant clearance to the unobserved rear during a subsequent rotation.
    const double co=std::cos(p.yaw),si=std::sin(p.yaw),h=config_.resolution_m*.5;
    const double radius=std::hypot(std::max(config_.body_front_m,config_.body_rear_m),
                                   std::max(config_.body_left_m,config_.body_right_m));
    Cell c=cell(p.x,p.y);int n=static_cast<int>(std::ceil(radius/config_.resolution_m))+1;
    for(int y=std::max(0,c.second-n);y<std::min(64,c.second+n+1);++y)
        for(int x=std::max(0,c.first-n);x<std::min(64,c.first+n+1);++x){
            Point q=center({x,y});bool contained=true;
            for(double dx:{-h,h})for(double dy:{-h,h}){
                double wx=q.first+dx-p.x,wy=q.second+dy-p.y;
                double lx=wx*co+wy*si,ly=-wx*si+wy*co;
                contained=contained&&lx>=-config_.body_rear_m&&lx<=config_.body_front_m&&
                           ly>=-config_.body_right_m&&ly<=config_.body_left_m;
            }
            if(contained)add(x,y,-4,stamp);
        }
    ++revision_;
}
void OccupancyGrid::update_sonar(const Pose& p,unsigned ch,double d,double stamp){
    if(ch>=3||!finite(d)||d<config_.sonar_min_m||d>config_.sonar_max_m||!finite(stamp))return;
    const bool censored=d>SONAR_CENSORED_RANGE_M;
    if(censored)d=SONAR_CENSORED_RANGE_M;
    const auto& m=config_.sonar_mounts[ch];double co=std::cos(p.yaw),si=std::sin(p.yaw);
    Point s{p.x+m[0]*co-m[1]*si,p.y+m[0]*si+m[1]*co};
    double angle=p.yaw+m[2],band=std::max(.12,config_.resolution_m*1.5),half=config_.sonar_half_angle_rad;
    Cell a=cell(s.first-d-band,s.second-d-band),b=cell(s.first+d+band,s.second+d+band);
    for(int y=std::max(0,a.second);y<std::min(64,b.second+1);++y)for(int x=std::max(0,a.first);x<std::min(64,b.first+1);++x){
        Point q=center({x,y});double dx=q.first-s.first,dy=q.second-s.second;
        double length=std::hypot(dx,dy),offset=std::abs(wrap(std::atan2(dy,dx)-angle));
        if(length<config_.sonar_min_m||length>d+band)continue;
        // Narrower free cone, broad occupied echo arc; a single range is not a
        // precise object point. A valid >4m report only clears the observed
        // near cone: it has NO occupied endpoint and never clears beyond 4m.
        // Missing/invalid/no-echo returns stay unknown.
        if(offset<=half*.65&&length<d-band)add(x,y,-.55,stamp);
        else if(!censored&&offset<=half&&std::abs(length-d)<=band)add(x,y,.9,stamp);
    }
    ++revision_;
}
Bitmap OccupancyGrid::planning_bitmap(double now)const{
    double body_radius=std::hypot(std::max(config_.body_front_m,config_.body_rear_m),
                                 std::max(config_.body_left_m,config_.body_right_m));
    double r=std::max(config_.robot_radius_m,body_radius)+config_.safety_margin_m+config_.resolution_m/std::sqrt(2.);
    int n=static_cast<int>(std::ceil(r/config_.resolution_m));std::vector<Cell> offsets;
    for(int dy=-n;dy<=n;++dy)for(int dx=-n;dx<=n;++dx)if(std::hypot(dx*config_.resolution_m,dy*config_.resolution_m)<=r)offsets.emplace_back(dx,dy);
    std::array<bool,4096> free{};for(unsigned i=0;i<4096;++i)free[i]=state({int(i%64),int(i/64)},now)==0;
    Bitmap out;out.fill(0xff);
    for(int y=0;y<64;++y)for(int x=0;x<64;++x){
        bool safe=true;
        for(Cell d:offsets){Cell c{x+d.first,y+d.second};if(!in_bounds(c)||!free[index(c)]){safe=false;break;}}
        if(safe){unsigned i=index({x,y});out[i>>3]&=static_cast<uint8_t>(~(1U<<(i&7)));}
    }
    return out;
}
bool OccupancyGrid::import_observations(const std::vector<int8_t>& cells,Point origin,double stamp){
    if(cells.size()!=4096||!finite(origin.first)||!finite(origin.second)||!finite(stamp))return false;
    for(int8_t v:cells)if(v < -1||v>1)return false;
    origin_=origin;
    for(unsigned i=0;i<4096;++i){values_[i]=cells[i]==0?-4.f:(cells[i]==1?4.f:0.f);observed_[i]=cells[i]==-1?-std::numeric_limits<double>::infinity():stamp;}
    ++revision_;return true;
}

Navigator::Navigator(Config c):config_(std::move(c)),grid_(config_){sonar_stamp_.fill(-std::numeric_limits<double>::infinity());}
Control Navigator::stop(const std::string& why){
    bootstrap_active_=false;
    sonar_wait_started_.reset();sonar_recovery_wait_=false;sonar_wait_resume_state_.clear();
    bypass_=Bypass::None;bypass_turning_=false;
    straight_assist_=false;
    straight_turn_wait_resume_.clear();
    straight_replan_started_s_.reset();straight_replans_=0;
    enabled_=false;avoidance_active_=obstacle_stopping_=false;turn_active_=turn_preparing_=turn_settling_=false;state_="STOPPED";reason_=why;pending_.reset();path_.clear();command_=Control{};
    command_.state=state_;command_.reason=reason_;return command_;
}
Control Navigator::fail(const std::string& why){stop(why);state_="FAULT";command_.state=state_;return command_;}
void Navigator::set_goal(double x,double y){
    if(!finite(x)||!finite(y))throw std::invalid_argument("goal must be finite metres");
    stop("goal_changed");goal_=Point{x,y};exploration_=false;original_route_.clear();
}
void Navigator::explore(){stop("exploration_selected");goal_.reset();exploration_=true;}
bool Navigator::go_to(double x,double y,double now,std::string& error){
    if(!finite(x)||!finite(y)||std::abs(x)>10000||std::abs(y)>10000){error="invalid_goal";return false;}
    if(!config_.configured&&!config_.velocity_control){error="calibration_not_confirmed";return false;}
    auto errors=config_.errors();
    if(!errors.empty()){error=errors.front();return false;}
    error=start_problem(now);if(!error.empty())return false;
    const auto cell=grid_.cell(x,y);
    if(!grid_.in_bounds(cell)){error="goal_outside_local_map";return false;}
    const auto bits=grid_.planning_bitmap(now);
    if(grid_.blocked(bits,grid_.index(cell))){error="goal_not_verified_free";return false;}
    const auto start=grid_.cell(pose_.x,pose_.y);
    if(!grid_.in_bounds(start)||grid_.blocked(bits,grid_.index(start))){error="start_not_verified_free";return false;}
    const auto center=grid_.center(cell);set_goal(center.first,center.second);
    if(!this->start(now)){error=reason_;return false;}
    return true;
}
bool Navigator::set_wheel_layout(unsigned mask){
    unsigned count=0;for(unsigned i=0;i<4;++i)count+=(mask>>i)&1U;
    if(mask>15||count!=2)return false;
    if(mask==config_.left_wheel_mask)return true;
    config_.left_wheel_mask=mask;reset_localization();reason_=command_.reason="wheel_layout_changed_reset_map";return true;
}
void Navigator::reset_localization(){Config saved=config_;*this=Navigator(saved);reason_=command_.reason="map_reset_requires_new_observations";}
std::string Navigator::sensor_problem(double now)const{
    if(localization_lost_)return "localization_lost_reset_map_required";
    if(!frame_||!valid_odometry_)return "odometry_not_ready";
    const auto& f=*frame_;double age=now-f.timestamp_s;
    if(!finite(now)||age<-.02||age>config_.sensor_timeout_s)return "telemetry_stale";
    if(!f.imu_valid||!finite(f.yaw_rad)||!finite(f.imu_age_s)||f.imu_age_s<0||age+f.imu_age_s>config_.sensor_timeout_s)return "imu_invalid_or_stale";
    for(bool ok:f.encoder_ok)if(!ok)return "encoder_invalid";
    for(unsigned i=0;i<3;++i){
        auto d=f.sonar_m[i];double a=f.sonar_age_s[i];
        if(!d||!finite(*d)||*d<config_.sonar_min_m||*d>config_.sonar_max_m)return "sonar_invalid";
        if(!finite(a)||a<0||age+a>config_.sonar_timeout_s)return "sonar_stale";
    }
    return {};
}
bool Navigator::start(double now){
    std::string why=start_problem(now);
    if(!why.empty()){fail(why);return false;}
    if(!goal_&&!exploration_){fail("no_goal");return false;}
    straight_assist_=false;
    enabled_=true;path_.clear();original_route_.clear();pending_.reset();detour_origin_.reset();rejoin_point_.reset();detour_departure_heading_.reset();
    avoidance_active_=obstacle_stopping_=false;turn_active_=turn_preparing_=turn_settling_=false;last_motion_mps_=0;
    state_="PLANNING";reason_="awaiting_fpga";last_tick_=now;progress_at_=now;progress_pose_=pose_;command_=Control{};
    if(exploration_&&!goal_)initialize_bootstrap(now);
    return true;
}
bool Navigator::start_straight(double now){
    const std::string why=start_problem(now);
    if(!why.empty()){fail(why);return false;}
    stop("straight_assist_selected");
    straight_assist_=true;straight_origin_=pose_;exploration_=false;goal_.reset();
    path_.clear();original_route_.clear();detour_origin_.reset();rejoin_point_.reset();detour_departure_heading_.reset();
    enabled_=true;last_tick_=now;last_motion_mps_=0;progress_at_=now;progress_pose_=pose_;
    stopped_requested_s_=now;stationary_since_.reset();
    state_="STRAIGHT_ADVANCE";reason_="straight_sonar_gated_heading_captured";waiting_control();
    return true;
}
std::string Navigator::start_problem(double now)const{
    const auto errors=config_.errors();if(!errors.empty())return errors.front();
    const auto localization=localization_problem(now);if(!localization.empty())return localization;
    const auto sensor=sensor_problem(now);
    // A short range-source gap accepts only a zero-speed waiting session;
    // tick() cannot move until all genuine ranges recover. IMU/odom stay strict.
    return sensor=="sonar_invalid"||sensor=="sonar_stale"?std::string{}:sensor;
}
std::string Navigator::localization_problem(double now) const {
    if(localization_lost_)return localization_diagnostics_.first_reason.empty()?"localization_lost_reset_map_required":localization_diagnostics_.first_reason;
    if(!frame_||!valid_odometry_)return "odometry_not_ready";
    if(!finite(now))return "telemetry_timestamp_invalid";
    double age=now-frame_->timestamp_s;
    if(age<0||age>config_.sensor_timeout_s)return "telemetry_stale";
    if(!frame_->imu_valid||!finite(frame_->imu_age_s)||frame_->imu_age_s<0||frame_->imu_age_s+age>.30)return "imu_invalid_or_stale";
    return {};
}
bool Navigator::localization_valid(double now) const {return localization_problem(now).empty();}
LocalizationDiagnostics Navigator::localization_evidence(const SensorFrame& incoming,double now) const {
    LocalizationDiagnostics d;d.timestamp_s=now;d.sample_timestamp_s=incoming.timestamp_s;
    d.sequence=incoming.sequence;d.imu_valid=incoming.imu_valid;
    d.sample_has_device_time=incoming.device_timestamp_s.has_value();
    if(incoming.device_timestamp_s)d.device_timestamp_s=*incoming.device_timestamp_s;
    for(unsigned i=0;i<4;++i)if(!incoming.encoder_ok[i])d.encoder_invalid_mask|=uint8_t(1U<<i);
    if(frame_){
        d.previous_timestamp_s=frame_->timestamp_s;d.previous_sequence=frame_->sequence;
        d.host_dt_s=incoming.timestamp_s-frame_->timestamp_s;d.dt_s=d.host_dt_s;
        d.previous_has_device_time=frame_->device_timestamp_s.has_value();
        if(frame_->device_timestamp_s)d.previous_device_timestamp_s=*frame_->device_timestamp_s;
        if(incoming.device_timestamp_s&&frame_->device_timestamp_s){
            d.using_device_time=true;d.dt_s=*incoming.device_timestamp_s-*frame_->device_timestamp_s;
        }
    }
    return d;
}
void Navigator::latch_localization(const std::string& reason,const LocalizationDiagnostics& evidence){
    localization_lost_=true;valid_odometry_=false;
    if(!localization_diagnostics_.latched){
        localization_diagnostics_=evidence;localization_diagnostics_.latched=true;
        localization_diagnostics_.first_reason=reason;
    }
}
Control Navigator::invalidate_localization(const std::string& reason,double now){
    SensorFrame sample=frame_?*frame_:SensorFrame{};
    auto evidence=localization_evidence(sample,now);evidence.external_invalidation=true;
    if(localization_lost_||valid_odometry_||!history_.empty())latch_localization(reason,evidence);
    valid_odometry_=false;history_.clear();return fail(reason);
}
std::optional<Pose> Navigator::pose_at(double stamp)const{
    if(history_.empty()||stamp<history_.front().first-1e-6||stamp>history_.back().first+1e-6)return {};
    for(size_t i=1;i<history_.size();++i){
        const auto& a=history_[i-1];const auto& b=history_[i];
        if(stamp>=a.first&&stamp<=b.first){double f=(stamp-a.first)/(b.first-a.first);return Pose{a.second.x+f*(b.second.x-a.second.x),a.second.y+f*(b.second.y-a.second.y),wrap(a.second.yaw+f*wrap(b.second.yaw-a.second.yaw))};}
    }
    return history_.back().second;
}
bool Navigator::ingest(const SensorFrame& f,double now){
    unsigned layout_count=0;for(unsigned i=0;i<4;++i)layout_count+=(config_.left_wheel_mask>>i)&1U;
    if(config_.left_wheel_mask>15||layout_count!=2){fail("invalid_left_wheel_mask");return false;}
    auto evidence=localization_evidence(f,now);
    bool ok=f.imu_valid&&finite(f.timestamp_s)&&finite(f.yaw_rad)&&finite(now);
    for(bool v:f.encoder_ok)ok=ok&&v;
    if(!ok){
        if(localization_lost_||valid_odometry_)latch_localization("invalid_odometry_frame",evidence);
        valid_odometry_=false;fail("invalid_odometry_frame");return false;
    }
    if(f.device_timestamp_s&&!finite(*f.device_timestamp_s)){
        if(frame_||localization_lost_||valid_odometry_)latch_localization("sensor_device_timestamp_invalid",evidence);
        valid_odometry_=false;fail("sensor_device_timestamp_invalid");return false;
    }
    if(now-f.timestamp_s>config_.sensor_timeout_s||f.timestamp_s>now+.02){fail("telemetry_timestamp_invalid");return false;}
    if(frame_){
        const auto& old=*frame_;
        // Host receipt time still bounds freshness/gaps. It must never stand in
        // for elapsed physical sample time when a device timebase is available.
        if(evidence.host_dt_s>config_.sensor_timeout_s){
            latch_localization("odometry_gap",evidence);frame_=f;history_.clear();fail("odometry_gap");return false;
        }
        if(f.device_timestamp_s.has_value()!=old.device_timestamp_s.has_value()){
            latch_localization("sensor_timebase_changed",evidence);frame_=f;history_.clear();fail("sensor_timebase_changed");return false;
        }
        if(evidence.using_device_time&&!(evidence.dt_s>0)){
            latch_localization("sensor_device_timestamp_nonmonotonic",evidence);frame_=f;history_.clear();fail("sensor_device_timestamp_nonmonotonic");return false;
        }
        if(f.timestamp_s<=old.timestamp_s)return false;
        const double dt=evidence.dt_s;
        if(dt>config_.sensor_timeout_s){
            latch_localization("sensor_device_time_gap",evidence);frame_=f;history_.clear();fail("sensor_device_time_gap");return false;
        }
        if(!positive(config_.encoder_ticks_per_turn)||!positive(config_.wheel_circumference_m)){frame_=f;valid_odometry_=false;return false;}
        double scale=config_.wheel_circumference_m/config_.encoder_ticks_per_turn;
        std::array<double,4> d{};for(unsigned i=0;i<4;++i)d[i]=double(count_delta(f.counts[i],old.counts[i]))*config_.encoder_signs[i]*scale;
        double dyaw=wrap(f.yaw_rad-old.yaw_rad);
        evidence.dyaw_rad=dyaw;evidence.yaw_limit_rad=config_.max_measured_turn_rps*dt+.06;
        evidence.wheel_limit_m=config_.max_measured_wheel_mps*dt+.005;
        evidence.wheel_delta_m=d;evidence.motion_limits_evaluated=true;
        evidence.yaw_jump=std::abs(dyaw)>evidence.yaw_limit_rad;
        for(unsigned i=0;i<4;++i)if(std::abs(d[i])>evidence.wheel_limit_m)evidence.jump_wheel_mask|=uint8_t(1U<<i);
        if(evidence.jump_wheel_mask||evidence.yaw_jump){
            const std::string why=evidence.jump_wheel_mask?"encoder_jump":"imu_yaw_jump";
            latch_localization(why,evidence);frame_=f;history_.clear();fail(why);return false;
        }
        measured_yaw_rps_=dyaw/dt;
        for(unsigned i=0;i<4;++i)measured_wheel_mps_[i]=d[i]/dt;
        // Compare physical speed over a fixed-duration window. A few encoder
        // edges in a 1ms sonar-event packet are neither a stuck wheel nor a
        // meaningful relative-speed estimate at the fixed 60527.7 CPR.
        wheel_window_s_+=dt;for(unsigned i=0;i<4;++i)wheel_window_distance_[i]+=d[i];
        if(wheel_window_s_+1e-12>=config_.encoder_compare_window_s){
            evidence.wheel_comparison_window_s=wheel_window_s_;
            for(unsigned i=0;i<4;++i)evidence.window_wheel_mps[i]=wheel_window_distance_[i]/wheel_window_s_;
            std::array<unsigned,2> left{},right{};unsigned nl=0,nr=0;
            for(unsigned i=0;i<4;++i){if(config_.left_wheel_mask&(1U<<i))left[nl++]=i;else right[nr++]=i;}
            for(auto pair:{std::pair<unsigned,unsigned>{left[0],left[1]}, {right[0],right[1]}}){
                unsigned a=pair.first,b=pair.second;
                const double largest=std::max(std::abs(evidence.window_wheel_mps[a]),std::abs(evidence.window_wheel_mps[b]));
                const bool mismatch=largest+1e-12>=config_.encoder_min_compare_mps&&
                    std::abs(evidence.window_wheel_mps[a]-evidence.window_wheel_mps[b])>largest*config_.encoder_mismatch_ratio;
                for(unsigned i:{a,b})wheel_bad_s_[i]=mismatch?wheel_bad_s_[i]+wheel_window_s_:0.;
            }
            wheel_window_s_=0;wheel_window_distance_.fill(0);
        }
        if(*std::max_element(wheel_bad_s_.begin(),wheel_bad_s_.end())>=config_.encoder_fault_after_s){
            latch_localization("encoder_wheel_disagreement",evidence);frame_=f;fail("encoder_wheel_disagreement");return false;
        }
        double travel=(d[0]+d[1]+d[2]+d[3])*.25;
        pose_.x+=travel*std::cos(pose_.yaw+dyaw*.5);pose_.y+=travel*std::sin(pose_.yaw+dyaw*.5);total_distance_+=std::abs(travel);valid_odometry_=true;
    }
    pose_.yaw=wrap(f.yaw_rad);frame_=f;history_.emplace_back(f.timestamp_s,pose_);
    history_.erase(history_.begin(),std::lower_bound(history_.begin(),history_.end(),f.timestamp_s-2.,[](const auto& p,double t){return p.first<t;}));
    grid_.recenter(pose_);grid_.mark_body(pose_,f.timestamp_s);
    for(unsigned i=0;i<3;++i){
        auto d=f.sonar_m[i];double age=f.sonar_age_s[i];
        if(!d||!finite(*d)||!finite(age)||age<0||age>config_.sonar_timeout_s||*d<config_.sonar_min_m||*d>config_.sonar_max_m)continue;
        if(f.sonar_observation_ids_valid&&have_sonar_id_[i]){
            const uint32_t advance=f.sonar_observation_id[i]-sonar_id_[i];
            if(!advance||advance>=0x80000000u)continue;
        }
        // The upstream age is the age of receipt. Account for A02 processed
        // measurement latency; delayed ranges must not be projected at today's pose.
        double stamp=f.timestamp_s-age-config_.sonar_latency_s;
        if(stamp<=sonar_stamp_[i]+1e-6)continue;
        auto p=pose_at(stamp);if(p){grid_.update_sonar(*p,i,*d,stamp);sonar_stamp_[i]=stamp;
            if(f.sonar_observation_ids_valid){sonar_id_[i]=f.sonar_observation_id[i];have_sonar_id_[i]=true;}}
    }
    if(trajectory_.empty()||distance(xy(pose_),xy(trajectory_.back()))>=.02){trajectory_.push_back(pose_);if(trajectory_.size()>2000)trajectory_.erase(trajectory_.begin(),trajectory_.begin()+100);}
    return valid_odometry_;
}
bool Navigator::free_world(Point p,const Bitmap& bits)const{Cell c=grid_.cell(p.first,p.second);return grid_.in_bounds(c)&&!grid_.blocked(bits,grid_.index(c));}
bool Navigator::segment_free(Point a,Point b,const Bitmap& bits)const{
    if(!free_world(a,bits)||!free_world(b,bits))return false;
    // Check EVERY grid square touched by the segment, including corner grazes.
    // Fixed-distance samples can miss a short crossing of an inflated obstacle
    // cell and incorrectly approve a shortcut around a tight A* corner.
    Cell low=grid_.cell(std::min(a.first,b.first),std::min(a.second,b.second));
    Cell high=grid_.cell(std::max(a.first,b.first),std::max(a.second,b.second));
    const auto origin=grid_.origin();const double res=grid_.resolution();
    const double dx=b.first-a.first,dy=b.second-a.second;
    for(int y=low.second-1;y<=high.second+1;++y)for(int x=low.first-1;x<=high.first+1;++x){
        if(grid_.in_bounds({x,y})&&!grid_.blocked(bits,grid_.index({x,y})))continue;
        double enter=0,exit=1;bool hit=true;
        for(auto axis:{std::array<double,4>{a.first,dx,origin.first+x*res,origin.first+(x+1)*res},
                       std::array<double,4>{a.second,dy,origin.second+y*res,origin.second+(y+1)*res}}){
            if(std::abs(axis[1])<1e-12){if(axis[0]<axis[2]-1e-12||axis[0]>axis[3]+1e-12)hit=false;continue;}
            double near=(axis[2]-axis[0])/axis[1],far=(axis[3]-axis[0])/axis[1];
            if(near>far)std::swap(near,far);
            enter=std::max(enter,near);exit=std::min(exit,far);
            if(enter>exit+1e-12)hit=false;
        }
        if(hit&&enter<=1+1e-12&&exit>=-1e-12)return false;
    }
    return true;
}
void Navigator::request(Point target,double now,const Bitmap& bits,const std::string& purpose,int side){
    Cell a=grid_.cell(pose_.x,pose_.y),b=grid_.cell(target.first,target.second);
    if(!grid_.in_bounds(a)||!grid_.in_bounds(b)){fail("goal_outside_local_map");return;}
    unsigned start=grid_.index(a),goal=grid_.index(b);
    if(grid_.blocked(bits,start)||grid_.blocked(bits,goal)){fail("start_or_goal_not_verified_free");return;}
    if(++request_id_==0)++request_id_;
    PlannerRequest r;r.request_id=request_id_;r.map_revision=grid_.revision();r.blocked=bits;r.start=static_cast<uint16_t>(start);r.goal=static_cast<uint16_t>(goal);
    r.origin=grid_.origin();r.resolution=grid_.resolution();r.created_s=now;r.purpose=purpose;r.side=side;pending_=r;
    state_="WAIT_PLAN";reason_="awaiting_fpga_"+purpose;
}
Control Navigator::reject_plan(uint32_t id,const std::string& why){if(pending_&&pending_->request_id==id)return fail("planner_"+why);return command_;}
bool Navigator::accept_plan(uint32_t id,uint32_t revision,const std::vector<uint16_t>& cells,double now){
    if(!enabled_||!pending_||pending_->request_id!=id||pending_->map_revision!=revision)return false;
    auto r=*pending_;
    if(now-r.created_s>config_.planner_timeout_s||now<r.created_s){fail("planner_timeout");return false;}
    const auto why=sensor_problem(now);
    const bool range_wait=why=="sonar_invalid"||why=="sonar_stale";
    if(!why.empty()&&!range_wait){fail(why);return false;}
    std::set<uint16_t> unique(cells.begin(),cells.end());
    if(cells.empty()||cells.size()>4096||cells.front()!=r.start||cells.back()!=r.goal||unique.size()!=cells.size()){fail("planner_invalid_endpoints_or_loop");return false;}
    for(unsigned c:cells)if(grid_.blocked(r.blocked,c)){fail("planner_path_hits_blocked_cell");return false;}
    for(size_t i=1;i<cells.size();++i){unsigned a=cells[i-1],b=cells[i];if(std::abs(int(a%64)-int(b%64))+std::abs(int(a/64)-int(b/64))!=1){fail("planner_path_not_four_connected");return false;}}
    std::vector<Point> points;points.reserve(cells.size());for(unsigned c:cells)points.emplace_back(r.origin.first+(c%64+.5)*r.resolution,r.origin.second+(c/64+.5)*r.resolution);
    Bitmap current=grid_.planning_bitmap(now);
    bool safe=segment_free(xy(pose_),points.front(),current);
    for(Point p:points)safe=safe&&free_world(p,current);
    if(!safe){fail("planner_path_no_longer_safe");return false;}
    if(r.purpose=="detour"&&detour_origin_){
        if(!detour_departure_heading_){fail("detour_missing_observed_direction");return false;}
        bool departed=false;
        for(Point p:points){
            double d=distance(p,xy(detour_departure_origin_));if(d<.25)continue;
            double heading=std::atan2(p.second-detour_departure_origin_.y,p.first-detour_departure_origin_.x);
            double coarse_allowance=std::atan2(config_.resolution_m/std::sqrt(2.),d);
            if(std::abs(wrap(heading-*detour_departure_heading_))>PI/2+coarse_allowance){
                fail("detour_did_not_use_selected_direction");return false;
            }
            departed=true;break;
        }
        if(!departed){fail("detour_departure_too_short");return false;}
        state_="DETOUR";detour_start_distance_=total_distance_;
    }else if(r.purpose=="rejoin"){state_="DETOUR";detour_start_distance_=total_distance_;}
    else{state_="FOLLOW_ROUTE";original_route_=points;}
    path_=std::move(points);reason_="fpga_path_verified";pending_.reset();progress_at_=now;progress_pose_=pose_;
    if(range_wait||sonar_recovery_wait_){
        const auto accepted_state=state_;
        if(range_wait&&!sonar_wait_started_)sonar_wait_started_=now;
        sonar_wait_resume_state_=accepted_state;
        state_=range_wait?"SENSOR_WAIT":"SENSOR_RECOVERY";
        reason_=range_wait?why+"_zero_wait":"waiting_post_stop_observations";
        waiting_control(); // accepting a safe plan never authorizes pre-recovery motion
    }
    return true;
}
std::optional<Point> Navigator::choose_frontier(const Bitmap& bitmap,double now)const{
    // Choose a known free observation goal; do NOT search a host-side route.
    double best=-std::numeric_limits<double>::infinity();std::optional<Point> result;
    int r=std::max(2,static_cast<int>(std::ceil((config_.robot_radius_m+config_.safety_margin_m)/grid_.resolution()))+1);
    for(unsigned i=0;i<4096;++i){
        if(grid_.blocked(bitmap,i))continue;
        Cell c{int(i%64),int(i/64)};Point p=grid_.center(c);double d=distance(p,xy(pose_));if(d<.30)continue;
        int unknown=0;for(Cell o:{Cell{r,0},Cell{-r,0},Cell{0,r},Cell{0,-r}})unknown+=grid_.state({c.first+o.first,c.second+o.second},now)==-1;
        double score=unknown-d*.3;if(unknown&&score>best){best=score;result=p;}
    }
    return result;
}
double Navigator::safety_speed() const{
    double speed=std::max(last_motion_mps_,std::abs(command_.linear_mps));
    for(double v:measured_wheel_mps_)speed=std::max(speed,std::abs(v));
    return speed;
}
double Navigator::reaction_delay(double now) const{
    return config_.reaction_s+config_.sonar_latency_s+frame_->sonar_age_s[0]+
           std::max(0.,now-frame_->timestamp_s);
}
double Navigator::front_stop_distance(double now) const{
    double v=safety_speed();return config_.stop_clearance_m+v*reaction_delay(now)+v*v/(2*config_.brake_mps2);
}
bool Navigator::fresh_observations_after(double stamp) const{
    for(unsigned i=0;i<3;++i){
        if(sonar_stamp_[i]<stamp)return false;
        if(frame_->sonar_observation_ids_valid&&(!have_sonar_id_[i]||sonar_id_[i]!=frame_->sonar_observation_id[i]))return false;
    }
    return true;
}
bool Navigator::stopped_and_observed(double now){
    bool still=frame_->timestamp_s>stopped_requested_s_&&std::abs(measured_yaw_rps_)<=.08;
    for(double v:measured_wheel_mps_)still=still&&std::abs(v)<=config_.stopped_wheel_mps;
    if(!still){stationary_since_.reset();reason_="waiting_encoder_stop";return false;}
    if(!stationary_since_)stationary_since_=now;
    if(now-*stationary_since_<config_.stop_dwell_s){reason_="waiting_stop_dwell";return false;}
    if(now-*stationary_since_<config_.scan_observation_dwell_s||!fresh_observations_after(*stationary_since_)){
        reason_="waiting_fresh_sonar_after_stop";return false;
    }
    last_motion_mps_=0;return true;
}
bool Navigator::footprint_increment_free(const Pose& current,const Pose& from,const Pose& target,double now,bool allow_unknown) const{
    const double margin=config_.safety_margin_m;
    const Polygon body=sweep_envelope(from,target,config_,0),padding=sweep_envelope(from,target,config_,margin);
    double minx=std::numeric_limits<double>::infinity(),miny=minx,maxx=-minx,maxy=-minx;
    for(Point p:padding){minx=std::min(minx,p.first);miny=std::min(miny,p.second);maxx=std::max(maxx,p.first);maxy=std::max(maxy,p.second);}
    const Cell low=grid_.cell(minx,miny),high=grid_.cell(maxx,maxy);
    if(!grid_.in_bounds(low)||!grid_.in_bounds(high))return false;
    const Point origin=grid_.origin();const double r=grid_.resolution();
    for(int y=low.second;y<=high.second;++y)for(int x=low.first;x<=high.first;++x){
        const double ax=origin.first+x*r,ay=origin.second+y*r;
        const Polygon square{{ax,ay},{ax+r,ay},{ax+r,ay+r},{ax,ay+r}};
        const int state=grid_.state({x,y},now);
        if(state==0)continue;
        // Clip the actual body against the cell. Testing whole intersecting
        // cells would either reject the old body's unknown boundary or exempt
        // a newly entered unknown strip in a partially occupied grid cell.
        if(newly_swept(clip_convex(square,body),current,config_,0)){
            if(state==1||!allow_unknown)return false;
        }
        // A known obstacle in the 80mm margin is never grandfathered as clear
        // merely because it was already in the current padding. Only actual
        // current body coverage is exempt; unknown padding remains unasserted.
        if(state==1&&newly_swept(clip_convex(square,padding),current,config_,0))return false;
    }
    return true;
}
bool Navigator::motion_increment_free(const Pose& target,double now,bool allow_unknown)const{
    if(!finite(now)||!finite(target.x)||!finite(target.y)||!finite(target.yaw))return false;
    const double length=distance(xy(pose_),xy(target)),angle=wrap(target.yaw-pose_.yaw);
    if(length>2)return false;
    const double dx=target.x-pose_.x,dy=target.y-pose_.y;
    if(allow_unknown&&(std::abs(angle)>1e-9||dx*std::cos(pose_.yaw)+dy*std::sin(pose_.yaw)<-1e-9||
       std::abs(-dx*std::sin(pose_.yaw)+dy*std::cos(pose_.yaw))>1e-9))return false;
    const unsigned n=std::max(1U,unsigned(std::ceil(std::max(length/(grid_.resolution()*.25),std::abs(angle)/(2*PI/180)))));
    Pose previous=pose_;
    for(unsigned i=1;i<=n;++i){
        const double fraction=double(i)/n;
        const Pose p{pose_.x+(target.x-pose_.x)*fraction,pose_.y+(target.y-pose_.y)*fraction,wrap(pose_.yaw+angle*fraction)};
        if(!footprint_increment_free(pose_,previous,p,now,allow_unknown))return false;
        previous=p;
    }
    return true;
}
bool Navigator::footprint_free(const Pose& p,double now) const{
    // Conservative rectangle/cell overlap test, including the left extension
    // and safety margin. Unknown, expired and out-of-map space all block turns.
    double co=std::cos(p.yaw),si=std::sin(p.yaw),h=grid_.resolution()*.5;
    double front=config_.body_front_m+config_.safety_margin_m;
    double rear=config_.body_rear_m+config_.safety_margin_m;
    double left=config_.body_left_m+config_.safety_margin_m;
    double right=config_.body_right_m+config_.safety_margin_m;
    double radius=std::hypot(std::max(front,rear),std::max(left,right));
    Cell a=grid_.cell(p.x-radius,p.y-radius),b=grid_.cell(p.x+radius,p.y+radius);
    if(!grid_.in_bounds(a)||!grid_.in_bounds(b))return false;
    const double projection=h*(std::abs(co)+std::abs(si));
    for(int y=a.second;y<=b.second;++y)for(int x=a.first;x<=b.first;++x){
        auto q=grid_.center({x,y});double wx=q.first-p.x,wy=q.second-p.y;
        double lx=wx*co+wy*si,ly=-wx*si+wy*co;
        if(lx>=-rear-projection&&lx<=front+projection&&ly>=-right-projection&&ly<=left+projection)
            if(grid_.state({x,y},now)!=0)return false;
    }
    return true;
}
bool Navigator::rotation_free(double target,double now) const{
    Pose p=pose_;p.yaw=target;return motion_increment_free(p,now,false);
}
void Navigator::initialize_bootstrap(double now){
    bootstrap_active_=true;bootstrap_origin_=bootstrap_step_origin_=bootstrap_step_target_=pose_;
    bootstrap_forward_s_=0;bootstrap_distance_start_=total_distance_;
    stopped_requested_s_=now;stationary_since_.reset();
    state_="BOOTSTRAP_STOP";reason_="observe_before_bounded_forward_exploration";
}
Control Navigator::service_bootstrap(double now,double dt){
    // This is a bounded local sensor reaction, not a host route search. A map
    // observation is written ONLY by actual sonar or measured body coverage.
    Bitmap bits=grid_.planning_bitmap(now);
    if(free_world(xy(pose_),bits)){
        auto frontier=choose_frontier(bits,now);
        if(frontier){bootstrap_active_=false;goal_=frontier;request(*goal_,now,bits);return waiting_control();}
    }
    if(*frame_->sonar_m[1]<=config_.side_clearance_m||*frame_->sonar_m[2]<=config_.side_clearance_m){
        obstacle_stop(now);reason_="bootstrap_side_clearance_stop_first";return waiting_control();
    }
    if(bootstrap_forward_s_>=config_.bootstrap_max_s||
       total_distance_-bootstrap_distance_start_>=config_.bootstrap_max_distance_m){
        state_="BOOTSTRAP_WAIT";reason_="bounded_bootstrap_limit_waiting_verified_corridor";
        return waiting_control();
    }
    if(std::abs(wrap(pose_.yaw-bootstrap_origin_.yaw))>5*PI/180){
        state_="BOOTSTRAP_WAIT";reason_="bootstrap_heading_changed_waiting_verified_turn";
        if(rotation_free(bootstrap_origin_.yaw,now))begin_turn(bootstrap_origin_.yaw,false,now);
        return waiting_control();
    }
    if(state_!="BOOTSTRAP_ADVANCE"){
        if(!stopped_and_observed(now))return waiting_control();
        bootstrap_step_origin_=pose_;bootstrap_step_target_=pose_;
        const double remain=config_.bootstrap_max_distance_m-(total_distance_-bootstrap_distance_start_);
        const double step=std::min(config_.bootstrap_step_m,remain);
        bootstrap_step_target_.x+=step*std::cos(pose_.yaw);bootstrap_step_target_.y+=step*std::sin(pose_.yaw);
        if(!motion_increment_free(bootstrap_step_target_,now,true)){
            obstacle_stop(now);reason_="known_obstacle_in_bootstrap_sweep";return waiting_control();
        }
        state_="BOOTSTRAP_ADVANCE";reason_="bounded_forward_sonar_gated_unknown_not_freed";
        progress_at_=now;progress_pose_=pose_;
    }
    const double travelled=distance(xy(pose_),xy(bootstrap_step_origin_));
    const double step=distance(xy(bootstrap_step_target_),xy(bootstrap_step_origin_));
    if(travelled>=std::max(.01,step-.01)){
        stopped_requested_s_=now;stationary_since_.reset();state_="BOOTSTRAP_STOP";reason_="stop_and_reobserve_after_short_step";
        return waiting_control();
    }
    // The motor command is straight in the CURRENT measured heading. Reusing
    // a fixed world target after tiny IMU/odometry drift would accidentally ask
    // the unknown-space guard to approve a rotation or lateral translation.
    Pose remaining=pose_;const double length=std::max(0.,step-travelled);
    remaining.x+=length*std::cos(pose_.yaw);remaining.y+=length*std::sin(pose_.yaw);
    if(!motion_increment_free(remaining,now,true)){
        obstacle_stop(now);reason_="known_obstacle_in_bootstrap_sweep";return waiting_control();
    }
    if(distance(xy(pose_),xy(progress_pose_))>.02){progress_pose_=pose_;progress_at_=now;}
    if(now-progress_at_>config_.no_progress_s)return fail("no_odometry_progress");
    const double available=std::max(0.,*frame_->sonar_m[0]-config_.stop_clearance_m),b=config_.brake_mps2,reaction=reaction_delay(now);
    const double safe=std::max(0.,-b*reaction+std::sqrt(b*b*reaction*reaction+2*b*available));
    reason_="bounded_forward_sonar_gated_unknown_not_freed";
    return emit(std::min({config_.bootstrap_speed_mps,config_.cruise_mps,safe}),0,dt);
}
Control Navigator::service_straight(double now,double dt){
    if(*frame_->sonar_m[1]<=config_.side_clearance_m||*frame_->sonar_m[2]<=config_.side_clearance_m){
        obstacle_stop(now);reason_="straight_side_clearance_stop_first";return waiting_control();
    }
    const double error=wrap(straight_origin_.yaw-pose_.yaw);
    if(std::abs(error)>10*PI/180){
        // A larger heading change remains a separately verified stopped
        // turn. The direct-drive request never authorizes a blind spin.
        if(rotation_free(straight_origin_.yaw,now)){
            begin_turn(straight_origin_.yaw,false,now);return waiting_control();
        }
        state_="STRAIGHT_WAIT";reason_="straight_heading_sweep_not_verified";
        return waiting_control();
    }
    if(state_!="STRAIGHT_ADVANCE"){
        progress_at_=now;progress_pose_=pose_;
    }
    const double available=std::max(0.,*frame_->sonar_m[0]-config_.stop_clearance_m);
    const double b=config_.brake_mps2,reaction=reaction_delay(now);
    const double safe=std::max(0.,-b*reaction+std::sqrt(b*b*reaction*reaction+2*b*available));
    const double speed=std::min({config_.straight_speed_mps,config_.max_wheel_mps,safe});
    // User-selected direct forward assist trusts current healthy ranges.
    // Historical occupied/unknown cells are retained for display/planning,
    // but do not create a pre-observation barrier or block this translation.
    if(distance(xy(pose_),xy(progress_pose_))>.02){progress_pose_=pose_;progress_at_=now;}
    if(now-progress_at_>config_.no_progress_s)return fail("no_odometry_progress");
    // emit clamps the correction after acceleration so both wheels remain
    // forward, with the inner wheel at least half the actual linear speed.
    state_="STRAIGHT_ADVANCE";
    reason_="straight_sonar_gated_heading_captured";
    return emit(speed,bounded(error*1.8,-.12,.12),dt);
}
#include "navcore_reactive.inc"
void Navigator::obstacle_stop(double now){
    straight_turn_wait_resume_.clear();
    if(exploration_&&original_route_.empty()&&path_.empty()&&!bootstrap_active_)initialize_bootstrap(now);
    last_motion_mps_=std::max(last_motion_mps_,std::abs(command_.linear_mps));
    if(!detour_origin_)detour_origin_=pose_; // preserve route frame before any scan
    pending_.reset();turn_active_=turn_preparing_=turn_settling_=false;
    avoidance_active_=obstacle_stopping_=true;stopped_requested_s_=now;stationary_since_.reset();
    scan_started_s_=now;scan_travel_rad_=0;state_="OBSTACLE_STOP";reason_="front_obstacle_stop_first";
    command_=Control{};
    // Capture heading/route before local motion. No return-point search occurs
    // until the new-direction measured .50m clearance leg is complete.
    if(original_route_.empty()&&!path_.empty())original_route_=path_;
    bypass_heading_=straight_assist_?straight_origin_.yaw:pose_.yaw;
    bypass_started_=now;bypass_motion_s_=0;bypass_scan_=0;bypass_attempts_=0;bypass_side_=0;
    bypass_clear_samples_=0;bypass_clear_id_.reset();
    bypass_departure_distance_=bypass_parallel_distance_=0;
    bypass_rotated_=bypass_turning_=false;
    bypass_=Bypass::Stop;bypass_after_stop_=Bypass::Select;
}
bool Navigator::begin_turn(double heading,bool scan,double now){
    double delta=wrap(heading-pose_.yaw);
    turn_target_yaw_=wrap(pose_.yaw+bounded(delta,-config_.avoidance_step_rad,config_.avoidance_step_rad));
    if(!rotation_free(turn_target_yaw_,now)){
        if(straight_assist_){
            straight_turn_wait_resume_=state_;state_="STRAIGHT_WAIT";
            reason_="straight_planned_turn_sweep_not_observed";progress_at_=now;progress_pose_=pose_;waiting_control();return false;
        }
        fail("rotation_sweep_not_verified_free");return false;
    }
    straight_turn_wait_resume_.clear();
    turn_resume_state_=state_;turn_is_scan_=scan;turn_preparing_=true;turn_active_=turn_settling_=false;
    stopped_requested_s_=now;stationary_since_.reset();state_=scan?"SCAN_STOP":"TURN_STOP";
    reason_="stop_before_turn_segment";command_=Control{};return true;
}
Control Navigator::service_turn(double now,double dt){
    if(turn_is_scan_&&(now-scan_started_s_>config_.scan_max_s||scan_travel_rad_>=2*PI))return fail("clockwise_scan_exhausted_no_verified_corridor");
    if(turn_preparing_){
        if(!stopped_and_observed(now))return waiting_control();
        double delta=wrap(turn_target_yaw_-pose_.yaw);
        turn_target_yaw_=wrap(pose_.yaw+bounded(delta,-config_.avoidance_step_rad,config_.avoidance_step_rad));
        if(!rotation_free(turn_target_yaw_,now))return fail("rotation_sweep_not_verified_free");
        turn_start_yaw_=turn_last_yaw_=pose_.yaw;turn_travel_rad_=0;
        turn_preparing_=false;turn_active_=true;state_=turn_is_scan_?(delta<0?"SCAN_CLOCKWISE":"SCAN_COUNTERCLOCKWISE"):"TURN_SEGMENT";
    }
    if(turn_active_){
        double travelled=std::abs(wrap(pose_.yaw-turn_last_yaw_));turn_travel_rad_+=travelled;turn_last_yaw_=pose_.yaw;
        if(turn_is_scan_)scan_travel_rad_+=travelled;
        if(!rotation_free(turn_target_yaw_,now))return fail("rotation_sweep_no_longer_verified_free");
        double err=wrap(turn_target_yaw_-pose_.yaw);
        const double desired=wrap(turn_target_yaw_-turn_start_yaw_);
        bool passed=desired*wrap(pose_.yaw-turn_start_yaw_)>=desired*desired;
        if(std::abs(err)<=2*PI/180||passed){
            turn_active_=false;turn_settling_=true;stopped_requested_s_=now;stationary_since_.reset();
            state_=turn_is_scan_?"SCAN_OBSERVE":"TURN_OBSERVE";reason_="stop_after_turn_segment";return waiting_control();
        }
        reason_=turn_is_scan_?(desired<0?"clockwise_scan_under_30deg":"counterclockwise_scan_under_30deg"):"heading_segment_under_30deg";
        // Bounded near-target speed reduces physical overshoot before stop.
        return emit(0,bounded(err*2.,-.45,.45),dt);
    }
    if(turn_settling_){
        if(!stopped_and_observed(now))return waiting_control();
        turn_settling_=false;progress_at_=now;progress_pose_=pose_;
        if(turn_is_scan_){
            obstacle_stopping_=true;state_="OBSTACLE_STOP";reason_="fresh_scan_echoes_ready";
            return service_bypass(now,dt);
        }
        state_=turn_resume_state_;reason_="turn_segment_stopped_and_observed";return waiting_control();
    }
    return waiting_control();
}
Control Navigator::waiting_control(){command_=Control{};command_.state=state_;command_.reason=reason_;command_.planner_request=pending_;return command_;}
Control Navigator::emit(double linear,double angular,double dt){
    // No reverse translation. Route turns use the verified-map sweep guard;
    // the explicit local bypass uses fresh ranges plus known-obstacle sweeps.
    linear=bounded(linear,0,config_.max_wheel_mps);
    if(linear>command_.linear_mps)linear=std::min(linear,command_.linear_mps+config_.accel_mps2*dt);
    angular=bounded(angular,-config_.max_turn_rps,config_.max_turn_rps);
    if(straight_assist_&&state_=="STRAIGHT_ADVANCE"){
        // Based on the ACTUAL post-ramp speed, not the .3m/s request. This
        // also produces zero angular command at a zero-speed first tick.
        const double correction_limit=linear/config_.track_m;
        angular=bounded(angular,-correction_limit,correction_limit);
    }
    double left=linear-angular*config_.track_m*.5,right=linear+angular*config_.track_m*.5;
    double scale=std::max({1.,std::abs(left)/config_.max_wheel_mps,std::abs(right)/config_.max_wheel_mps});
    if(config_.velocity_control){
        // Physical setpoints go to MOT4. Legacy PWM calibration cannot shrink
        // or amplify a closed-loop velocity command. Diagnostic duties stay 0.
        command_=Control{};command_.linear_mps=linear/scale;command_.angular_rps=angular/scale;
        command_.state=state_;command_.reason=reason_;last_motion_mps_=std::abs(command_.linear_mps);return command_;
    }
    left/=scale*config_.drive_mps_per_full_pwm;right/=scale*config_.drive_mps_per_full_pwm;
    const double pwm_limit=straight_assist_?config_.straight_max_pwm:config_.max_pwm;
    double ps=std::max({1.,std::abs(left)/pwm_limit,std::abs(right)/pwm_limit});
    command_=Control{};command_.left=left/ps;command_.right=right/ps;command_.linear_mps=linear/scale/ps;command_.angular_rps=angular/scale/ps;command_.state=state_;command_.reason=reason_;last_motion_mps_=std::abs(command_.linear_mps);return command_;
}
Control Navigator::tick(double now){
    if(!enabled_)return command_;
    const auto why=sensor_problem(now);
    if(!why.empty()&&why!="sonar_invalid"&&why!="sonar_stale")return fail(why);
    double dt=last_tick_?now-*last_tick_:.02;
    if(dt<0||dt>config_.sensor_timeout_s)return fail("control_watchdog");
    last_tick_=now;
    if(why=="sonar_invalid"||why=="sonar_stale"){
        if(!sonar_wait_started_){sonar_wait_started_=now;if(!sonar_recovery_wait_)sonar_wait_resume_state_=state_;}
        if(now-*sonar_wait_started_>=config_.sonar_wait_timeout_s)return fail("sonar_unavailable_timeout");
        state_="SENSOR_WAIT";reason_=why+"_zero_wait";
        progress_at_=now;progress_pose_=pose_;
        // The owner lease, CPU/ACK and localization remain independently strict.
        // Neither this wait nor repeated old packets refresh a source timestamp.
        return waiting_control();
    }
    if(!why.empty())return fail(why);
    if(sonar_wait_started_){
        sonar_wait_started_.reset();sonar_recovery_wait_=true;
        stopped_requested_s_=now;stationary_since_.reset();
        progress_at_=now;progress_pose_=pose_;
        if(bypass_==Bypass::TurnOut||bypass_==Bypass::Select){
            bypass_=Bypass::Select;bypass_clear_samples_=0;bypass_clear_id_.reset();
        }
    }
    if(sonar_recovery_wait_){
        state_="SENSOR_RECOVERY";
        if(!bypass_stopped(now))return waiting_control();
        sonar_recovery_wait_=false;progress_at_=now;progress_pose_=pose_;
        state_=sonar_wait_resume_state_;sonar_wait_resume_state_.clear();
    }
    if(bypass_!=Bypass::None&&(std::abs(command_.linear_mps)>1e-9||std::abs(command_.angular_rps)>1e-9))
        bypass_motion_s_+=dt;
    // Account for the interval actually covered by the previous forward
    // command. Charging wall time here used up the entire bootstrap budget
    // while safely waiting for a real obstacle echo arc to clear/expire; when
    // clearance was finally observed the car could never resume. A stopped
    // BOOTSTRAP_WAIT/STOP or observation interval is not a movement attempt.
    // The independent measured-distance and no-progress limits remain active.
    if(bootstrap_active_&&command_.state=="BOOTSTRAP_ADVANCE"&&command_.linear_mps>0)
        bootstrap_forward_s_+=dt;
    const bool front_blocked=*frame_->sonar_m[0]<=front_stop_distance(now);
    const bool straight_side_blocked=straight_assist_&&
        (*frame_->sonar_m[1]<=config_.side_clearance_m||*frame_->sonar_m[2]<=config_.side_clearance_m);
    // Test the obstacle BEFORE pending plans, detours and heading restoration.
    // A pending FPGA result can never bypass a newly observed stop condition.
    if((front_blocked||straight_side_blocked)&&!avoidance_active_){obstacle_stop(now);return waiting_control();}
    if(bypass_!=Bypass::None)return service_bypass(now,dt);

    if(turn_active_||turn_preparing_||turn_settling_)return service_turn(now,dt);
    if(pending_){if(now-pending_->created_s>config_.planner_timeout_s)return fail("planner_timeout");return waiting_control();}
    if(bootstrap_active_)return service_bootstrap(now,dt);
    if(straight_assist_&&!straight_turn_wait_resume_.empty()){
        // A pending zero-speed turn wait retains the detour/rejoin intent;
        // it must not fall through into the sensor-only forward mode.
        state_=straight_turn_wait_resume_;progress_at_=now;progress_pose_=pose_;
    }
    if(straight_assist_&&state_!="DETOUR"&&state_!="REJOIN_HEADING")return service_straight(now,dt);
    Bitmap bits=grid_.planning_bitmap(now);
    if(!free_world(xy(pose_),bits))return fail("footprint_clearance_not_observed");
    if(state_=="REJOIN_HEADING"){
        double err=wrap(rejoin_heading_-pose_.yaw);
        if(std::abs(err)>config_.heading_tolerance_rad){begin_turn(rejoin_heading_,false,now);return waiting_control();}
        path_.assign(original_route_.begin()+std::min<size_t>(original_route_.size(),rejoin_index_+1),original_route_.end());
        detour_origin_.reset();rejoin_point_.reset();avoidance_active_=false;
        if(straight_assist_){
            path_.clear();original_route_.clear();state_="STRAIGHT_STOP";reason_="straight_original_heading_restored";
            straight_replan_started_s_.reset();straight_replans_=0;
            stopped_requested_s_=now;stationary_since_.reset();return waiting_control();
        }
        state_="FOLLOW_ROUTE";reason_="original_route_resumed";
        return waiting_control(); // expose the stopped, restored-heading transition
    }
    if(path_.empty()){
        if(state_=="FOLLOW_ROUTE"||state_=="DETOUR"){
            if(state_=="DETOUR"){state_="REJOIN_HEADING";reason_="detour_position_rejoined";return emit(0,0,dt);}
            if(!exploration_){stop("goal_reached");state_=command_.state="ARRIVED";return command_;}goal_.reset();
        }
        if(exploration_&&!goal_){goal_=choose_frontier(bits,now);if(!goal_)return fail("no_known_safe_frontier");}
        if(!goal_)return fail("no_goal");
        request(*goal_,now,bits);return waiting_control();
    }
    while(!path_.empty()&&distance(path_.front(),xy(pose_))<=config_.waypoint_tolerance_m){
        // A reached-radius test alone can discard a corner too early. Keep it
        // until the actual pose has a verified segment to the next waypoint.
        if(path_.size()>1&&!segment_free(xy(pose_),path_[1],bits))break;
        path_.erase(path_.begin());
    }
    if(path_.empty())return emit(0,0,dt);
    for(Point p:path_)if(!free_world(p,bits)){
        if(detour_origin_){
            if(straight_assist_){
                if(!straight_replan_started_s_)straight_replan_started_s_=now;
                if(straight_replans_>=8||now-*straight_replan_started_s_>config_.straight_replan_max_s)
                    return fail("straight_detour_replan_limit");
                ++straight_replans_;path_.clear();obstacle_stop(now);
                reason_="straight_detour_map_changed_stop_replan";return waiting_control();
            }
            return fail("detour_map_changed");
        }
        path_.clear();if(!goal_)return fail("no_goal");request(*goal_,now,bits);return waiting_control();
    }
    Point target=path_.front();size_t target_index=0;
    for(size_t i=1;i<path_.size();++i){
        Point p=path_[i];if(distance(p,xy(pose_))>.24)break;
        Point previous=i==1?xy(pose_):path_[i-2];
        double incoming=std::atan2(path_[i-1].second-previous.second,path_[i-1].first-previous.first);
        double outgoing=std::atan2(p.second-path_[i-1].second,p.first-path_[i-1].first);
        // Shortcuts may retire straight intermediate points, never an unreached
        // change of direction. A later curved control step otherwise loses the
        // corner it needs to remain inside a tight four-connected free corridor.
        if(std::abs(wrap(outgoing-incoming))>10*PI/180)break;
        if(segment_free(xy(pose_),p,bits)){target=p;target_index=i;}
    }
    if(target_index)path_.erase(path_.begin(),path_.begin()+target_index);
    if(!segment_free(xy(pose_),target,bits))return fail("route_segment_blocked");
    double bearing=wrap(std::atan2(target.second-pose_.y,target.first-pose_.x)-pose_.yaw);
    if(distance(xy(pose_),xy(progress_pose_))>.03||std::abs(wrap(pose_.yaw-progress_pose_.yaw))>.08){progress_pose_=pose_;progress_at_=now;}
    if(now-progress_at_>config_.no_progress_s)return fail("no_odometry_progress");
    // Planned-map turns retain bounded targets and stop/observe. The local
    // reactive scan independently allows cumulative large turns.
    const double align_threshold=state_=="DETOUR"?config_.heading_tolerance_rad:25*PI/180;
    if(std::abs(bearing)>align_threshold){begin_turn(pose_.yaw+bearing,false,now);return waiting_control();}
    if(front_blocked||straight_side_blocked){
        // No translation merely because a side beam exceeded 0.30 m. The
        // newly aligned FRONT must also exceed its speed/latency stop bound.
        if(avoidance_active_){obstacle_stop(now);return waiting_control();}
        return fail("front_clearance_insufficient");
    }
    avoidance_active_=false;
    double speed=config_.cruise_mps;
    if(state_=="DETOUR"&&total_distance_-detour_start_distance_<config_.detour_accel_distance_m)speed=config_.detour_mps;
    if(straight_assist_)speed=std::min(config_.straight_speed_mps,config_.max_wheel_mps);
    double available=std::max(0.,*frame_->sonar_m[0]-config_.stop_clearance_m),b=config_.brake_mps2,reaction=reaction_delay(now);
    double safe=std::max(0.,-b*reaction+std::sqrt(b*b*reaction*reaction+2*b*available));
    speed=std::min(speed,safe)*std::max(.25,std::cos(bearing));reason_=state_=="DETOUR"?"verified_detour":"following_verified_route";
    return emit(speed,bearing*1.8,dt);
}
} // namespace robot_nav
