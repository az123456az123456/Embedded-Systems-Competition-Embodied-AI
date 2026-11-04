// Source-executed production Navigator tests. Encoder/IMU observations follow
// the preceding actual emitted wheel command; no board or map-free area is
// fabricated. Private inspection is used only for budget boundary/preference
// preconditions which would otherwise require a two-minute drive per case.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#define private public
#include "../navcore.h"
#undef private
using namespace robot_nav;
namespace {
unsigned checks=0;
void check(bool yes,const char* why){++checks;if(!yes)throw std::runtime_error(why);}
Config config(){Config c;c.velocity_control=true;c.reaction_s=.12;c.sonar_latency_s=.10;return c;}
struct Rig {
    Config c=config();Navigator nav{c};double now=10,yaw=0;Control out;
    std::array<double,4> counts{};std::array<double,3> range{4.5,2.,1.};
    unsigned invalid=0;double extra_age=0;bool freeze_ids=false;uint32_t observation=0;
    explicit Rig(bool navigation=false){
        for(unsigned i=0;i<80;++i)step(false);
        if(navigation){nav.explore();check(nav.start(now),"navigation starts with genuinely fresh odometry");}
        else check(nav.start_straight(now),"straight starts with genuinely fresh odometry");
        out=nav.command();
    }
    void step(bool tick=true){
        constexpr double dt=.02;
        const double left=out.linear_mps-out.angular_rps*c.track_m*.5;
        const double right=out.linear_mps+out.angular_rps*c.track_m*.5;
        for(unsigned w=0;w<4;++w)
            counts[w]+=((c.left_wheel_mask&(1u<<w))?left:right)*dt*c.encoder_ticks_per_turn/c.wheel_circumference_m*c.encoder_signs[w];
        yaw+=out.angular_rps*dt;now+=dt;
        if(!freeze_ids)observation=uint32_t(std::floor((now+1e-8)*20))*50u;
        SensorFrame f;f.timestamp_s=now;f.device_timestamp_s=now;f.sequence=uint32_t(std::llround(now*1000));
        f.yaw_rad=wrap(yaw);f.imu_valid=true;f.sonar_observation_ids_valid=true;
        for(unsigned w=0;w<4;++w)f.counts[w]=uint32_t(int64_t(std::llround(counts[w])));
        for(unsigned ch=0;ch<3;++ch){
            if(!(invalid&(1u<<ch)))f.sonar_m[ch]=range[ch];
            f.sonar_observation_id[ch]=observation;
            f.sonar_age_s[ch]=std::max(0.,now-observation/1000.)+extra_age;
        }
        const bool first=!nav.frame();check(first||nav.ingest(f,now),"physical previous-command encoder/IMU frame is accepted");
        if(first)nav.ingest(f,now);
        if(!tick)return;
        const auto cells=nav.grid().values();const auto observed=nav.grid().observed_s();const auto rev=nav.grid().revision();
        out=nav.tick(now+.00001);
        check(nav.grid().values()==cells&&nav.grid().observed_s()==observed&&nav.grid().revision()==rev,
              "direction decisions do not assert free map cells or refresh map evidence");
        check(std::abs(out.linear_mps)<=.300001&&std::abs(out.angular_rps)<=.600001,"physical output limits retained");
    }
    void obstacle(std::array<double,3> r){range=r;step();check(out.linear_mps==0&&out.angular_rps==0,"obstacle always first emits zero");}
    int turn(unsigned limit=150){for(unsigned i=0;i<limit;++i){step();check(nav.enabled(),nav.reason().c_str());if(std::abs(out.angular_rps)>1e-8)return out.angular_rps>0?1:-1;}return 0;}
    void remain_zero(unsigned steps,const char* why){for(unsigned i=0;i<steps;++i){step();check(nav.enabled(),"blocked fresh observation retains navigation intent");check(out.linear_mps==0&&out.angular_rps==0,why);}}
};
void initial_preference_falls_back(){
    for(bool navigation:{false,true}){
        Rig r(navigation);r.obstacle({.29,.8,.4});
        // The left preference was chosen before a newer processed source
        // arrives while braking. Both flanks are <=.30m, so the generic
        // fallback (not single-close-flank shortcut) must test the other side.
        r.nav.bypass_side_=1;r.range={.29,.195,.24};r.step(false);
        check(!r.nav.bypass_rotation_clear(wrap(r.yaw+.02),r.now),"left20cm hard floor is genuinely blocked");
        check(r.nav.bypass_rotation_clear(wrap(r.yaw-.02),r.now),"opposite full physical guard is genuinely clear");
        check(r.turn()==-1,"parked left preference falls back to physically verified right");
        const auto d=r.nav.local_avoidance_diagnostics();
        check(d.blocked_side_mask&1u,"left failure remains recorded for this encounter");
        check(d.attempted_side_mask==2u&&d.direction_switches==0,"only actually commanded directions count as attempts");
        check(!r.out.planner_request,"local fallback has no map planning prerequisite");
    }
}
void turn_interruption_and_hysteresis(){
    for(bool navigation:{false,true}){
        Rig r(navigation);r.obstacle({.29,1.8,.8});check(r.turn()==1,"actual left turn starts");
        for(unsigned i=0;i<12;++i)r.step();
        const double scan=r.nav.bypass_scan_,motion=r.nav.bypass_motion_s_;
        r.range={.29,.195,.24};r.step();
        check(r.out.linear_mps==0&&r.out.angular_rps==0,"new left obstruction first emits zero before right targets");
        check(r.nav.bypass_scan_>=scan&&r.nav.bypass_motion_s_>=motion,"direction reversal never replenishes whole-encounter budgets");
        check(r.nav.bypass_turn_travel_==0&&!r.nav.bypass_clear_id_&&r.nav.bypass_clear_samples_==0,
              "new direction discards old segment and clear confirmation");
        check(r.turn()==-1,"left scan changes to safe right after measured stopped proof");
        check(r.nav.bypass_direction_switches_==1,"first real left-to-right reversal counted once");
        // Right becomes blocked; left is barely above the20cm floor but has
        // not recovered its22cm release margin. Do not ping-pong to left.
        r.range={.29,.205,.195};r.step();
        check(r.out.linear_mps==0&&r.out.angular_rps==0,"second new obstruction first stops");
        r.remain_zero(60,"failed left direction cannot re-open from threshold chatter");
        check(r.nav.state()=="AVOID_WAIT"&&r.nav.bypass_direction_switches_==1,"both blocked waits without alternating motors");
        r.range[1]=.225;r.freeze_ids=true;
        r.remain_zero(15,"repeated source IDs cannot satisfy direction release confirmation");
        r.freeze_ids=false;check(r.turn()==1,"two genuinely new improved observations permit left recovery");
        check(r.nav.bypass_direction_switches_==2,"clear recovery adds exactly one actual reversal");
        check(r.nav.bypass_scan_>=scan&&r.nav.bypass_motion_s_>=motion,"repeated switches preserve whole encounter progress");
    }
}
void both_physically_blocked(){
    for(bool navigation:{false,true}){
        Rig r(navigation);r.obstacle({.29,.05,.05});
        r.remain_zero(550,"real5cm side obstacles never get an unsafe forced turn");
        check(r.nav.state()=="AVOID_WAIT"&&r.nav.bypass_attempted_sides_==0,"stationary both-side wait consumes no angle attempt");
        check(r.nav.bypass_scan_==0&&r.nav.bypass_motion_s_==0,"stationary blocked wait consumes no movement budget");
        r.range={.29,.24,.65};check(r.turn()==-1,"new fresh physically clear opposite side recovers without restart");
    }
}
void source_wait_never_resets_direction_history(){
    for(bool navigation:{false,true})for(bool stale:{false,true}){
        Rig r(navigation);r.obstacle({.29,1.8,.8});check(r.turn()==1,"outage case first reaches actual turn");
        for(unsigned i=0;i<10;++i)r.step();
        r.range={.29,.195,.24};r.step();check(r.turn()==-1,"outage case records blocked left and starts right");
        const double scan=r.nav.bypass_scan_,motion=r.nav.bypass_motion_s_;
        if(stale)r.extra_age=.8;else r.invalid=1;
        r.remain_zero(15,"invalid or expired source pauses at zero with intent retained");
        check(r.out.state=="SENSOR_WAIT"&&(r.nav.local_avoidance_diagnostics().blocked_side_mask&1u),
              "source wait preserves failed-side history");
        r.invalid=0;r.extra_age=0;
        check(r.turn()==-1,"fresh source recovery resumes safe right rather than old failed left");
        check(r.nav.bypass_scan_>=scan&&r.nav.bypass_motion_s_>=motion,"source recovery cannot reset budgets");
    }
}
void immediate_latest_sonar_keeps_encounter(){
    Rig r(true);r.obstacle({.29,1.8,.8});check(r.turn()==1,"range-only interrupt fixture rotates left");
    for(unsigned i=0;i<10;++i)r.step();
    r.nav.original_route_={{0,0},{.8,0},{1.6,0}};
    const auto saved=r.nav.original_route_;const double scan=r.nav.bypass_scan_,motion=r.nav.bypass_motion_s_;
    LiveSonarSample live;live.timestamp_s=r.now;live.sequence=4000;live.metres={.29,.195,.24};live.age_s={0,0,0};
    live.observation_id.fill(uint32_t(std::llround(r.now*1000)));
    check(r.nav.observe_latest_sonar(live,r.now),"new raw range immediately interrupts unsafe left turn");r.out=r.nav.command();
    check(r.out.linear_mps==0&&r.out.angular_rps==0&&r.nav.bypass_side_==-1,"latest range is reduce-only zero then opposite preference");
    check(r.nav.bypass_scan_>=scan&&r.nav.bypass_motion_s_>=motion&&r.nav.original_route_==saved,
          "range-only prealignment stop retains scan budget and original map route");
    r.nav.live_sonar_.reset();r.range={.29,.195,.24};check(r.turn()==-1,"aligned fresh source can subsequently authorize safe right");
}
void budget_boundaries_and_new_task(){
    Rig r(true);r.obstacle({.29,1.8,.8});check(r.turn()==1,"budget fixture starts actual turn");
    r.nav.original_route_={{0,0},{.8,0},{1.6,0}};const auto saved=r.nav.original_route_;
    r.nav.bypass_scan_=6.5;r.nav.bypass_motion_s_=70.;
    r.range={.29,.195,.24};r.step();check(r.turn()==-1,"budget fixture performs safe physical reversal");
    check(r.nav.bypass_scan_>=6.5&&r.nav.bypass_motion_s_>=70.&&r.nav.original_route_==saved,
          "scan overone revolution is retained across new-side round");
    // A new near obstacle in the same navigation .50m departure is still the
    // same encounter, including the immediate range-only path.
    r.nav.bypass_=Navigator::Bypass::MoveOut;r.nav.bypass_target_=r.nav.pose_.yaw;
    r.nav.bypass_leg_start_=r.nav.total_distance_;r.nav.command_.linear_mps=.05;r.nav.command_.angular_rps=0;
    LiveSonarSample live;live.timestamp_s=r.now;live.sequence=5000;live.metres={.25,.6,.9};live.age_s={0,0,0};
    live.observation_id.fill(uint32_t(std::llround(r.now*1000)));
    check(r.nav.observe_latest_sonar(live,r.now),"new near front immediately stops a local departure");r.out=r.nav.command();
    check(r.nav.bypass_scan_>=6.5&&r.nav.bypass_motion_s_>=70.&&r.nav.original_route_==saved,
          "new obstacle during same departure cannot replenish allowance or discard route");
    r.nav.live_sonar_.reset();r.range={.29,.6,.9};r.nav.bypass_motion_s_=120.01;r.step();
    check(!r.nav.enabled()&&r.nav.reason()=="avoid_local_sequence_timeout"&&r.out.linear_mps==0&&r.out.angular_rps==0,
          "whole-encounter120s budget remains a real zero-output task fault");
    r.nav.stop("operator_stop");check(r.nav.start_straight(r.now),"explicit new task can start with current healthy inputs");r.out=r.nav.command();
    r.obstacle({.29,.8,.4});check(r.nav.bypass_scan_==0&&r.nav.bypass_motion_s_==0&&r.nav.bypass_attempted_sides_==0,
                             "only a genuinely new encounter/task resets scan, motion and direction history");
    check(r.turn()==1,"new encounter independently selects left");
    r.nav.bypass_scan_=4*vehicle_profile::pi;r.step();
    check(!r.nav.enabled()&&r.nav.reason()=="avoid_scan_exhausted"&&r.out.linear_mps==0&&r.out.angular_rps==0,
          "720degree exhaustion remains enforced after fresh-encounter reset");
}
}
int main(){try{
    initial_preference_falls_back();turn_interruption_and_hysteresis();both_physically_blocked();
    source_wait_never_resets_direction_history();immediate_latest_sonar_keeps_encounter();budget_boundaries_and_new_task();
    std::cout<<"PASS v36 direction fallback "<<checks<<" checks; both modes, stop-before-reverse,2source hysteresis, both-blocked wait, source recovery,720deg/120s retained, latest-range interrupts\n";
}catch(const std::exception& e){std::cerr<<"FAIL v36 direction fallback after "<<checks<<": "<<e.what()<<'\n';return 1;}}
