// Regression of the two real-v31 stop/clearance failures. This harness drives
// public production Navigator APIs with actual previous wheel commands.
#include "../navcore.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
using namespace robot_nav;
namespace {
unsigned checks=0;
void ck(bool yes,const char* why){++checks;if(!yes)throw std::runtime_error(why);}
Config config(){Config c;c.velocity_control=true;c.reaction_s=.12;c.sonar_latency_s=.10;return c;}
struct Rig{
    Config c;Navigator nav;double now=10,yaw=0;Control out;
    std::array<double,4> counts{};std::array<double,3> sonar{4.5,2.,.65};
    bool freeze=false;double external_yaw_step=0;bool duplicate_front=false;
    uint32_t front_id=0;
    explicit Rig(Config cfg=config()):c(cfg),nav(cfg){for(unsigned i=0;i<80;++i)step(false);}
    void step(bool tick=true){
        constexpr double dt=.02;
        double left=out.linear_mps-out.angular_rps*c.track_m*.5;
        double right=out.linear_mps+out.angular_rps*c.track_m*.5;
        if(!freeze){
            for(unsigned w=0;w<4;++w)counts[w]+=((c.left_wheel_mask&(1U<<w))?left:right)*dt*c.encoder_ticks_per_turn/c.wheel_circumference_m*c.encoder_signs[w];
            yaw+=(right-left)/c.track_m*dt;
        }
        yaw+=external_yaw_step;now+=dt;
        SensorFrame f;f.timestamp_s=now;f.device_timestamp_s=now;f.sequence=unsigned(std::llround(now*1000));f.yaw_rad=wrap(yaw);f.imu_valid=true;
        const uint32_t observation=uint32_t(std::floor((now+1e-8)*10))*100;
        f.sonar_observation_ids_valid=true;
        if(!duplicate_front)front_id=observation;
        for(unsigned w=0;w<4;++w)f.counts[w]=uint32_t(int64_t(std::llround(counts[w])));
        for(unsigned ch=0;ch<3;++ch){f.sonar_m[ch]=sonar[ch];f.sonar_observation_id[ch]=ch==0?front_id:observation;f.sonar_age_s[ch]=std::max(0.,now-(ch==0?front_id:observation)/1000.);}
        const bool accepted=nav.ingest(f,now);
        if(tick)ck(accepted,"fixture source frame accepted");
        if(tick)out=nav.tick(now+.0001);
    }
    void start(){ck(nav.start_straight(now),"straight starts on fresh public inputs");out=nav.command();}
    void moving(){for(unsigned i=0;i<160;++i){step();ck(nav.enabled(),nav.reason().c_str());}ck(out.linear_mps>.2,"fixture reaches intended straight speed");}
    void near(){sonar[0]=.373;step();ck(out.linear_mps==0&&out.angular_rps==0&&out.state=="OBSTACLE_STOP","real .373m obstacle first commands stop");}
};
void false_far_requires_measured_scan(){
    Rig r;r.start();r.moving();r.near();r.freeze=true;r.sonar[0]=1.508;
    bool turning=false;
    for(unsigned i=0;i<70;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());ck(r.out.linear_mps==0,"same-heading near->far never self-starts forward");turning=turning||r.out.angular_rps>0;}
    ck(turning,"far rebound initiates scan toward genuinely larger opening");
    r.freeze=false;double origin=r.yaw;bool handoff=false;
    for(unsigned i=0;i<200&&!handoff;++i){
        const Control previous=r.out;r.step();ck(r.nav.enabled(),r.nav.reason().c_str());
        const double net=wrap(r.yaw-origin);
        if(net+1e-7<r.c.avoid_verify_turn_min_rad)ck(r.out.linear_mps==0,"less than minimum measured turn never permits forward");
        if(r.out.linear_mps>0){handoff=true;ck(net>=r.c.avoid_verify_turn_min_rad,"handoff proves actual minimum measured turn");ck(previous.angular_rps>0,"valid rotating-to-forward handoff inserts no zero-command frame");}
    }
    ck(handoff,"measured new direction and fresh distinct clear returns complete");
}
void wait_revokes_old_stopped_evidence(){
    Rig r;r.start();r.moving();r.sonar[0]=.10;r.step();
    for(unsigned i=0;i<50;++i){r.step();ck(r.out.linear_mps==0&&r.out.angular_rps==0,"physical near-front rotation bound waits at zero");}
    ck(r.out.state=="AVOID_WAIT","fixture is genuinely parked in blocked Select");
    r.sonar[0]=1.508;r.external_yaw_step=.02;
    for(unsigned i=0;i<25;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());ck(r.out.linear_mps==0&&r.out.angular_rps==0,"moving chassis revokes historical stop proof before motion selection");}
    r.external_yaw_step=0;bool turn=false,forward=false;
    for(unsigned i=0;i<200&&!forward;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());turn=turn||r.out.angular_rps!=0;forward=r.out.linear_mps>0;}
    ck(turn&&forward,"fresh still-window allows verified scan/recovery without moving stop-request timestamp forever");
}
void interrupted_scan_rechecks_current_stop(){
    Rig r;r.start();r.moving();r.near();
    for(unsigned i=0;i<120&&r.out.angular_rps==0;++i)r.step();
    ck(r.out.angular_rps>0,"fixture enters left scan");
    for(unsigned i=0;i<8;++i)r.step();
    r.sonar[0]=.10;r.step();ck(r.out.linear_mps==0&&r.out.angular_rps==0,"new physical limit interrupts turn immediately");
    for(unsigned i=0;i<30;++i)r.step();
    r.sonar[0]=1.508;r.external_yaw_step=-.02;
    for(unsigned i=0;i<20;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());ck(r.out.linear_mps==0&&r.out.angular_rps==0,"TurnOut AVOID_WAIT also invalidates stale stop/clear data after manual motion");}
    r.external_yaw_step=0;
    bool turn=false,forward=false;for(unsigned i=0;i<220&&!forward;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());turn=turn||r.out.angular_rps!=0;forward=r.out.linear_mps>0;}
    ck(turn&&forward,"interrupted manoeuvre resumes through new scan with no permanent parked deadlock");
}
void no_clearance_from_old_observations(){
    Rig r;r.start();r.moving();r.near();
    for(unsigned i=0;i<120&&r.out.angular_rps==0;++i)r.step();
    ck(r.out.angular_rps>0,"fixture enters turn");
    r.sonar[0]=1.508;r.duplicate_front=true;
    for(unsigned i=0;i<25;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());ck(r.out.linear_mps==0,"repeated pre-scan observation IDs never discharge clear confirmation");}
    r.duplicate_front=false;bool forward=false;
    for(unsigned i=0;i<180&&!forward;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());forward=r.out.linear_mps>0;}
    ck(forward,"new actual observations restore bounded handoff");
}
void configurable_minimum(){
    Config c=config();c.avoid_verify_turn_min_rad=12*vehicle_profile::pi/180.;
    Rig r(c);r.start();r.moving();r.near();r.sonar[0]=1.508;
    bool forward=false;double start=r.yaw;
    for(unsigned i=0;i<250&&!forward;++i){r.step();ck(r.nav.enabled(),r.nav.reason().c_str());if(wrap(r.yaw-start)<c.avoid_verify_turn_min_rad)ck(r.out.linear_mps==0,"configured 12degree minimum is enforced");forward=r.out.linear_mps>0;}
    ck(forward,"configurable angular scan completes");
}
}
int main(){try{false_far_requires_measured_scan();wait_revokes_old_stopped_evidence();interrupted_scan_rechecks_current_stop();no_clearance_from_old_observations();configurable_minimum();std::cout<<"reactive-v32-safety PASS checks="<<checks<<'\n';return 0;}catch(const std::exception& e){std::cerr<<"reactive-v32-safety FAIL after="<<checks<<" reason="<<e.what()<<'\n';return 1;}}
