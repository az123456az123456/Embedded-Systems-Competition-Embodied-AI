// Actual production profile + three sonar beams + wheel/IMU feedback. No map
// import is used in the bootstrap-to-FPGA test; host only compiles the same A*
// source that runs in the FPGA firmware, never a second route implementation.
#include "../navcore.h"
extern "C" {
#include "astar.h"
}
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace robot_nav;
namespace {
unsigned checks=0;
void check(bool value,const char* label){++checks;if(!value)throw std::runtime_error(label);}
Config calibrated(){Config c;c.configured=true;c.drive_mps_per_full_pwm=.6;c.left_wheel_mask=12;return c;}
struct Robot{
    Config c;Navigator nav;double t=0,yaw=0;std::array<double,4> counts{};Control out;
    std::array<double,3> ranges{4.5,4.5,4.5};
    explicit Robot(Config config=calibrated()):c(config),nav(config){sample();for(unsigned i=0;i<50;++i)step(false);}
    void sample(){
        SensorFrame f;f.timestamp_s=t;f.device_timestamp_s=t;f.imu_valid=true;f.yaw_rad=wrap(yaw);
        f.sonar_observation_ids_valid=true;const unsigned observation=unsigned(std::floor((t+1e-8)*10))*100;
        for(unsigned i=0;i<4;++i)f.counts[i]=uint32_t(int64_t(std::llround(counts[i])));
        for(unsigned i=0;i<3;++i){f.sonar_m[i]=ranges[i];f.sonar_age_s[i]=std::max(0.,t-observation/1000.);f.sonar_observation_id[i]=observation;}
        nav.ingest(f,t);
    }
    void step(bool tick=true){
        constexpr double dt=.02;
        const double left=out.linear_mps-out.angular_rps*c.track_m*.5,right=out.linear_mps+out.angular_rps*c.track_m*.5;
        for(unsigned i=0;i<4;++i)counts[i]+=((c.left_wheel_mask&(1U<<i))?left:right)*dt*c.encoder_ticks_per_turn/c.wheel_circumference_m*c.encoder_signs[i];
        yaw+=out.angular_rps*dt;t+=dt;sample();if(tick)out=nav.tick(t+.0001);
    }
};
std::vector<uint16_t> firmware_plan(const PlannerRequest&r){
    astar_context ctx{};std::copy(r.blocked.begin(),r.blocked.end(),ctx.occupied);
    const int n=astar_plan(&ctx,r.start%64,r.start/64,r.goal%64,r.goal/64);
    check(n>1,"actual FPGA A* must have a nonempty route");return {ctx.path,ctx.path+n};
}
void test_real_beams_bootstrap_to_fpga(){
    Robot r;check(r.nav.start_problem(r.t).empty(),"real fresh beams allow exploration start");
    const auto bits=r.nav.grid().planning_bitmap(r.t);const auto start=r.nav.grid().index(r.nav.grid().cell(0,0));
    check(OccupancyGrid::blocked(bits,start),"no fictitious initial surrounding disk");
    r.nav.explore();check(r.nav.start(r.t),"start without a map import/manual circle");
    bool moved=false,stopped=false,planned=false;double handoff=0;
    for(unsigned i=0;i<2300&&!planned;++i){
        r.step();check(r.nav.state()!="FAULT",r.nav.reason().c_str());
        if(r.out.linear_mps>0){moved=true;check(r.out.linear_mps<=.0600001,"bootstrap bounded speed");check(r.out.angular_rps==0,"bootstrap never spins through unknown");}
        if(moved&&r.out.state=="BOOTSTRAP_STOP")stopped=true;
        if(r.out.planner_request){
            const auto p=*r.out.planner_request;handoff=r.nav.pose().x;
            check(moved&&stopped,"short steps stop and reobserve before A* handoff");
            check(handoff>.45&&handoff<1.20,"actual sonar history forms a corridor within bounded budget");
            check(r.nav.accept_plan(p.request_id,p.map_revision,firmware_plan(p),r.t+.0002),"actual FPGA route accepted by unchanged strict validation");
            planned=true;r.out=r.nav.command();
        }
    }
    check(planned,"bootstrap actually reaches FPGA A* with only real production beams");
    bool route_motion=false;
    for(unsigned i=0;i<700&&!route_motion;++i){r.step();check(r.nav.state()!="FAULT",r.nav.reason().c_str());route_motion=r.out.linear_mps>0;}
    check(route_motion,"handoff can execute a verified FPGA route");
    std::cout<<"real_beams_handoff_x_m="<<handoff<<" t_s="<<r.t<<'\n';
}
void test_bootstrap_reacts_and_waits(){
    for(auto ranges:{std::array<double,3>{.2,.2,.2},std::array<double,3>{4.5,.25,4.5}}){
        Robot r;r.ranges=ranges;for(unsigned i=0;i<30;++i)r.step(false);
        r.nav.explore();check(r.nav.start(r.t),"close obstacle does not reject Start itself");
        for(unsigned i=0;i<180;++i){r.step();
            check(r.nav.reason()!="no_original_route_to_rejoin","initial avoidance never demands an old route");
            if(ranges[0]<=.2){check(r.out.linear_mps==0&&r.out.angular_rps==0,"real front/rotation-radius near protects body");}
            else check(r.nav.reason()!="bootstrap_side_clearance_stop_first","side .25m cannot by itself replace front sonar stop policy");
        }
        check(r.nav.enabled(),"physical wait or measured exploration keeps accepted session");
        r.ranges={4.5,4.5,4.5};bool moved=false;
        for(unsigned i=0;i<250&&!moved;++i){r.step();check(r.nav.state()!="FAULT",r.nav.reason().c_str());moved=r.out.linear_mps>0;}
        // Map evidence remains for normal exploration/planning. During the
        // local bypass only current sonar controls clearance; a later normal
        // route handoff may still wait for an observed-map corridor.
        check(moved||(r.nav.enabled()&&r.out.linear_mps==0&&r.out.angular_rps==0),"front-clear bypass either advances or reaches normal-map handoff wait");
    }
}
void test_increment_geometry_and_drift(){
    Robot r;const auto p=r.nav.pose();Pose forward=p;forward.x+=.08;
    check(!r.nav.motion_increment_free(forward,r.t,false),"strict unknown strips remain blocked");
    check(r.nav.motion_increment_free(forward,r.t,true),"only bounded sensor-gated mode may translate into unknown");
    for(Pose forbidden:{Pose{p.x,p.y,p.yaw+.001},Pose{p.x-.01,p.y,p.yaw},Pose{p.x+.01,p.y+.001,p.yaw}})
        check(!r.nav.motion_increment_free(forbidden,r.t,true),"unknown exception excludes turns/lateral/reverse");
    Pose same=p;check(r.nav.motion_increment_free(same,r.t,false),"the actual current footprint needs no surrounding unknown disk");
    const auto rev=r.nav.grid().revision();r.nav.motion_increment_free(forward,r.t,true);check(r.nav.grid().revision()==rev,"geometry guard writes no map evidence");
    // A partially covered grid cell is not wholly exempted from new occupancy.
    std::vector<int8_t> map(4096,-1);auto cell=r.nav.grid().cell(.20,.10);map[r.nav.grid().index(cell)]=1;
    check(r.nav.grid().import_observations(map,r.nav.grid().origin(),r.t),"explicit obstacle fixture");
    check(!r.nav.motion_increment_free(forward,r.t,true),"new known occupied area blocks forward bootstrap");
    Robot drift;drift.nav.explore();check(drift.nav.start(drift.t),"drift test starts");
    for(unsigned i=0;i<120&&drift.out.linear_mps==0;++i)drift.step();
    check(drift.out.linear_mps>0,"drift test actually advances");
    drift.yaw+=.0001;drift.step();check(drift.out.linear_mps>0&&drift.out.state=="BOOTSTRAP_ADVANCE","tiny IMU drift is not mistaken for a requested unknown turn");
    drift.yaw+=6*vehicle_profile::pi/180;drift.step();check(drift.out.linear_mps==0,"significant heading drift stops bootstrap");
}
void test_continuous_sweep_regression(){
    auto c=calibrated();Navigator n(c);SensorFrame f;f.timestamp_s=1;f.imu_valid=true;f.yaw_rad=2.6041813788792258;f.sonar_m={4.5,4.5,4.5};
    n.ingest(f);f.timestamp_s=1.05;n.ingest(f);
    std::vector<int8_t> cells(4096,0);cells[33*64+31]=-1;
    check(n.grid().import_observations(cells,{-3.1683030421111464,-3.1337210619595894},1.05),"continuous sweep fixture");
    Pose target=n.pose();target.yaw+=1.9*vehicle_profile::pi/180;
    check(!n.motion_increment_free(target,1.05,false),"an unknown cell swept between endpoint angles still blocks rotation");
    Navigator margin(c);f.timestamp_s=2;f.yaw_rad=2.6449047698912103;margin.ingest(f);f.timestamp_s=2.05;margin.ingest(f);
    cells.assign(4096,0);cells[34*64+31]=1;
    check(margin.grid().import_observations(cells,{-3.183049751494905,-3.1630556896335102},2.05),"occupied current-padding fixture");
    target={-.016933186494679113,.0091779502514790084,f.yaw_rad};
    check(!margin.motion_increment_free(target,2.05,true),"known obstacle in current padding is not exempted from actual80mm margin");
}
void test_bootstrap_limits_and_goto(){
    auto c=calibrated();c.bootstrap_max_distance_m=.12;Robot r(c);r.nav.explore();check(r.nav.start(r.t),"bounded budget starts");
    for(unsigned i=0;i<1000;++i)r.step();
    check(r.nav.enabled()&&r.out.linear_mps==0&&r.out.state=="BOOTSTRAP_WAIT","distance limit stops without an endless unknown-space walk");
    check(r.nav.pose().x<=.14,"step budget includes final partial step");
    Robot target;std::string error;check(!target.nav.go_to(.5,0,target.t,error),"map click remains strict known-safe");
    check(!target.nav.enabled(),"unknown destination never enters bootstrap automatically");
}
}
int main(){
    test_real_beams_bootstrap_to_fpga();test_bootstrap_reacts_and_waits();test_increment_geometry_and_drift();
    test_continuous_sweep_regression();test_bootstrap_limits_and_goto();
    std::cout<<"PASS bootstrap/navigation "<<checks<<" checks\n";
}
