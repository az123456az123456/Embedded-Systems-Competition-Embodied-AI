// Production Navigator and the exact FPGA firmware A*. The unknown-map
// scenarios use only real production sonar mapping; the detour scenario
// explicitly imports a previously surveyed simulator map, never clears it.
#include "../navcore.h"
extern "C" {
#include "astar.h"
}
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace robot_nav;
namespace {
unsigned checks=0;
void ck(bool v,const char* message){++checks;if(!v)throw std::runtime_error(message);}
Config configured(){Config c;c.configured=true;c.drive_mps_per_full_pwm=.6;return c;}
struct Robot {
    Config config;Navigator nav;double time=0,yaw=0;Control out;
    std::array<double,4> counts{};
    std::array<double,3> ranges{4.5,4.5,4.5};
    unsigned invalid=0;double sonar_age=0,side_bias_m=0;bool imu_valid=true,freeze=false,freeze_m4=false,obstacle=false;
    explicit Robot(Config c=configured(),double heading=0):config(c),nav(c),yaw(heading){sample();for(unsigned i=0;i<60;++i)step(false);}
    double ray(double sx,double sy,double angle)const {
        if(!obstacle)return 4.5;
        double lo=0,hi=4.5;
        for(auto axis:{std::array<double,4>{sx,std::cos(angle),.75,1.0},
                       std::array<double,4>{sy,std::sin(angle),-.40,.40}}){
            if(std::abs(axis[1])<1e-10){if(axis[0]<axis[2]||axis[0]>axis[3])return 4.5;continue;}
            double a=(axis[2]-axis[0])/axis[1],b=(axis[3]-axis[0])/axis[1];if(a>b)std::swap(a,b);
            lo=std::max(lo,a);hi=std::min(hi,b);if(lo>hi)return 4.5;
        }
        return lo>.03?lo:4.5;
    }
    void sample(){
        SensorFrame f;f.timestamp_s=time;f.device_timestamp_s=time;f.sequence=unsigned(time*1000);
        f.imu_valid=imu_valid;f.yaw_rad=wrap(yaw);f.sonar_observation_ids_valid=true;
        unsigned obs=unsigned(std::floor((time+1e-8)*10))*100;
        for(unsigned i=0;i<4;++i)f.counts[i]=uint32_t(int64_t(std::llround(counts[i])));
        for(unsigned i=0;i<3;++i){
            double range=ranges[i];
            if(obstacle){const auto& m=config.sonar_mounts[i];const Pose p=nav.pose();
                range=ray(p.x+m[0]*std::cos(yaw)-m[1]*std::sin(yaw),p.y+m[0]*std::sin(yaw)+m[1]*std::cos(yaw),yaw+m[2]);
                // A 1mm side difference models the real digital range
                // resolution and selects either mirrored departure safely.
                if(i==1)range=std::min(4.5,range+side_bias_m);
                if(i==2)range=std::min(4.5,range-side_bias_m);}
            if(!(invalid&(1U<<i)))f.sonar_m[i]=range;
            f.sonar_age_s[i]=std::max(0.,time-obs/1000.)+sonar_age;f.sonar_observation_id[i]=obs;
        }
        nav.ingest(f,time);
    }
    void step(bool tick=true){
        constexpr double dt=.02;
        double left=out.linear_mps-out.angular_rps*config.track_m*.5;
        double right=out.linear_mps+out.angular_rps*config.track_m*.5;
        for(unsigned i=0;i<4;++i)if(!freeze&&!(freeze_m4&&i==3))counts[i]+=((config.left_wheel_mask&(1U<<i))?left:right)*dt*config.encoder_ticks_per_turn/config.wheel_circumference_m*config.encoder_signs[i];
        yaw+=out.angular_rps*dt;time+=dt;sample();if(tick)out=nav.tick(time+.0001);
    }
    void start(){ck(nav.start_straight(time),"straight start");out=nav.command();}
    bool plan(){
        if(!out.planner_request)return false;
        const auto r=*out.planner_request;
        ck(r.purpose=="detour","straight only requests FPGA detours");
        astar_context context{};std::copy(r.blocked.begin(),r.blocked.end(),context.occupied);
        int count=astar_plan(&context,r.start%64,r.start/64,r.goal%64,r.goal/64);
        ck(count>1,"actual FPGA A* finds surveyed detour");
        ck(nav.accept_plan(r.request_id,r.map_revision,{context.path,context.path+count},time+.0002),"unchanged full FPGA path validation");
        out=nav.command();return true;
    }
};
void sensor_gated_start(){
    Robot r;auto bits=r.nav.grid().planning_bitmap(r.time);
    ck(OccupancyGrid::blocked(bits,r.nav.grid().index(r.nav.grid().cell(0,0))),"start remains unknown for full map planner");
    r.start();r.step();ck(r.out.linear_mps>0,"first healthy tick advances without stop-observation barrier");bool moved=true;
    for(unsigned i=0;i<160;++i){r.step();ck(r.nav.enabled(),"fresh straight remains enabled");ck(!r.out.planner_request,"no global map/A* needed for straight");
        ck(r.out.linear_mps<=.3000001&&r.out.angular_rps==0,"bounded pure-forward unknown step");moved=moved||r.out.linear_mps>0;}
    ck(moved&&r.nav.pose().x>.1,"pure sonar mode actually travels");
    r.nav.stop();ck(!r.nav.straight_assist_active()&&!r.nav.enabled(),"STOP cancels straight mode");
    ck(r.nav.command().left==0&&r.nav.command().right==0,"STOP commands zero");
    r.nav.explore();ck(!r.nav.straight_assist_active()&&r.nav.start(r.time),"ordinary exploration remains separate");
    Robot restart;restart.start();for(unsigned i=0;i<90;++i)restart.step();ck(restart.out.linear_mps>0,"restart test is moving");
    ck(restart.nav.start_straight(restart.time),"new explicit straight intent accepted");restart.out=restart.nav.command();
    ck(restart.out.left==0&&restart.out.right==0&&restart.nav.state()=="STRAIGHT_ADVANCE","repeat start resets output/ramp without an observation barrier");
    restart.step();ck(restart.out.linear_mps>0,"repeat start uses current healthy ranges immediately");
    restart.nav.set_goal(.5,0);ck(!restart.nav.straight_assist_active(),"goal selection clears straight mode");
    Robot across(configured(),vehicle_profile::pi-.01);across.start();for(unsigned i=0;i<90;++i)across.step();
    across.yaw+=.02;across.step();ck(across.nav.enabled()&&across.out.linear_mps>0,"small wrapped heading error across +/-pi does not jump");
    across.yaw+=.04;across.step();ck(across.nav.enabled()&&across.out.linear_mps>0&&across.out.angular_rps<0,"small wrapped error gets forward correction across +/-pi");
}
void unknown_obstacle_wait(){
    Robot r;r.start();for(unsigned i=0;i<90;++i)r.step();ck(r.out.linear_mps>0,"unknown obstacle test moves first");
    r.ranges={.2,.2,.2};r.step();ck(r.out.left==0&&r.out.right==0&&r.out.state=="OBSTACLE_STOP","front guard stops before choosing/turning");
    for(unsigned i=0;i<150;++i){r.step();ck(r.nav.enabled()&&r.nav.straight_assist_active(),"unknown rotation WAIT stays enabled");ck(r.out.left==0&&r.out.right==0,"unknown rotation never blind-spins");}
    ck(r.nav.state()=="AVOID_WAIT"&&r.nav.reason()=="avoid_selected_side_physical_clearance_insufficient","too-close ranges block the local rotation envelope");
    Robot side;side.ranges={4.5,.25,4.5};for(unsigned i=0;i<30;++i)side.step(false);side.start();
    side.step();ck(side.out.state=="OBSTACLE_STOP"&&side.out.linear_mps==0&&side.out.angular_rps==0,"left .25m first commands a full stop even with clear front");
    bool turned=false;
    for(unsigned i=0;i<120;++i){side.step();ck(side.nav.enabled()&&side.out.linear_mps==0&&side.out.angular_rps<=0,"left .25m permits only the physically clear away/right rotation");turned=turned||side.out.angular_rps<0;}
    ck(turned,"fresh close-side observations actually initiate an away turn");
}
void immediate_current_ranges_and_forward_correction(){
    Robot held;const double start=held.time+.001;
    const auto values=held.nav.grid().values();const auto observed=held.nav.grid().observed_s();
    const auto revision=held.nav.grid().revision();
    ck(held.nav.start_straight(start),"current healthy frame starts without another source measurement");
    held.out=held.nav.tick(start+.001);
    ck(held.out.linear_mps>0&&held.out.left>0&&held.out.right>0,"first tick before post-stop frame commands limited forward motion");
    ck(held.out.linear_mps<=held.config.accel_mps2*.001001,"first direct tick retains acceleration ramp");
    ck(held.nav.grid().values()==values&&held.nav.grid().observed_s()==observed&&held.nav.grid().revision()==revision,"direct start/tick writes no fictitious free map evidence");

    Robot recover;recover.start();for(unsigned i=0;i<60;++i)recover.step();recover.ranges[0]=.20;recover.step();
    ck(recover.out.left==0&&recover.out.right==0,"real close front first commands zero");
    auto frame=*recover.nav.frame();const double time=recover.time+.02;
    frame.timestamp_s=time;frame.device_timestamp_s=time;frame.sonar_m={4.5,4.5,4.5};
    // Every channel is healthy/current, but these source observation IDs are
    // deliberately unchanged from before the stop. v29 must NOT release from
    // these old-direction observations; a stopped new source set is required.
    frame.sonar_age_s={.08,.09,.10};
    ck(recover.nav.ingest(frame,time),"fresh recovery packet ingested");
    const auto before=recover.nav.grid().values();const auto before_seen=recover.nav.grid().observed_s();
    recover.out=recover.nav.tick(time+.0001);
    ck(recover.nav.enabled()&&recover.out.linear_mps==0&&recover.out.angular_rps==0,"old source IDs cannot bypass post-stop confirmation");
    ck(recover.nav.grid().values()==before&&recover.nav.grid().observed_s()==before_seen,"recovery does not erase historical map cells");

    for(int sign:{-1,1}){
        Robot turn;turn.start();turn.yaw=sign*5*vehicle_profile::pi/180.;
        const double initial_error=std::abs(turn.yaw);
        for(unsigned i=0;i<100;++i){turn.step();ck(turn.nav.enabled(),"small correction stays enabled without map gate");
            ck(turn.out.linear_mps>0&&turn.out.left>0&&turn.out.right>0,"small correction keeps both physical wheels forward from first ramp tick");
            ck(std::abs(turn.out.angular_rps)<=.1200001,"forward correction angular cap");
            ck(std::abs(turn.out.angular_rps)*turn.config.track_m<=turn.out.linear_mps+1e-8,"post-ramp clamp keeps inner wheel at least half actual linear speed");
            ck(turn.out.angular_rps*sign<=1e-8,"positive/negative yaw errors correct toward original yaw");}
        ck(std::abs(wrap(turn.nav.pose().yaw))<initial_error*.2,"forward feedback converges without resetting the original heading");
    }
}
void safeguards(){
    for(unsigned kind=0;kind<5;++kind){Robot r;r.start();for(unsigned i=0;i<90;++i)r.step();
        if(kind==0)r.invalid=1;
        if(kind==1)r.sonar_age=.8;
        if(kind==2)r.imu_valid=false;
        if(kind==3)r.freeze=true;
        if(kind==4)r.freeze_m4=true;
        for(unsigned i=0;i<350&&r.nav.enabled();++i)r.step();
        ck(!r.nav.enabled()&&!r.nav.straight_assist_active(),"real sensor/odometry fault disarms straight");
        ck(r.out.left==0&&r.out.right==0,"fault produces zero motor command");
        const std::string why=r.nav.reason();
        ck(why==(kind==0?"sonar_unavailable_timeout":kind==1?"sonar_unavailable_timeout":kind==2?"invalid_odometry_frame":kind==3?"no_odometry_progress":"encoder_wheel_disagreement"),"original safety reason preserved");
    }
    Robot watchdog;watchdog.start();for(unsigned i=0;i<20;++i)watchdog.step(false);
    watchdog.out=watchdog.nav.tick(watchdog.time);ck(!watchdog.nav.enabled()&&watchdog.nav.reason()=="control_watchdog","control watchdog unchanged");
    Robot drift;drift.start();for(unsigned i=0;i<90;++i)drift.step();drift.yaw+=3*vehicle_profile::pi/180;drift.step();
    ck(drift.out.linear_mps>0&&drift.out.angular_rps<0,"small heading error corrects forward without old-map observation barrier");
    drift.yaw=11*vehicle_profile::pi/180;drift.step();
    ck(drift.nav.enabled()&&drift.out.linear_mps==0&&drift.out.angular_rps==0&&drift.nav.reason()=="straight_heading_reacquire_with_sonar","greater than10degree correction first stops before the verified sonar-local reacquire scan");
    Robot dynamic;dynamic.start();for(unsigned i=0;i<90;++i)dynamic.step();dynamic.ranges[0]=.40;dynamic.step();
    ck(dynamic.nav.enabled()&&dynamic.out.left==0&&dynamic.out.right==0,"speed/braking/delay guard stops above static 0.30m");
    Robot known;known.start();for(unsigned i=0;i<90;++i)known.step();
    std::vector<int8_t> cells(4096,-1);Pose ahead=known.nav.pose();ahead.x+=.20;ahead.y+=.10;
    cells[known.nav.grid().index(known.nav.grid().cell(ahead.x,ahead.y))]=1;
    ck(known.nav.grid().import_observations(cells,known.nav.grid().origin(),known.time),"known side-of-beam obstacle fixture");known.step();
    ck(known.out.linear_mps>0&&known.nav.enabled(),"direct forward assist follows current healthy ranges despite historical occupied map cells");
}
void requested_speed_and_original_pwm_caps(){
    auto c=configured();c.drive_mps_per_full_pwm=.5;c.max_pwm=.4;
    Robot r(c);r.start();double previous=0,previous_tick=r.time;bool reached=false;
    for(unsigned i=0;i<200;++i){r.step();ck(r.nav.enabled(),"open straight speed run remains healthy");
        ck(std::abs(r.out.left)<=.6000001&&std::abs(r.out.right)<=.6000001,"assist hard PWM cap never exceeds .60");
        const double tick=r.time+.0001;
        ck(r.out.linear_mps<=previous+c.accel_mps2*(tick-previous_tick)+1e-9,"assist retains original acceleration limit using actual first tick time");
        previous_tick=tick;previous=r.out.linear_mps;reached=reached||std::abs(r.out.linear_mps-.3)<1e-6;}
    ck(reached&&std::abs(r.out.left-.6)<1e-6&&std::abs(r.out.right-.6)<1e-6,".5mps calibration reaches requested .3mps through dedicated .6 cap");
    // Ordinary navigation keeps its original .4 cap even when a caller asks
    // it to cruise at .3. This fixture only raises the requested cruise, not
    // the cap being tested; the production defaults remain .15/.22.
    c.cruise_mps=c.detour_mps=.3;Robot old(c);
    ck(old.nav.grid().import_observations(std::vector<int8_t>(4096,0),old.nav.grid().origin(),old.time),"surveyed ordinary-route fixture");
    old.nav.set_goal(1.5,.05);ck(old.nav.start(old.time),"normal navigation after speed config");old.out=old.nav.tick(old.time+.0001);
    ck(old.out.planner_request.has_value(),"ordinary mode still asks FPGA route");const auto request=*old.out.planner_request;
    astar_context context{};std::copy(request.blocked.begin(),request.blocked.end(),context.occupied);
    const int length=astar_plan(&context,request.start%64,request.start/64,request.goal%64,request.goal/64);
    ck(length>1&&old.nav.accept_plan(request.request_id,request.map_revision,{context.path,context.path+length},old.time+.0002),"actual FPGA ordinary path accepted");
    old.out=old.nav.command();bool normal_capped=false;
    for(unsigned i=0;i<200;++i){old.step();ck(old.nav.enabled(),"normal cap run remains healthy");
        ck(!old.nav.straight_assist_active(),"normal route cannot borrow assist PWM cap");
        ck(std::abs(old.out.left)<=.4000001&&std::abs(old.out.right)<=.4000001,"ordinary navigation retains .40 PWM cap");
        ck(old.out.linear_mps<=.2000001,"ordinary linear speed respects the .4 PWM physical bound");
        normal_capped=normal_capped||(old.out.linear_mps>.19&&std::max(std::abs(old.out.left),std::abs(old.out.right))>.3999);}
    ck(normal_capped,"ordinary requested .3 is still physically capped to .2mps");
    for(unsigned kind=0;kind<2;++kind){auto limited=configured();limited.drive_mps_per_full_pwm=.5;
        if(kind==0)limited.max_wheel_mps=.25;else limited.straight_max_pwm=.5;
        Robot capped(limited);capped.start();for(unsigned i=0;i<180;++i)capped.step();
        ck(std::abs(capped.out.linear_mps-.25)<1e-6,"assist honors lower configured wheel/PWM physical cap");}
    for(unsigned kind=0;kind<2;++kind){auto invalid=configured();if(kind==0)invalid.straight_speed_mps=.301;else invalid.straight_max_pwm=.601;
        Robot rejected(invalid);ck(!rejected.nav.start_straight(rejected.time),"out-of-range assist settings refuse start");ck(!rejected.nav.enabled(),"invalid assist settings stay stopped");}
}
// Local .30m three-beam bypass and later map-route A* rejoin are covered by
// test_reactive_avoidance.cc; heading assist no longer requests A* detours.

}
int main(){try{sensor_gated_start();unknown_obstacle_wait();immediate_current_ranges_and_forward_correction();safeguards();requested_speed_and_original_pwm_caps();
    std::cout<<"PASS straight-assist "<<checks<<" checks\n";
}catch(const std::exception& e){std::cerr<<"FAIL straight-assist: "<<e.what()<<" checks="<<checks<<'\n';return 1;}}
