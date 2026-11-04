// v27.2: local turn/scan/detour ordering tests moved to test_reactive_avoidance.cc.
// The old tests required FPGA rejoin planning BEFORE any bypass movement,
// which conflicts with the new .30m/.20m local-first requirement.
#include "../navcore.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace robot_nav;
#define CHECK(expr) do { if(!(expr)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #expr); } while(0)

Config calibrated(){
    Config c;c.configured=true;c.wheel_circumference_m=.2;c.encoder_ticks_per_turn=2000;
    c.track_m=.22;c.robot_radius_m=.12;c.encoder_signs={1,1,1,1};
    c.body_front_m=c.body_rear_m=.08;c.body_left_m=c.body_right_m=.07;
    c.sonar_mounts={{{0,0,0},{0,0,1.5707963267948966},{0,0,-1.5707963267948966}}};c.drive_mps_per_full_pwm=.60;c.safety_margin_m=.03;
    c.sonar_latency_s=0;c.map_ttl_s=60;c.stop_clearance_m=.35;return c;
}
SensorFrame frame(double stamp,uint32_t count=0){
    SensorFrame f;f.timestamp_s=stamp;f.counts.fill(count);f.imu_valid=true;
    f.sonar_m={2.,2.,2.};return f;
}
void known_free(Navigator& n,double stamp){CHECK(n.grid().import_observations(std::vector<int8_t>(4096,0),{-3.2,-3.2},stamp));}
struct Fixture{
    Navigator n;
    Fixture(Config c=calibrated()):n(c){n.ingest(frame(0));CHECK(n.ingest(frame(.05)));known_free(n,.05);}
    PlannerRequest request(double x=1.05,double y=.05){n.set_goal(x,y);CHECK(n.start(.05));auto out=n.tick(.06);CHECK(out.left==0&&out.right==0&&out.planner_request);return *out.planner_request;}
    std::vector<uint16_t> straight(const PlannerRequest& r){
        std::vector<uint16_t> cells;int x=r.start%64,y=r.start/64,tx=r.goal%64,ty=r.goal/64;
        cells.push_back(static_cast<uint16_t>(y*64+x));
        while(x!=tx){x+=x<tx?1:-1;cells.push_back(static_cast<uint16_t>(y*64+x));}
        while(y!=ty){y+=y<ty?1:-1;cells.push_back(static_cast<uint16_t>(y*64+x));}
        return cells;
    }
    void route(){auto r=request();CHECK(n.accept_plan(r.request_id,r.map_revision,straight(r),.07));}
};

void test_configuration_gate(){Navigator n(Config{});n.ingest(frame(0));n.ingest(frame(.05));n.set_goal(1,0);CHECK(!n.start(.05));CHECK(n.command().left==0);CHECK(n.state()=="FAULT");}
void test_first_boot_invalid_imu_can_recover(){Navigator n(calibrated());auto s=frame(0);s.imu_valid=false;CHECK(!n.ingest(s));n.ingest(frame(.05));CHECK(n.ingest(frame(.10)));known_free(n,.1);n.set_goal(1.05,.05);CHECK(n.start(.10));}
void test_config_bounds(){auto c=calibrated();c.max_pwm=.61;CHECK(!c.errors().empty());c=calibrated();c.robot_radius_m=1e20;CHECK(!c.errors().empty());Navigator n(c);n.ingest(frame(0));n.ingest(frame(.05));n.set_goal(1,0);CHECK(!n.start(.05));c=calibrated();c.encoder_ticks_per_turn=10000001;CHECK(!c.errors().empty());}
void test_wrap_and_odometry(){
    CHECK(count_delta(2,0xfffffffcU)==6);CHECK(count_delta(0xfffffffcU,2)==-6);
    Navigator n(calibrated());n.ingest(frame(0,0xfffffffcU));CHECK(n.ingest(frame(.1,2)));
    CHECK(std::abs(n.pose().x-.0006)<1e-8);CHECK(std::abs(n.pose().y)<1e-8);
    auto f=frame(.2,102);f.yaw_rad=.2;CHECK(n.ingest(f));CHECK(n.pose().y>0);
}
void test_encoder_signs(){auto c=calibrated();c.encoder_signs={-1,1,-1,1};Navigator n(c);n.ingest(frame(0));auto f=frame(.1,100);f.counts[0]=f.counts[2]=uint32_t(-100);CHECK(n.ingest(f));CHECK(std::abs(n.pose().x-.01)<1e-7);}
void test_missing_encoder_and_latched_stop(){Fixture f;f.route();auto s=frame(.10);s.encoder_ok[3]=false;CHECK(!f.n.ingest(s));CHECK(f.n.state()=="FAULT");CHECK(!f.n.enabled());CHECK(f.n.tick(.11).left==0);CHECK(f.n.ingest(frame(.15)));CHECK(!f.n.enabled());}
void test_wrong_encoder(){auto c=calibrated();c.encoder_fault_after_s=.25;Navigator n(c);n.ingest(frame(0));for(unsigned i=1;i<=3;++i){auto f=frame(i*.1,i*100);f.counts[3]=uint32_t(-int(i*100));n.ingest(f);}CHECK(n.reason()=="encoder_wheel_disagreement");CHECK(!n.enabled());}
void test_stuck_encoder(){auto c=calibrated();c.encoder_fault_after_s=.25;Navigator n(c);n.ingest(frame(0));for(unsigned i=1;i<=3;++i){auto f=frame(i*.1,i*100);f.counts[0]=0;n.ingest(f);}CHECK(n.reason()=="encoder_wheel_disagreement");}
void test_jump_and_yaw_reset(){Fixture f;auto s=frame(.1,100000);CHECK(!f.n.ingest(s));CHECK(f.n.reason()=="encoder_jump");Fixture g;s=frame(.1);s.yaw_rad=2.;CHECK(!g.n.ingest(s));CHECK(g.n.reason()=="imu_yaw_jump");}
void test_duplicate_does_not_refresh(){Fixture f;f.route();CHECK(!f.n.ingest(frame(.05),.2));auto out=f.n.tick(.4);CHECK(out.left==0);CHECK(f.n.reason()=="telemetry_stale");}
void test_invalid_sonar(){Fixture f;f.route();auto s=frame(.1);s.sonar_m[1].reset();CHECK(f.n.ingest(s));auto out=f.n.tick(.11);CHECK(out.left==0&&out.right==0);CHECK(f.n.reason()=="sonar_invalid_zero_wait"&&f.n.enabled());}
void test_stale_sonar(){Fixture f;f.route();auto s=frame(.1);s.sonar_age_s[2]=.8;f.n.ingest(s);CHECK(f.n.tick(.11).left==0);CHECK(f.n.reason()=="sonar_stale_zero_wait"&&f.n.enabled());}
void test_control_watchdog(){Fixture f;f.route();for(double t:{.15,.25,.35})f.n.ingest(frame(t));CHECK(f.n.tick(.36).left==0);CHECK(f.n.reason()=="control_watchdog");}
void test_unknown_inflation_and_expiry(){auto c=calibrated();OccupancyGrid g(c);auto b=g.planning_bitmap(0);for(auto v:b)CHECK(v==255);std::vector<int8_t> map(4096,0);map[32*64+36]=1;CHECK(g.import_observations(map,{-3.2,-3.2},1));b=g.planning_bitmap(1);CHECK(g.blocked(b,32*64+36));CHECK(g.blocked(b,32*64+35));CHECK(!g.blocked(b,32*64+30));CHECK(g.blocked(b,0));b=g.planning_bitmap(62);for(auto v:b)CHECK(v==255);}
void test_no_echo_is_unknown(){Navigator n(calibrated());auto s=frame(0);s.sonar_m={std::nullopt,std::nullopt,std::nullopt};n.ingest(s);CHECK(n.grid().state(n.grid().cell(1,0),0)==-1);}
void test_sonar_uses_historical_pose(){auto c=calibrated();c.sonar_latency_s=.2;Navigator n(c);auto s=frame(0);s.sonar_m={std::nullopt,std::nullopt,std::nullopt};n.ingest(s);s=frame(.1,1000);s.sonar_m={std::nullopt,std::nullopt,std::nullopt};n.ingest(s);s=frame(.2,2000);s.sonar_m={1.,std::nullopt,std::nullopt};n.ingest(s);CHECK(n.grid().state(n.grid().cell(.95,.05),.2)==1);CHECK(n.grid().state(n.grid().cell(1.25,.05),.2)==-1);}
void test_map_recentre(){auto c=calibrated();OccupancyGrid g(c);std::vector<int8_t> map(4096,0);map[32*64+40]=1;g.import_observations(map,{-3.2,-3.2},0);Point obstacle=g.center({40,32});g.recenter(Pose{2.5,0,0});CHECK(g.state(g.cell(obstacle.first,obstacle.second),0)==1);CHECK(g.origin().first>-3.2);}
void test_new_echo_overrides_old_free(){auto c=calibrated();OccupancyGrid g(c);g.import_observations(std::vector<int8_t>(4096,0),{-3.2,-3.2},0);g.update_sonar(Pose{},0,1.,.1);CHECK(g.state(g.cell(.95,.05),.1)==1);}
void test_localization_loss_requires_new_map(){Fixture f;f.route();CHECK(!f.n.ingest(frame(.5,100)));CHECK(f.n.ingest(frame(.55,100)));CHECK(!f.n.start(.55));CHECK(f.n.reason()=="odometry_gap");f.n.reset_localization();CHECK(f.n.grid().state({32,32},.55)==-1);f.n.ingest(frame(.6));CHECK(f.n.ingest(frame(.65)));known_free(f.n,.65);f.n.set_goal(1.05,.05);CHECK(f.n.start(.65));}
void test_valid_route_and_acceleration_bound(){Fixture f;f.route();auto out=f.n.tick(.08);CHECK(out.linear_mps>0);CHECK(out.linear_mps<=calibrated().accel_mps2*.02+1e-9);CHECK(out.left<=.45&&out.right<=.45);CHECK(out.state=="FOLLOW_ROUTE");}
void test_wrong_request_ignored(){Fixture f;auto r=f.request();CHECK(!f.n.accept_plan(r.request_id+1,r.map_revision,f.straight(r),.07));CHECK(f.n.pending_request());CHECK(f.n.enabled());}
void test_nonadjacent_rejected(){Fixture f;auto r=f.request();CHECK(!f.n.accept_plan(r.request_id,r.map_revision,{r.start,r.goal},.07));CHECK(f.n.reason()=="planner_path_not_four_connected");CHECK(!f.n.enabled());}
void test_path_obstacle_rejected(){Fixture f;auto r=f.request();auto map=std::vector<int8_t>(4096,0);map[32*64+37]=1;f.n.grid().import_observations(map,{-3.2,-3.2},.06);CHECK(!f.n.accept_plan(r.request_id,r.map_revision,f.straight(r),.07));CHECK(f.n.reason()=="planner_path_no_longer_safe");}
void test_planner_timeout(){auto c=calibrated();c.planner_timeout_s=.15;Fixture f(c);f.request();f.n.ingest(frame(.15));f.n.tick(.15);f.n.ingest(frame(.25));auto out=f.n.tick(.25);CHECK(out.left==0);CHECK(f.n.reason()=="planner_timeout");}
void test_new_obstacle_stops_before_replan(){Fixture f;f.route();f.n.tick(.08);auto map=std::vector<int8_t>(4096,0);map[32*64+39]=1;f.n.grid().import_observations(map,{-3.2,-3.2},.08);auto out=f.n.tick(.10);CHECK(out.left==0&&out.right==0);CHECK(out.planner_request);CHECK(out.state=="WAIT_PLAN");}
Control stopped_observation(Fixture& f,std::array<std::optional<double>,3> ranges,double& t){
    auto s=frame(t);s.sonar_m=ranges;CHECK(f.n.ingest(s));known_free(f.n,t);
    auto out=f.n.tick(t+.001);
    for(unsigned step=0;step<30&&!out.planner_request&&out.angular_rps==0&&f.n.enabled();++step){
        t+=.05;s=frame(t);s.sonar_m=ranges;CHECK(f.n.ingest(s));known_free(f.n,t);out=f.n.tick(t+.001);
    }
    return out;
}
void test_manual_scan_bootstraps_measured_map(){
    auto c=calibrated();c.sonar_latency_s=.3;Navigator n(c);
    // Manual 0.5 rad/s rotation, 20 Hz telemetry, new A02 observations only 2 Hz.
    // No imported or fabricated free-space map is used in this test.
    for(unsigned i=0;i<=260;++i){auto s=frame(i*.05);s.yaw_rad=wrap(i*.025);s.sonar_age_s.fill((i%10)*.05);n.ingest(s);}
    CHECK(n.grid().state(n.grid().cell(1.,0),13)==0);
    auto bits=n.grid().planning_bitmap(13);CHECK(!OccupancyGrid::blocked(bits,OccupancyGrid::index(n.grid().cell(0,0))));
    n.set_goal(1.05,.05);CHECK(n.start(13));auto out=n.tick(13.01);CHECK(out.planner_request);CHECK(out.left==0);
}
void test_reject_path_loop(){Fixture f;auto r=f.request();auto p=f.straight(r);p.insert(p.begin()+1,r.start);CHECK(!f.n.accept_plan(r.request_id,r.map_revision,p,.07));CHECK(f.n.reason()=="planner_invalid_endpoints_or_loop");}
void test_expired_plan_stops_not_resumes(){Fixture f;f.route();f.n.tick(.4);CHECK(!f.n.enabled());CHECK(f.n.ingest(frame(.15)));CHECK(f.n.tick(.16).left==0);CHECK(!f.n.enabled());}
void test_sonar_event_duplicates_do_not_repaint(){
    Navigator n(calibrated());auto s=frame(1.0);s.sonar_observation_ids_valid=true;s.sonar_observation_id.fill(0xfffffff0u);
    n.ingest(s);auto revision=n.grid().revision();
    s.timestamp_s=1.051;s.sonar_age_s.fill(.050);CHECK(n.ingest(s));
    // Host receipt jitter moved inferred time by 1 ms; device says same echo.
    CHECK(n.grid().revision()==revision+1); // body only, no repeated sonar evidence
    revision=n.grid().revision();s.timestamp_s=1.10;s.sonar_age_s.fill(0);s.sonar_observation_id.fill(0x10u);
    CHECK(n.ingest(s));CHECK(n.grid().revision()==revision+4); // valid device wrap + 3 new echoes
    revision=n.grid().revision();s.timestamp_s=1.15;s.sonar_observation_id.fill(0xffffff00u);
    CHECK(n.ingest(s));CHECK(n.grid().revision()==revision+1); // reordered observation ignored
}

void test_fixed_profile_precision_and_rectangle(){
    Config c;CHECK(c.encoder_ticks_per_turn==60527.7);CHECK((c.encoder_signs==std::array<int,4>{-1,1,-1,1}));
    CHECK(std::abs(c.body_left_m-.137)<1e-12);CHECK(std::abs(c.body_right_m-.115)<1e-12);
    c.configured=true;c.drive_mps_per_full_pwm=.6;Navigator n(c);auto s=frame(0);s.sonar_m={std::nullopt,std::nullopt,std::nullopt};n.ingest(s);
    s.timestamp_s=.1;for(unsigned i=0;i<4;++i)s.counts[i]=uint32_t(c.encoder_signs[i]*6053);CHECK(n.ingest(s));
    CHECK(std::abs(n.pose().x-(6053*.20507/60527.7))<1e-12);
    OccupancyGrid grid(c);grid.mark_body(Pose{},0);
    CHECK(grid.state(grid.cell(-.05,.05),0)==0);
    CHECK(grid.state(grid.cell(.15,.15),0)==-1); // inside circumcircle, outside physical body
    CHECK(grid.state(grid.cell(-.15,-.15),0)==-1);
}
void test_live_side_mapping_pairs_and_reset(){
    auto c=calibrated();c.left_wheel_mask=12;c.encoder_fault_after_s=.2;Navigator n(c);
    n.ingest(frame(0));
    for(unsigned step=1;step<=5;++step){auto s=frame(step*.1);s.counts={0,0,step*100,step*100};CHECK(n.ingest(s));}
    CHECK(n.reason()!="encoder_wheel_disagreement");CHECK(n.config().left_wheel_mask==12);
    CHECK(!n.set_wheel_layout(1));CHECK(!n.set_wheel_layout(16));CHECK(n.config().left_wheel_mask==12);
    CHECK(n.set_wheel_layout(10));CHECK(!n.enabled());CHECK(n.grid().state({32,32},.5)==-1);
    CHECK(n.reason()=="wheel_layout_changed_reset_map");CHECK(n.set_wheel_layout(10));
}
void test_pending_plan_obstacle_uses_measured_braking_speed(){
    Fixture f;auto request=f.request();auto s=frame(.10,100);s.sonar_m={.4,2.,2.};
    CHECK(f.n.ingest(s));known_free(f.n,.10);
    // Instantaneous speed is .20m/s; v32's measured short-window body speed
    // is .10m/s while accelerating from rest. Even though WAIT_PLAN requests
    // zero, .40m is below .35+.10*.851+.10^2/(2*.4); cancel the old plan.
    auto out=f.n.tick(.101);CHECK(out.left==0&&out.right==0);CHECK(out.state=="OBSTACLE_STOP");
    CHECK(!out.planner_request);CHECK(!f.n.accept_plan(request.request_id,request.map_revision,f.straight(request),.102));
}
void test_stop_waits_for_actual_encoder_coast_to_finish(){
    Fixture f;f.route();f.n.tick(.08);double t=.1;uint32_t count=0;
    auto s=frame(t,count);s.sonar_m={.3,1.5,.6};f.n.ingest(s);known_free(f.n,t);
    auto out=f.n.tick(t+.001);CHECK(out.state=="OBSTACLE_STOP");
    for(unsigned i=0;i<16;++i){t+=.05;count+=50;s=frame(t,count);s.sonar_m={.3,1.5,.6};CHECK(f.n.ingest(s));known_free(f.n,t);out=f.n.tick(t+.001);CHECK(out.left==0&&out.right==0);CHECK(!out.planner_request);}
    CHECK(out.reason=="avoid_waiting_encoder_stop");
    bool turned=false;
    for(unsigned i=0;i<20&&!turned;++i){t+=.05;s=frame(t,count);s.sonar_m={.3,1.5,.6};CHECK(f.n.ingest(s));known_free(f.n,t);out=f.n.tick(t+.001);CHECK(out.linear_mps==0&&!out.planner_request);turned=std::abs(out.angular_rps)>0;}
    CHECK(turned);
}
void test_old_device_echo_cannot_finish_stop_observation(){
    Fixture f;f.route();double t=.1;auto s=frame(t);s.sonar_m={.3,1.5,.6};
    s.sonar_observation_ids_valid=true;s.sonar_observation_id.fill(100);CHECK(f.n.ingest(s));known_free(f.n,t);auto out=f.n.tick(t+.001);
    for(unsigned i=0;i<10;++i){t+=.05;s=frame(t);s.sonar_m={.3,1.5,.6};s.sonar_age_s.fill(t-.1);s.sonar_observation_ids_valid=true;s.sonar_observation_id.fill(100);CHECK(f.n.ingest(s));known_free(f.n,t);out=f.n.tick(t+.001);CHECK(out.left==0&&out.right==0);CHECK(!out.planner_request);}
    CHECK(out.reason=="avoid_waiting_new_sonar");
    t+=.05;s=frame(t);s.sonar_m={.3,1.5,.6};s.sonar_observation_ids_valid=true;s.sonar_observation_id.fill(200);CHECK(f.n.ingest(s));known_free(f.n,t);out=f.n.tick(t+.001);CHECK(out.state=="AVOID_TURN_CLEAR"&&!out.planner_request);
}
void test_unknown_rear_uses_bounded_local_turn(){
    Fixture f;f.route();auto map=std::vector<int8_t>(4096,0);
    map[OccupancyGrid::index(f.n.grid().cell(-.15,-.05))]=-1;
    CHECK(f.n.grid().import_observations(map,{-3.2,-3.2},.1));
    // Isolate unknown-rear policy from conservative occupied sonar endpoint cells.
    // Geometric rays and known-occupied turn guards are covered independently in
    // test_reactive_avoidance.cc; this fixture explicitly supplies a surveyed map.
    double t=.1;auto s=frame(t);s.sonar_m={.3,2.,2.};CHECK(f.n.ingest(s));
    CHECK(f.n.grid().import_observations(map,{-3.2,-3.2},t));
    auto out=f.n.tick(t+.001);CHECK(out.left==0&&out.right==0);
    for(unsigned i=0;i<20&&out.angular_rps==0;++i){t+=.05;s=frame(t);s.sonar_m={.3,2.,2.};CHECK(f.n.ingest(s));CHECK(f.n.grid().import_observations(map,{-3.2,-3.2},t));out=f.n.tick(t+.001);CHECK(out.linear_mps==0);}
    CHECK(f.n.enabled());CHECK(std::abs(out.angular_rps)>0);CHECK(!out.planner_request);
}
void test_sonar_delay_is_part_of_braking_guard(){
    auto c=calibrated();c.sonar_latency_s=.3;Fixture f(c);f.route();
    // With .10m/s short-window body motion, .48m is below the delayed
    // threshold .4995m but above the .4695m bound without acoustic latency.
    auto s=frame(.10,100);s.sonar_m={.48,2.,2.};s.sonar_age_s[0]=.2;
    CHECK(f.n.ingest(s));known_free(f.n,.10);auto out=f.n.tick(.12);
    CHECK(out.state=="OBSTACLE_STOP");CHECK(out.left==0&&out.right==0);
}
void test_valid_large_turn_not_artificial_segment_fault(){
    Fixture f;f.route();double t=.1;auto out=stopped_observation(f,{.3,.25,.25},t);CHECK(out.angular_rps<0); // equal sides retain clockwise/right fallback
    auto s=frame(t+.2);s.yaw_rad=-29.5*vehicle_profile::pi/180; // within physical IMU jump bound
    s.sonar_m={.3,.25,.25};CHECK(f.n.ingest(s));known_free(f.n,t+.2);out=f.n.tick(t+.201);
    CHECK(f.n.enabled());CHECK(f.n.reason()!="turn_segment_overshoot_stop");CHECK(out.linear_mps==0); // a valid large scan is not an artificial segment fault
}


void test_late_plan_while_sonar_wait_preserves_session(){
    for(bool stopped:{false,true}){
        Fixture f;auto request=f.request();auto s=frame(.10);s.sonar_m[0].reset();
        CHECK(f.n.ingest(s));known_free(f.n,.10);auto out=f.n.tick(.11);
        CHECK(f.n.enabled()&&out.state=="SENSOR_WAIT");
        if(stopped){f.n.stop("operator_stop");CHECK(!f.n.accept_plan(request.request_id,request.map_revision,f.straight(request),.12));CHECK(!f.n.enabled());continue;}
        CHECK(f.n.accept_plan(request.request_id,request.map_revision,f.straight(request),.12));
        CHECK(f.n.enabled()&&f.n.state()=="SENSOR_WAIT"&&f.n.command().linear_mps==0);
        bool resumed=false;
        for(unsigned i=0;i<18;++i){double t=.15+i*.05;s=frame(t);CHECK(f.n.ingest(s));known_free(f.n,t);out=f.n.tick(t+.001);
            // Motion waits for a real 80 ms source window after STOP, rather
            // than the former fixed .25 s dwell plus acoustic-latency timer.
            if(t<.23)CHECK(out.linear_mps==0);
            resumed=resumed||out.linear_mps>0;}
        CHECK(resumed&&f.n.enabled()&&f.n.state()=="FOLLOW_ROUTE");
    }
}
void test_plan_during_range_recovery_and_repeated_flap(){
    for(bool plan_during_recovery:{false,true}){
        Fixture f;auto request=f.request();auto s=frame(.10);s.sonar_m[0].reset();CHECK(f.n.ingest(s));known_free(f.n,.10);CHECK(f.n.tick(.11).state=="SENSOR_WAIT");
        if(!plan_during_recovery)CHECK(f.n.accept_plan(request.request_id,request.map_revision,f.straight(request),.12));
        s=frame(.15);CHECK(f.n.ingest(s));known_free(f.n,.15);CHECK(f.n.tick(.151).state=="SENSOR_RECOVERY");
        if(plan_during_recovery)CHECK(f.n.accept_plan(request.request_id,request.map_revision,f.straight(request),.16));
        CHECK(f.n.state()=="SENSOR_RECOVERY"&&f.n.command().linear_mps==0);
        for(unsigned i=0;i<4;++i){const double t=.20+i*.10;s=frame(t);s.sonar_m[0].reset();CHECK(f.n.ingest(s));known_free(f.n,t);CHECK(f.n.tick(t+.001).state=="SENSOR_WAIT");
            s=frame(t+.05);CHECK(f.n.ingest(s));known_free(f.n,t+.05);auto healthy=f.n.tick(t+.051);
            CHECK(f.n.enabled()&&healthy.state!="FAULT");
            if(healthy.linear_mps>0)CHECK(f.n.motion_gate_diagnostics(t+.051).stop_evidence_window_s>=.08-1e-9);}
        bool resumed=false;
        for(unsigned i=0;i<20;++i){const double t=.60+i*.05;s=frame(t);CHECK(f.n.ingest(s));known_free(f.n,t);auto out=f.n.tick(t+.001);resumed=resumed||out.linear_mps>0;}
        CHECK(resumed&&f.n.enabled()&&f.n.state()=="FOLLOW_ROUTE");
    }
}
void test_extreme_signed_encoder_delta_is_rejected_without_overflow(){
    auto c=calibrated();c.encoder_signs={-1,1,-1,1};Navigator n(c);n.ingest(frame(0));
    auto s=frame(.1);s.counts[0]=0x80000000u;CHECK(!n.ingest(s));CHECK(n.reason()=="encoder_jump");
}

int main(){
    struct Test{const char* name;void(*run)();};
    const Test tests[]={
#define T(name) {#name,name}
        T(test_configuration_gate),T(test_first_boot_invalid_imu_can_recover),T(test_config_bounds),
        T(test_wrap_and_odometry),T(test_encoder_signs),T(test_sonar_event_duplicates_do_not_repaint),
        T(test_missing_encoder_and_latched_stop),T(test_wrong_encoder),T(test_stuck_encoder),
        T(test_jump_and_yaw_reset),T(test_duplicate_does_not_refresh),T(test_invalid_sonar),
        T(test_stale_sonar),T(test_control_watchdog),T(test_unknown_inflation_and_expiry),
        T(test_no_echo_is_unknown),T(test_sonar_uses_historical_pose),T(test_map_recentre),
        T(test_new_echo_overrides_old_free),T(test_localization_loss_requires_new_map),
        T(test_valid_route_and_acceleration_bound),T(test_wrong_request_ignored),T(test_nonadjacent_rejected),
        T(test_path_obstacle_rejected),T(test_planner_timeout),T(test_new_obstacle_stops_before_replan),
        
        T(test_manual_scan_bootstraps_measured_map),
        T(test_reject_path_loop),T(test_expired_plan_stops_not_resumes),
        T(test_fixed_profile_precision_and_rectangle),T(test_live_side_mapping_pairs_and_reset),T(test_pending_plan_obstacle_uses_measured_braking_speed),T(test_stop_waits_for_actual_encoder_coast_to_finish),T(test_old_device_echo_cannot_finish_stop_observation),T(test_unknown_rear_uses_bounded_local_turn),T(test_sonar_delay_is_part_of_braking_guard),T(test_valid_large_turn_not_artificial_segment_fault),T(test_late_plan_while_sonar_wait_preserves_session),T(test_plan_during_range_recovery_and_repeated_flap),T(test_extreme_signed_encoder_delta_is_rejected_without_overflow)
#undef T
    };
    unsigned failures=0;
    for(const auto& t:tests){try{t.run();std::cout<<"PASS "<<t.name<<"\n";}catch(const std::exception& e){++failures;std::cerr<<"FAIL "<<e.what()<<"\n";}}
    std::cout<<"navcore cases="<<sizeof(tests)/sizeof(tests[0])<<" failures="<<failures<<"\n";
    return failures?1:0;
}
