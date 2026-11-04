#include "../map_snapshot.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace robot_nav;
int main(){
    unsigned checks=0;
    auto check=[&](bool b){++checks;assert(b);};
    Config c;c.configured=true;c.wheel_circumference_m=.20106193;c.encoder_ticks_per_turn=60000;
    c.track_m=.177;c.robot_radius_m=.18;c.drive_mps_per_full_pwm=.6;c.sonar_latency_s=0;
    c.body_front_m=c.body_rear_m=c.body_left_m=c.body_right_m=.10; // Explicit small synthetic body, not the production CAD profile.
    Navigator n(c);SensorFrame f;f.timestamp_s=10;f.imu_valid=true;f.sonar_m={2.,2.,2.};
    n.ingest(f);f.timestamp_s=10.05;check(n.ingest(f));
    std::vector<int8_t> cells(4096,0);cells[40*64+40]=1;
    check(n.grid().import_observations(cells,{-3.2,-3.2},10.05));
    const auto revision=n.grid().revision();MapSnapshotCache cache;std::string error;
    cache.publish(map_snapshot_json(n,10.06,.01,false,"boot-0"),10.06);
    check(cache.validate_goal("boot-0",revision,1.05,.05,10.07,error));
    check(!cache.validate_goal("boot-1",revision,1.05,.05,10.07,error));
    check(!cache.validate_goal("boot-0",revision+1,1.05,.05,10.07,error));
    check(!cache.validate_goal("boot-0",revision,1.05,.05,12.061,error)&&error=="map_snapshot_expired");
    check(!cache.validate_goal("boot-0",revision,1.05,.05,9.99,error));
    check(!cache.validate_goal("boot-0",revision,10000,.05,10.07,error)&&error=="goal_outside_local_map");
    check(!cache.validate_goal("boot-0",revision,INFINITY,.05,10.07,error));
    check(!cache.validate_goal("boot-0",revision,.85,.85,10.07,error)&&error=="goal_not_verified_free");
    check(!cache.validate_goal("boot-0",revision,.65,.85,10.07,error)); // inflated clearance
    check(n.go_to(1.08,.08,10.07,error));
    check(std::abs(n.goal()->first-1.05)<1e-9&&std::abs(n.goal()->second-.05)<1e-9);
    auto control=n.tick(10.08);check(control.planner_request.has_value()&&control.left==0&&control.right==0);
    const auto old=*control.planner_request;
    check(n.go_to(.55,.05,10.09,error));control=n.tick(10.10);
    check(control.planner_request&&control.planner_request->request_id!=old.request_id);
    check(!n.accept_plan(old.request_id,old.map_revision,{old.start,old.goal},10.10));
    n.stop();check(!n.go_to(.85,.85,10.10,error)&&!n.enabled());
    check(!n.go_to(.65,.85,10.10,error)&&error=="goal_not_verified_free");
    // A visible old free cell does not override a new obstacle in the live map.
    cells[32*64+42]=1;check(n.grid().import_observations(cells,{-3.2,-3.2},10.11));
    check(cache.validate_goal("boot-0",revision,1.05,.05,10.12,error));
    check(!n.go_to(1.05,.05,10.12,error)&&error=="goal_not_verified_free");
    check(!n.go_to(.55,.05,10.5,error)&&error=="telemetry_stale");
    c.configured=false;Navigator uncalibrated(c);
    check(!uncalibrated.go_to(.55,.05,10.1,error)&&error=="calibration_not_confirmed");
    // Unknown cells and footprint margin stay blocked; no invented map on tap.
    Navigator unknown(n.config());f.timestamp_s=11;unknown.ingest(f);f.timestamp_s=11.05;unknown.ingest(f);
    check(!unknown.go_to(-1.05,.05,11.06,error)&&error=="goal_not_verified_free");
    // Cache survives moving-map updates, but never changes the clicked world coordinate.
    cache.publish(map_snapshot_json(n,10.13,.08,false,"boot-0"),10.13);
    check(cache.validate_goal("boot-0",revision,.55,.05,10.14,error));
    std::cout<<"PASS map goal "<<checks<<" checks: snapshot age/epoch/revision, metre coordinates, live obstacle clearance, FPGA request generation, unknown space, calibration/staleness\n";
}
