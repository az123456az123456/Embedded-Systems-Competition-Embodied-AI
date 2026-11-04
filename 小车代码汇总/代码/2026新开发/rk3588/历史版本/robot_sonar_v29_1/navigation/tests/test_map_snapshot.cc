#include "../map_snapshot.h"
#include "../vendor/picojson.h"
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace robot_nav;
static picojson::value parse(const std::string& s){picojson::value v;assert(picojson::parse(v,s).empty());return v;}
static bool flag(const picojson::value& v,const char* k){return v.get(k).get<bool>();}
static std::string text(const picojson::value& v,const char* k){return v.get(k).get<std::string>();}
int main(int argc,char**argv){
    Config c;c.configured=true;c.wheel_circumference_m=.20106193;c.encoder_ticks_per_turn=60000;
    c.track_m=.177;c.robot_radius_m=.18;c.drive_mps_per_full_pwm=.6;c.sonar_latency_s=0;
    c.body_front_m=c.body_rear_m=c.body_left_m=c.body_right_m=.10; // Explicit small synthetic body, not the production CAD profile.
    Navigator nav(c);MapSnapshotCache cache;
    assert(!flag(parse(cache.read(10)),"available"));
    auto empty=parse(map_snapshot_json(nav,10,-1,false,"boot-0"));
    assert(!flag(empty,"available")&&text(empty,"cells")==std::string(4096,'?'));
    SensorFrame f;f.timestamp_s=10;f.imu_valid=true;f.sonar_m={2.,2.,2.};nav.ingest(f);
    f.timestamp_s=10.05;assert(nav.ingest(f));
    std::vector<int8_t> cells(4096,0);cells[0]=-1;cells[1]=1;cells[63*64+63]=1;
    assert(nav.grid().import_observations(cells,{-3.2,-3.2},10.05));
    nav.set_goal(1.05,.05);assert(nav.start(10.05));auto request=*nav.tick(10.06).planner_request;
    std::vector<uint16_t> route;for(int x=request.start%64;x<=int(request.goal%64);++x)route.push_back(uint16_t((request.start/64)*64+x));
    assert(nav.accept_plan(request.request_id,request.map_revision,route,10.07));
    const auto body=map_snapshot_json(nav,10.08,.03,false,"boot-0");cache.publish(body,10.08);
    auto snapshot=parse(cache.read(10.1));
    assert(flag(snapshot,"available")&&flag(snapshot,"pose_valid"));
    assert(std::abs(snapshot.get("footprint_m").get("left").get<double>()-.10)<1e-6);
    assert(snapshot.get("encoder_ticks_per_turn").get<double>()==60000);
    assert(snapshot.get("left_wheel_mask").get<double>()==10);
    assert(text(snapshot,"cells").size()==4096&&text(snapshot,"cells").substr(0,3)=="?#.");
    assert(text(snapshot,"cells")[4095]=='#');
    assert(snapshot.get("path").get<picojson::array>().size()==route.size());
    assert(snapshot.get("goal").get<picojson::array>()[0].get<double>()==1.05);
    assert(std::abs(snapshot.get("snapshot_age_ms").get<double>()-20)<1e-5);
    // Old data never becomes a live pose because the mapper still runs.
    assert(!flag(parse(map_snapshot_json(nav,10.5,.45,false,"boot-0")),"available"));
    assert(!flag(parse(map_snapshot_json(nav,10.08,.03,true,"boot-0")),"available"));
    assert(parse(cache.read(13)).get("snapshot_age_ms").get<double>()>2000);
    f.timestamp_s=10.1;f.imu_valid=false;nav.ingest(f);
    assert(!flag(parse(map_snapshot_json(nav,10.1,0,false,"boot-0")),"pose_valid"));
    nav=Navigator(c);auto reset=parse(map_snapshot_json(nav,11,-1,false,"boot-1"));
    assert(text(reset,"session_id")=="boot-1"&&text(reset,"cells")==std::string(4096,'?'));
    assert(reset.get("path").get<picojson::array>().empty()&&reset.get("goal").is<picojson::null>());
    if(argc==2){std::ofstream fixture(argv[1]);fixture<<cache.read(10.1);assert(fixture.good());}
    std::cout<<"PASS map snapshot: world orientation, complete paths/goals, unknown cells, freshness, reset epoch and cache age\n";
}
