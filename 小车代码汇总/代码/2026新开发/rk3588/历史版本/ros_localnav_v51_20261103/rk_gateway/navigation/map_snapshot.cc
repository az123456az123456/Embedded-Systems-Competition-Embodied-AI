#include "map_snapshot.h"
#include "vendor/picojson.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace robot_nav {
namespace {
std::string quote(const std::string& s){return picojson::value(s).serialize();}
}
std::string map_snapshot_json(const Navigator& nav,double now,double age,
                              bool reset,const std::string& session){
    const auto& grid=nav.grid();const auto origin=grid.origin();const auto p=nav.pose();
    const bool valid=nav.localization_valid(now)&&!reset&&std::isfinite(age)&&age>=0&&age<=nav.config().sensor_timeout_s;
    std::ostringstream out;out<<std::fixed<<std::setprecision(5);
    out<<"{\"schema\":1,\"width\":64,\"height\":64,\"resolution_m\":"<<grid.resolution()
       <<",\"origin\":["<<origin.first<<','<<origin.second<<"],\"cells\":\"";
    for(int y=0;y<64;++y)for(int x=0;x<64;++x){int v=grid.state({x,y},now);out<<(v<0?'?':v==0?'.':'#');}
    out<<"\",\"navigable\":\"";
    const auto blocked=grid.planning_bitmap(now);
    for(unsigned i=0;i<4096;++i)out<<(grid.blocked(blocked,i)?'#':'.');
    out<<"\",\"pose\":["<<p.x<<','<<p.y<<','<<p.yaw<<"],\"pose_valid\":"<<(valid?"true":"false")
       <<",\"available\":"<<(valid?"true":"false")<<",\"updated_monotonic_ms\":"<<std::setprecision(0)<<now*1000
       <<",\"sensor_age_ms\":"<<(std::isfinite(age)&&age>=0?age*1000:-1)<<std::setprecision(5)
       <<",\"session_id\":"<<quote(session)<<",\"map_revision\":"<<grid.revision()
       <<",\"robot_radius_m\":"<<nav.config().robot_radius_m
       <<",\"footprint_m\":{\"front\":"<<nav.config().body_front_m<<",\"rear\":"<<nav.config().body_rear_m
       <<",\"left\":"<<nav.config().body_left_m<<",\"right\":"<<nav.config().body_right_m<<'}'
       <<",\"encoder_ticks_per_turn\":"<<nav.config().encoder_ticks_per_turn
       <<",\"left_wheel_mask\":"<<nav.config().left_wheel_mask
       <<",\"state\":"<<quote(nav.state())<<",\"reason\":"<<quote(nav.reason())
       <<",\"availability_reason\":"<<quote(valid?"live":reset?"reset_map_required":"waiting_for_valid_pose")
       <<",\"goal\":";
    if(nav.goal())out<<'['<<nav.goal()->first<<','<<nav.goal()->second<<']';else out<<"null";
    out<<",\"path\":[";bool first=true;
    for(const auto& q:nav.path()){if(!first)out<<',';first=false;out<<'['<<q.first<<','<<q.second<<']';}
    out<<"],\"trajectory\":[";first=true;
    for(const auto& q:nav.trajectory()){if(!first)out<<',';first=false;out<<'['<<q.x<<','<<q.y<<']';}
    out<<"],\"units\":\"metres,radians\"}";return out.str();
}
void MapSnapshotCache::publish(std::string json,double now){
    std::lock_guard<std::mutex> lock(mutex_);json_=std::move(json);updated_s_=now;
    history_.emplace_back(now,json_);
    while(history_.size()>6||(!history_.empty()&&now-history_.front().first>2.0))history_.pop_front();
}
bool MapSnapshotCache::validate_goal(const std::string& session,uint32_t revision,
                                    double x,double y,double now,std::string& error)const{
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(now)){error="invalid_goal";return false;}
    std::lock_guard<std::mutex> lock(mutex_);
    for(auto it=history_.rbegin();it!=history_.rend();++it){
        if(now<it->first||now-it->first>2.0)continue;
        picojson::value v;if(!picojson::parse(v,it->second).empty()||!v.is<picojson::object>())continue;
        if(!v.get("session_id").is<std::string>()||v.get("session_id").get<std::string>()!=session||
           !v.get("map_revision").is<double>()||v.get("map_revision").get<double>()!=revision)continue;
        if(!v.get("available").is<bool>()||!v.get("available").get<bool>()){error="map_not_ready";return false;}
        const auto& origin=v.get("origin");const auto& r=v.get("resolution_m");
        const auto& cells=v.get("navigable");
        if(!origin.is<picojson::array>()||origin.get<picojson::array>().size()!=2||!r.is<double>()||
           !cells.is<std::string>()||cells.get<std::string>().size()!=4096){error="map_not_ready";return false;}
        const auto& a=origin.get<picojson::array>();
        if(!a[0].is<double>()||!a[1].is<double>()||r.get<double>()<=0){error="map_not_ready";return false;}
        const double cx=(x-a[0].get<double>())/r.get<double>(),cy=(y-a[1].get<double>())/r.get<double>();
        if(cx<0||cx>=64||cy<0||cy>=64){error="goal_outside_local_map";return false;}
        if(cells.get<std::string>()[unsigned(cy)*64+unsigned(cx)]!='.'){error="goal_not_verified_free";return false;}
        return true;
    }
    error="map_snapshot_expired";return false;
}
std::string MapSnapshotCache::read(double now)const{
    std::string body;double updated;
    {std::lock_guard<std::mutex> lock(mutex_);body=json_;updated=updated_s_;}
    if(body.empty())return "{\"schema\":1,\"available\":false,\"pose_valid\":false,\"reason\":\"waiting_for_map\",\"snapshot_age_ms\":-1}";
    const double age=std::isfinite(now)&&now>=updated?(now-updated)*1000:1e9;
    body.pop_back();body+=",\"snapshot_age_ms\":"+std::to_string(age)+"}";return body;
}
}
