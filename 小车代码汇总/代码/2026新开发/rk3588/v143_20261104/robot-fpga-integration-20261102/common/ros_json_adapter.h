#pragma once
#include "ros_bridge_codec.h"
#include "../vendor/json.h"
#include <charconv>
#include <cmath>
#include <stdexcept>
namespace arb1 {
using Json=nlohmann::json;
inline uint64_t exact_u64(const Json&j){
 if(j.is_number_unsigned())return j.get<uint64_t>();
 if(j.is_number_integer()){const auto v=j.get<int64_t>();if(v>=0)return uint64_t(v);}
 if(j.is_string()){const auto s=j.get<std::string>();uint64_t v=0;const auto r=std::from_chars(s.data(),s.data()+s.size(),v);if(!s.empty()&&r.ec==std::errc{}&&r.ptr==s.data()+s.size())return v;}
 throw std::runtime_error("invalid exact uint64 field");
}
inline Id parse_id(const std::string&s){
 if(s.size()!=32)throw std::runtime_error("invalid goal UUID");
 Id out{};
 auto digit=[](char c)->int{return c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:c>='A'&&c<='F'?c-'A'+10:-1;};
 for(unsigned i=0;i<16;++i){const int a=digit(s[2*i]),b=digit(s[2*i+1]);if(a<0||b<0)throw std::runtime_error("invalid goal UUID digit");out[i]=uint8_t(a*16+b);}return out;
}
inline std::string id_hex(const Id&id){const char*s="0123456789abcdef";std::string out;for(auto b:id){out+=s[b>>4];out+=s[b&15];}return out;}
inline Identity identity_from_json(const Json&j){
 Identity i;i.session=exact_u64(j.at("session_id"));i.run=exact_u64(j.at("run_generation"));i.goal=parse_id(j.at("goal_id").get<std::string>());
 i.nav_epoch=exact_u64(j.at("nav_epoch"));i.map_epoch=exact_u64(j.at("map_epoch"));i.map_revision=exact_u64(j.at("map_revision"));
 i.localization_epoch=exact_u64(j.at("localization_epoch"));i.plan_id=exact_u64(j.at("plan_id"));
 if(j.find("frame_token")!=j.end())i.frame_token=j.at("frame_token").get<uint32_t>();
 return i;
}
inline Json identity_json(const Identity&i){return {{"session_id",std::to_string(i.session)},{"run_generation",std::to_string(i.run)},{"goal_id",id_hex(i.goal)},
 {"nav_epoch",std::to_string(i.nav_epoch)},{"map_epoch",std::to_string(i.map_epoch)},{"map_revision",std::to_string(i.map_revision)},
 {"localization_epoch",std::to_string(i.localization_epoch)},{"plan_id",std::to_string(i.plan_id)},{"frame_token",i.frame_token}};}
inline FpgaState fpga_state_from_json(const Json&j){
 if(!j.at("available").get<bool>()||j.value("state_stale",true)||j.value("stream_fault",true))throw std::runtime_error("FPGA state unavailable/stale");
 FpgaState s;s.identity=identity_from_json(j);s.state_seq=exact_u64(j.at("state_seq"));s.pose_seq=exact_u64(j.at("pose_seq"));
 const auto name=j.at("state").get<std::string>();const char*names[]={"IDLE","PLANNING","FOLLOWING","AVOIDING","ARRIVED","CANCELLED"};unsigned k=0;while(k<6&&name!=names[k])++k;if(k==6)throw std::runtime_error("unknown FPGA state");s.state=k;
 s.avoid_phase=j.at("avoid_stage").get<uint8_t>();s.reason=j.at("reason").get<uint16_t>();
 const auto&p=j.at("pose_mm_urad");if(!p.is_array()||p.size()!=3)throw std::runtime_error("pose missing");
 s.x_mm=p[0].get<int32_t>();s.y_mm=p[1].get<int32_t>();s.yaw_urad=p[2].get<int32_t>();s.pose_age_ms=j.at("pose_age_ms").get<uint32_t>();
 s.state_flags=j.at("flags").get<uint32_t>();s.cursor=j.at("waypoint_index").get<uint16_t>();s.count=j.at("path_count").get<uint16_t>();
 s.remaining_mm=j.at("remaining_mm").get<uint32_t>();s.permit_ms=j.at("permit_remaining_ms").get<uint32_t>();s.auth_ms=j.at("authorization_remaining_ms").get<uint32_t>();
 s.active_crc=j.at("active_crc32").get<uint32_t>();s.device_ms=j.at("device_ms").get<uint32_t>();s.owner=j.at("owner").get<uint32_t>();s.frame_id="map";return s;
}
inline PlanResult plan_from_json(const Json&j,uint64_t command){
 PlanResult p;p.identity=identity_from_json(j);p.command_id=command;p.start_pose_seq=exact_u64(j.at("start_pose_seq"));p.status=j.at("status").get<uint16_t>();
 p.path_revision=uint32_t(p.identity.map_revision);p.expanded=j.at("expanded").get<uint32_t>();p.cycles=j.at("cycles").get<uint32_t>();
 const auto&o=j.at("origin_mm");if(!o.is_array()||o.size()!=2)throw std::runtime_error("plan geometry missing");p.origin_x_mm=o[0].get<int32_t>();p.origin_y_mm=o[1].get<int32_t>();
 p.bitmap_crc32=j.at("bitmap_crc32").get<uint32_t>();const auto&v=j.at("cells");
 if(!v.is_array()||v.size()>4096||v.size()!=j.at("path_count").get<size_t>())throw std::runtime_error("incomplete plan cells");
 for(const auto&c:v){const auto id=exact_u64(c);if(id>=4096)throw std::runtime_error("plan cell out of range");p.cells.push_back(uint16_t(id));}return p;
}
inline ClockAnchor anchor_from_json(const Json&j){
 ClockAnchor c;c.session=exact_u64(j.at("session_id"));c.anchor_id=exact_u64(j.at("anchor_id"));c.info_request_id=exact_u64(j.at("info_request_id"));
 c.host_send_ns=exact_u64(j.at("host_send_ns"));c.host_receive_ns=exact_u64(j.at("host_receive_ns"));c.device_ms=j.at("device_ms").get<uint32_t>();
 c.anchor_request_id=exact_u64(j.at("anchor_request_id"));c.anchor_ack_device_ms=j.at("anchor_ack_device_ms").get<uint32_t>();
 if(!j.at("anchor_ack_verified").get<bool>()||!c.anchor_request_id||c.host_receive_ns<c.host_send_ns||c.host_receive_ns-c.host_send_ns>100000000)throw std::runtime_error("unverified clock anchor");
 c.rtt_bound_ms=uint32_t((c.host_receive_ns-c.host_send_ns+999999)/1000000);c.flags=1;return c;
}
}
