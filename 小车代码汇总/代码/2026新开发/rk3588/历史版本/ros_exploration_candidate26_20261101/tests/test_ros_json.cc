#include "ros_json_adapter.h"
#include <cassert>
#include <fstream>
#include <filesystem>
using namespace arb1;
int main(int argc,char**argv){
 if(argc!=2)return 2;
 std::filesystem::create_directories(argv[1]);std::ofstream cases(std::filesystem::path(argv[1])/"cases.jsonl");unsigned count=0;
 auto check=[&](bool b,const char*n){cases<<"{\"case\":\""<<n<<"\",\"pass\":"<<(b?"true":"false")<<"}\n";cases.flush();assert(b);++count;};
 auto rejects=[](auto f){try{f();return false;}catch(const std::exception&){return true;}};
 check(exact_u64(Json::parse("18446744073709551615"))==UINT64_MAX,"numeric_u64_exact");
 check(exact_u64(Json("9007199254740993"))==9007199254740993ull,"string_u64_above_double_precision");
 check(rejects([]{exact_u64(Json(1.5));}),"floating_id_rejected");
 check(rejects([]{parse_id(std::string(32,'g'));}),"malformed_goal_id_rejected");
 Identity id;id.session=UINT64_MAX;id.run=9007199254740993ull;id.goal=parse_id("00112233445566778899aabbccddeeff");id.nav_epoch=1;id.map_epoch=1;id.map_revision=3;id.localization_epoch=1;id.plan_id=5;
 check(identity_from_json(identity_json(id)).run==id.run,"identity_round_trip_exact");
 auto plan=identity_json(id);plan["start_pose_seq"]="12";plan["status"]=0;plan["expanded"]=123;plan["cycles"]=500;plan["origin_mm"]={-3200,-3200};plan["bitmap_crc32"]=123;plan["path_count"]=4096;plan["cells"]=Json::array();for(unsigned i=0;i<4096;++i)plan["cells"].push_back(i);
 check(plan_from_json(plan,99).cells.size()==4096,"complete_4096_cell_plan");
 auto bad=plan;bad["path_count"]=4095;check(rejects([&]{plan_from_json(bad,99);}),"truncated_plan_rejected");
 bad=plan;bad["cells"][0]=4096;check(rejects([&]{plan_from_json(bad,99);}),"out_of_window_cell_rejected");
 ClockAnchor anchor;Json a={{"session_id","10"},{"anchor_id","1"},{"info_request_id","2"},{"host_send_ns","1000000000"},{"host_receive_ns","1001000000"},{"device_ms",100},{"anchor_request_id","3"},{"anchor_ack_device_ms",101},{"anchor_ack_verified",true}};
 anchor=anchor_from_json(a);check(anchor.flags==1&&anchor.anchor_request_id==3,"real_ack_anchor_fields");
 a["anchor_ack_verified"]=false;check(rejects([&]{anchor_from_json(a);}),"unverified_anchor_rejected");
 Json s=identity_json(id);s["available"]=true;s["state_stale"]=false;s["stream_fault"]=false;s["state_seq"]="1";s["pose_seq"]="2";s["state"]="FOLLOWING";s["avoid_stage"]=0;s["reason"]=0;s["pose_mm_urad"]={11,22,33};s["pose_age_ms"]=20;s["flags"]=32;s["waypoint_index"]=1;s["path_count"]=5;s["remaining_mm"]=100;s["permit_remaining_ms"]=400;s["authorization_remaining_ms"]=25000;s["active_crc32"]=123;s["device_ms"]=101;s["owner"]=1;
 check(fpga_state_from_json(s).x_mm==11,"state_keeps_actual_pose");
 s.erase("pose_mm_urad");check(rejects([&]{fpga_state_from_json(s);}),"missing_pose_never_filled_with_zero");
 std::ofstream(std::filesystem::path(argv[1])/"summary.json")<<"{\"cases\":"<<count<<",\"failed\":0}\n";
}
