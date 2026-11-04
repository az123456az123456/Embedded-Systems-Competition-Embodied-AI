#include "phone_nav_adapter.h"
#include "http_client.h"
#include "../vendor/json.h"
#include <filesystem>
#include <fstream>
#include <cassert>
// Only the bottom packet transport is replaced. Production private HTTP parsing,
// source-age admission and FpgaDelegation execute unchanged. No raw socket opens.
namespace robotnav {
NvaSocket::~NvaSocket()=default;
bool NvaSocket::open(const std::string&,std::string&){return true;}
bool NvaSocket::send(const acg138::nav3::Wire&){return true;}
bool NvaSocket::receive(acg138::nav3::Wire&){return false;}
}
int main(int argc,char**argv){
 if(argc!=2)return 2;
 using J=nlohmann::json;
 std::filesystem::create_directories(argv[1]);
 std::ofstream cases(std::filesystem::path(argv[1])/"cases.jsonl");
 robotnav::PhoneNavAdapter adapter;assert(adapter.open("test-only"));
 unsigned checks=0,failed=0;const uint64_t now=100000;
 auto run=[&](const char*name,const char*op,J body,int expected,bool idle=true){
  int code=0;std::string response;
  bool matched=adapter.request(op==std::string("status")?"GET":"POST",std::string("/api/navigation/fpga/")+op,body.dump(),"phone",idle,now,code,response);
  bool ok=matched&&code==expected;
  ++checks;if(!ok)++failed;
  cases<<J({{"case",name},{"pass",ok},{"actual_code",code},{"expected_code",expected},{"response",J::parse(response)}}).dump()<<'\n';
  return J::parse(response);
 };
 auto task=[&](uint64_t generation,uint64_t expected){return J{{"session","phone"},{"generation",std::to_string(generation)},{"expected_generation",std::to_string(expected)},{"produced_mono_ns",std::to_string(now*1000000)},{"valid_for_ms","500"}};};
 auto first=run("startup_no_authority","status",J::object(),200);
 assert(!first.at("quiet").get<bool>()&&!first.at("enabled").get<bool>());
 auto fractional=task(10,0);fractional["produced_mono_ns"]=std::to_string(now*1000000+999999);
 run("fractional_ns_reproduces_dashboard_quantization_rejection","prepare",fractional,409);
 fractional["produced_mono_ns"]=std::to_string(robotnav::dashboard_request_timestamp(now*1000000+999999));
 run("sender_floors_to_dashboard_ms_without_extending_TTL","prepare",fractional,200);
 run("explicit_prepare_suppresses_MOT_but_no_permit","prepare",task(10,10),200);
 adapter.takeover();
 run("late_prepare_larger_generation_cannot_undo_manual","prepare",task(100,10),409);
 auto manual=run("manual_remains_owner","status",J::object(),200);
 assert(!manual.at("quiet").get<bool>()&&!manual.at("enabled").get<bool>());
 run("new_explicit_task_can_prepare","prepare",task(101,11),200);
 auto no_plan=task(101,0);no_plan["command_id"]="123";
 run("claim_requires_real_plan_state_clock","claim",no_plan,409);
 run("late_release_cannot_revoke_new_generation","release",task(10,0),409);
 run("nonidle_prepare_rejected","prepare",task(102,101),409,false);
 auto expired=task(102,101);expired["produced_mono_ns"]=std::to_string((now-500)*1000000);
 run("exact_500ms_is_expired","prepare",expired,409);
 auto zero=task(102,101);zero["valid_for_ms"]="0";run("zero_TTL_cannot_execute","prepare",zero,409);
 auto wrong=task(102,101);wrong["session"]="old-phone";run("old_phone_rejected","prepare",wrong,409);
 auto numeric=task(102,101);numeric["generation"]=102;run("numeric_u64_does_not_lose_precision","prepare",numeric,409);
 auto missing=task(102,101);missing.erase("expected_generation");run("missing_atomic_prepare_barrier_rejected","prepare",missing,409);
 run("unknown_endpoint_rejected","unknown",J::object(),404);
 adapter.takeover();adapter.takeover();
 const auto next=robotnav::next_dashboard_generation(102,103);
 assert(next==104);
 run("explicit_task_after_STOP_and_DISARM_uses_observed_generation","prepare",task(next,103),200);
 adapter.takeover();
 run("new_generation_still_cannot_bypass_atomic_manual_barrier","prepare",task(106,103),409);
 std::ofstream(std::filesystem::path(argv[1])/"summary.json")<<J({{"checks",checks},{"failed",failed},{"real_private_HTTP_handler",true},{"bottom_transport","stub; no raw packets"},{"real_Controller_HTTP_integration","NOT_RUN"}}).dump(2)<<'\n';
 return failed?1:0;
}
