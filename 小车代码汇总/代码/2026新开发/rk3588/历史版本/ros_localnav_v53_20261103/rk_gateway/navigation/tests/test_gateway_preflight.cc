// Actual Shared status + Gateway + loopback backend. No hardware or test-only
// production hooks; preflight must preserve manual authority before claim.
#define GATEWAY_HANDOFF_NO_MAIN
#include "test_gateway_core_handoff.cc"
int main(){
 for(bool latched:{false,true})for(const std::string operation:{"straight","explore","goto","local"}){
  Fixture f;f.prepare();
  {std::lock_guard<std::mutex> lock(f.shared->mutex);
   if(latched)f.shared->nav.invalidate_localization("encoder_wheel_disagreement",navwire::monotonicSeconds());
   else f.shared->reset_required=true;
  }
  const auto zeros=f.backend.zero_entered.load(),claims=f.backend.claim_entered.load(),attempts=f.attempts.load();
  unsigned epoch;{std::lock_guard<std::mutex> lock(f.shared->mutex);epoch=f.shared->origin_epoch;}
  if(operation=="local"){
   std::string error;check(!f.gateway->requestNavigation("straight",0,0,error));
   check(error==(latched?"encoder_wheel_disagreement":"reset-map_required_after_fpga_reset_or_imu_zero_or_wheel_mapping_change"));
  }else{
   const auto reply=f.start_http(operation,operation=="goto"?",\"x\":0.5,\"y\":0.0,\"map_session\":\"fixture-map\",\"map_revision\":0":"");
   check(reply.code==409);check(reply.json.get("error").get<std::string>()==(latched?"encoder_wheel_disagreement":"reset-map_required_after_fpga_reset_or_imu_zero_or_wheel_mapping_change"));
  }
  check(f.backend.zero_entered==zeros&&f.backend.claim_entered==claims&&f.attempts==attempts);
  check(f.backend.is_armed()&&!f.gateway->navigationActive()&&!f.gateway->navigationPreparing()&&!f.core_enabled());
  {std::lock_guard<std::mutex> lock(f.shared->mutex);check(f.shared->origin_epoch==epoch);if(latched)check(f.shared->nav.localization_diagnostics().latched);}
  check(f.control("drive",",\"throttle\":0.1,\"steer\":0").code==200);check(f.backend.is_armed());
  check(f.control("stop").code==200);
 }
 std::printf("PASS %u actual Shared/Gateway localization preflight checks: precise latched/reset reason, zero claim/zero-drive side effects, manual authority retained, HTTP straight/explore/goto and console; no map reset\n",checks);
}
