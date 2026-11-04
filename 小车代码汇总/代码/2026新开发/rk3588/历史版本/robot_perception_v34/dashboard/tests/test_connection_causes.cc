#include "../phone/control_audit.h"
#include <cassert>
#include <cstdio>
int main(){
 unsigned checks=0;
 for(const char* reason:{"window_focus_lost","activity_pause","connection_closed","control_ack_timeout","app_background"}){
  robotphone::JsonObject input;std::string error;
  assert(robotphone::parseFlatJson(std::string("{\"client_reason\":\"")+reason+"\",\"client_mono_ms\":12345,\"client_state_age_ms\":20,\"client_lease_age_ms\":30,\"client_diag_id\":2}",input,error));
  auto d=robotphone::ControlAudit::clientReported(input);assert(d.present&&d.reason==reason&&d.mono_ms==12345&&d.state_age_ms==20&&d.lease_age_ms==30&&d.id==2);checks++;
 }
 robotphone::JsonObject input;std::string error;assert(robotphone::parseFlatJson("{\"client_reason\":\"arbitrary_string_should_not_be_recorded\"}",input,error));
 assert(!robotphone::ControlAudit::clientReported(input).present);checks++;
 std::printf("PASS %u bounded connection cause allowlist checks; metadata does not authorize any motor command\n",checks);
}
