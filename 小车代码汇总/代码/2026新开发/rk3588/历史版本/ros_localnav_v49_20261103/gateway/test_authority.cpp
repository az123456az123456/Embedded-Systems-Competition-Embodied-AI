#include "nav_authority.h"
#include "../acg138_fpga_map_bridge/tests/stage3_plan_device.hpp"
#include <cassert>
using A=robotnav::NavAuthority;
static uint64_t host(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Setup {
 Fixture f;A a;n::PlanResult p;robotnav::DeviceClock clock;uint64_t now;
 Setup(){p=f.c.plan(f.goal(),f.pose(),350,50);now=host();const auto &i=f.c.last_info();clock={f.c.session(),f.c.anchor(),now,now,i.ack.device_ms};}
 bool begin(){a.phone("owner",1,now,true,now);return a.begin(p,clock,1,now);}
 n::Wire ack(const n::Wire&w,unsigned status=0){ne_message m{};assert(ne_decode(w.data(),w.size(),&m)==0);auto h=m.h;h.opcode|=128;h.payload_bytes=32;n::Payload payload{};ne_put16(payload.data(),status);ne_put64(payload.data()+8,p.nav_epoch);n::Wire out{};assert(ne_encode(&h,payload.data(),out.data())==0);return out;}
};
int main(int argc,char**argv){if(argc!=2)return 2;std::filesystem::create_directories(argv[1]);std::ofstream out(std::filesystem::path(argv[1])/"cases.jsonl");unsigned count=0,failed=0;
 auto test=[&](const char*name,auto fn){Setup s;bool ok=false;try{ok=fn(s);}catch(const std::exception&e){std::cerr<<e.what()<<'\n';}++count;if(!ok){++failed;std::cerr<<"FAILED "<<name<<'\n';}out<<"{\"case\":\""<<name<<"\",\"pass\":"<<(ok?"true":"false")<<"}\n";};
 test("no_implicit_enable",[](Setup&s){return !s.a.permit(s.now)&&!s.a.begin(s.p,s.clock,1,s.now);});
 test("real_guard_accepts_permit_and_exact_ack",[](Setup&s){if(!s.begin())return false;auto w=s.a.permit(s.now);if(!w)return false;ne_message m{};if(ne_decode(w->data(),w->size(),&m))return false;auto rc=nav_exec_handle(&s.f.d.exec,&m,s.f.d.now,1000);return rc==0&&s.a.accept_ack(s.ack(*w),s.now)&&s.a.ticket(s.now).has_value();});
 test("wrong_ack_identity_not_authority",[](Setup&s){s.begin();auto w=s.a.permit(s.now);auto ack=s.ack(*w);ack[24]^=1;ne_put16(ack.data()+256,ne_crc16(ack.data(),256));return !s.a.accept_ack(ack,s.now)&&!s.a.ticket(s.now);});
 test("negative_ack_revokes",[](Setup&s){s.begin();auto w=s.a.permit(s.now);return !s.a.accept_ack(s.ack(*w,NE_NO_PERMIT),s.now)&&!s.a.enabled();});
 test("old_phone_sequence_cannot_renew_30s",[](Setup&s){s.begin();s.a.phone("owner",1,s.now+29900,true,s.now+29900);s.a.permit(s.now+30000);return !s.a.enabled();});
 test("manual_takeover_stop_invalidates_ack",[](Setup&s){s.begin();auto w=s.a.permit(s.now);s.a.stop();return !s.a.accept_ack(s.ack(*w),s.now)&&!s.a.ticket(s.now)&&!s.a.permit(s.now);});
 test("500ms_gap_no_late_resume",[](Setup&s){s.begin();s.a.permit(s.now);s.clock.host_send_ms=s.clock.host_receive_ms=s.now+500;s.clock.device_ms+=500;s.a.clock(s.clock);return !s.a.permit(s.now+500)&&!s.a.enabled();});
 test("clock_source_age_not_receipt_refreshed",[](Setup&s){s.begin();s.a.permit(s.now);return !s.a.permit(s.now+1001)&&!s.a.enabled();});
 test("new_anchor_cannot_extend_same_phone_authorization",[](Setup&s){s.begin();auto w=s.a.permit(s.now);s.a.accept_ack(s.ack(*w),s.now);s.a.started(1,s.a.ticket(s.now)->permit_seq);const auto before=ne_get32(w->data()+96+40);s.clock.anchor++;s.clock.host_send_ms=s.clock.host_receive_ms=s.now+100;s.clock.device_ms+=103;s.a.clock(s.clock);auto next=s.a.permit(s.now+100);return next&&int32_t(ne_get32(next->data()+96+40)-before)<=0;});
 test("renewal_cadence_50ms",[](Setup&s){s.begin();auto w=s.a.permit(s.now);s.a.accept_ack(s.ack(*w),s.now);s.a.started(1,s.a.ticket(s.now)->permit_seq);return !s.a.permit(s.now+49)&&s.a.permit(s.now+50).has_value();});
 test("new_owner_never_inherits_exploration",[](Setup&s){s.begin();s.a.phone("new-owner",1,s.now,true,s.now);return !s.a.enabled()&&!s.a.permit(s.now);});
 test("old_generation_rejected",[](Setup&s){s.begin();s.a.stop();return !s.a.begin(s.p,s.clock,1,s.now);});
 test("permit_seq_cannot_overtake_START",[](Setup&s){s.begin();auto w=s.a.permit(s.now);s.a.accept_ack(s.ack(*w),s.now);return !s.a.permit(s.now+50)&&s.a.ticket(s.now+50).has_value()&&!s.a.permit(s.now+350)&&!s.a.enabled();});
 test("replan_negative_ack_waits_without_renewing_until_new_STATE",[](Setup&s){
  s.begin();auto w=*s.a.permit(s.now);s.a.accept_ack(s.ack(w),s.now);s.a.started(1,s.a.ticket(s.now)->permit_seq);
  auto old=*s.a.permit(s.now+50);const auto auth=ne_get32(old.data()+136);
  if(s.a.accept_ack(s.ack(old,NE_IDENTITY),s.now+51)||!s.a.enabled()||s.a.ticket(s.now+51)||s.a.permit(s.now+100))return false;
  n::StateView state;state.identity=s.p.identity;state.nav_epoch=s.p.nav_epoch;state.state=2;state.flags=32;
  if(!s.a.state(state,s.now+151)||s.a.permit(s.now+150))return false; // old STATE cannot end reconciliation
  ++state.identity.plan_id;if(!s.a.state(state,s.now+151))return false;
  auto next=s.a.permit(s.now+151);return next&&ne_get64(next->data()+72)==state.identity.plan_id&&int32_t(ne_get32(next->data()+136)-auth)<=0;
 });
 test("replan_without_new_STATE_times_out_no_resume",[](Setup&s){
  s.begin();auto w=*s.a.permit(s.now);s.a.accept_ack(s.ack(w),s.now);s.a.started(1,s.a.ticket(s.now)->permit_seq);
  auto old=*s.a.permit(s.now+50);s.a.accept_ack(s.ack(old,NE_IDENTITY),s.now+51);
  if(s.a.permit(s.now+250)||s.a.enabled())return false;
  n::StateView state;state.identity=s.p.identity;++state.identity.plan_id;state.nav_epoch=s.p.nav_epoch;state.state=2;state.flags=32;
  return !s.a.state(state,s.now+151)&&!s.a.permit(s.now+251);
 });
 test("identity_error_before_START_still_revokes",[](Setup&s){s.begin();auto w=*s.a.permit(s.now);s.a.accept_ack(s.ack(w,NE_IDENTITY),s.now);return !s.a.enabled();});
 test("replan_manual_takeover_never_restored",[](Setup&s){s.begin();auto w=*s.a.permit(s.now);s.a.accept_ack(s.ack(w),s.now);s.a.started(1,s.a.ticket(s.now)->permit_seq);auto old=*s.a.permit(s.now+50);s.a.accept_ack(s.ack(old,NE_IDENTITY),s.now+51);s.a.stop();n::StateView state;state.identity=s.p.identity;++state.identity.plan_id;state.nav_epoch=s.p.nav_epoch;state.state=2;state.flags=32;return !s.a.state(state,s.now+151)&&!s.a.permit(s.now+100);});
 std::ofstream summary(std::filesystem::path(argv[1])/"summary.json");summary<<"{\"cases\":"<<count<<",\"failed\":"<<failed<<"}\n";return failed?1:0;
}
