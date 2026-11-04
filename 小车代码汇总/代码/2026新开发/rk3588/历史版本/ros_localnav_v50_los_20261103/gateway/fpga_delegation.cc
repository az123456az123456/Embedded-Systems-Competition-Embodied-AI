#include "fpga_delegation.h"
#include <algorithm>
#include <limits>
namespace robotnav {
namespace {
acg138::nav3::Header header(const arb1::Identity& a){
 acg138::nav3::Header h;h.session_id=a.session;h.run_generation=a.run;h.goal_id=a.goal;
 h.map_epoch=a.map_epoch;h.map_revision=a.map_revision;h.localization_epoch=a.localization_epoch;h.plan_id=a.plan_id;return h;
}
bool same(const arb1::Identity&a,const arb1::Identity&b){
 return a.session==b.session&&a.run==b.run&&a.goal==b.goal&&a.nav_epoch==b.nav_epoch&&
 a.map_epoch==b.map_epoch&&a.map_revision==b.map_revision&&a.localization_epoch==b.localization_epoch&&a.plan_id==b.plan_id;
}
}
FpgaDelegation::FpgaDelegation(Send s):send_(std::move(s)){}
void FpgaDelegation::phone(const std::string&o,uint64_t seq,uint64_t source,bool armed,uint64_t now){
 authority_.phone(o,seq,source,armed,now);
 if(!owner_.empty()&&o!=owner_)takeover();
 if(!armed&&authority_.enabled())authority_.stop();
}
bool FpgaDelegation::context(const arb1::Message&m,uint64_t now){
 if(m.header.type!=ARB1_MSG_SNAPSHOT||m.blocks.empty()||m.blocks.size()>3)return false;
 // Validate the complete context before mutating any cached object.
 for(const auto&b:m.blocks){
  if(!std::holds_alternative<arb1::FpgaState>(b.payload)&&!std::holds_alternative<arb1::PlanResult>(b.payload)&&!std::holds_alternative<arb1::ClockAnchor>(b.payload))return false;
  if(b.metadata.received_mono_ns>now)return false;
  if(std::holds_alternative<arb1::FpgaState>(b.payload)&&
    (b.metadata.source_age_ns==ARB1_UNKNOWN_SOURCE_AGE||b.metadata.source_age_ns>250000000||now-b.metadata.received_mono_ns>250000000-b.metadata.source_age_ns))return false;
 }
 for(const auto&b:m.blocks){
  if(auto p=std::get_if<arb1::PlanResult>(&b.payload)){
   if(!plan_||p->identity.session!=plan_->identity.session||p->command_id>plan_->command_id){plan_=*p;plan_received_ns_=b.metadata.received_mono_ns;}
  }else if(auto s=std::get_if<arb1::FpgaState>(&b.payload)){
   // A retransmitted cached STATE cannot become fresh merely by arriving again.
   if(state_&&s->identity.session==state_->identity.session&&s->pose_seq<=state_->pose_seq&&s->state_seq<=state_->state_seq)continue;
   if(authority_.enabled()&&authorized_identity_){
    auto a=*authorized_identity_,b=s->identity;a.plan_id=b.plan_id;a.nav_epoch=b.nav_epoch;
    if(!same(a,b)){authority_.stop();reason_="FPGA_session_or_task_changed_no_renewal";}
   }
   state_=*s;state_sample_ns_=b.metadata.received_mono_ns-b.metadata.source_age_ns;
   acg138::nav3::StateView v;v.identity=header(s->identity);v.nav_epoch=s->identity.nav_epoch;
   v.state=s->state;v.pose_age_ms=s->pose_age_ms;v.flags=s->state_flags;
   const bool was_enabled=authority_.enabled();authority_.state(v,now/1000000);
   if(was_enabled&&!authority_.enabled())reason_=authority_.stop_reason();
  }else if(auto c=std::get_if<arb1::ClockAnchor>(&b.payload)){
   if(!(c->flags&1)||!c->anchor_request_id||c->host_receive_ns>now)continue;
   DeviceClock v;v.session=c->session;v.anchor=c->anchor_id;v.device_ms=c->device_ms;
   v.host_send_ms=c->host_send_ns/1000000;v.host_receive_ms=c->host_receive_ns/1000000;
   if(!clock_||v.session!=clock_->session||v.anchor>clock_->anchor){clock_=v;authority_.clock(v);}
  }
 }
 return true;
}
bool FpgaDelegation::prepare(const std::string& owner,uint64_t generation,bool idle){
 if(owner.empty()||!generation||generation<generation_||!idle||
    (generation==generation_&&(!quiet_||owner!=owner_))){reason_="prepare_requires_current_owner_idle_new_generation";return false;}
 if(generation>generation_){authority_.stop();plan_.reset();generation_=generation;}
 owner_=owner;quiet_=true;reason_="prepared_no_MOT_probes";return true;
}
bool FpgaDelegation::healthy(uint64_t now)const{
 return state_&&now*1000000>=state_sample_ns_&&state_->pose_age_ms<=250&&
 now*1000000-state_sample_ns_<=250000000-uint64_t(state_->pose_age_ms)*1000000&&
 (state_->state_flags&32u);
}
bool FpgaDelegation::claim(uint64_t command,uint64_t generation,uint64_t now){
 if(!quiet_||generation!=generation_||!plan_||plan_->command_id!=command||plan_->status||plan_->cells.empty()||
 !state_||!same(plan_->identity,state_->identity)||!healthy(now)||!clock_||clock_->session!=plan_->identity.session){reason_="plan_state_clock_or_generation_mismatch";return false;}
 acg138::nav3::PlanResult p;p.identity=header(plan_->identity);p.nav_epoch=plan_->identity.nav_epoch;
 p.status=acg138::nav3::Status::Ok;p.cells=plan_->cells;p.start_pose_seq=plan_->start_pose_seq;
 if(!authority_.begin(p,*clock_,generation,now)){reason_="explicit_armed_phone_authority_required";return false;}
 authorized_identity_=plan_->identity;reason_="waiting_real_NVA_ACK";return true;
}
void FpgaDelegation::tick(uint64_t now){
 if(!quiet_||!authority_.enabled())return;
 if(!healthy(now)){authority_.stop();reason_="FPGA_state_stale_no_permit_renewal";return;}
 if(auto packet=authority_.permit(now)){if(!send_(*packet)){authority_.stop();reason_="NVA_send_failed";}}
 else if(!authority_.enabled())reason_=authority_.stop_reason();
}
bool FpgaDelegation::ack(const acg138::nav3::Wire&w,uint64_t now){const bool was_enabled=authority_.enabled();const bool ok=authority_.accept_ack(w,now);if(was_enabled&&!authority_.enabled())reason_=authority_.stop_reason();return ok;}
bool FpgaDelegation::started(uint64_t g,uint64_t seq){const bool ok=g==generation_&&authority_.started(g,seq);if(ok)reason_="FPGA_execution_started";return ok;}
void FpgaDelegation::takeover(){authority_.stop();authorized_identity_.reset();quiet_=false;owner_.clear();plan_.reset();if(generation_!=UINT64_MAX)++generation_;reason_="manual_or_stop_takeover";}
}
