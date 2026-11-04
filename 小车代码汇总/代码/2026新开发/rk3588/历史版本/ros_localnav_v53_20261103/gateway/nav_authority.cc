#include "nav_authority.h"
#include <algorithm>
#include <limits>
namespace robotnav {
void NavAuthority::stop(const std::string& reason){stop_reason_=reason;enabled_=false;pending_.reset();ticket_.reset();permit_started_=false;navigation_started_=false;replan_wait_until_ms_=0;}
void NavAuthority::phone(const std::string& owner,uint64_t seq,uint64_t original,bool armed,uint64_t now){
 if(owner.empty()||!seq||original>now||now-original>=30000){if(enabled_)stop("NVA_phone_owner_missing_or_expired");armed_=false;return;}
 if(owner!=phone_owner_){stop("NVA_phone_owner_changed");phone_owner_=owner;phone_sequence_=phone_source_ms_=0;}
 if(seq<=phone_sequence_||original<phone_source_ms_)return; // duplicate never renews
 phone_sequence_=seq;phone_source_ms_=original;armed_=armed;if(!armed&&enabled_)stop("NVA_phone_disarmed");
}
bool NavAuthority::clock(const DeviceClock& c){
 if(c.session!=plan_.identity.session_id||!c.anchor||c.host_receive_ms<c.host_send_ms||c.host_receive_ms-c.host_send_ms>100)return false;
 if(clock_.session==c.session&&(c.anchor<clock_.anchor||c.host_receive_ms<clock_.host_receive_ms))return false;
 clock_=c;return true;
}
bool NavAuthority::begin(const Plan& p,const DeviceClock& c,uint64_t gen,uint64_t now){
 if(enabled_||!armed_||!phone_sequence_||now<phone_source_ms_||now-phone_source_ms_>=30000||
 !gen||gen<=generation_||!p.identity.session_id||!p.identity.run_generation||!p.identity.plan_id||!p.nav_epoch||
 p.status!=acg138::nav3::Status::Ok||p.cells.empty()||auth_epoch_==UINT64_MAX)return false;
 plan_=p;if(!clock(c)||now<c.host_receive_ms||now-c.host_receive_ms>1000)return false;
 generation_=gen;++auth_epoch_;last_auth_phone_=0;enabled_=true;stop_reason_.clear();permit_started_=false;navigation_started_=false;last_send_ms_=0;replan_wait_until_ms_=0;ticket_.reset();pending_.reset();return true;
}
bool NavAuthority::state(const acg138::nav3::StateView& s,uint64_t now){
 if(replan_wait_until_ms_&&now>=replan_wait_until_ms_){stop("NVA_replan_STATE_not_confirmed");return false;}
 if(!enabled_||!acg138::nav3::same_task(s.identity,plan_.identity,false))return false;
 if(s.state==4||s.state==5||(s.flags&8u)||s.nav_epoch!=plan_.nav_epoch){stop("NVA_FPGA_state_or_epoch_changed:state="+std::to_string(s.state)+":expected_epoch="+std::to_string(plan_.nav_epoch)+":actual_epoch="+std::to_string(s.nav_epoch));return false;}
 if(s.identity.plan_id<plan_.identity.plan_id||s.pose_age_ms>250||!(s.flags&32u))return false;
 if(s.identity.plan_id>plan_.identity.plan_id){plan_.identity.plan_id=s.identity.plan_id;pending_.reset();ticket_.reset();replan_wait_until_ms_=0;}
 return true;
}
std::optional<NavAuthority::Wire> NavAuthority::permit(uint64_t now){
 if(!enabled_)return {};
 if(!armed_){stop("NVA_phone_disarmed");return {};}
 if(now<phone_source_ms_||now-phone_source_ms_>=30000){stop("NVA_phone_authorization_expired");return {};}
 if(now<clock_.host_receive_ms||now-clock_.host_receive_ms>1000){stop("NVA_clock_anchor_stale");return {};}
 if(sequence_==UINT64_MAX){stop("NVA_sequence_exhausted");return {};}
 if(permit_started_&&(now<last_send_ms_||now-last_send_ms_>=500)){stop("NVA_sender_gap_500ms");return {};}
 if(replan_wait_until_ms_){
   if(now>=replan_wait_until_ms_)stop("NVA_replan_STATE_not_confirmed");
   return {}; // Never renew the old plan or infer a new one from a rejection.
 }
 if(ticket_&&!navigation_started_){
   if(!ticket(now)){stop("NVA_START_ticket_expired");}return {}; // no new permit seq can overtake START
 }
 if(permit_started_&&now-last_send_ms_<50)return {};
 // Stop-and-wait first permit until its ACK mints START; do not replace its
 // identity while a delayed ACK is in flight. No retransmission renews a lease.
 if(pending_&&now-last_send_ms_<100)return {};
 ne_header h{};h.family=NE_NVA1;h.opcode=NE_PERMIT;h.payload_bytes=64;
 h.transport_seq=++sequence_;h.request_id=sequence_;const auto&a=plan_.identity;
 h.session_id=a.session_id;h.run_generation=a.run_generation;std::copy(a.goal_id.begin(),a.goal_id.end(),h.goal_id);
 h.map_epoch=a.map_epoch;h.map_revision=a.map_revision;h.localization_epoch=a.localization_epoch;h.plan_id=a.plan_id;
 uint8_t p[160]{};ne_put32(p,NE_OWNER_NAV);ne_put64(p+4,plan_.nav_epoch);ne_put64(p+12,auth_epoch_);
 ne_put64(p+20,phone_sequence_);ne_put64(p+28,sequence_);
 // INFO device timestamp precedes host receive. Use that lower device-time
 // bound (minus one ms for quantisation), never a fresh "now+30s" lease.
 const uint32_t lower=clock_.device_ms+uint32_t(now-clock_.host_receive_ms)-1u;
 ne_put32(p+36,lower+450u);
 uint32_t ad=lower+uint32_t(30000-(now-phone_source_ms_));
 if(last_auth_phone_==phone_sequence_&&int32_t(ad-auth_deadline_)>0)ad=auth_deadline_;
 last_auth_phone_=phone_sequence_;auth_deadline_=ad;ne_put32(p+40,ad);
 ne_put64(p+44,clock_.anchor);Wire w{};
 if(ne_encode(&h,p,w.data())!=NE_CODEC_OK){stop("NVA_encode_failed");return {};}
 pending_=h;last_send_ms_=now;if(!permit_started_)first_send_ms_=now;permit_started_=true;return w;
}
bool NavAuthority::accept_ack(const Wire& w,uint64_t now){
 if(!enabled_||!pending_||now<last_send_ms_||now-last_send_ms_>=100)return false;
 ne_message m{};if(ne_decode(w.data(),w.size(),&m)!=NE_CODEC_OK)return false;const auto&a=m.h;const auto&b=*pending_;
 if(a.family!=NE_NVA1||a.opcode!=(NE_PERMIT|0x80)||a.transport_seq!=b.transport_seq||a.request_id!=b.request_id||
 a.session_id!=b.session_id||a.run_generation!=b.run_generation||!std::equal(a.goal_id,a.goal_id+16,b.goal_id)||
 a.map_epoch!=b.map_epoch||a.map_revision!=b.map_revision||a.localization_epoch!=b.localization_epoch||a.plan_id!=b.plan_id)return false;
 // A replan can commit between our last STATE and this PERMIT. The negative
 // ACK grants nothing: wait (at most 200ms from the original send) for a newer
 // same-task, same-epoch, fresh STATE before changing plan_id. All other errors
 // revoke immediately; existing 500ms/30s/pose guards keep their original ages.
 if(ne_get16(m.payload)==NE_IDENTITY&&navigation_started_&&
    ne_get64(m.payload+8)==plan_.nav_epoch){
  replan_wait_until_ms_=last_send_ms_+200;pending_.reset();ticket_.reset();return false;
 }
 if(ne_get16(m.payload)!=NE_OK||ne_get64(m.payload+8)!=plan_.nav_epoch){
  stop("NVA_FPGA_ACK_rejected:status="+std::to_string(ne_get16(m.payload))+":request="+std::to_string(b.request_id)+":expected_epoch="+std::to_string(plan_.nav_epoch)+":actual_epoch="+std::to_string(ne_get64(m.payload+8)));return false;
 }
 acg138::nav3::StartTicket t;t.identity=plan_.identity;t.nav_epoch=plan_.nav_epoch;t.permit_seq=b.transport_seq;t.auth_epoch=auth_epoch_;
 // Expiry uses the original send deadline; accepting an ACK does not re-age it.
 t.expires_at=std::chrono::steady_clock::time_point(std::chrono::milliseconds(last_send_ms_+350));
 ticket_=t;pending_.reset();return true;
}
bool NavAuthority::started(uint64_t generation,uint64_t sequence){
 if(!enabled_||!ticket_||generation!=generation_||sequence!=ticket_->permit_seq)return false;
 navigation_started_=true;return true;
}
std::optional<acg138::nav3::StartTicket> NavAuthority::ticket(uint64_t now)const {
 if(!enabled_||!ticket_||now<phone_source_ms_||now-phone_source_ms_>=30000||
 std::chrono::steady_clock::time_point(std::chrono::milliseconds(now))>=ticket_->expires_at)return {};
 return ticket_;
}
}
