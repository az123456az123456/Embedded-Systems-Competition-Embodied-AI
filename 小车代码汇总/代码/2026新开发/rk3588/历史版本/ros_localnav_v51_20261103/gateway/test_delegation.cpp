#include "fpga_delegation.h"
#include "../acg138_fpga_map_bridge/tests/stage3_plan_device.hpp"
#include <functional>
#include <cassert>
using namespace robotnav;
struct DelegateFixture {
 Fixture f;n::PlanResult real;arb1::Message context;std::vector<n::Wire> sent;
 FpgaDelegation d{[this](const n::Wire&w){sent.push_back(w);return true;}};
 uint64_t now=100000;
 DelegateFixture(){
  real=f.c.plan(f.goal(),f.pose(),350,50);
  arb1::Identity id;id.session=real.identity.session_id;id.run=real.identity.run_generation;id.goal=real.identity.goal_id;
  id.nav_epoch=real.nav_epoch;id.map_epoch=real.identity.map_epoch;id.map_revision=real.identity.map_revision;
  id.localization_epoch=real.identity.localization_epoch;id.plan_id=real.identity.plan_id;
  arb1::PlanResult p;p.identity=id;p.command_id=123;p.status=unsigned(real.status);p.cells=real.cells;
  arb1::FpgaState s;s.identity=id;s.pose_seq=1;s.state_seq=1;s.state=1;s.state_flags=32;s.pose_age_ms=0;
  arb1::ClockAnchor c;c.session=id.session;c.anchor_id=f.c.anchor();c.anchor_request_id=42;c.flags=1;
  c.device_ms=f.c.last_info().ack.device_ms;c.host_send_ns=c.host_receive_ns=now*1000000;
  arb1::Metadata meta;meta.received_mono_ns=now*1000000;meta.source_age_ns=0;
  context.blocks={{meta,p},{meta,s},{meta,c}};
 }
 bool setup(){d.phone("phone",1,now,true,now);return d.prepare("phone",10,true)&&d.context(context,now*1000000)&&d.claim(123,10,now);}
 n::Wire ack(){ne_message m{};assert(ne_decode(sent.back().data(),258,&m)==0);auto h=m.h;h.opcode|=0x80;h.payload_bytes=32;uint8_t p[160]{};ne_put64(p+8,real.nav_epoch);n::Wire w{};assert(ne_encode(&h,p,w.data())==0);return w;}
};
int main(int argc,char**argv){
 if(argc!=2)return 2;
 std::filesystem::create_directories(argv[1]);std::ofstream cases(std::filesystem::path(argv[1])/"cases.jsonl");unsigned count=0,failed=0;
 auto test=[&](const char*name,std::function<bool(DelegateFixture&)>fn){DelegateFixture f;bool ok=false;try{ok=fn(f);}catch(const std::exception&e){std::cerr<<e.what()<<'\n';}++count;if(!ok){++failed;std::cerr<<"FAIL "<<name<<'\n';}cases<<"{\"case\":\""<<name<<"\",\"pass\":"<<(ok?"true":"false")<<"}\n";};
 test("startup_no_motion_or_MOT_quiet",[](auto&f){f.d.tick(f.now);return f.sent.empty()&&!f.d.quiet();});
 test("preview_prepares_but_does_not_send_permit",[](auto&f){f.d.prepare("phone",10,true);f.d.context(f.context,f.now*1000000);f.d.tick(f.now);return f.d.quiet()&&f.sent.empty()&&!f.d.enabled();});
 test("explicit_claim_permit_accepted_by_frozen_firmware",[](auto&f){if(!f.setup())return false;f.d.tick(f.now);ne_message m{};return f.sent.size()==1&&ne_decode(f.sent[0].data(),258,&m)==0&&nav_exec_handle(&f.f.d.exec,&m,f.f.d.now,1000)==0;});
 test("only_real_matching_ack_mints_start_ticket",[](auto&f){if(!f.setup())return false;f.d.tick(f.now);if(f.d.ticket(f.now))return false;return f.d.ack(f.ack(),f.now)&&f.d.ticket(f.now).has_value();});
 test("no_auto_arm",[](auto&f){f.d.prepare("phone",10,true);f.d.context(f.context,f.now*1000000);return !f.d.claim(123,10,f.now)&&f.sent.empty();});
 test("wrong_plan_or_generation_rejected",[](auto&f){f.d.phone("phone",1,f.now,true,f.now);f.d.prepare("phone",10,true);f.d.context(f.context,f.now*1000000);return !f.d.claim(124,10,f.now)&&!f.d.claim(123,9,f.now);});
 test("state_identity_mismatch_rejected",[](auto&f){auto&s=std::get<arb1::FpgaState>(f.context.blocks[1].payload);s.identity.map_revision++;return !f.setup();});
 test("stale_context_not_reaged",[](auto&f){f.context.blocks[1].metadata.source_age_ns=251000000;return !f.d.context(f.context,f.now*1000000);});
 test("combined_pose_and_transport_age",[](auto&f){auto&s=std::get<arb1::FpgaState>(f.context.blocks[1].payload);s.pose_age_ms=200;f.context.blocks[1].metadata.source_age_ns=100000000;return !f.setup();});
 test("state_timeout_stops_permits_keeps_MOT_quiet",[](auto&f){if(!f.setup())return false;f.d.tick(f.now);f.d.tick(f.now+251);return !f.d.enabled()&&f.d.quiet()&&f.sent.size()==1;});
 test("duplicate_state_cannot_refresh_age",[](auto&f){if(!f.setup())return false;f.d.tick(f.now);for(auto&b:f.context.blocks)b.metadata.received_mono_ns=(f.now+240)*1000000;f.d.context(f.context,(f.now+240)*1000000);f.d.tick(f.now+251);return !f.d.enabled();});
 test("manual_takeover_invalidates_old_prepare_and_ack",[](auto&f){if(!f.setup())return false;f.d.tick(f.now);auto ack=f.ack();f.d.takeover();return !f.d.quiet()&&!f.d.ack(ack,f.now)&&!f.d.prepare("phone",10,true)&&!f.d.claim(123,10,f.now);});
 test("new_owner_cannot_inherit_navigation",[](auto&f){if(!f.setup())return false;f.d.phone("another",1,f.now,true,f.now);return !f.d.quiet()&&!f.d.enabled();});
 test("unconfirmed_anchor_rejected",[](auto&f){std::get<arb1::ClockAnchor>(f.context.blocks[2].payload).flags=0;return !f.setup();});
 test("ack_rejected_after_takeover_cannot_zero_manual",[](auto&f){if(!f.setup())return false;f.d.tick(f.now);auto ack=f.ack();f.d.takeover();f.d.context(f.context,f.now*1000000);return !f.d.ack(ack,f.now)&&!f.d.enabled()&&!f.d.quiet();});
 test("new_FPGA_session_immediately_stops_old_authority",[](auto&f){if(!f.setup())return false;f.d.tick(f.now);auto&s=std::get<arb1::FpgaState>(f.context.blocks[1].payload);s.identity.session++;s.state_seq++;f.d.context(f.context,f.now*1000000);f.d.tick(f.now+50);return !f.d.enabled()&&f.sent.size()==1;});
 test("changed_map_reference_immediately_stops_old_authority",[](auto&f){if(!f.setup())return false;f.d.tick(f.now);auto&s=std::get<arb1::FpgaState>(f.context.blocks[1].payload);s.identity.localization_epoch++;s.state_seq++;f.d.context(f.context,f.now*1000000);f.d.tick(f.now+50);return !f.d.enabled()&&f.sent.size()==1;});
 std::ofstream(std::filesystem::path(argv[1])/"summary.json")<<"{\"cases\":"<<count<<",\"failed\":"<<failed<<",\"firmware_modified\":false}\n";return failed?1:0;
}
