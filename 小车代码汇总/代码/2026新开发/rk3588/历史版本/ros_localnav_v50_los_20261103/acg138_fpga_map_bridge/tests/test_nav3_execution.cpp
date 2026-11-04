#include "stage3_plan_device.hpp"
struct ExecutionDevice:Device {
 void emit_state(){auto h=event(NE_STATE,160);n::Payload p{};p[0]=exec.state;
  ne_put32(p.data()+4,(exec.owner?1u:0u)|(exec.plan_ready?2u:0u));
  ne_put64(p.data()+8,exec.nav_epoch);ne_put64(p.data()+16,exec.state_seq);
  ne_put32(p.data()+124,exec.owner);ne_put32(p.data()+64,exec.active.crc32);enqueue(h,p);}
 void send(const n::Wire&w)override{Device::send(w);if(w[5]==NE_CANCEL||w[5]==NE_START)emit_state();}
 int permit(const n::PlanResult&r,unsigned seq=1){
   ne_header h{};h.family=NE_NVA1;h.opcode=NE_PERMIT;h.payload_bytes=64;h.transport_seq=seq;h.request_id=seq;
   h.session_id=r.identity.session_id;h.run_generation=r.identity.run_generation;std::copy_n(r.identity.goal_id.begin(),16,h.goal_id);
   h.map_epoch=1;h.map_revision=r.identity.map_revision;h.localization_epoch=1;h.plan_id=r.identity.plan_id;
   n::Payload p{};ne_put32(p.data(),1);ne_put64(p.data()+4,r.nav_epoch);ne_put64(p.data()+12,1);
   ne_put64(p.data()+20,1);ne_put64(p.data()+28,seq);ne_put32(p.data()+36,now+500);ne_put32(p.data()+40,now+30000);ne_put64(p.data()+44,exec.guard.anchors[0].id);
   n::Wire w{};if(ne_encode(&h,p.data(),w.data()))throw std::runtime_error("permit encode");
   ne_message m{};if(ne_decode(w.data(),w.size(),&m))throw std::runtime_error("permit decode");
   return nav_exec_handle(&exec,&m,now,1000);
 }
};
struct F {ExecutionDevice d;n::Client c;F():c(d,10,{},true){c.handshake(123,1,"map");c.upload(n::Bitmap{},{0,0,1});}
 n::PlanResult plan(){n::PoseSample p;p.x_mm=50;p.y_mm=50;p.source_stamp_ns=1000000000+int64_t(d.poses)*20000000;std::array<uint8_t,16>g{};g[0]=1;return c.plan(g,p,350,50);}
 n::StartTicket ticket(const n::PlanResult&r){return {r.identity,r.nav_epoch,1,1,std::chrono::steady_clock::now()+std::chrono::milliseconds(100)};}
};
int main(int argc,char**argv){if(argc!=2)return 2;std::filesystem::create_directories(argv[1]);std::ofstream cases(std::filesystem::path(argv[1])/"cases.jsonl");unsigned total=0,failed=0;
 const auto test=[&](const char*id,auto fn){F f;bool ok=false;std::string why;try{ok=fn(f);}catch(const std::exception&e){why=e.what();}++total;if(!ok){++failed;std::cerr<<id<<": "<<why<<'\n';}
 cases<<"{\"case\":\""<<id<<"\",\"pass\":"<<(ok?"true":"false")<<"}\n";
 std::ofstream frames(std::filesystem::path(argv[1])/(std::string(id)+".jsonl"));for(auto&s:f.d.trace)frames<<"{\"wire_hex\":\""<<s<<"\"}\n";};
 test("goal_plan_never_motion",[](F&f){auto r=f.plan();return r.cells.size()==4&&!f.d.exec.owner&&!f.d.exec.resume;});
 test("start_requires_real_fpga_permit",[](F&f){auto r=f.plan();try{f.c.start(f.ticket(r));}catch(const n::Rejected&){return !f.d.exec.owner;}return false;});
 test("matching_start_then_cancel",[](F&f){auto r=f.plan();if(f.d.permit(r))return false;f.c.start(f.ticket(r));if(f.d.exec.state!=NV_FOLLOWING)return false;f.c.cancel(r.identity);f.c.pump();return f.d.exec.state==NV_CANCELLED&&!f.d.exec.owner&&f.c.stopped_for_map();});
 test("late_start_after_cancel",[](F&f){auto r=f.plan();auto t=f.ticket(r);f.c.cancel(r.identity);auto sent=f.c.statistics().sent;try{f.c.start(t);}catch(const n::Rejected&){return sent==f.c.statistics().sent;}return false;});
 test("expired_start_ticket",[](F&f){auto r=f.plan();auto t=f.ticket(r);t.expires_at=std::chrono::steady_clock::now();auto sent=f.c.statistics().sent;try{f.c.start(t);}catch(const n::Rejected&){return sent==f.c.statistics().sent;}return false;});
 test("ticket_longer_than_500ms_rejected",[](F&f){auto r=f.plan();auto t=f.ticket(r);t.expires_at+=std::chrono::seconds(1);try{f.c.start(t);}catch(const n::Rejected&){return !f.d.exec.owner;}return false;});
 test("each_start_identity_field_rejected",[](F&f){auto r=f.plan();auto sent=f.c.statistics().sent;for(unsigned i=0;i<10;++i){auto t=f.ticket(r);switch(i){case 0:++t.identity.session_id;break;case 1:++t.identity.run_generation;break;case 2:++t.identity.goal_id[0];break;case 3:++t.identity.map_epoch;break;case 4:++t.identity.map_revision;break;case 5:++t.identity.localization_epoch;break;case 6:++t.identity.plan_id;break;case 7:++t.nav_epoch;break;case 8:t.permit_seq=0;break;case 9:t.auth_epoch=0;}
 try{f.c.start(t);return false;}catch(const n::Rejected&){} }return sent==f.c.statistics().sent;});
 test("moving_map_rejected_without_begin",[](F&f){auto r=f.plan();if(f.d.permit(r))return false;f.c.start(f.ticket(r));f.c.pump();auto begins=f.d.begins;n::Bitmap b{};b[100]=1;try{f.c.upload(b,{0,0,1});}catch(const n::Rejected&e){return e.status==n::Status::Busy&&f.d.begins==begins;}return false;});
 test("cancel_then_replace_map_then_new_goal_no_autostart",[](F&f){auto r=f.plan();f.c.cancel(r.identity);f.c.pump();n::Bitmap b{};b[100]=1;auto map=f.c.upload(b,{0,0,1});auto p=f.plan();return map.revision==2&&map.state_verified&&p.cells.size()==4&&!f.d.exec.owner&&p.identity.run_generation>r.identity.run_generation;});
 test("old_cancel_cannot_cancel_new_goal",[](F&f){auto r=f.plan();f.c.cancel(r.identity);f.c.pump();auto p=f.plan();auto sent=f.c.statistics().sent;try{f.c.cancel(r.identity);}catch(const n::Rejected&){return sent==f.c.statistics().sent&&f.d.exec.state==NV_PLANNING&&p.identity.run_generation>r.identity.run_generation;}return false;});
 test("wrong_session_cancel_rejected_locally",[](F&f){auto r=f.plan();++r.identity.session_id;auto sent=f.c.statistics().sent;try{f.c.cancel(r.identity);}catch(const n::Rejected&){return sent==f.c.statistics().sent;}return false;});
 test("default_line_A_still_cannot_start",[](F&f){n::Client c(f.d,10);try{c.start({});}catch(const std::logic_error&){return !f.d.exec.owner;}return false;});
 test("async_replan_fetches_new_same_goal_path_without_START",[](F&f){
  auto r=f.plan();if(f.d.permit(r))return false;f.c.start(f.ticket(r));f.c.pump();
  n::PoseSample pose;pose.x_mm=150;pose.y_mm=50;pose.source_stamp_ns=2000000000;f.c.stream_pose(pose);f.c.pump();
  nav_exec_begin_avoid(&f.d.exec);nav_exec_set_avoid_phase(&f.d.exec,NV_ADVANCE);
  if(nav_exec_finish_avoid(&f.d.exec,f.d.now)||nav_exec_plan(&f.d.exec,f.d.now))return false;
  f.d.plan_event();f.d.emit_state();f.c.pump();auto updated=f.c.poll_replan();
  return updated&&updated->identity.plan_id==r.identity.plan_id+1&&updated->cells.front()==1&&updated->cells.back()==3&&updated->cells.size()==3&&
   f.c.execution_plan()->identity.plan_id==updated->identity.plan_id&&std::count(f.d.ops.begin(),f.d.ops.end(),NE_START)==1;
 });
 test("cancel_during_replan_does_not_publish_or_restart",[](F&f){
  auto r=f.plan();if(f.d.permit(r))return false;f.c.start(f.ticket(r));f.c.pump();
  nav_exec_begin_avoid(&f.d.exec);nav_exec_set_avoid_phase(&f.d.exec,NV_ADVANCE);
  if(nav_exec_finish_avoid(&f.d.exec,f.d.now)||nav_exec_plan(&f.d.exec,f.d.now))return false;
  f.d.plan_event();f.d.emit_state();f.c.pump();f.c.cancel(r.identity);f.c.pump();
  return !f.c.poll_replan()&&!f.c.execution_plan()&&!f.d.exec.resume;
 });
 test("wrong_replan_page_not_published",[](F&f){
  auto r=f.plan();if(f.d.permit(r))return false;f.c.start(f.ticket(r));f.c.pump();
  nav_exec_begin_avoid(&f.d.exec);nav_exec_set_avoid_phase(&f.d.exec,NV_ADVANCE);
  if(nav_exec_finish_avoid(&f.d.exec,f.d.now)||nav_exec_plan(&f.d.exec,f.d.now))return false;
  f.d.plan_event();f.d.emit_state();f.c.pump();f.d.wrong_page_plan=true;
  try{f.c.poll_replan();}catch(const n::Timeout&){return f.c.execution_plan()->identity.plan_id==r.identity.plan_id;}return false;
 });
 std::ofstream summary(std::filesystem::path(argv[1])/"summary.json");summary<<"{\"cases\":"<<total<<",\"failed\":"<<failed<<",\"oracle\":\"unchanged candidate26 nav_exec + guard + A*\"}\n";return failed?1:0;
}
