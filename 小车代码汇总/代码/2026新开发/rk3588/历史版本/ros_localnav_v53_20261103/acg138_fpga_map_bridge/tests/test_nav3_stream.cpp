#include "stage3_plan_device.hpp"
#include "acg138_fpga_map_bridge/nav3_pose_schedule.hpp"
int main(int argc,char**argv){
 if(argc!=2)return 2;
 std::filesystem::create_directories(argv[1]);std::ofstream cases(std::filesystem::path(argv[1])/"cases.jsonl");unsigned total=0,failed=0;
 const auto test=[&](const char* id,auto fn){++total;Fixture f;bool ok=false;std::string why;
  try{ok=fn(f);}catch(const std::exception&e){why=e.what();}if(!ok){++failed;std::cerr<<id<<": "<<why<<'\n';}
  cases<<"{\"case\":\""<<id<<"\",\"pass\":"<<(ok?"true":"false")<<"}\n";
  std::ofstream trace(std::filesystem::path(argv[1])/(std::string(id)+".frames.jsonl"));unsigned i=0;
  for(const auto&w:f.d.trace)trace<<"{\"sequence\":"<<++i<<",\"wire_hex\":\""<<w<<"\"}\n";
 };
 test("pose_stream_independent_sequence_20_samples",[](Fixture&f){for(unsigned i=0;i<20;++i){auto p=f.pose();p.source_stamp_ns+=i*20000000LL;if(!f.c.stream_pose(p))return false;f.c.pump();}auto v=f.c.state();return f.d.poses==20&&v&&v->pose_seq==20&&!f.d.exec.owner&&!f.d.exec.resume;});
 test("duplicate_tf_no_transport_or_renewal",[](Fixture&f){auto p=f.pose();f.c.stream_pose(p);const auto tx=f.c.statistics().sent;const auto seq=f.d.exec.guard.pose_seq;return !f.c.stream_pose(p)&&tx==f.c.statistics().sent&&seq==f.d.exec.guard.pose_seq;});
 test("STATE_delivery_trigger_uses_transport_not_state_change",[](Fixture&f){
  f.c.stream_pose(f.pose());f.c.pump();const auto old=f.c.state();const auto seq=f.c.state_transport_sequence();
  if(!old||!seq)return false;
  auto h=f.d.event(NE_STATE,160);n::Payload p{};p[0]=old->state;ne_put64(p.data()+16,old->state_seq);ne_put64(p.data()+24,old->pose_seq);
  f.d.enqueue(h,p);f.c.pump();const auto fresh=f.c.state_transport_sequence();
  if(fresh<=seq||f.c.state()->state_seq!=old->state_seq)return false;
  f.d.enqueue(h,p);h.session_id=999;h.transport_seq++;f.d.enqueue(h,p);f.c.pump();
  return f.c.state_transport_sequence()==fresh;
 });
 test("old_tf_rejected_before_send",[](Fixture&f){auto p=f.pose();p.source_age_ns=250000001;try{f.c.stream_pose(p);}catch(const n::Rejected&e){return e.status==n::Status::PoseStale&&!f.d.poses;}return false;});
 test("queued_tf_age_not_refreshed",[](Fixture&f){auto p=f.pose();p.source_age_ns=230000000;p.sampled_at-=std::chrono::milliseconds(25);try{f.c.stream_pose(p);}catch(const n::Rejected&){return !f.d.poses;}return false;});
 test("time_reverse_rejected",[](Fixture&f){auto p=f.pose();f.c.stream_pose(p);p.source_stamp_ns--;try{f.c.stream_pose(p);}catch(const n::Rejected&){return f.d.poses==1;}return false;});
 test("outside_window_no_pose",[](Fixture&f){auto p=f.pose();p.x_mm=6400;try{f.c.stream_pose(p);}catch(const n::Rejected&e){return e.status==n::Status::GoalOutsideWindow&&!f.d.poses;}return false;});
 test("uncommitted_map_no_pose",[](Fixture&f){f.c.info();f.c.session_bind(456);return !f.c.stream_pose(f.pose())&&!f.d.poses;});
 test("state_signed_coordinates_and_fields",[](Fixture&){n::Message m;m.header.opcode=0x40;m.header.payload_bytes=160;m.header.transport_seq=1;m.header.session_id=123;m.payload[0]=3;m.payload[1]=2;ne_put32(m.payload.data()+32,uint32_t(-123));ne_put32(m.payload.data()+36,456);ne_put32(m.payload.data()+40,uint32_t(-999));ne_put16(m.payload.data()+48,4);ne_put16(m.payload.data()+50,9);ne_put32(m.payload.data()+52,370);ne_put32(m.payload.data()+56,400);ne_put32(m.payload.data()+60,29000);ne_put32(m.payload.data()+124,1);ne_put32(m.payload.data()+144,1);n::StateView v;return n::parse_state(m,v)==n::Error::Ok&&v.state==3&&v.avoid_stage==2&&v.x_mm==-123&&v.y_mm==456&&v.yaw_urad==-999&&v.cursor==4&&v.path_count==9&&v.remaining_mm==370&&v.permit_ms==400&&v.authorization_ms==29000&&v.owner==1&&v.frame_token==1;});
 test("wrong_session_and_old_state_ignored",[](Fixture&f){f.c.stream_pose(f.pose());f.c.pump();const auto v=f.c.state();auto h=f.d.event(NE_STATE,160);h.session_id=999;n::Payload p{};ne_put64(p.data()+24,999);f.d.enqueue(h,p);h.session_id=123;h.transport_seq=1;f.d.enqueue(h,p);f.c.pump();return v&&f.c.state()->pose_seq==v->pose_seq;});
 test("stream_hook_preserves_exact_goal_pose_and_page_ack",[](Fixture&f){unsigned calls=0;f.c.set_service([&]{auto p=f.pose();p.source_stamp_ns+=(++calls)*20000000LL;f.c.stream_pose(p);});const auto r=f.c.plan(f.goal(),f.pose(),6350,6350);const auto g=std::find(f.d.ops.begin(),f.d.ops.end(),NE_GOAL_PLAN);return r.cells.size()==127&&calls>0&&std::count(f.d.ops.begin(),g,NE_POSE)==1&&f.d.pages==2&&!f.d.exec.owner;});
 test("refresh_anchor_not_session_rebind",[](Fixture&f){auto p=f.pose();f.c.stream_pose(p);std::this_thread::sleep_for(std::chrono::milliseconds(410));f.d.now+=410;f.c.refresh_anchor();p.source_stamp_ns+=420000000;p.sampled_at=std::chrono::steady_clock::now();return f.c.stream_pose(p)&&f.c.anchor()==2&&f.c.session()==123&&f.d.poses==2;});
 test("no_start_or_permit_api_used",[](Fixture&f){f.c.stream_pose(f.pose());f.c.pump();return std::find(f.d.ops.begin(),f.d.ops.end(),NE_START)==f.d.ops.end()&&!f.d.exec.owner&&!f.d.exec.intent.linear_mmps&&!f.d.exec.intent.angular_mradps;});
 test("schedule_slow_send_no_extra_period",[](Fixture&){using namespace std::chrono;const steady_clock::time_point t{};return n::next_pose_slot(t,t+milliseconds(2))==t+milliseconds(20)&&n::next_pose_slot(t+milliseconds(20),t+milliseconds(35))==t+milliseconds(40);});
 test("schedule_skips_backlog_no_old_samples",[](Fixture&){using namespace std::chrono;const steady_clock::time_point t{};return n::next_pose_slot(t,t+milliseconds(105))==t+milliseconds(120)&&n::next_pose_slot(t,t+milliseconds(120))==t+milliseconds(140)&&n::next_pose_slot(t+milliseconds(20),t)==t+milliseconds(20);});
 std::ofstream out(std::filesystem::path(argv[1])/"summary.json");out<<"{\"stage\":4,\"cases\":"<<total<<",\"failed\":"<<failed<<",\"pass\":"<<(failed?"false":"true")<<",\"oracle\":\"candidate26 frozen nav_exec/nx_guard, synthetic I/O only\"}\n";
 std::cout<<"stage4 cases="<<total<<" failed="<<failed<<'\n';return failed?1:0;
}
