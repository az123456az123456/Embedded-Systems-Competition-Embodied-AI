#include "stage3_device.hpp"
#include <sstream>
struct Device:MapDevice {
  bool wrong_result_run=false,wrong_result_pose=false,wrong_page_plan=false,bad_path=false;
  bool result_before_ack=false,drop_pose=false;
  unsigned pages=0,goals=0,poses=0;std::vector<std::string> trace;
  void enqueue(ne_header h,n::Payload p){n::Wire w;if(ne_encode(&h,p.data(),w.data()))throw std::runtime_error("oracle encode");rx.push_back(w);}
  ne_header event(unsigned op,unsigned bytes){
    ne_header h{};h.opcode=op;h.payload_bytes=bytes;h.session_id=exec.guard.session;h.transport_seq=++event_seq;
    h.run_generation=exec.task.run;std::copy_n(exec.task.goal,16,h.goal_id);h.map_epoch=exec.active.epoch;h.map_revision=exec.active.revision;
    h.localization_epoch=exec.task.localization_epoch;h.plan_id=exec.task.plan;return h;
  }
  void state(){
    auto h=event(NE_STATE,160);n::Payload p{};p[0]=exec.state;ne_put32(p.data()+4,nx_pose_valid(&exec.guard,now)?32:0);
    ne_put64(p.data()+24,exec.guard.pose_seq);ne_put32(p.data()+44,exec.pose.source_age_ms);ne_put32(p.data()+64,exec.active.crc32);enqueue(h,p);
  }
  void plan_event(){
    auto h=event(NE_PLAN_RESULT,64);n::Payload p{};
    const auto status=exec.reason==NV_R_NO_ROUTE?NE_NO_ROUTE:exec.plan_ready?NE_OK:NE_CANCELLED;
    ne_put16(p.data(),status);ne_put16(p.data()+2,exec.reason);ne_put32(p.data()+4,now);
    ne_put64(p.data()+8,exec.nav_epoch);ne_put64(p.data()+16,exec.state_seq);ne_put64(p.data()+24,exec.start_pose_seq+(wrong_result_pose?1:0));
    ne_put32(p.data()+32,server.planner.expanded);ne_put16(p.data()+40,exec.plan_ready?exec.path_count:0);
    ne_put16(p.data()+42,exec.start_cell);ne_put16(p.data()+44,exec.goal_cell);ne_put16(p.data()+46,exec.plan_ready?1:0);
    ne_put32(p.data()+48,exec.planned_revision);ne_put32(p.data()+52,exec.active.frame_token);ne_put32(p.data()+56,50000000);
    if(wrong_result_run)++h.run_generation;
    enqueue(h,p);
  }
  void send(const n::Wire& w)override{
    static const char hex[]="0123456789abcdef";std::string s;
    for(auto b:w){s+=hex[b>>4];s+=hex[b&15];}trace.push_back(s);
    ne_message m{};if(ne_decode(w.data(),w.size(),&m))throw std::runtime_error("decode");
    if(m.h.opcode!=NE_POSE && m.h.opcode!=NE_GOAL_PLAN && m.h.opcode!=NE_PATH_PAGE){MapDevice::send(w);return;}
    ops.push_back(m.h.opcode);++now;
    if(m.h.opcode==NE_POSE){++poses;if(drop_pose)return;const auto rc=nav_exec_handle(&exec,&m,now,10000);if(rc)throw std::runtime_error("real POSE rejected "+std::to_string(rc));state();return;}
    const auto rc=nav_exec_handle(&exec,&m,now,10000);auto h=m.h;h.opcode|=0x80;h.payload_bytes=32;n::Payload p{};
    ne_put16(p.data(),rc);ne_put32(p.data()+4,now);ne_put64(p.data()+8,exec.nav_epoch);ne_put64(p.data()+16,exec.state_seq);
    if(m.h.opcode==NE_GOAL_PLAN){
      ++goals;if(!rc)nav_exec_plan(&exec,now);
      if(!rc&&result_before_ack)plan_event();
      enqueue(h,p);if(!rc&&!result_before_ack)plan_event();return;
    }
    ++pages;
    if(!rc){std::uint16_t cells[64];unsigned count=0;const int page_rc=nav_exec_page(&exec,&m,cells,64,&count);
      ne_put16(p.data(),page_rc);if(!page_rc){ne_put16(p.data()+24,ne_get16(m.payload));ne_put16(p.data()+26,count);h.payload_bytes=32+2*count;
       for(unsigned i=0;i<count;++i)ne_put16(p.data()+32+2*i,(bad_path&&i==1)?4095:cells[i]);}}
    if(wrong_page_plan)++h.plan_id;
    enqueue(h,p);
  }
};
struct Fixture {
 Device d;n::Client c;Fixture():c(d,5){c.handshake(123,1,"map");c.upload(n::Bitmap{},{0,0,1});}
 n::PoseSample pose(){n::PoseSample p;p.x_mm=50;p.y_mm=50;p.source_stamp_ns=1000000000;return p;}
 std::array<std::uint8_t,16> goal(){std::array<std::uint8_t,16> a{};a[0]=1;return a;}
};
int main(int argc,char**argv){
 if(argc!=2)return 2;
 std::filesystem::create_directories(argv[1]);std::ofstream cases(std::filesystem::path(argv[1])/"cases.jsonl");
 unsigned total=0,failed=0;
 const auto test=[&](const char* id,auto fn){++total;Fixture f;bool ok=false;std::string why;
  try{ok=fn(f);}catch(const std::exception&e){why=e.what();}if(!ok){++failed;std::cerr<<id<<": "<<why<<'\n';}
  cases<<"{\"case\":\""<<id<<"\",\"pass\":"<<(ok?"true":"false")<<"}\n";
  std::ofstream trace(std::filesystem::path(argv[1])/(std::string(id)+".frames.jsonl"));unsigned i=0;
  for(const auto&w:f.d.trace)trace<<"{\"sequence\":"<<++i<<",\"wire_hex\":\""<<w<<"\"}\n";
 };
 test("real_astar_127_cells_two_pages",[](Fixture&f){auto r=f.c.plan(f.goal(),f.pose(),6350,6350);return r.status==n::Status::Ok&&r.cells.size()==127&&f.d.pages==2&&r.cells.front()==0&&r.cells.back()==4095;});
 test("plan_before_ack_preserved",[](Fixture&f){f.d.result_before_ack=true;return f.c.plan(f.goal(),f.pose(),350,50).cells.size()==4;});
 test("no_start_no_drive",[](Fixture&f){f.c.plan(f.goal(),f.pose(),350,50);return !f.d.exec.owner&&!f.d.exec.resume&&!f.d.exec.intent.linear_mmps&&!f.d.exec.intent.angular_mradps&&std::find(f.d.ops.begin(),f.d.ops.end(),NE_START)==f.d.ops.end();});
 test("no_route_explicit_empty",[](Fixture&f){n::Bitmap b{};b[0]=1;f.c.upload(b,{0,0,1});auto r=f.c.plan(f.goal(),f.pose(),350,50);return r.status==n::Status::NoRoute&&r.cells.empty()&&f.d.pages==0;});
 test("wrong_result_run_rejected",[](Fixture&f){f.d.wrong_result_run=true;try{f.c.plan(f.goal(),f.pose(),350,50);}catch(const n::Timeout&){return f.d.pages==0;}return false;});
 test("wrong_result_pose_rejected",[](Fixture&f){f.d.wrong_result_pose=true;try{f.c.plan(f.goal(),f.pose(),350,50);}catch(const std::runtime_error&){return f.d.pages==0;}return false;});
 test("wrong_page_plan_rejected",[](Fixture&f){f.d.wrong_page_plan=true;try{f.c.plan(f.goal(),f.pose(),350,50);}catch(const n::Timeout&){return true;}return false;});
 test("nonadjacent_page_rejected",[](Fixture&f){f.d.bad_path=true;try{f.c.plan(f.goal(),f.pose(),350,50);}catch(const std::runtime_error&){return true;}return false;});
 test("old_pose_no_send",[](Fixture&f){auto p=f.pose();p.source_age_ns=250000001;try{f.c.plan(f.goal(),p,350,50);}catch(const n::Rejected&e){return e.status==n::Status::PoseStale&&!f.d.poses&&!f.d.goals;}return false;});
 test("repeated_tf_no_renewal",[](Fixture&f){auto p=f.pose();f.c.plan(f.goal(),p,350,50);try{f.c.plan(f.goal(),p,350,50);}catch(const n::Rejected&e){return e.status==n::Status::PoseStale&&f.d.poses==1&&f.d.goals==1;}return false;});
 test("unconfirmed_pose_no_goal",[](Fixture&f){f.d.drop_pose=true;try{f.c.plan(f.goal(),f.pose(),350,50);}catch(const n::Timeout&){return !f.d.goals;}return false;});
 test("goal_outside_no_send",[](Fixture&f){const auto count=f.c.statistics().sent;try{f.c.plan(f.goal(),f.pose(),6400,50);}catch(const n::Rejected&e){return e.status==n::Status::GoalOutsideWindow&&count==f.c.statistics().sent;}return false;});
 test("pose_outside_no_send",[](Fixture&f){auto p=f.pose();p.x_mm=-1;try{f.c.plan(f.goal(),p,350,50);}catch(const n::Rejected&e){return e.status==n::Status::GoalOutsideWindow&&!f.d.poses&&!f.d.goals;}return false;});
 test("source_time_rollback_rejected",[](Fixture&f){auto p=f.pose();f.c.plan(f.goal(),p,350,50);p.source_stamp_ns--;try{f.c.plan(f.goal(),p,350,50);}catch(const n::Rejected&){return f.d.poses==1;}return false;});
 std::ofstream out(std::filesystem::path(argv[1])/"summary.json");out<<"{\"stage\":3,\"cases\":"<<total<<",\"failed\":"<<failed<<",\"pass\":"<<(failed?"false":"true")<<",\"oracle\":\"candidate26 nav_exec + nx_guard + astar unchanged\"}\n";
 std::cout<<"stage3 cases="<<total<<" failed="<<failed<<'\n';return failed?1:0;
}
