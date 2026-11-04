#include "stage3_plan_device.hpp"
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
 test("await_plan_notification_not_terminal",[](Fixture&f){f.d.interim_plan=true;auto p=f.c.plan(f.goal(),f.pose(),350,50);return p.status==n::Status::Ok&&p.cells.size()==4&&f.d.pages==1;});
 test("await_plan_then_real_cancel_stays_cancelled",[](Fixture&f){f.d.interim_plan=true;f.d.cancel_during_plan=true;auto p=f.c.plan(f.goal(),f.pose(),350,50);return p.status==n::Status::Cancelled&&p.cells.empty()&&!f.d.pages&&!f.d.exec.resume;});
 test("await_plan_without_final_times_out_no_path",[](Fixture&f){f.d.interim_plan=true;f.d.interim_only=true;try{f.c.plan(f.goal(),f.pose(),350,50);}catch(const n::Timeout&){return !f.d.pages&&!f.d.exec.resume;}return false;});
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
