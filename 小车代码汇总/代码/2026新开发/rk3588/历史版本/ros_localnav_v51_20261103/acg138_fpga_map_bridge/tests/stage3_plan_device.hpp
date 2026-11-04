#pragma once
#include "stage3_device.hpp"
#include <sstream>
struct Device:MapDevice {
  bool wrong_result_run=false,wrong_result_pose=false,wrong_page_plan=false,bad_path=false;
  bool result_before_ack=false,drop_pose=false;
  bool interim_plan=false,interim_only=false,cancel_during_plan=false;
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
      ++goals;
      if(!rc&&interim_plan)plan_event(); // actual unmodified firmware AWAIT_PLAN fields
      if(!rc&&interim_only){enqueue(h,p);return;}
      if(!rc&&cancel_during_plan){exec.state=NV_CANCELLED;exec.reason=NV_R_CANCEL;}
      else if(!rc)nav_exec_plan(&exec,now);
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
