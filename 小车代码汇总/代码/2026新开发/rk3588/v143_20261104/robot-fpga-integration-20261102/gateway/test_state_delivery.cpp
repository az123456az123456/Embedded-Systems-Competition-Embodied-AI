// Scheduled-delay replay through the unchanged production authority/delegation.
// Only the transport clock/ACK are synthetic; no socket, ROS or motor is used.
#define main existing_delegation_test_main
#include "test_delegation.cpp"
#undef main
#include <iostream>

struct Result {bool enabled=false;uint64_t stopped=0,sends=0,max_age=0;std::string reason;};
static Result replay(bool immediate,unsigned phase,unsigned forwarding,unsigned duration,std::ostream* trace=nullptr,bool cut_source=false){
 DelegateFixture f;auto&s=std::get<arb1::FpgaState>(f.context.blocks[1].payload);s.pose_age_ms=32;
 if(!f.setup())throw std::runtime_error("fixture claim");
 f.d.tick(f.now);if(!f.d.ack(f.ack(),f.now)||!f.d.started(10,f.d.ticket(f.now)->permit_seq))throw std::runtime_error("fixture START");
 auto wire=f.context;auto ros=f.context;uint64_t ros_source=0,delivered_sample_ns=f.now*1000000,delivered_seq=1;Result result;
 for(unsigned t=1;t<=duration;++t){
  const auto now=f.now+t;
  if(t%100==phase&&(!cut_source||t<500)){
   wire=f.context;auto&v=std::get<arb1::FpgaState>(wire.blocks[1].payload);
   v.pose_seq=2+t/100;v.state_seq=2;v.state=2;v.state_flags=226;v.pose_age_ms=32;
   for(auto&b:wire.blocks)b.metadata.received_mono_ns=now*1000000;
   // A real independently verified anchor is supplied every 400 ms in production.
   auto&c=std::get<arb1::ClockAnchor>(wire.blocks[2].payload);
   c.anchor_id=1+t/400;c.host_send_ns=c.host_receive_ns=(f.now+(t/400)*400)*1000000;
   c.device_ms=1000+(t/400)*400;
  }
  if((immediate&&t%100==(phase+2)%100)||(!immediate&&t%100==99)){
   ros=wire;ros_source=std::get<arb1::FpgaState>(wire.blocks[1].payload).pose_seq;
   // Re-publishing does not change sample time: actual body capture remains old.
   for(auto&b:ros.blocks){const auto original=b.metadata.received_mono_ns;b.metadata.received_mono_ns=now*1000000;b.metadata.source_age_ns=now*1000000-original;}
  }
  if(t%forwarding==forwarding-2){
   f.d.context(ros,now*1000000);
   auto&b=ros.blocks[1];const auto seq=std::get<arb1::FpgaState>(b.payload).pose_seq;
   if(seq>delivered_seq){delivered_seq=seq;delivered_sample_ns=b.metadata.received_mono_ns-b.metadata.source_age_ns;}
  }
  f.d.phone("phone",1+t,now,true,now);
  const auto age=(now*1000000-delivered_sample_ns)/1000000+32;result.max_age=std::max(result.max_age,age);
  const auto n=f.sent.size();f.d.tick(now);
  if(f.sent.size()>n)f.d.ack(f.ack(),now);
  if(trace&&(f.sent.size()>n||!f.d.enabled()))*trace<<"{\"t_ms\":"<<t<<",\"ros_pose_seq\":"<<ros_source<<",\"enabled\":"<<(f.d.enabled()?"true":"false")<<",\"sends\":"<<f.sent.size()<<",\"reason\":\""<<f.d.reason()<<"\"}\n";
  if(!f.d.enabled()){result.stopped=t;break;}
 }
 result.enabled=f.d.enabled();result.sends=f.sent.size();result.reason=f.d.reason();return result;
}
int main(int argc,char**argv){
 if(argc!=2)return 2;std::filesystem::create_directories(argv[1]);auto dir=std::filesystem::path(argv[1]);
 std::ofstream before(dir/"legacy-minimal.jsonl"),after(dir/"fixed-trace.jsonl"),cases(dir/"cases.jsonl");unsigned count=0,failed=0;
 auto record=[&](const char*name,const Result&r,bool pass){++count;if(!pass)++failed;cases<<"{\"case\":\""<<name<<"\",\"pass\":"<<(pass?"true":"false")<<",\"authority_enabled\":"<<(r.enabled?"true":"false")<<",\"stop_ms\":"<<r.stopped<<",\"permits\":"<<r.sends<<",\"max_forwarded_pose_age_ms\":"<<r.max_age<<",\"reason\":\""<<r.reason<<"\"}\n";};
 const auto old=replay(false,0,50,5000,&before);record("legacy_100ms_50ms_false_stale_reproduced",old,!old.enabled&&old.reason=="FPGA_state_stale_no_permit_renewal");
 const auto fixed=replay(true,0,20,5000,&after);record("immediate_STATE_20ms_forwarding_5s_renewal",fixed,fixed.enabled&&fixed.sends>=95);
 for(unsigned phase=0;phase<100;++phase){const auto r=replay(true,phase,20,5000);record(("phase_"+std::to_string(phase)).c_str(),r,r.enabled&&r.sends>=95);}
 const auto lost=replay(true,0,20,5000,nullptr,true);record("actual_missing_STATE_still_revokes_without_recovery",lost,!lost.enabled&&lost.stopped<=650);
 std::ofstream(dir/"summary.json")<<"{\"cases\":"<<count<<",\"failed\":"<<failed<<",\"production_admission_unchanged\":true,\"transport_and_clock\":\"scheduled model, not hardware measurement\",\"motors_started\":false}\n";
 return failed?1:0;
}
