// Production Navigator; every odometry update follows the PREVIOUS emitted
// physical command. No motor, network, map or sensor observation is fabricated.
#include "../navcore.h"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace robot_nav;
namespace {
unsigned checks=0;
void ck(bool b,const char* why){++checks;if(!b)throw std::runtime_error(why);}
Config cfg(){Config c;c.velocity_control=true;c.reaction_s=.12;c.sonar_latency_s=.10;return c;}
struct Rig {
 Config c=cfg();Navigator nav{c};Control out;double now=10,yaw=0;
 std::array<double,4> counts{};std::array<double,3> distances{4.5,4.5,4.5};
 std::array<uint32_t,3> ids{};std::array<double,3> source_time{};
 unsigned invalid=0,freeze=0;double host_receipt_delay=0;
 Rig(){for(unsigned i=0;i<80;++i)step(false);}
 void step(bool tick=true,bool live=false){
  constexpr double dt=.02;now+=dt;
  const double l=out.linear_mps-out.angular_rps*c.track_m*.5,r=out.linear_mps+out.angular_rps*c.track_m*.5;
  for(unsigned w=0;w<4;++w)counts[w]+=((c.left_wheel_mask&(1u<<w))?l:r)*dt*c.encoder_ticks_per_turn/c.wheel_circumference_m*c.encoder_signs[w];
  yaw+=out.angular_rps*dt;
  SensorFrame f;f.timestamp_s=now-host_receipt_delay;f.device_timestamp_s=now;f.sequence=uint32_t(std::llround(now*1000));
  f.imu_valid=true;f.yaw_rad=wrap(yaw);f.sonar_observation_ids_valid=true;
  for(unsigned w=0;w<4;++w)f.counts[w]=uint32_t(int64_t(std::llround(counts[w])));
  for(unsigned ch=0;ch<3;++ch){
   if(!(freeze&(1u<<ch))){ids[ch]=f.sequence;source_time[ch]=f.timestamp_s;}
   if(!(invalid&(1u<<ch)))f.sonar_m[ch]=distances[ch];
   f.sonar_observation_id[ch]=ids[ch];f.sonar_age_s[ch]=f.timestamp_s-source_time[ch];
  }
  if(live){LiveSonarSample s;s.timestamp_s=f.timestamp_s;s.sequence=f.sequence;s.metres=f.sonar_m;s.age_s=f.sonar_age_s;s.observation_id=f.sonar_observation_id;
   nav.observe_latest_sonar(s,now+.000001);out=nav.command();}
  const bool seeded=nav.frame().has_value();const bool accepted=nav.ingest(f,now);ck(!seeded||accepted,"fixture keeps real encoder/IMU continuity");
  if(tick)out=nav.tick(now+.000001);
 }
 void start(){ck(nav.start_straight(now),"fresh source starts");out=nav.command();for(unsigned i=0;i<110;++i)step();ck(out.linear_mps>.29,"reaches .3m/s");}
};
void grace_boundary(bool live,bool invalid){
 Rig r;r.start();const Control old=r.out;const double source=r.source_time[0];r.freeze=1;if(invalid)r.invalid=1;
 bool stopped=false;unsigned preserved=0;
 for(unsigned i=0;i<80;++i){r.step(true,live);double age=r.now+.000001-source;
  ck(r.nav.enabled(),"source-only fault never drops navigation authorization");
  if(age<=1.){ck(r.out.linear_mps>0,"source gap through one second continues straight");ck(r.out.linear_mps<=old.linear_mps+1e-9,"grace never ramps target above preceding command");++preserved;}
  else{ck(r.out.linear_mps==0&&r.out.angular_rps==0,"strictly more than one second stops");stopped=true;}
 }
 ck(stopped&&preserved>40,"both sides of one-second boundary exercised");
 for(unsigned i=0;i<300;++i){r.step(true,live);ck(r.nav.enabled()&&r.out.linear_mps==0&&r.out.angular_rps==0,"long missing sonar stays authorized at zero");}
 r.freeze=r.invalid=0;bool recovered=false;for(unsigned i=0;i<90;++i){r.step(true,live);ck(r.nav.enabled(),"fresh range recovery retains session");recovered=recovered||r.out.linear_mps>0;}
 ck(recovered,"new genuine sonar plus measured stopped proof resumes same intent");
}
void partial_near_preempts_grace(){
 for(bool live:{false,true})for(unsigned near:{0u,1u,2u}){
  Rig r;r.start();const unsigned bad=near?0:1;r.invalid=1u<<bad;r.freeze=1u<<bad;
  for(unsigned i=0;i<10;++i)r.step(true,live);
  ck(r.out.linear_mps>0,"short partial gap does not stop before real obstacle");
  r.distances[near]=.15;r.step(true,live);
  ck(r.out.linear_mps==0&&r.out.angular_rps==0,"new genuine near echo wins over unrelated missing channel");
 }
}
void source_id_and_start_boundaries(){
 Rig r;r.start();r.freeze=7;const double origin=r.source_time[0];
 for(unsigned i=0;i<55;++i)r.step(true,true);
 ck(r.now-origin>1&&r.out.linear_mps==0,"new SNS1 sequences never rejuvenate repeated source IDs");
 Rig idle;idle.invalid=1;idle.freeze=1;idle.step(false);ck(idle.nav.start_straight(idle.now),"missing source accepts only zero waiting intent");idle.out=idle.nav.command();idle.step();
 ck(idle.out.linear_mps==0&&idle.out.angular_rps==0,"cached clear observation cannot start a new straight command");
 Rig stopped;stopped.start();stopped.nav.stop("phone_stop");stopped.out=stopped.nav.command();stopped.invalid=1;for(unsigned i=0;i<30;++i)stopped.step();
 ck(!stopped.nav.enabled()&&stopped.out.linear_mps==0,"explicit STOP cannot be resurrected by grace");
 Rig imu;imu.start();auto bad=*imu.nav.frame();bad.timestamp_s+=.02;bad.device_timestamp_s=*bad.device_timestamp_s+.02;bad.imu_valid=false;
 imu.nav.ingest(bad,bad.timestamp_s);auto out=imu.nav.tick(bad.timestamp_s);ck(!imu.nav.enabled()&&out.linear_mps==0,"real IMU fault still stops immediately");
}
void no_turn_from_missing_ranges(){
 Rig r;r.start();r.distances={.25,2.,1.};r.step();
 bool turning=false;for(unsigned i=0;i<80&&!turning;++i){r.step();turning=std::abs(r.out.angular_rps)>0;}
 ck(turning,"fixture actually enters obstacle turn");r.invalid=1;r.freeze=1;r.step();
 ck(r.nav.enabled()&&r.out.linear_mps==0&&r.out.angular_rps==0,"turn cannot use straight grace to ignore missing range");
}
}
int main(){try{for(bool live:{false,true})for(bool invalid:{false,true})grace_boundary(live,invalid);partial_near_preempts_grace();source_id_and_start_boundaries();no_turn_from_missing_ranges();std::cout<<"PASS v38 source-grace "<<checks<<" checks: real source age, one-second boundary, partial near priority, measured recovery, retained authorization\n";}catch(const std::exception& e){std::cerr<<"FAIL v38 source-grace checks="<<checks<<" "<<e.what()<<'\n';return 1;}}
