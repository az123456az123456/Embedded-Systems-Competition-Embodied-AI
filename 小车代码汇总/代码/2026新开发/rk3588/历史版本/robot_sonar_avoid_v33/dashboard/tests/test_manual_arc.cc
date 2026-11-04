#include "../phone/controller.h"
#include "../phone/motor_link.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
using namespace robotphone;
static int checks;
static void ck(bool v,const char *label){++checks;if(!v){std::fprintf(stderr,"FAIL %s\n",label);std::exit(1);}}
static Config cfg(){Config c;c.left_motor_mask=10;c.slew_per_second=1000;return c;}
static void ready(Controller&c,uint64_t t,bool arc=true){c.updateMotorLink(t,true);c.updateArcProtocol(arc);c.updateImu({t,true,0,0,0});}
static int signed16(const uint8_t*p){unsigned u=(p[0]<<8)|p[1];return u>=32768?int(u)-65536:int(u);}
static void physical(const State&s,int l,int r){if(std::lround(s.left*1000)!=l||std::lround(s.right*1000)!=r)std::fprintf(stderr,"actual=(%f,%f) expected=(%d,%d)\n",s.left,s.right,l,r);ck(std::lround(s.left*1000)==l&&std::lround(s.right*1000)==r,"physical left/right golden");}
static void golden(){
 struct V{double y,x;int speed,curve,l,r;};
 const V v[]={{1,0,600,0,600,600},{-1,0,-600,0,-600,-600},{1,-1,600,-999,150,600},{1,1,600,999,600,150},{-1,-1,-600,-999,-150,-600},{-1,1,-600,999,-600,-150},{0,-1,600,-999,150,600},{0,1,600,999,600,150}};
 const uint8_t mac[]={2,1,2,3,4,5};
 for(const auto&a:v){
  Controller c(cfg());ready(c,1000);ck(c.arm(1,1000),"arm");
  ck(c.drive(2,a.y,a.x,false,1000),"continuous joystick accepted");const auto&s=c.tick(1010);physical(s,a.l,a.r);
  if(a.curve==0){
   ck(s.arc_mode&&s.arc_outer_permille==a.speed&&s.arc_curve_q8==0,"straight MOT3 physical fields");
   const auto b=arcMotorFrame(mac,2,true,s.arc_outer_permille,s.arc_curve_q8,10,15);
   ck(std::memcmp(b.data()+14,"MOT3",4)==0&&signed16(b.data()+24)==a.speed,"straight real MOT3 codec");
   ck(motorCrc16(b.data()+14,16)==unsigned((b[30]<<8)|b[31]),"MOT3 real CRC");
  }else{
   ck(!s.arc_mode,"sharp same-sign arc uses exact MOT1 physical duties");
   const auto b=motorFrame(mac,2,true,std::lround(s.left*1000),std::lround(s.right*1000),10,15);
   ck(std::memcmp(b.data()+14,"MOT1",4)==0&&signed16(b.data()+24)==a.l&&signed16(b.data()+26)==a.r,"sharp real MOT1 preserves exact duties and physical signs");
   ck(motorCrc16(b.data()+14,16)==unsigned((b[30]<<8)|b[31]),"MOT1 real CRC");
  }
 }

}
static void coneAndContinuity(){
 for(int reverse:{-1,1})for(int degrees=-89;degrees<=89;degrees++){
  Controller c(cfg());ready(c,1000);c.arm(1,1000);c.drive(2,reverse,0,false,1000);c.tick(1010);double angle=degrees*3.141592653589793/180.;c.drive(3,reverse*std::cos(angle),std::sin(angle),false,1020);const auto&s=c.tick(1030);
  ck(s.left*reverse>0&&s.right*reverse>0,"both sides translate without manual spin");
  ck(std::min(std::abs(s.left),std::abs(s.right))>=std::max(std::abs(s.left),std::abs(s.right))*.25-.001,"inner target at least 25 percent outer");
  if(std::abs(degrees)<=15)ck(s.left==s.right&&s.arc_curve_q8==0,"expanded exact equal straight/reverse cone");
  else if(degrees<0)ck(std::abs(s.right)>std::abs(s.left),"left path right side faster both directions");
  else ck(std::abs(s.left)>std::abs(s.right),"right path left side faster both directions");
 }
 Controller c(cfg());ready(c,1000);c.arm(1,1000);c.drive(2,-1,0,false,1000);c.tick(1010);c.drive(3,0,-1,false,1020);auto s=c.tick(1030);physical(s,-150,-600);ck(!s.arc_mode,"sharp horizontal retains reverse translation through physical MOT1");
 c.drive(4,.06,-1,false,1040);s=c.tick(1050);ck(s.left<0&&s.right<0,"small horizontal noise cannot reverse travel");
 c.drive(5,.13,-1,false,1060);s=c.tick(1070);ck(s.left>0&&s.right>0,"intentional vertical sign changes direction");
 c.drive(6,0,0,false,1080);ck(c.state().left==0&&c.state().right==0&&!c.state().arc_mode,"release immediate zero");
 c.drive(7,0,-1,false,1090);s=c.tick(1100);physical(s,150,600);
}
static void tinyTravelSign(){
 for(double magnitude:{.001,.01,.061,.08,.119,.12,.2})for(int sign:{-1,1})for(double ratio:{0.,.1,.3}){
  Controller c(cfg());ready(c,1000);c.arm(1,1000);
  ck(c.drive(2,sign*magnitude,magnitude*ratio,false,1000),"small stick accepted");
  const auto &s=c.tick(1010);ck(s.left*sign>0&&s.right*sign>0,"tiny straight or near-straight movement preserves requested forward/reverse sign");
 }
 for(int sign:{-1,1}){
  Controller c(cfg());ready(c,1000);c.arm(1,1000);c.drive(2,sign*.061,0,false,1000);c.tick(1010);
  c.drive(3,-sign*.05,.95,false,1020);const auto&s=c.tick(1030);
  ck(s.left*sign>0&&s.right*sign>0,"angular horizontal noise retains last travel sign independent of radial magnitude");
 }
}
static void slewTransitions(){
 Config config=cfg();config.slew_per_second=1.2;
 double largest_step=0;int transition_cases=0;
 struct Gesture{bool rotate;bool cw;double y,x;};
 const Gesture starts[]={{true,true,0,0},{true,false,0,0},{false,false,1,-1},{false,false,1,1},{false,false,-1,-1},{false,false,-1,1}};
 const Gesture targets[]={{false,false,1,-1},{false,false,1,1},{false,false,-1,-1},{false,false,-1,1},{false,false,0,-1},{false,false,0,1},{false,false,1,0},{false,false,-1,0}};
 for(const auto &start:starts)for(const auto &target:targets){
  Controller c(config);uint64_t t=1000,seq=1;ready(c,t);c.arm(seq++,t);
  for(int i=0;i<100;i++){t+=10;ready(c,t);if(start.rotate)c.rotate(seq++,start.cw,.3,t);else c.drive(seq++,start.y,start.x,false,t);c.tick(t);}
  double previous_left=c.state().left,previous_right=c.state().right;
  bool raw_transition=false;
  for(int i=0;i<150;i++){
   t+=10;ready(c,t);ck(c.drive(seq++,target.y,target.x,false,t),"transition command accepted");const auto state=c.tick(t);
   const double step=std::max(std::abs(state.left-previous_left),std::abs(state.right-previous_right));
   largest_step=std::max(largest_step,step);
   ck(step<=.013000001,"spin/opposite arc slew bound plus at most one permille wire quantization");
   if(state.arc_mode)ck(state.left*state.right>=0&&std::min(std::abs(state.left),std::abs(state.right))*2>=std::max(std::abs(state.left),std::abs(state.right))-1e-9,"only representable intermediate uses MOT3");
   else raw_transition=true;
   previous_left=state.left;previous_right=state.right;
  }
  ck(c.state().arc_mode==(target.x==0),"transition reaches correct physical MOT3 straight or exact MOT1 sharp arc");
  if(start.rotate)ck(raw_transition,"spin transition preserves exact MOT1 until arc representable");
  ++transition_cases;
 }
 std::printf("PASS slew transitions %d cases maximum 10ms side step %.6f (slew .012 + wire quantization)\n",transition_cases,largest_step);
}
static void compatibilityAndBypass(){
 Controller c(cfg());ready(c,1000,false);c.arm(1,1000);ck(!c.drive(2,1,0,false,1000)&&c.state().reason=="arc_protocol_waiting"&&!c.state().armed,"legacy FPGA cannot enter new manual semantics");
 ready(c,1010);c.arm(3,1010);c.drive(4,1,1,false,1010);c.tick(1020);c.updateArcProtocol(false);ck(!c.state().armed&&c.state().left==0&&c.state().reason=="arc_protocol_lost","loss of arc capability disarms immediately");
 ready(c,1030);c.arm(5,1030);ck(c.driveWheels(6,-.11,-.29,1030),"private navigation exact wheel API");auto s=c.tick(1040);ck(!s.arc_mode&&!s.angle_mode,"navigation bypass MOT3");physical(s,-110,-290);
 c.rotate(7,true,.3,1050);s=c.tick(1060);physical(s,300,-300);ck(!s.arc_mode,"dedicated rotation uses MOT1");
 c.drive(8,-.5,0,true,1070);c.tick(1080);c.updateImu({1090,true,0,0,5});c.drive(9,-.5,0,true,1090);s=c.tick(1100);ck(s.left>s.right&&s.right<0&&!s.arc_mode,"reverse heading keeps physical yaw correction sign");
 ready(c,1401);c.tick(1451);ck(!c.state().armed&&c.state().left==0,"command watchdog independent of fresh sensors");
 MotorAckWindow w;w.noteSent(8,2000,10,15,3);MotorAck a;a.sequence=8;a.configured=true;a.left_mask=10;a.invert_mask=15;a.protocol=1;ck(!w.accept(a,2010),"ACK1 cannot satisfy MOT3 request");a.protocol=3;ck(w.accept(a,2010)&&w.fresh(2010),"matching ACK3 capability response accepted");ck(!w.accept(a,2020),"ACK3 replay does not refresh");ck(!w.fresh(2301),"ACK3 capability liveness expires");
}
static void boundariesAndTuning(){
 for(int direction:{-1,1})for(int steer_sign:{-1,1})for(double angle:{0.,14.999,15.,15.001,30.999,31.,31.001,34.999,35.,35.001,44.999,45.,45.001,60.,89.999,90.}){
  Controller c(cfg());ready(c,1000);c.arm(1,1000);
  c.drive(2,direction,0,false,1000);c.tick(1010);
  const double rad=angle*3.14159265358979323846/180.;
  ck(c.drive(3,direction*std::cos(rad),steer_sign*std::sin(rad),false,1020),"boundary command");
  const auto &s=c.tick(1030);
  const int inner=std::lround(600*(1-.75*std::min(1.,std::max(0.,(angle-15)/30.))));
  physical(s,direction*(steer_sign<0?inner:600),direction*(steer_sign<0?600:inner));
  ck(s.left*direction>0&&s.right*direction>0,"full sharp two-sided same-sign translation");
  if(inner<300)ck(!s.arc_mode,"below physical 50 percent floor always exact MOT1");
  else ck(s.arc_mode,"representable boundary stays MOT3");
 }
 for(double ratio:{.1,.25,.4,.5,.6,.8})for(double full:{15.001,30.,45.,60.,90.}){
  Config config=cfg();config.turn_inner_ratio=ratio;config.turn_full_deg=full;Controller c(config);ready(c,1000);
  ck(c.arm(1,1000),"finite in-range tuning can arm");
  double a=full*3.14159265358979323846/180.;c.drive(2,std::cos(a),std::sin(a),false,1000);
  const auto&s=c.tick(1010);physical(s,600,std::lround(600*ratio));
 }
 for(double bad:{-.1,0.,.099,.801,1.,double(INFINITY),double(NAN)}){
  Config config=cfg();config.turn_inner_ratio=bad;Controller c(config);ready(c,1000);ck(!c.arm(1,1000),"invalid inner ratio disables arm");
 }
 for(double bad:{0.,15.,90.1,double(INFINITY),double(NAN)}){
  Config config=cfg();config.turn_full_deg=bad;Controller c(config);ready(c,1000);ck(!c.arm(1,1000),"invalid angle disables arm");
 }
 Controller c(cfg());ready(c,1000);c.arm(1,1000);c.drive(2,1,1,false,1000);c.tick(1010);
 ck(!c.state().arc_mode&&c.state().armed,"sharp manual uses MOT1 while retaining capability guard");
 c.updateArcProtocol(false);ck(!c.state().armed&&c.state().left==0&&c.state().right==0&&c.state().reason=="arc_protocol_lost","capability loss immediately disarms sharp MOT1 manual");
 ready(c,1020);c.arm(3,1020);c.drive(4,1,1,true,1020);c.tick(1030);c.updateArcProtocol(false);
 ck(!c.state().armed&&c.state().left==0,"capability loss disarms heading override sharp arc");
 ready(c,1040);c.arm(5,1040);c.driveWheels(6,.15,.6,1040);c.tick(1050);c.updateArcProtocol(false);
 ck(c.state().armed,"private navigation physical MOT1 does not acquire manual arc dependency");
}
static void physicalSweep(){
 for(int direction:{-1,1})for(int steer_sign:{-1,1}){
  int previous_inner=600;
  for(int step=0;step<=9000;step++){
   const double angle=step*.01,rad=angle*3.14159265358979323846/180.;
   Controller c(cfg());ready(c,1000);c.arm(1,1000);c.drive(2,direction,0,false,1000);c.tick(1010);
   c.drive(3,direction*std::cos(rad),steer_sign*std::sin(rad),false,1020);const auto &s=c.tick(1030);
   const int outer=std::lround(std::max(std::abs(s.left),std::abs(s.right))*1000);
   const int inner=std::lround(std::min(std::abs(s.left),std::abs(s.right))*1000);
   const int desired=std::lround(600*(1-.75*std::min(1.,std::max(0.,(angle-15)/30.))));
   ck(outer==600&&std::abs(inner-desired)<=1,"actual wire wheel pair tracks strong target within one permille");
   ck(inner<=previous_inner&&previous_inner-inner<=2,"fine angle physical sweep is monotonic and continuous across MOT3/MOT1 boundary");
   ck(s.left*direction>0&&s.right*direction>0,"all sweep requests translate on both sides");
   previous_inner=inner;
  }
 }
}
int main(){golden();coneAndContinuity();tinyTravelSign();slewTransitions();compatibilityAndBypass();boundariesAndTuning();physicalSweep();std::printf("PASS strong manual arcs %d checks: 8 direction goldens, 15/31/35/45 degree boundaries, 25 percent sharp ratio, tuning validation, reverse/horizontal hysteresis, navigation bypass, heading sign, capability loss during MOT1, watchdogs\n",checks);}
