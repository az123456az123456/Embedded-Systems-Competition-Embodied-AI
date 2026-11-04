#include "controller.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace robotphone;
namespace {
int checks=0;
void ck(bool b,const char*m){++checks;if(!b){std::cerr<<"FAIL "<<m<<'\n';std::exit(1);}}
Config config(){Config c;c.follow_front_sonar_distance=false;c.follow_distance_m=.4;c.left_motor_mask=10;c.bearing.enabled=true;for(auto&e:c.bearing.eyes){e.calibrated=true;e.fx_px=600;e.cx_px=480;e.lateral_right_m=0;}return c;}
Detection box(){Detection d;d.class_id=0;d.name="Person";d.score=.9;d.eye=0;d.x1=185;d.x2=247;d.y1=60;d.y2=420;return d;}
struct Rig {
 Controller c{config()};uint64_t seq=0,frame=0;
 void sensors(uint64_t t){c.updateMotorLink(t,true);c.updateAngleProtocol(true);c.updateVelocityProtocol(true,true);c.updateImu({t,true,0,0,0});c.updateRanges({t,{true,true,true},{3,3,3}});}
 void vision(uint64_t t,bool visible=true){c.updateVision({++frame,t,visible?std::vector<Detection>{box()}:std::vector<Detection>{}});}
 Rig(){sensors(1000);vision(1000);ck(c.arm(++seq,1000),"arm");ck(c.select(frame,c.vision().boxes[0].track_id,1000),"select");ck(c.follow(++seq,1000),"follow admission");}
 FollowObservation observation(uint64_t t,double distance=2,double angle=0){FollowObservation o;o.target_id=c.state().target_id;o.selection_ms=c.state().selection_ms;o.vision_ms=c.vision().time_ms;o.scan_ms=o.pose_ms=t;o.matched=true;o.forward_clear=o.reverse_clear=o.turn_clear=true;o.distance_m=distance;o.bearing_right_deg=angle;o.x_body_m=distance*std::cos(angle*M_PI/180);o.y_body_m=-distance*std::sin(angle*M_PI/180);return o;}
 void step(uint64_t t,double distance,double angle=0){sensors(t);vision(t);ck(c.updateFollowObservation(observation(t,distance,angle),t),"fresh association accepted");ck(c.follow(++seq,t),"owner heartbeat");c.tick(t);}
};
void lifecycle(){
 Rig r;r.c.tick(1001);ck(r.c.state().mode==Mode::Follow&&r.c.state().linear_mps==0&&r.c.state().reason=="follow_wait_lidar","missing lidar waits without motion");
 r.step(1020,2);ck(r.c.state().twist_mode&&r.c.state().linear_mps>.2,"large image box uses actual lidar distance");
 r.step(1040,2,30);ck(r.c.state().linear_mps==0&&r.c.state().angular_rps<0,"target right turns right before approaching");
 r.step(1060,.42);ck(r.c.state().target_id&&r.c.state().mode==Mode::Follow&&r.c.state().linear_mps==0,"0.4m hold keeps selected identity and task");
 r.step(1080,2);ck(r.c.state().linear_mps>0,"moving target resumes approach without new start");
 r.step(1100,.25);ck(r.c.state().linear_mps<0&&r.c.state().linear_mps>=-.10,"target too close slowly backs up");
 r.c.updateRanges({1100,{true,true,true},{.15,3,3}});r.c.tick(1100);ck(r.c.state().linear_mps<0,"near front target permits retreat into verified clear rear");
 r.c.updateRanges({1100,{true,true,true},{3,3,3}});
 auto o=r.observation(1100,.25);o.reverse_clear=false;r.c.updateFollowObservation(o,1100);r.c.tick(1101);ck(r.c.state().linear_mps==0&&r.c.state().mode==Mode::Follow,"rear obstacle prevents reversing and preserves task");
 r.step(1120,2);o=r.observation(1120);o.forward_clear=false;r.c.updateFollowObservation(o,1120);r.c.tick(1121);ck(r.c.state().linear_mps==0,"front obstacle stops approach");
 r.step(1140,2,20);o=r.observation(1140,2,20);o.turn_clear=false;r.c.updateFollowObservation(o,1140);r.c.tick(1141);ck(r.c.state().linear_mps==0&&r.c.state().angular_rps==0,"side obstacle stops rotation");
 r.step(1160,2);auto id=r.c.state().target_id;r.sensors(1180);r.vision(1180,false);r.c.tick(1180);ck(r.c.state().linear_mps==0&&r.c.state().target_id==id,"empty detection frame immediately stops without changing identity");
 r.step(1200,2);ck(r.c.state().target_id==id&&r.c.state().linear_mps>0,"brief uniquely matched dropout reacquires same target");
 r.sensors(1500);r.c.follow(++r.seq,1500);r.c.tick(1500);ck(r.c.state().linear_mps==0&&r.c.state().mode==Mode::Follow,"original scan timestamp expires despite heartbeat");
 r.c.stop(1501);o=r.observation(1501);ck(!r.c.updateFollowObservation(o,1501)&&!r.c.follow(++r.seq,1501),"stop cannot be undone by late observation or heartbeat");
}
void provenance(){
 Rig r;double left=0,right=0;ck(r.c.selectedRays(1000,left,right)&&left<0&&right>0,"box edges project to calibrated optical rays");auto o=r.observation(1000);o.target_id++;ck(!r.c.updateFollowObservation(o,1000),"wrong target rejected");
 o=r.observation(1000);o.selection_ms++;ck(!r.c.updateFollowObservation(o,1000),"old selection rejected");
 o=r.observation(1000);o.scan_ms=1001;ck(!r.c.updateFollowObservation(o,1000),"future scan rejected");
 o=r.observation(1000);o.pose_ms=1;ck(!r.c.updateFollowObservation(o,1000),"stale pose rejected");
 o=r.observation(1000);o.distance_m=std::numeric_limits<double>::quiet_NaN();ck(!r.c.updateFollowObservation(o,1000),"nonfinite range rejected");
 o=r.observation(1000);o.x_body_m=4;ck(!r.c.updateFollowObservation(o,1000),"inconsistent geometry rejected");
 o=r.observation(1000);o.matched=false;o.distance_m=o.x_body_m=o.y_body_m=0;ck(r.c.updateFollowObservation(o,1000),"fresh unmatched scan accepted for centering only");r.c.tick(1001);ck(r.c.state().linear_mps==0,"unmatched scan cannot authorize forward speed");
 // Even perfect sensor updates cannot extend the phone's 350ms motion command.
 r.sensors(1351);r.vision(1351);o=r.observation(1351);r.c.updateFollowObservation(o,1351);r.c.tick(1351);ck(r.c.state().mode==Mode::Follow&&r.c.state().linear_mps==0,"observation cannot renew motion authority; follow waits for owner command");

}
void temporaryInterruptions(){
 Rig r;r.step(1050,2.5);const auto id=r.c.state().target_id;
 ck(r.c.state().linear_mps>0,"far target starts approach before interruption");
 r.c.updateImu({1080,false,0,0,0});
 ck(r.c.state().mode==Mode::Follow&&r.c.state().target_id==id&&r.c.state().linear_mps==0,"brief IMU outage stops output without cancelling explicit follow");
 ck(r.c.follow(++r.seq,1090),"owner heartbeat remains accepted while waiting for IMU");
 r.step(1120,2.4);ck(r.c.state().linear_mps>0,"fresh IMU and new observations resume the existing follow");
 r.c.updateMotorLink(1150,false);
 ck(r.c.state().mode==Mode::Follow&&r.c.state().linear_mps==0,"brief motor ACK outage retains follow intent with zero output");
 ck(r.c.follow(++r.seq,1160),"owner can maintain follow while motor ACK is unavailable");
 r.c.tick(1160);ck(r.c.state().linear_mps==0&&r.c.state().angular_rps==0,"heartbeat alone cannot bypass missing motor ACK");
 r.step(1200,2.3);ck(r.c.state().linear_mps>0,"fresh ACK permits existing follow to resume");
 r.sensors(1580);r.vision(1580);r.c.updateFollowObservation(r.observation(1580,2.2),1580);r.c.tick(1580);
 ck(r.c.state().mode==Mode::Follow&&r.c.state().linear_mps==0,"350ms command gap parks follow without dropping the task");
 r.sensors(1600);r.vision(1600);r.c.updateFollowObservation(r.observation(1600,2.2),1600);r.c.tick(1600);
 ck(r.c.state().linear_mps==0,"fresh sensors cannot renew the expired motion command");
 ck(r.c.follow(++r.seq,1600),"new owner heartbeat renews command after short gap");r.c.tick(1600);
 ck(r.c.state().linear_mps>0,"far target continues after command delivery recovers");
 r.c.stop(1610);ck(!r.c.follow(++r.seq,1620)&&!r.c.state().armed,"explicit stop still requires a fresh arm");
 Rig expiry;expiry.step(1050,2.5);expiry.c.tick(31051);
 ck(!expiry.c.state().armed&&expiry.c.state().target_id==0,"30s owner lease still revokes follow and selection");
 Rig manual;manual.step(1050,2.5);manual.c.updateMotorLink(1080,false);
 manual.c.drive(++manual.seq,.3,0,false,1090);
 ck(manual.c.state().mode!=Mode::Follow,"manual override cancels follow even while motor link is unavailable");
 manual.sensors(1120);manual.vision(1120);manual.c.tick(1120);
 ck(manual.c.state().linear_mps==0&&manual.c.state().left==0,"sensor recovery after manual override cannot resume old follow");
}
void imageServo(){
 auto cfg=config();cfg.bearing.enabled=false;Controller c(cfg);
 auto sensors=[&](uint64_t t){c.updateMotorLink(t,true);c.updateVelocityProtocol(true,true);c.updateImu({t,true,0,0,0});c.updateRanges({t,{true,true,true},{3,3,3}});};
 sensors(1000);auto d=box();d.x1+=40;d.x2+=40;
 auto f=c.updateVision({1,1000,{d}});ck(c.arm(1,1000)&&c.select(1,f.boxes[0].track_id,1000),"select without intrinsics");
 ck(c.follow(2,1000),"image servo admits without focal length");
 c.tick(1000);ck(c.state().linear_mps==0&&c.state().angular_rps==0,"pixel centering still requires lidar clearance");
 FollowObservation o;o.target_id=c.state().target_id;o.selection_ms=c.state().selection_ms;o.vision_ms=o.scan_ms=o.pose_ms=1000;o.turn_clear=o.forward_clear=o.reverse_clear=true;
 ck(c.updateFollowObservation(o,1000),"unmatched lidar supplies fresh clearance");c.tick(1001);
 ck(c.state().angular_rps<0&&c.state().linear_mps==0,"dimensionless right image error rotates right only");
 o.matched=true;o.distance_m=o.x_body_m=2;c.updateFollowObservation(o,1001);c.tick(1002);
 ck(c.state().linear_mps==0,"off-center image blocks even an old matched range");
 sensors(1020);c.updateVision({2,1020,{box()}});o.vision_ms=o.scan_ms=o.pose_ms=1020;c.updateFollowObservation(o,1020);c.follow(3,1020);c.tick(1020);
 ck(c.state().linear_mps>0&&c.state().linear_mps<=.16,"centered image approaches using lidar metres");
 o.distance_m=o.x_body_m=.42;c.updateFollowObservation(o,1020);c.tick(1021);
 ck(c.state().linear_mps==0&&c.state().mode==Mode::Follow,"uncalibrated mode holds 0.4m continuously");
 o.distance_m=o.x_body_m=.25;c.updateFollowObservation(o,1020);c.tick(1022);ck(c.state().linear_mps<0,"uncalibrated mode retreats when rear is clear");
 o.reverse_clear=false;c.updateFollowObservation(o,1020);c.tick(1023);ck(c.state().linear_mps==0,"rear obstacle blocks image-mode reverse");
 o.turn_clear=false;o.matched=false;c.updateFollowObservation(o,1020);sensors(1040);c.updateVision({3,1040,{d}});c.tick(1040);
 ck(c.state().angular_rps==0,"side clearance required for image servo");
 c.stop(1041);ck(!c.follow(4,1042),"late image heartbeat cannot undo cancel");
}
void movingTarget(){
 Rig r;double robot=0,target=2.5,t=1.;
 // Closed-loop longitudinal plant: target walks away for 8 seconds, then waits.
 for(int i=1;i<=600;i++){
   auto ms=uint64_t(1000+i*50);if(i<160)target+=.003;
   r.step(ms,target-robot);robot+=r.c.state().linear_mps*.05;t+=.05;
 }
 ck(std::abs((target-robot)-.4)<=.065,"moving target converges to 0.4m deadband over sustained follow");
 ck(r.c.state().mode==Mode::Follow&&r.c.state().target_id,"task persists after reaching distance");
 (void)t;
}
void fourDecimetres(){
 auto cfg=config();cfg.bearing.enabled=false;Controller c(cfg);
 ck(c.config().follow_distance_m==.4&&c.config().follow_distance_deadband_m==.05,"legacy fixture setpoint is 4dm with 5cm deadband");
 auto step=[&](uint64_t t,double distance,double sonar,bool front_clear=true,bool rear_clear=true){
  c.updateMotorLink(t,true);c.updateVelocityProtocol(true,true);c.updateImu({t,true,0,0,0});
  c.updateRanges({t,{true,true,true},{sonar,3,3}});
  auto f=c.updateVision({t,t,{box()}});
  if(!c.state().armed){ck(c.arm(1,t),"4dm arm");ck(c.select(f.frame_id,f.boxes[0].track_id,t),"4dm select");}
  ck(c.follow(t,t),"4dm owner heartbeat");
  FollowObservation o;o.target_id=c.state().target_id;o.selection_ms=c.state().selection_ms;
  o.vision_ms=o.scan_ms=o.pose_ms=t;o.matched=true;o.distance_m=o.x_body_m=distance;
  o.forward_clear=front_clear;o.reverse_clear=rear_clear;o.turn_clear=true;
  ck(c.updateFollowObservation(o,t),"4dm fresh observation");c.tick(t);
 };
 step(1000,.65,.48);ck(c.state().linear_mps>0,"4dm approach is not stopped by the old 50cm sonar threshold");
 step(1020,.42,.26);ck(c.state().linear_mps==0&&c.state().reason=="follow_holding_distance","4dm target holds inside 5cm deadband");
 step(1040,.30,.14);ck(c.state().linear_mps<0,"close target retreats with clear rear despite near front sonar");
 step(1060,.30,.14,true,false);ck(c.state().linear_mps==0,"rear obstruction prevents retreat at 4dm");
 step(1080,.65,.48,false);ck(c.state().linear_mps==0,"lidar obstruction prevents 4dm approach");
 step(1100,.65,.19);ck(c.state().linear_mps==0&&c.state().reason=="follow_obstacle_front","20cm front sonar hard stop retained");
 c.stop(1101);ck(!c.follow(1102,1102)&&!c.state().armed,"4dm cancel remains final");
}
}
int main(){fourDecimetres();temporaryInterruptions();lifecycle();provenance();imageServo();movingTarget();std::cout<<"PASS lidar follow "<<checks<<" checks\n";}
