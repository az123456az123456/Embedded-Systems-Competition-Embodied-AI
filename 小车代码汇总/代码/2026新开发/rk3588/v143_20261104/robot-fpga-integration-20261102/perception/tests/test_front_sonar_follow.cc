#include "controller.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace robotphone;
namespace {
int checks=0;
void ck(bool ok,const char* text){++checks;if(!ok){std::cerr<<"FAIL "<<text<<'\n';std::exit(1);}}
Config config(){Config c;c.left_motor_mask=10;c.bearing.enabled=false;return c;}
Detection box(double offset=0){Detection d;d.class_id=0;d.name="Person";d.score=.9;d.eye=0;d.x1=185+offset;d.x2=247+offset;d.y1=60;d.y2=420;return d;}
struct Rig {
 Controller c{config()};uint64_t seq=0;
 void sensors(uint64_t t,double distance,bool valid=true){c.updateMotorLink(t,true);c.updateVelocityProtocol(true,true);c.updateImu({t,true,0,0,0});c.updateRanges({t,{valid,true,true},{distance,2,2}});}
 void vision(uint64_t t,double offset=0){c.updateVision({t,t,{box(offset)}});}
 Rig(){sensors(1000,2);vision(1000);ck(c.arm(++seq,1000),"arm");ck(c.select(1000,c.vision().boxes[0].track_id,1000),"select");ck(c.follow(++seq,1000),"follow");}
 void observation(uint64_t t,bool matched=false,double lidar_distance=0,bool front=true,bool rear=true,bool turn=true,double bearing=0){
  FollowObservation o;o.target_id=c.state().target_id;o.selection_ms=c.state().selection_ms;
  o.vision_ms=c.vision().time_ms;o.scan_ms=o.pose_ms=t;o.matched=matched;
  o.distance_m=lidar_distance;o.bearing_right_deg=bearing;o.x_body_m=lidar_distance*std::cos(bearing*M_PI/180);o.y_body_m=-lidar_distance*std::sin(bearing*M_PI/180);
  o.forward_clear=front;o.reverse_clear=rear;o.turn_clear=turn;
  ck(c.updateFollowObservation(o,t),"fresh lidar clearance accepted");
 }
 void step(uint64_t t,double distance,bool matched=false,double lidar_distance=0,bool front=true,bool rear=true,bool turn=true,double offset=0){
  sensors(t,distance);vision(t,offset);observation(t,matched,lidar_distance,front,rear,turn);
  ck(c.follow(++seq,t),"new owner command");c.tick(t);
 }
};
void sourceAndMotion(){
 Rig r;ck(r.c.config().follow_front_sonar_distance,"production default ranges from front sonar");
 r.c.tick(1000);ck(r.c.state().reason=="following"&&r.c.state().linear_mps>0,"fresh visual target and sonar approach without any lidar observation");
 r.step(1020,2);ck(r.c.state().linear_mps>0,"sonar approach ignores unmatched lidar");
 r.step(1040,.3,true,3);ck(r.c.state().linear_mps==0&&r.c.state().reason=="follow_holding_distance","30cm sonar holds despite far lidar range");
 ck(r.c.followDistanceValid(1040)&&r.c.followDistanceMetres()==.3&&r.c.followDistanceTimeMs()==1040,"telemetry reports actual front echo and source time");
 r.step(1060,.8,true,.2);ck(r.c.state().linear_mps>0,"near lidar range cannot replace far sonar target range");
 r.step(1080,.2,true,3);ck(r.c.state().linear_mps==0&&r.c.state().angular_rps==0&&r.c.state().reason=="follow_sonar_too_close","20cm echo stops because rear has no ultrasonic coverage");
 r.step(1100,.23,false,0,true,false);ck(r.c.state().linear_mps==0&&r.c.state().reason=="follow_sonar_too_close","close echo never authorizes blind reverse");
 r.step(1120,.15);ck(r.c.state().linear_mps==0,"lidar rear clearance cannot grant reverse in ultrasonic mode");
 r.step(1140,.8,false,0,false);ck(r.c.state().linear_mps>0,"lidar clearance cannot block front-sonar approach");
 r.step(1160,.8,false,0,true,true,true,35);ck(r.c.state().linear_mps==0&&r.c.state().angular_rps<0,"off-center image only turns before using sonar for translation");
 ck(!r.c.followPositionValid(1160),"off-axis image cannot draw a trusted frontal target point");
 r.step(1180,.8,false,0,true,true,false,35);ck(r.c.state().angular_rps<0,"lidar turn clearance cannot block visual steering");
 r.sensors(1190,.8);r.c.updateRanges({1190,{true,true,true},{.8,2,.2}});r.vision(1190,35);ck(r.c.follow(++r.seq,1190),"right obstacle command accepted");r.c.tick(1190);
 ck(r.c.state().angular_rps==0&&r.c.state().reason=="follow_obstacle_side","right ultrasonic obstacle blocks right turn");
 r.vision(1191,0);
 r.sensors(1192,.8);r.c.updateRanges({1192,{true,true,true},{.8,.2,2}});r.vision(1192,-35);ck(r.c.follow(++r.seq,1192),"left obstacle command accepted");r.c.tick(1192);
 ck(r.c.state().angular_rps==0&&r.c.state().reason=="follow_obstacle_side","left ultrasonic obstacle blocks left turn");
 r.step(1200,.25);ck(r.c.state().linear_mps==0&&r.c.state().reason=="follow_holding_distance","25cm inclusive hold boundary is stable");
 r.step(1220,.35);ck(r.c.state().linear_mps==0&&r.c.state().reason=="follow_holding_distance","35cm inclusive hold boundary is stable");
}
void invalidAndLeases(){
 Rig r;r.step(1020,.8);
 for(int which=0;which<4;++which){
  uint64_t t=1040+which*20;r.sensors(t,which==1?std::numeric_limits<double>::quiet_NaN():which==2?4.6:.8,which!=0);
  r.vision(t);r.observation(t,true,2);
  if(which==3)r.c.updateRanges({t-600,{true,true,true},{.8,2,2}});
  ck(r.c.follow(++r.seq,t),"task persists through bad sonar");r.c.tick(t);
  ck(r.c.state().linear_mps==0&&r.c.state().reason=="follow_wait_sonar"&&!r.c.followDistanceValid(t),"invalid or stale sonar never falls back to valid lidar target range");
 }
 r.step(1140,.8);ck(r.c.state().linear_mps>0,"fresh echo and owner command resume task");
 r.sensors(1520,.8);r.vision(1520);r.observation(1520);r.c.tick(1520);
 ck(r.c.state().mode==Mode::Follow&&r.c.state().linear_mps==0&&r.c.state().reason=="follow_wait_command","fresh sonar cannot renew expired phone motion command");
 ck(r.c.follow(++r.seq,1520),"new owner command accepted");r.c.tick(1520);ck(r.c.state().linear_mps>0,"command recovery resumes sonar follow");
 r.c.updateVision({1540,1540,{}});r.c.tick(1540);ck(r.c.state().linear_mps==0&&!r.c.followDistanceValid(1540),"lost selected visual target stops even with good echo");
 r.c.stop(1541);ck(!r.c.follow(++r.seq,1542)&&!r.c.state().armed,"cancel cannot be undone by echo or command replay");
}
void admissionAndSteering(){
 Rig r;
 // Reselecting cancels the old task; a miss before the first follow must wait,
 // rather than reject the command and let the gateway disarm the whole owner.
 ck(r.c.select(r.c.vision().frame_id,r.c.state().target_id,1001),"reselect before initial miss");
 r.c.updateVision({1020,1020,{}});r.sensors(1020,.8);
 ck(r.c.follow(++r.seq,1020),"initial follow accepts selected identity during a missed detection");r.c.tick(1020);
 ck(r.c.state().mode==Mode::Follow&&r.c.state().armed&&r.c.state().reason=="follow_wait_target"&&r.c.state().linear_mps==0&&r.c.state().angular_rps==0,"initial miss retains authorization but produces zero motion");
 r.step(1040,.8);ck(r.c.state().linear_mps>0,"fresh target and echo recover after initial admission miss");
 r.step(1060,.8,false,0,true,true,true,35);ck(r.c.state().linear_mps==0&&r.c.state().angular_rps<0,"target moving right commands right turn before approach");
 r.step(1080,.8,false,0,true,true,true,0);
 r.step(1100,.8,false,0,true,true,true,-35);ck(r.c.state().linear_mps==0&&r.c.state().angular_rps>0,"target moving left commands left turn before approach");
 r.step(1120,.3);ck(r.c.state().linear_mps==0,"30cm echo holds");
 r.step(1140,.5);ck(r.c.state().linear_mps>0,"target moving beyond 40cm resumes approach to 30cm");
 for(uint64_t t=1160;t<=2160;t+=20){
  r.sensors(t,.8);r.vision(t);ck(r.c.follow(++r.seq,t),"sonar-only heartbeat");r.c.tick(t);
  ck(r.c.state().linear_mps>0,"lidar observation expiry cannot halt sonar-only approach");
 }
 r.sensors(2180,.8);r.vision(2180,35);ck(r.c.follow(++r.seq,2180),"sonar-only right heartbeat");r.c.tick(2180);
 ck(r.c.state().angular_rps<0&&r.c.state().linear_mps==0,"right steering works without fresh lidar");
 r.sensors(2200,.8);r.vision(2200);r.c.follow(++r.seq,2200);r.c.tick(2200);
 r.sensors(2220,.8);r.vision(2220,-35);r.c.follow(++r.seq,2220);r.c.tick(2220);
 ck(r.c.state().angular_rps>0&&r.c.state().linear_mps==0,"left steering works without fresh lidar");
 r.c.stop(2221);ck(!r.c.follow(++r.seq,2222)&&!r.c.state().armed,"initial-wait acceptance never rearms after explicit cancel");
}
void sustained(){
 Rig r;double gap=1.5;
 for(int i=1;i<=1600;i++){
  r.step(1000+i*50,gap);gap-=r.c.state().linear_mps*.05;
 }
 ck(std::abs(gap-.3)<.065&&r.c.state().mode==Mode::Follow,"closed-loop sonar gap approaches 30cm without completing task");
}
}
int main(){sourceAndMotion();invalidAndLeases();admissionAndSteering();sustained();std::cout<<"PASS front sonar follow "<<checks<<" checks\n";}
