#include "controller.h"
#include "motor_link.h"
#include "follow_sensors.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
using namespace robotphone;
namespace {
int checks=0;void ck(bool b,const char*m){++checks;if(!b){std::cerr<<m<<'\n';std::exit(1);}}
bool near(double a,double b){return std::abs(a-b)<.011;}
Config configuration(){Config c;c.follow_front_sonar_distance=false;c.left_motor_mask=10;c.bearing.enabled=true;for(auto&e:c.bearing.eyes){e.calibrated=true;e.fx_px=600;e.cx_px=480;}return c;}
Detection target(int eye=0,double u=480){Detection d;d.class_id=39;d.score=.9;d.eye=eye;double x=(eye*960+u)*.45;d.x1=x-15;d.x2=x+15;d.y1=200;d.y2=300;return d;}
void sensors(Controller&c,uint64_t t,double yaw=0){c.updateMotorLink(t,true);c.updateAngleProtocol(true);c.updateArcProtocol(true);c.updateImu({t,true,0,0,yaw});c.updateRanges({t,{true,true,true},{2,2,2}});}
void measured(Controller&c,uint64_t t){FollowObservation o;o.target_id=c.state().target_id;o.selection_ms=c.state().selection_ms;o.vision_ms=c.vision().time_ms;o.scan_ms=o.pose_ms=t;o.matched=true;o.forward_clear=o.turn_clear=true;o.distance_m=o.x_body_m=2;auto b=c.selectedBearing(t);o.bearing_right_deg=b.turn_deg;o.x_body_m=2*std::cos(b.turn_deg*M_PI/180);o.y_body_m=-2*std::sin(b.turn_deg*M_PI/180);ck(c.updateFollowObservation(o,t),"scan observation accepted");}
void prepare(Controller&c,int eye=0,double u=480){sensors(c,1000);auto f=c.updateVision({1,1000,{target(eye,u)}});ck(c.select(1,f.boxes[0].track_id,1000),"disarmed select must work");ck(!c.state().armed&&!c.state().angle_mode,"select cannot move");ck(c.arm(1,1000),"arm");ck(c.select(1,f.boxes[0].track_id,1000),"reselect after explicit arm");ck(c.follow(2,1000),"follow");measured(c,1000);c.tick(1010);}
void geometryControl(){
 Controller left(configuration());prepare(left);ck(left.state().angle_mode,"MOT2 requested in follow");ck(near(left.state().turn_angle_deg,-3.43363),"left camera -6cm gives left bearing at virtual 1m");ck(left.state().left<left.state().right,"negative angle turns left");
 Controller right(configuration());prepare(right,1);ck(near(right.state().turn_angle_deg,-1.14576),"right camera -2cm uses its own optical center");ck(near(right.state().left,right.state().right),"FPGA two-degree deadband is matched");
 // A fresh snapshot's camera angle must be corrected by motion since capture.
 left.updateImu({1100,true,0,0,10});left.updateMotorLink(1100,true);left.updateRanges({1100,{true,true,true},{2,2,2}});left.follow(3,1100);left.tick(1100);
 double expect=-3.43363+10;ck(near(left.state().turn_angle_deg,expect),"turn left since capture moves target bearing right");
 ck(left.state().left>left.state().right,"compensated angle reverses correction when appropriate");
 left.drive(4,.2,0,false,1101);left.tick(1110);ck(!left.state().angle_mode&&left.state().mode==Mode::Manual,"manual command leaves angle protocol");
 Controller far(configuration());prepare(far,0,900);ck(far.state().follow_forward==0&&far.state().left>0&&far.state().right<0,"large bearing rotates in place");
 Controller expiry(configuration());prepare(expiry);sensors(expiry,1351);expiry.tick(1351);ck(expiry.state().armed&&!expiry.state().angle_mode,"lease expiry disables angle output independent of fresh sensors");
 Controller lost(configuration());prepare(lost);lost.updateVision({2,1020,{}});ck(!lost.state().angle_mode&&lost.state().mode==Mode::Follow&&lost.state().left==0&&lost.state().right==0,"lost target immediately zeroes angle output");
 Controller dead(configuration());prepare(dead);dead.updateMotorLink(1020,false);ck(dead.state().armed&&!dead.state().angle_mode,"ACK failure disarms angle mode");
}
void interlocks(){
 for(int which=0;which<5;which++){
  Controller c(configuration());prepare(c,0,700);FollowRanges r{1020,{true,true,true},{2,2,2}};
  if(which==0)r.valid[0]=false;
  if(which==1)r.time_ms=1;
  if(which==2)r.metres[0]=.2;
  if(which==3)r.metres[2]=.22;
  if(which==4)r.metres[0]=std::numeric_limits<double>::quiet_NaN();
  c.updateRanges(r);c.tick(1020);ck(!c.state().angle_mode&&c.state().left==0&&c.state().right==0,"invalid/old/close sonar immediately stops");
 }
 auto cfg=configuration();cfg.bearing.enabled=false;Controller disabled(cfg);sensors(disabled,1000);auto f=disabled.updateVision({1,1000,{target()}});disabled.select(1,f.boxes[0].track_id,1000);
 ck(near(disabled.selectedBearing(1000).u_px,480),"unconfigured lens still exposes measured pixel for calibration");disabled.arm(1,1000);disabled.select(1,f.boxes[0].track_id,1000);ck(!disabled.follow(2,1000),"uncalibrated lens cannot start motion");ck(disabled.drive(3,.2,0,false,1001),"manual remains available without lens calibration");
 Controller old(configuration());sensors(old,1000);f=old.updateVision({1,1000,{target()}});old.select(1,f.boxes[0].track_id,1000);old.arm(1,1000);old.select(1,f.boxes[0].track_id,1000);old.updateAngleProtocol(false);ck(!old.follow(2,1000)&&old.state().reason=="angle_protocol_waiting","old FPGA cannot enter angle follow");
 Controller gap(configuration());sensors(gap,1000);sensors(gap,1200,10);f=gap.updateVision({1,1100,{target()}});gap.select(1,f.boxes[0].track_id,1200);ck(!gap.selectedBearing(1200).valid,"gap in yaw history rejects guessed interpolation");
 cfg=configuration();cfg.follow_stereo_view=false;Controller view(cfg);sensors(view,1000);f=view.updateVision({1,1000,{target()}});view.select(1,f.boxes[0].track_id,1000);ck(view.selectedBearing(1000).reason=="view_not_supported","geometry cannot be applied to a different crop/view");
 Controller wrap(configuration());sensors(wrap,1000,179);sensors(wrap,1100,-179);f=wrap.updateVision({1,1050,{target(0,600)}});wrap.select(1,f.boxes[0].track_id,1100);auto b=wrap.selectedBearing(1100);double expected=std::atan(std::tan(std::atan(.2)+M_PI/180)-.06)*180/M_PI;ck(b.valid&&near(b.turn_deg,expected),"capture-yaw interpolation handles +/-180 wrap");
}
void wire(){
 const uint8_t mac[]={2,1,2,3,4,5};auto bytes=angleMotorFrame(mac,17,true,-1234,220,10,15);
 ck(bytes.size()==60&&!std::memcmp(bytes.data()+14,"MOT2",4)&&bytes[18]==2,"MOT2 magic/version/length");ck(bytes[24]==0xfb&&bytes[25]==0x2e&&bytes[26]==0&&bytes[27]==220,"signed centidegrees plus positive forward permille big endian");
 auto stop=angleMotorFrame(mac,18,false,-9000,220,10,15);ck(stop[19]==0&&stop[24]==0&&stop[25]==0&&stop[26]==0&&stop[27]==0,"disarmed MOT2 always zero");
 auto clipped=angleMotorFrame(mac,19,true,-10000,999,10,15);ck(clipped[24]==0xdc&&clipped[25]==0xd8&&clipped[27]==220,"wire hard bounds");
 // Build ACK2 independently from a duty ACK payload, never reinterpret angle as PWM.
 bytes=motorFrame(mac,17,true,-40,180,10,15);std::memset(bytes.data(),255,6);const uint8_t src[]={2,0xac,0x13,0x80,0,1};std::memcpy(bytes.data()+6,src,6);std::memcpy(bytes.data()+14,"ACK2",4);bytes[18]=2;bytes[19]=5;
 auto crc=motorCrc16(bytes.data()+14,16);bytes[30]=crc>>8;bytes[31]=crc;MotorAck ack;
 ck(decodeMotorAck(bytes.data(),bytes.size(),mac,ack)&&ack.protocol==2&&ack.left==-40&&ack.right==180,"ACK2 carries applied PWM, not requested angle");
 MotorAckWindow window;window.noteSent(17,1000,10,15,1);ck(!window.accept(ack,1010),"ACK2 cannot acknowledge a MOT1 sequence");window.noteSent(17,1000,10,15,2);ck(window.accept(ack,1010),"ACK2 must match actual MOT2 transmission");ck(!window.accept(ack,1020),"ACK2 replay cannot renew watchdog");
 bytes[18]=1;crc=motorCrc16(bytes.data()+14,16);bytes[30]=crc>>8;bytes[31]=crc;ck(!decodeMotorAck(bytes.data(),bytes.size(),mac,ack),"mixed ACK2 magic/version rejected even with valid CRC");
}
void referenceAndSensorAge(){
 Controller c(configuration());prepare(c);c.resetYawReference(1050);
 ck(c.state().armed&&!c.state().angle_mode,"applied zero reference stops movement");
 sensors(c,1060,0);c.select(1,c.vision().boxes[0].track_id,1060);
 ck(c.selectedBearing(1060).reason=="yaw_history_missing","pre-zero image cannot use post-zero yaw");
 auto f=c.updateVision({2,1050,{target()}});c.select(2,f.boxes[0].track_id,1060);
 ck(c.selectedBearing(1060).reason=="yaw_history_missing","same-millisecond reference/capture is ambiguous and rejected");
 f=c.updateVision({3,1060,{target()}});c.select(3,f.boxes[0].track_id,1060);
 ck(c.selectedBearing(1060).valid,"same-epoch image and yaw become usable");
 sensor_state raw={};raw.have_sample=1;raw.received_ms=1010;raw.sample.valid_mask=7;
 for(unsigned i=0;i<3;i++)raw.sample.distance_mm[i]=2000;
 raw.sample.age_ms[1]=490;auto ranges=followRanges(raw,1010);
 ck(ranges.valid[0]&&ranges.valid[1]&&ranges.valid[2]&&ranges.time_ms==520,"oldest true sonar time retained across publication");
 Controller age(configuration());prepare(age);age.updateRanges(ranges);age.tick(1010);
 ck(age.state().angle_mode,"490ms measurement is initially accepted");age.tick(1021);
 ck(!age.state().angle_mode&&age.state().reason=="follow_wait_sonar","measurement expires at real age500, not UI age500");
 raw.sample.age_ms[2]=501;ranges=followRanges(raw,1010);ck(!ranges.valid[2],"already expired sensor never accepted");
 raw.sample.age_ms[2]=65535;ranges=followRanges(raw,1010);ck(!ranges.valid[2],"underflow sentinel rejected");
 raw.sample.age_ms[2]=0;raw.sample.fault_mask=1;ranges=followRanges(raw,1010);ck(!ranges.valid[0],"sensor fault rejected independently of distance");
}
}
int main(){geometryControl();interlocks();wire();referenceAndSensorAge();std::cout<<"PASS angle follow "<<checks<<" checks\n";}
