#include "../backend/controller.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace robotphone;
namespace {
int checks=0;
void check(bool value, const char* message) {
  ++checks;
  if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
bool near(double a,double b,double e=1e-8) { return std::abs(a-b)<e; }
Config mapped() { Config c; c.left_motor_mask=3; c.slew_per_second=100; return c; }
void sensors(Controller& c,std::uint64_t t,double yaw=0,bool calibrated=true) {
  c.updateMotorLink(t,true); c.updateImu({t,calibrated,0,0,yaw});
}
Detection box(double cx=216,int eye=0,int cls=0,double size=80) {
  Detection d; d.class_id=cls; d.name="person"; d.score=.8; d.eye=eye;
  d.x1=cx-size/2; d.x2=cx+size/2; d.y1=180; d.y2=180+size;
  return d;
}
VisionFrame frame(std::uint64_t id,std::uint64_t t,std::vector<Detection> boxes) {
  return {id,t,std::move(boxes)};
}
void interlocks() {
  Controller c;
  sensors(c,1000);
  check(!c.arm(1,1000),"unmapped robot cannot arm");
  check(c.configure(mapped()),"valid map accepted");
  check(c.arm(1,1000),"fresh FPGA ACK permits arm");
  check(c.drive(2,1,.5,false,1000),"new manual command accepted");
  c.tick(1010);
  check(near(c.state().left,.6)&&near(c.state().right,.2),"differential normalization preserves ratio at 60% limit");
  check(!c.drive(2,-1,0,false,1010),"replay rejected");
  c.stop(1011,true);
  check(c.state().estop && !c.state().armed && c.state().left==0 && c.state().right==0,"emergency stop immediate and latched");
  check(!c.drive(3,1,0,false,1012),"drive cannot clear emergency stop");
  check(!c.arm(2,1012),"replayed arm cannot clear emergency stop");
  check(c.arm(3,1012) && !c.state().estop,"separate fresh arm clears latch");
  c.drive(4,1,0,false,1012); c.tick(1022);
  c.setForeground(false,1023);
  check(!c.state().armed && c.state().left==0,"background immediately disarms");
  check(!c.arm(5,1023),"cannot arm in background");
  c.setForeground(true,1024); check(c.arm(5,1024),"foreground requires new arm");
  c.drive(6,1,0,false,1024); c.tick(1034); c.ownerDisconnected(1035);
  check(!c.state().armed && c.state().left==0 && !c.state().target_id,"owner disconnect stops and clears target");
  sensors(c,1100); c.arm(7,1100); c.drive(8,1,0,false,1100); c.tick(1110);
  c.configure(mapped());
  check(!c.state().armed && c.state().left==0,"even same configuration disarms");
  c.arm(9,1110); c.drive(10,1,0,false,1110); c.tick(1120);
  c.updateMotorLink(1121,false);
  check(!c.state().armed && c.state().left==0,"negative motor readiness stops immediately");
}

void leasesAndSlew() {
  Config config=mapped(); config.slew_per_second=1;
  Controller c(config); sensors(c,1000); c.arm(1,1000); c.drive(2,1,.5,false,1000);
  c.tick(1100);
  check(near(c.state().left,.1)&&near(c.state().right,.1/3),"slew ramp preserves target ratio from stopped");
  c.drive(3,0,0,false,1101);
  check(c.state().left==0 && c.state().right==0,"zero joystick stops immediately without deceleration wait");
  check(c.state().armed && c.state().mode==Mode::Idle,"joystick release stays armed and idle");
  sensors(c,1400); c.tick(1400);
  check(c.state().armed && c.state().left==0,"idle released joystick has no moving command lease to expire");
  check(c.drive(4,1,0,false,1400),"fresh joystick gesture works without another arm after release");
  c.tick(1410);
  sensors(c,1698); c.tick(1698);
  check(c.state().armed,"fresh ACK alone does not prematurely expire lease");
  sensors(c,1751); c.tick(1751);
  check(!c.state().armed && c.state().reason=="lease_expired" && c.state().left==0,"sensor polling never extends command lease");
  sensors(c,1800); c.arm(5,1800); c.drive(6,1,0,false,1800); c.tick(1810);
  c.updateImu({2050,true,0,0,0}); c.drive(7,1,0,false,2050); c.tick(2101);
  check(!c.state().armed && c.state().reason=="motor_link_stale","missing actual FPGA ACK stops despite fresh control commands");
  sensors(c,2200); c.arm(8,2200); c.drive(9,1,0,false,2200); c.tick(2210); c.tick(2209);
  check(!c.state().armed && c.state().reason=="clock_reversed","monotonic clock reversal fails closed");
  sensors(c,2300); c.arm(10,2300);
  check(!c.drive(11,std::numeric_limits<double>::quiet_NaN(),0,false,2300) && !c.state().armed,"NaN command disarms");
  sensors(c,2400); c.arm(12,2400); c.drive(13,1,0,false,2400); c.tick(2410);
  sensors(c,2751);
  check(!c.drive(14,1,0,false,2751) && !c.state().armed && c.state().left==0,"late new command cannot revive an expired lease between ticks");
}

void heading() {
  Controller c(mapped()); sensors(c,1000,179); c.arm(1,1000);
  c.drive(2,.5,0,true,1000); c.tick(1010);
  sensors(c,1020,-179); c.drive(3,.5,0,true,1020); c.tick(1030);
  check(near(c.state().heading_error_deg,2),"yaw wrap produces +2 instead of -358 degrees");
  check(c.state().left>c.state().right,"positive yaw deviation corrects right");
  c.drive(4,-.5,0,true,1040); c.tick(1050);
  check(c.state().left>c.state().right && c.state().right<0,"reverse velocity does not invert heading correction");
  c.drive(5,.5,.4,true,1060); c.tick(1070);
  sensors(c,1080,135); c.drive(6,.5,0,true,1080); c.tick(1090);
  check(near(c.state().heading_error_deg,0) && near(c.state().left,c.state().right),"manual turn override recaptures heading on release");
  sensors(c,1100,140); c.drive(7,.5,0,true,1100); c.tick(1110);
  check(near(c.state().heading_error_deg,5),"new heading retained after override");
  c.updateMotorLink(1401,true); c.tick(1401);
  check(c.state().left==0 && c.state().reason=="imu_stale" && c.state().mode==Mode::Idle,"stale calibrated IMU stops heading mode");
  check(c.drive(8,.2,0,false,1402),"manual mode remains available without IMU");
  c.tick(1412); check(c.state().left>0,"manual motion does not pretend to require calibrated attitude");
  check(!c.drive(9,.2,0,true,1413) && c.state().left==0,"uncalibrated/stale attitude cannot enable hold");
  check(c.drive(10,0,0,true,1414) && c.state().armed && c.state().mode==Mode::Idle,"release with hold requested stops even when IMU is stale");
}

void tracking() {
  Controller c(mapped()); sensors(c,1000); c.arm(1,1000);
  auto a=c.updateVision(frame(1,1000,{box(150),box(300)}));
  const auto left=a.boxes[0].track_id,right=a.boxes[1].track_id;
  auto b=c.updateVision(frame(2,1030,{box(302),box(152)}));
  check(b.boxes[0].track_id==right && b.boxes[1].track_id==left,"reordered detection array preserves object identities");
  check(c.select(1,left,1040),"tap on retained recent displayed frame resolves to current ID");
  check(!c.select(999,left,1040),"unknown frame cannot select");
  check(!c.select(1,left,1501),"old displayed frame cannot select");
  check(c.state().target_id==left,"failed selection cannot silently replace existing target");
  c.select(2,left,1040); c.follow(2,1040); c.tick(1050);
  auto d=c.updateVision(frame(3,1060,{box(304)}));
  check(c.state().target_id==0 && c.state().left==0 && c.state().reason=="target_lost","missing selected target immediately stops");
  c.updateVision(frame(4,1090,{box(150),box(306)}));
  check(!c.follow(3,1100) && c.state().left==0,"reappearing object never automatically reacquires selected target");
  auto old=c.vision();
  c.updateVision(frame(3,900,{box(150)}));
  check(c.vision().frame_id==old.frame_id,"out of order vision cannot refresh freshness");

  Controller ambiguous(mapped());
  auto first=ambiguous.updateVision(frame(1,1000,{box(206),box(226)}));
  const auto selected=first.boxes[0].track_id;
  ambiguous.select(1,selected,1000);
  auto second=ambiguous.updateVision(frame(2,1030,{box(216)}));
  check(second.boxes[0].track_id!=first.boxes[0].track_id && second.boxes[0].track_id!=first.boxes[1].track_id,"ambiguous merge gets new identity");
  check(ambiguous.state().target_id==0,"ambiguous association clears selected target");

  Controller split(mapped());
  auto one=split.updateVision(frame(1,1000,{box(216)}));
  auto two=split.updateVision(frame(2,1030,{box(206),box(226)}));
  check(two.boxes[0].track_id!=one.boxes[0].track_id && two.boxes[1].track_id!=one.boxes[0].track_id,"ambiguous split never duplicates track ID");
  auto changed=split.updateVision(frame(3,1060,{box(206,0,1)}));
  check(changed.boxes[0].track_id!=two.boxes[0].track_id,"different class cannot inherit target ID");
  auto gap=split.updateVision(frame(4,1500,{box(206,0,1)}));
  check(gap.boxes[0].track_id!=changed.boxes[0].track_id,"long tracking gap cannot silently retain identity");

  Controller moving(mapped()); sensors(moving,2000); moving.arm(1,2000);
  moving.drive(2,.5,0,true,2000); moving.tick(2010);
  auto tap=moving.updateVision(frame(1,2010,{box(216)}));
  check(moving.select(1,tap.boxes[0].track_id,2011) && moving.state().armed &&
        moving.state().left==0 && moving.state().mode==Mode::Idle,
        "valid new target tap cancels prior moving command without starting follow");
  moving.tick(2020);
  check(moving.state().left==0,"selecting a box cannot resume previous heading motion");
}

void following() {
  Controller c(mapped()); sensors(c,1000); c.arm(1,1000);
  auto f=c.updateVision(frame(1,1000,{box(648,1)}));
  check(near(f.boxes[0].center_x,0),"right-eye optical center is not interpreted as right steering");
  c.select(1,f.boxes[0].track_id,1000); check(c.follow(2,1000),"valid tap allows follow"); c.tick(1010);
  check(near(c.state().left,.22) && near(c.state().right,.22),"centered target approaches slowly at configured duty");
  c.stop(1011); check(!c.state().target_id,"stop clears target");
  check(!c.drive(3,1,0,false,1012) && c.state().left==0,"explicit normal stop requires separate arm before further drive");
  sensors(c,1100); c.arm(3,1100);
  f=c.updateVision(frame(2,1100,{box(820,1)}));
  c.select(2,f.boxes[0].track_id,1100); c.follow(4,1100); c.tick(1110);
  check(c.state().left>0 && c.state().right<0 && near(c.state().left,-c.state().right),"off-center target turns in place before approaching");
  c.stop(1111); sensors(c,1200); c.arm(5,1200);
  auto large=box(216); large.y1=100; large.y2=400;
  f=c.updateVision(frame(3,1200,{large})); c.select(3,f.boxes[0].track_id,1200); c.follow(6,1200); c.tick(1210);
  check(c.state().left==0 && !c.state().target_id && c.state().reason=="target_reached","large image target stops and requires a fresh selection");
  sensors(c,1300); f=c.updateVision(frame(4,1300,{box(216)}));
  c.select(4,f.boxes[0].track_id,1300); c.follow(7,1300); c.tick(1310);
  sensors(c,1500); c.follow(8,1500); c.tick(1510);
  sensors(c,1700); c.follow(9,1700); c.tick(1710);
  sensors(c,1801); c.tick(1801);
  check(c.state().left==0 && !c.state().target_id,"vision expiry stops and clears target");
  check(c.state().reason=="target_lost" && c.state().armed,"vision expiry is checked independently from fresh IMU/ACK/control lease");
  check(!c.follow(10,1802),"expired vision cannot renew follow command");
  sensors(c,1900,0,false); f=c.updateVision(frame(5,1900,{box(216)}));
  c.select(5,f.boxes[0].track_id,1900);
  check(!c.follow(11,1900) && c.state().reason=="imu_stale","follow requires calibrated fresh attitude");
}
}  // namespace

int main() {
  interlocks(); leasesAndSlew(); heading(); tracking(); following();
  std::cout << "PASS controller: " << checks << " checks (tracking, differential, heading, leases, ACK, stop/rearm)\n";
}
