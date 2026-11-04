#include "../phone/controller.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace robotphone;
static unsigned checks;
static void check(bool ok,const char* message){++checks;if(!ok){std::cerr<<"FAIL "<<message<<'\n';std::exit(1);}}
static Config config(){Config c;c.left_motor_mask=10;c.slew_per_second=100;return c;}
static void ready(Controller& c,uint64_t now){c.updateMotorLink(now,true);c.updateArcProtocol(true);c.updateVelocityProtocol(true,true);c.updateImu({now,true,0,0,0});}
static bool zero(const State& s){return s.left==0&&s.right==0&&s.linear_mps==0&&s.angular_rps==0;}
int main(){
 Controller c(config());ready(c,1000);check(c.arm(1,1000),"arm starts authorization");
 check(c.state().authorization_ms==1000,"arm source recorded");check(c.drive(2,.5,0,false,1000),"drive accepted");c.tick(1010);check(c.state().left>0,"valid action moves");
 ready(c,1351);c.tick(1351);check(c.state().armed&&c.state().motion_paused&&zero(c.state()),"351ms action age zeros but stays logically armed");
 ready(c,1600);c.tick(1600);check(zero(c.state())&&c.state().mode==Mode::Idle,"hardware recovery never replays old action");
 check(c.idle(3,2000),"valid owner idle renews lease");check(c.state().authorization_ms==2000,"idle is owner command");
 ready(c,31999);c.tick(31999);check(c.state().armed,"29999ms from accepted idle still armed");
 ready(c,32000);c.tick(32000);check(!c.state().armed&&zero(c.state())&&c.state().reason=="command_timeout_30s","exact 30000ms revokes logical authority");
 check(!c.drive(4,.5,0,false,32001),"late drive cannot rearm");ready(c,32002);check(c.arm(5,32002),"new explicit arm required after expiry");
 check(!c.drive(5,.5,0,false,32100),"same sequence rejected");check(c.state().authorization_ms==32002,"replay does not renew lease");
 check(!c.drive(6,std::numeric_limits<double>::quiet_NaN(),0,false,32101),"invalid command rejected");check(c.state().armed&&zero(c.state())&&c.state().authorization_ms==32002,"invalid command zeros without revoking or renewing");
 c.updateMotorLink(32102,false);check(c.state().armed&&c.state().motion_paused&&zero(c.state()),"negative ACK pauses");
 check(c.drive(7,.5,0,false,32103),"valid blocked intent acknowledged as paused");check(c.state().authorization_ms==32103&&zero(c.state()),"fresh owner blocked by ACK renews authority only");
 ready(c,32110);c.tick(32110);check(zero(c.state()),"new ACK alone cannot restore blocked throttle");
 check(c.drive(8,.5,0,false,32111),"fresh current owner action after recovery");c.tick(32121);check(c.state().left>0,"fresh action resumes");
 c.updateArcProtocol(false);check(c.state().armed&&zero(c.state()),"MOT3 capability loss pauses");ready(c,32130);c.tick(32130);check(zero(c.state()),"protocol recovery does not replay");
 c.setForeground(false,32131);check(c.state().armed&&zero(c.state()),"focus loss only pauses");c.setForeground(true,32132);check(zero(c.state()),"focus return never replays");
 c.resetYawReference(32133);check(c.state().armed&&zero(c.state()),"IMU zero retains authorization");
 check(c.configure(config())&&c.state().armed&&zero(c.state()),"wheel configuration pauses without logical disarm");
 Config bad=config();bad.left_motor_mask=0;check(!c.configure(bad)&&c.state().armed&&c.config().left_motor_mask==10,"invalid config preserves working map and logical arm");
 ready(c,32200);check(c.driveVelocity(9,.2,.1,32200,32111),"internal navigation physical units accepted");check(c.state().authorization_ms==32111,"internal command carries last real phone time");
 for(unsigned i=0;i<100;i++){const auto t=32300+i*20;ready(c,t);check(c.driveVelocity(10+i,.2,.1,t,32111),"periodic navigation accepted under unchanged real owner time");}
 check(c.state().authorization_ms==32111,"100 internal ticks never extend phone authorization");
 ready(c,62110);c.tick(62110);check(c.state().armed&&zero(c.state()),"29999ms retains arm even when motion expired");ready(c,62111);c.tick(62111);check(!c.state().armed,"internal ticks do not defeat 30s timeout");
 ready(c,63000);check(c.arm(200,63000),"explicit rearm");check(c.lease(201,63100,63100),"private owner lease accepted");
 check(!c.lease(201,63200,63200)&&c.state().authorization_ms==63100,"private lease replay cannot renew");check(!c.lease(202,63200,63201)&&c.state().authorization_ms==63100,"future source rejected");
 check(c.driveVelocity(203,.2,0,63200,63100),"new nav command");c.updateVelocityProtocol(true,false);check(c.state().armed&&zero(c.state())&&c.state().motion_paused,"softcore calibration wait zeros only");
 ready(c,63210);c.tick(63210);check(zero(c.state()),"softcore recovery cannot restart old command");
 c.commandSourceTime(63211);check(c.idle(204,63300,63211),"trusted source accepts current real owner time");c.commandSourceTime();check(c.state().authorization_ms==63211,"loopback transport time does not extend lease");
 c.stop(63301);check(!c.state().armed&&zero(c.state()),"explicit stop revokes");check(!c.idle(205,63302),"idle cannot rearm");ready(c,63400);check(c.arm(206,63400),"explicit arm again");c.ownerDisconnected(63401);check(!c.state().armed,"disconnect revokes");
 std::cout<<"PASS v36 control authorization "<<checks<<" checks: 30s boundary, short motion TTL, sensor/link pauses, internal-owner clock, replay/invalid protection and explicit stop/disconnect\n";
}
