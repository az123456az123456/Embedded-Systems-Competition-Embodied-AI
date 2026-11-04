// Production MotorLink socket dispatch + Controller + ACK window. Only the
// AF_PACKET endpoint is replaced; no hardware socket or motor is accessed.
#include "../phone/motor_link.h"
#include "../phone/controller.h"
#include <arpa/inet.h>
#include <cassert>
#include <cerrno>
#include <cstdarg>
#include <cstring>
#include <deque>
#include <iostream>
#include <linux/if_packet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
using namespace robotphone;
namespace {
constexpr int motor_fd=12345,nav_fd=12346;
constexpr uint8_t host[]={2,1,2,3,4,5},fpga[]={2,0xac,0x13,0x80,0,1};
unsigned checks=0,sends=0,nav_sends=0;
std::deque<std::vector<uint8_t>> motor_rx,nav_rx;
std::vector<uint8_t> sent;
void check(bool value){++checks;assert(value);}
void put16(uint8_t*p,uint16_t v){p[0]=v>>8;p[1]=v;}
void put32(uint8_t*p,uint32_t v){for(unsigned i=0;i<4;++i)p[i]=v>>(24-8*i);}
std::vector<uint8_t> nav(uint32_t beat,uint8_t flags=3,uint16_t error=0){
 std::vector<uint8_t>b(60);std::memset(b.data(),255,6);std::memcpy(b.data()+6,fpga,6);b[12]=0x88;b[13]=0xb7;
 auto*p=b.data()+14;std::memcpy(p,"NAV1",4);p[4]=1;p[5]=flags;put16(p+6,127);put32(p+8,beat);
 put32(p+12,150000);put32(p+16,4096);put32(p+20,1234);put16(p+24,error);put16(p+26,motorCrc16(p,26));return b;
}
std::vector<uint8_t> ack(bool ready){
 auto b=sent;std::memset(b.data(),255,6);std::memcpy(b.data()+6,fpga,6);std::memcpy(b.data()+14,"ACK4",4);
 b[19]=(b[19]&1)|4|(ready?8:0);put16(b.data()+24,0);put16(b.data()+26,0);put16(b.data()+30,motorCrc16(b.data()+14,16));return b;
}
std::vector<uint8_t> oldAck(uint32_t sequence,bool configured=true){
 auto b=motorFrame(host,sequence,false,0,0,10,15);std::memset(b.data(),255,6);std::memcpy(b.data()+6,fpga,6);
 std::memcpy(b.data()+14,"ACK1",4);b[18]=1;b[19]=configured?4:0;put16(b.data()+30,motorCrc16(b.data()+14,16));return b;
}
uint32_t sequenceOf(const std::vector<uint8_t>& b){return uint32_t(b[20])<<24|uint32_t(b[21])<<16|uint32_t(b[22])<<8|b[23];}
void restartSequenceCase(uint32_t previous){
 MotorLink restarted;std::string error;check(restarted.open("test0",error));
 auto old=oldAck(previous);auto corrupt=old;corrupt[30]^=1;motor_rx.push_back(corrupt);restarted.receive(20000);
 check(restarted.diagnostics().configuration_seeds==0&&!restarted.fresh(20000));
 corrupt=old;corrupt[6]^=1;motor_rx.push_back(corrupt);restarted.receive(20001);check(restarted.diagnostics().configuration_seeds==0);
 motor_rx.push_back(old);restarted.receive(20002);
 check(restarted.diagnostics().configuration_seeds==1&&restarted.diagnostics().last_seed_sequence==previous);
 check(restarted.configurationNeeded()&&!restarted.fresh(20002)&&restarted.diagnostics().ack_frames==0&&!restarted.velocitySupported());
 check(!restarted.sendVelocity(true,.2,0,10,15,20003));check(!restarted.sendVelocity(true,0,0,10,15,20003)); // an old status can only authorize ZERO configuration
 check(restarted.sendVelocity(false,0,0,10,15,20004));check(sequenceOf(sent)==uint32_t(previous+1));
 check((sent[19]&1)==0&&sent[24]==0&&sent[25]==0&&sent[26]==0&&sent[27]==0);
 // FPGA's actual signed-modular newer rule, including ffffffff -> 0.
 const auto delta=sequenceOf(sent)-previous;check(delta&&!(delta&0x80000000u));
 motor_rx.push_back(ack(false));restarted.receive(20014);
 check(restarted.fresh(20014)&&!restarted.configurationNeeded()&&restarted.diagnostics().ack_frames==1);
 check(restarted.sendVelocity(true,0,0,10,15,20015));check(sequenceOf(sent)==uint32_t(previous+2));motor_rx.push_back(ack(true));restarted.receive(20025);
 check(restarted.softcoreReady(20025)); // recovery takes23ms, inside both500ms and30s
 Controller unarmed([]{Config c;c.left_motor_mask=10;return c;}());unarmed.updateMotorLink(20025,restarted.fresh(20025));
 check(!unarmed.state().armed&&!unarmed.driveVelocity(1,.2,0,20025)); // ACK cannot supply a phone arm
 // Neither delayed older statuses nor a much larger old status may reseed an
 // already-confirmed process, even when the fresh ACK window later expires.
 motor_rx.push_back(oldAck(previous-1,false));restarted.receive(20400);
 check(restarted.configurationNeeded()&&!restarted.fresh(20400)&&restarted.diagnostics().configuration_seeds==1);
 motor_rx.push_back(oldAck(previous+1000,true));restarted.receive(20401);check(restarted.diagnostics().configuration_seeds==1);
 check(restarted.sendVelocity(false,0,0,10,15,20402));check(sequenceOf(sent)==uint32_t(previous+3));
 motor_rx.push_back(ack(false));restarted.receive(20412);check(restarted.fresh(20412));
 motor_rx.push_back(old);restarted.receive(20413);check(restarted.fresh(20413)&&restarted.diagnostics().configuration_seeds==1);
 check(restarted.sendVelocity(true,0,0,10,15,20414));check(sequenceOf(sent)==uint32_t(previous+4));
 restarted.close();
}

}
extern "C" unsigned __wrap_if_nametoindex(const char*){return 7;}
extern "C" int __wrap_socket(int domain,int,int protocol){assert(domain==AF_PACKET);return ntohs(protocol)==0x88b6?motor_fd:nav_fd;}
extern "C" int __wrap_bind(int fd,const sockaddr*a,socklen_t){const auto*s=reinterpret_cast<const sockaddr_ll*>(a);assert(s->sll_ifindex==7);assert(ntohs(s->sll_protocol)==(fd==motor_fd?0x88b6:0x88b7));return 0;}
extern "C" int __wrap_ioctl(int fd,unsigned long request,...){assert(fd==motor_fd&&request==SIOCGIFHWADDR);va_list args;va_start(args,request);auto*r=va_arg(args,ifreq*);va_end(args);std::memcpy(r->ifr_hwaddr.sa_data,host,6);return 0;}
extern "C" ssize_t __wrap_sendto(int fd,const void*b,size_t n,int,const sockaddr*,socklen_t){++sends;if(fd==nav_fd)++nav_sends;assert(fd==motor_fd);sent.assign(static_cast<const uint8_t*>(b),static_cast<const uint8_t*>(b)+n);return n;}
extern "C" ssize_t __wrap_recvfrom(int fd,void*b,size_t n,int,sockaddr*a,socklen_t*){auto&q=fd==motor_fd?motor_rx:nav_rx;if(q.empty()){errno=EAGAIN;return -1;}auto v=q.front();q.pop_front();assert(v.size()<=n);std::memcpy(b,v.data(),v.size());reinterpret_cast<sockaddr_ll*>(a)->sll_pkttype=PACKET_HOST;return v.size();}
extern "C" int __wrap_close(int fd){assert(fd==motor_fd||fd==nav_fd);return 0;}
int main(){
 SoftcoreStatus decoded;auto b=nav(42);
 check(decodeSoftcoreStatus(b.data(),b.size(),host,decoded));
 check(decoded.flags==3&&decoded.heartbeat==42&&decoded.path_length==127&&decoded.astar_cycles==150000&&decoded.expanded==4096&&decoded.imu_sequence==1234);
 for(size_t n=0;n<42;++n)check(!decodeSoftcoreStatus(b.data(),n,host,decoded));
 check(!decodeSoftcoreStatus(nullptr,60,host,decoded));
 for(size_t i=14;i<42;++i){auto bad=b;bad[i]^=1;check(!decodeSoftcoreStatus(bad.data(),bad.size(),host,decoded));}
 for(size_t i:{size_t(0),size_t(6),size_t(12)}){auto bad=b;bad[i]^=1;check(!decodeSoftcoreStatus(bad.data(),bad.size(),host,decoded));}
 auto bad=nav(42,0x13);check(!decodeSoftcoreStatus(bad.data(),bad.size(),host,decoded));
 b.resize(64,0x5a);check(decodeSoftcoreStatus(b.data(),b.size(),host,decoded));
 SoftcoreMonitor monitor;monitor.socketState(true);check(monitor.reason(1000,false)=="softcore_diagnostics_waiting_nav1");
 b=nav(42);check(monitor.accept(b.data(),b.size(),host,1000));check(!monitor.heartbeatLive(1000));
 check(monitor.reason(1000,false)=="softcore_heartbeat_unverified");
 check(monitor.accept(b.data(),b.size(),host,1100));check(!monitor.heartbeatLive(1100));
 b=nav(43);check(monitor.accept(b.data(),b.size(),host,1200));check(monitor.heartbeatLive(1200));
 check(monitor.reason(1200,false)=="softcore_local_calibration_or_commit_not_ready");
 check(monitor.reason(1200,true)=="ready");
 check(monitor.reason(1200,false,false)=="softcore_idle_protocol_probe");
 check(monitor.json(1200,false,false).find("softcore_idle_protocol_probe")!=std::string::npos);
 check(monitor.reason(1200,false,true)=="softcore_local_calibration_or_commit_not_ready");
 b=nav(43,7);monitor.accept(b.data(),b.size(),host,1300);check(monitor.reason(1300,true)=="softcore_cpu_trap");check(monitor.reason(1300,false,false)=="softcore_cpu_trap");
 b=nav(43,11,0x1001);monitor.accept(b.data(),b.size(),host,1400);check(monitor.reason(1400,true)=="softcore_firmware_fault");
 b=nav(43,1);monitor.accept(b.data(),b.size(),host,1500);check(monitor.reason(1500,false)=="softcore_selftest_pending");
 b=nav(43,3);monitor.accept(b.data(),b.size(),host,5000);check(monitor.reason(5000,false)=="softcore_heartbeat_stalled");
 check(monitor.reason(5501,false)=="softcore_diagnostics_stale");
 b=nav(1);monitor.accept(b.data(),b.size(),host,5600);check(monitor.status().resets==1&&!monitor.heartbeatLive(5600));
 b=nav(2);monitor.accept(b.data(),b.size(),host,5700);check(monitor.heartbeatLive(5700));
 monitor.clear();monitor.socketState(true);b=nav(0xffffffffu);monitor.accept(b.data(),b.size(),host,6000);
 b=nav(0);monitor.accept(b.data(),b.size(),host,6100);check(monitor.heartbeatLive(6100)&&monitor.status().resets==0);
 // Actual two-socket dispatch: NAV1 can observe a healthy CPU, but cannot
 // establish motor liveness, set capability, arm, or satisfy an ACK sequence.
 MotorLink link;std::string error;check(link.open("test0",error));
 nav_rx.push_back(nav(1));link.receive(10000);nav_rx.push_back(nav(2));link.receive(10100);
 check(link.softcore().heartbeatLive(10100)&&!link.fresh(10100)&&!link.velocitySupported()&&!link.softcoreReady(10100));
 check(link.diagnostics().ack_frames==0&&link.diagnostics().rx_frames==0);
 check(link.sendVelocity(false,0,0,10,15,10100));motor_rx.push_back(ack(false));link.receive(10110);
 check(link.fresh(10110)&&link.velocitySupported()&&!link.softcoreReady(10110));
 Config c;c.left_motor_mask=10;Controller controller(c);controller.updateMotorLink(10110,link.fresh(10110));controller.updateVelocityProtocol(true,false);
 check(controller.arm(1,10110));check(controller.driveVelocity(2,0,0,10110));check(controller.state().armed);
 check(link.sendVelocity(true,0,0,10,15,10150));motor_rx.push_back(ack(true));link.receive(10160);
 check(link.softcoreReady(10160));controller.updateMotorLink(10160,true);controller.updateVelocityProtocol(true,true);
 check(controller.driveVelocity(3,.3,.1,10160));check(controller.state().linear_mps==.3);
 // An unsolicited NAV1 containing a larger heartbeat never refreshes ACK4.
 nav_rx.push_back(nav(99));link.receive(10461);check(!link.fresh(10461)&&!link.softcoreReady(10461));
 controller.updateMotorLink(10461,false);check(controller.tick(10461).armed&&controller.state().linear_mps==0);
 // Fresh ACK4 ready=0 remains authoritative even when NAV1 selftest/heartbeat
 // is healthy, and exact ACK4 replay cannot extend the command lifetime.
 check(link.sendVelocity(false,0,0,10,15,10500));auto fresh=ack(false);motor_rx.push_back(fresh);nav_rx.push_back(nav(100));link.receive(10510);
 check(link.fresh(10510)&&!link.softcoreReady(10510));motor_rx.push_back(fresh);link.receive(10801);check(!link.fresh(10801));
 // Rejected ARM may echo the OLD sequence. It is a configuration hint only.
 auto stale_unconfigured=fresh;stale_unconfigured[19]=0;put16(stale_unconfigured.data()+30,motorCrc16(stale_unconfigured.data()+14,16));
 const auto ack_count=link.diagnostics().ack_frames;motor_rx.push_back(stale_unconfigured);link.receive(11000);
 check(link.configurationNeeded()&&!link.fresh(11000));check(link.diagnostics().ack_frames==ack_count);check(link.diagnostics().configuration_hints==1);
 check(!link.sendVelocity(true,.2,0,10,15,11001)); // hint never permits movement
 check(link.sendVelocity(false,0,0,10,15,11002));motor_rx.push_back(ack(false));link.receive(11003);
 check(!link.configurationNeeded()&&link.fresh(11003));check(!controller.state().twist_mode&&controller.state().armed); // recovery is zero only
 check(link.sendVelocity(true,0,0,10,15,11004));motor_rx.push_back(ack(true));link.receive(11005);
 check(link.softcoreReady(11005));check(controller.state().linear_mps==0); // a NEW owner action is required
 check(nav_sends==0&&sends==5);check(link.softcore().status().frames==4);
 link.close();check(!link.softcore().status().seen&&!link.softcore().status().socket_open);
 restartSequenceCase(0x70001000u);restartSequenceCase(0xffffffffu);restartSequenceCase(0x80000010u);
 std::cout<<"PASS "<<checks<<" production NAV1/ACK4 lifecycle checks: startup sequence rebase, zero-only config, wrap/late ACK isolation, CRC/MAC/version, heartbeat/reset/wrap, CPU faults, independent sockets, calibration zero, strict ACK4 and lease loss; no physical hardware\n";
}
