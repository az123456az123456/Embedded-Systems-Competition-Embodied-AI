#include "motor_link.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <linux/if_packet.h>
#include <net/if.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
namespace robotphone {
static const uint8_t fpga_mac[]={2,0xac,0x13,0x80,0,1};
static unsigned u16(const uint8_t *p){return (p[0]<<8)|p[1];}
static uint32_t u32(const uint8_t *p){return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(p[2]<<8)|p[3];}
static int signed16(const uint8_t *p){unsigned n=u16(p);return n<32768?int(n):int(n)-65536;}
static void put16(uint8_t *p,unsigned n){p[0]=n>>8;p[1]=n;}
uint16_t motorCrc16(const uint8_t *p,size_t n){uint16_t c=65535;for(size_t i=0;i<n;i++){c^=uint16_t(p[i])<<8;for(int b=0;b<8;b++)c=(c&0x8000)?(c<<1)^0x1021:c<<1;}return c;}
std::vector<uint8_t> motorFrame(const uint8_t source[6],uint32_t seq,bool armed,int left,int right,unsigned mask,unsigned invert){
 std::vector<uint8_t> b(60,0);std::memcpy(b.data(),fpga_mac,6);std::memcpy(b.data()+6,source,6);b[12]=0x88;b[13]=0xb6;
 auto *p=b.data()+14;std::memcpy(p,"MOT1",4);p[4]=1;p[5]=armed?1:0;
 for(int i=0;i<4;i++)p[6+i]=seq>>(24-i*8);
 put16(p+10,armed?std::max(-600,std::min(600,left)):0);put16(p+12,armed?std::max(-600,std::min(600,right)):0);
 p[14]=mask;p[15]=invert;put16(p+16,motorCrc16(p,16));return b;
}
std::vector<uint8_t> angleMotorFrame(const uint8_t source[6],uint32_t seq,bool armed,int angle,int forward,unsigned mask,unsigned invert){
 auto b=motorFrame(source,seq,false,0,0,mask,invert);auto *p=b.data()+14;
 std::memcpy(p,"MOT2",4);p[4]=2;p[5]=armed?1:0;
 put16(p+10,armed?std::max(-9000,std::min(9000,angle)):0);
 put16(p+12,armed?std::max(0,std::min(220,forward)):0);
 put16(p+16,motorCrc16(p,16));return b;
}
std::vector<uint8_t> arcMotorFrame(const uint8_t source[6],uint32_t seq,bool armed,int outer,int curve,unsigned mask,unsigned invert){
 auto b=motorFrame(source,seq,false,0,0,mask,invert);auto *p=b.data()+14;
 std::memcpy(p,"MOT3",4);p[4]=3;p[5]=armed?1:0;
 put16(p+10,armed?std::max(-600,std::min(600,outer)):0);
 put16(p+12,armed?std::max(-256,std::min(256,curve)):0);
 put16(p+16,motorCrc16(p,16));return b;
}
std::vector<uint8_t> velocityMotorFrame(const uint8_t source[6],uint32_t seq,bool armed,int linear,int angular,unsigned mask,unsigned invert){
 auto b=motorFrame(source,seq,false,0,0,mask,invert);auto *p=b.data()+14;
 std::memcpy(p,"MOT4",4);p[4]=4;p[5]=armed?1:0;
 put16(p+10,armed?std::max(-300,std::min(300,linear)):0);
 put16(p+12,armed?std::max(-1000,std::min(1000,angular)):0);
 put16(p+16,motorCrc16(p,16));return b;
}
bool decodeMotorAck(const uint8_t *b,size_t n,const uint8_t host[6],MotorAck &out){
 if(!b||!host||(n!=60&&n!=64))return false;
 static const uint8_t broadcast[]={255,255,255,255,255,255};
 if((std::memcmp(b,host,6)&&std::memcmp(b,broadcast,6))||std::memcmp(b+6,fpga_mac,6)||b[12]!=0x88||b[13]!=0xb6)return false;
 const uint8_t *p=b+14;
 const unsigned protocol=!std::memcmp(p,"ACK1",4)&&p[4]==1?1:!std::memcmp(p,"ACK2",4)&&p[4]==2?2:!std::memcmp(p,"ACK3",4)&&p[4]==3?3:!std::memcmp(p,"ACK4",4)&&p[4]==4?4:0;
 if(!protocol||(p[5]&~(protocol==4?15u:7u))||motorCrc16(p,16)!=u16(p+16))return false;
 for(size_t i=32;i<60;i++)if(b[i])return false;
 int left=signed16(p+10),right=signed16(p+12);
 if(std::abs(left)>600||std::abs(right)>600||p[14]==0||p[14]>=15||p[15]>15)return false;
 if(n==64){uint32_t crc=~0u;for(size_t i=0;i<60;i++){crc^=b[i];for(int bit=0;bit<8;bit++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}uint32_t f=b[60]|(uint32_t(b[61])<<8)|(uint32_t(b[62])<<16)|(uint32_t(b[63])<<24);if(~crc!=f)return false;}
 out.sequence=u32(p+6);out.armed=p[5]&1;out.watchdog=p[5]&2;out.configured=p[5]&4;out.left=left;out.right=right;out.left_mask=p[14];out.invert_mask=p[15];out.protocol=protocol;out.softcore_ready=protocol==4&&(p[5]&8);return true;
}
bool decodeSoftcoreStatus(const uint8_t *b,size_t n,const uint8_t host[6],SoftcoreStatus &out){
 if(!b||!host||n<42||n>64)return false;
 static const uint8_t broadcast[]={255,255,255,255,255,255};
 if((std::memcmp(b,host,6)&&std::memcmp(b,broadcast,6))||std::memcmp(b+6,fpga_mac,6)||b[12]!=0x88||b[13]!=0xb7)return false;
 const uint8_t *p=b+14;
 // NAV1 is 26 data bytes plus its own CRC16. Ethernet padding/FCS is outside
 // that CRC and may be retained or removed by the host NIC.
 if(std::memcmp(p,"NAV1",4)||p[4]!=1||(p[5]&0xf0)||motorCrc16(p,26)!=u16(p+26))return false;
 SoftcoreStatus s;s.flags=p[5];s.path_length=u16(p+6);s.heartbeat=u32(p+8);
 s.astar_cycles=u32(p+12);s.expanded=u32(p+16);s.imu_sequence=u32(p+20);s.firmware_error=u16(p+24);out=s;return true;
}
bool SoftcoreMonitor::accept(const uint8_t *b,size_t n,const uint8_t host[6],uint64_t now){
 SoftcoreStatus decoded;
 if(!decodeSoftcoreStatus(b,n,host,decoded)){++status_.rejected;return false;}
 if(status_.seen&&decoded.heartbeat!=status_.heartbeat){
  const uint32_t delta=decoded.heartbeat-status_.heartbeat;
  if(!(delta&0x80000000u)){++status_.advances;status_.last_advance_ms=now;}
  else {++status_.resets;status_.last_advance_ms=0;}
 }
 status_.flags=decoded.flags;status_.path_length=decoded.path_length;status_.heartbeat=decoded.heartbeat;
 status_.astar_cycles=decoded.astar_cycles;status_.expanded=decoded.expanded;status_.imu_sequence=decoded.imu_sequence;
 status_.firmware_error=decoded.firmware_error;status_.received_ms=now;status_.seen=true;++status_.frames;return true;
}
bool SoftcoreMonitor::packetFresh(uint64_t now)const{
 return status_.seen&&now>=status_.received_ms&&now-status_.received_ms<=500;
}
bool SoftcoreMonitor::heartbeatLive(uint64_t now)const{
 return status_.advances&&status_.last_advance_ms&&now>=status_.last_advance_ms&&now-status_.last_advance_ms<=3500;
}
std::string SoftcoreMonitor::reason(uint64_t now,bool ack_ready,bool autonomous_requested)const{
 if(!status_.socket_open)return "softcore_diagnostics_socket_unavailable";
 if(!status_.seen)return "softcore_diagnostics_waiting_nav1";
 if(!packetFresh(now))return "softcore_diagnostics_stale";
 if(status_.flags&4)return "softcore_cpu_trap";
 if((status_.flags&8)||status_.firmware_error)return "softcore_firmware_fault";
 if(!(status_.flags&1))return "softcore_boot_not_ready";
 if(!(status_.flags&2))return "softcore_selftest_pending";
 if(!status_.last_advance_ms)return "softcore_heartbeat_unverified";
 if(!heartbeatLive(now))return "softcore_heartbeat_stalled";
 if(!status_.imu_sequence)return "softcore_local_imu_not_observed";
 if(!ack_ready)return autonomous_requested?"softcore_local_calibration_or_commit_not_ready":"softcore_idle_protocol_probe";
 return "ready";
}
std::string SoftcoreMonitor::json(uint64_t now,bool ack_ready,bool autonomous_requested)const{
 const auto &s=status_;std::ostringstream o;
 o<<"{\"protocol\":\"NAV1\",\"passive\":true,\"socket_open\":"<<(s.socket_open?"true":"false")
  <<",\"received\":"<<(s.seen?"true":"false")<<",\"packet_fresh\":"<<(packetFresh(now)?"true":"false")
  <<",\"sample_mono_ms\":"<<s.received_ms<<",\"packet_age_ms\":"<<(s.seen&&now>=s.received_ms?std::to_string(now-s.received_ms):"null")
  <<",\"flags\":"<<unsigned(s.flags)<<",\"firmware_ready\":"<<((s.flags&1)?"true":"false")
  <<",\"selftest_ok\":"<<((s.flags&2)?"true":"false")<<",\"cpu_trap\":"<<((s.flags&4)?"true":"false")
  <<",\"firmware_failed\":"<<((s.flags&8)?"true":"false")<<",\"firmware_error\":"<<s.firmware_error
  <<",\"heartbeat\":"<<s.heartbeat<<",\"heartbeat_advances\":"<<s.advances<<",\"heartbeat_resets\":"<<s.resets
  <<",\"heartbeat_advance_mono_ms\":"<<s.last_advance_ms
  <<",\"heartbeat_live\":"<<(heartbeatLive(now)?"true":"false")
  <<",\"heartbeat_age_ms\":"<<(s.last_advance_ms&&now>=s.last_advance_ms?std::to_string(now-s.last_advance_ms):"null")
  <<",\"observed_imu_sequence\":"<<s.imu_sequence<<",\"path_length\":"<<s.path_length<<",\"astar_cycles\":"<<s.astar_cycles
  <<",\"expanded\":"<<s.expanded<<",\"frames\":"<<s.frames<<",\"rejected\":"<<s.rejected
  <<",\"last_errno\":"<<s.last_errno<<",\"matching_ack4_ready\":"<<(ack_ready?"true":"false")
  <<",\"reason\":\""<<reason(now,ack_ready,autonomous_requested)<<"\",\"detail_limit\":\"NAV1 cannot distinguish local calibration, stale control commit, or local IMU age\"}";
 return o.str();
}
MotorLink::~MotorLink(){close();}
void MotorLink::close(){if(fd_>=0)::close(fd_);if(nav_fd_>=0)::close(nav_fd_);fd_=nav_fd_=-1;window_.clear();softcore_.clear();angle_supported_=arc_supported_=velocity_supported_=false;configuration_needed_=true;configuration_verified_=false;sequence_seeded_=false;softcore_ack_ms_=0;softcore_ready_=false;}
bool MotorLink::open(const std::string &interface,std::string &error){
 close();diagnostics_={};if(interface.empty()||interface.size()>=IFNAMSIZ){error="Invalid FPGA interface";return false;}
 index_=if_nametoindex(interface.c_str());if(!index_){error="FPGA network interface not found";return false;}
 fd_=socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(0x88b6));
 if(fd_<0){error=std::strerror(errno);return false;}
 ifreq request={};std::strncpy(request.ifr_name,interface.c_str(),IFNAMSIZ-1);
 if(ioctl(fd_,SIOCGIFHWADDR,&request)){error=std::strerror(errno);close();return false;}
 std::memcpy(mac_,request.ifr_hwaddr.sa_data,6);
 sockaddr_ll address={};address.sll_family=AF_PACKET;address.sll_protocol=htons(0x88b6);address.sll_ifindex=index_;
 if(bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))){error=std::strerror(errno);close();return false;}
 // This separate receive-only socket cannot feed the ACK matcher. Failure to
 // open diagnostics is reported explicitly and does not hide a healthy ACK.
 nav_fd_=socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(0x88b7));
 if(nav_fd_<0)softcore_.socketState(false,errno);
 else {
  address.sll_protocol=htons(0x88b7);
  if(bind(nav_fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))){const int e=errno;::close(nav_fd_);nav_fd_=-1;softcore_.socketState(false,e);}
  else softcore_.socketState(true);
 }
 return true;
}
bool MotorLink::send(bool armed,double left,double right,unsigned mask,unsigned invert,uint64_t now){
 if(fd_<0||!std::isfinite(left)||!std::isfinite(right)||mask==0||mask>=15||invert>15||(armed&&configuration_needed_))return false;
 // Clamp before multiplying/rounding, including unusually large finite inputs.
 left=std::max(-.6,std::min(.6,left));right=std::max(-.6,std::min(.6,right));
 uint32_t seq=++sequence_;auto bytes=motorFrame(mac_,seq,armed,std::lround(left*1000),std::lround(right*1000),mask,invert);
 return transmit(bytes,seq,mask,invert,1,now);
}
bool MotorLink::sendAngle(bool armed,double angle,double forward,unsigned mask,unsigned invert,uint64_t now){
 if(fd_<0||!std::isfinite(angle)||!std::isfinite(forward)||mask==0||mask>=15||invert>15||(armed&&configuration_needed_))return false;
 angle=std::max(-90.,std::min(90.,angle));forward=std::max(0.,std::min(.22,forward));
 uint32_t seq=++sequence_;auto bytes=angleMotorFrame(mac_,seq,armed,std::lround(angle*100),std::lround(forward*1000),mask,invert);
 return transmit(bytes,seq,mask,invert,2,now);
}
bool MotorLink::sendArc(bool armed,int outer,int curve,unsigned mask,unsigned invert,uint64_t now){
 if(fd_<0||mask==0||mask>=15||invert>15||(armed&&(configuration_needed_||(!arc_supported_&&(outer!=0||curve!=0)))))return false;
 uint32_t seq=++sequence_;return transmit(arcMotorFrame(mac_,seq,armed,outer,curve,mask,invert),seq,mask,invert,3,now);
}
bool MotorLink::sendVelocity(bool armed,double linear,double angular,unsigned mask,unsigned invert,uint64_t now){
 if(fd_<0||!std::isfinite(linear)||!std::isfinite(angular)||mask==0||mask>=15||invert>15||
    (armed&&(configuration_needed_||(!velocity_supported_&&(linear!=0||angular!=0))))||std::abs(linear)>.300000001||std::abs(angular)>1.000000001)return false;
 const uint32_t seq=++sequence_;
 return transmit(velocityMotorFrame(mac_,seq,armed,std::lround(linear*1000),std::lround(angular*1000),mask,invert),seq,mask,invert,4,now);
}
bool MotorLink::softcoreReady(uint64_t now)const{
 return !configuration_needed_&&velocity_supported_&&softcore_ready_&&now>=softcore_ack_ms_&&now-softcore_ack_ms_<=300&&window_.fresh(now);
}
bool MotorLink::transmit(const std::vector<uint8_t>& bytes,uint32_t seq,unsigned mask,unsigned invert,unsigned protocol,uint64_t now){
 sockaddr_ll address={};address.sll_family=AF_PACKET;address.sll_protocol=htons(0x88b6);address.sll_ifindex=index_;address.sll_halen=6;std::memcpy(address.sll_addr,fpga_mac,6);
 if(sendto(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=static_cast<ssize_t>(bytes.size())){diagnostics_.tx_errors++;diagnostics_.last_errno=errno;return false;}
 diagnostics_.tx_frames++;diagnostics_.last_errno=0;
 window_.noteSent(seq,now,mask,invert,protocol);return true;
}
void MotorLink::receive(uint64_t now){
 if(fd_<0)return;
 if(!window_.fresh(now)){arc_supported_=false;angle_supported_=false;velocity_supported_=false;softcore_ready_=false;}
 for(int count=0;count<64;count++){
  uint8_t bytes[2048];sockaddr_ll from={};socklen_t size=sizeof(from);ssize_t n=recvfrom(fd_,bytes,sizeof(bytes),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&from),&size);
  if(n<0&&errno==EINTR)continue;
  if(n<0)break;
  if(from.sll_pkttype==PACKET_OUTGOING)continue;
  diagnostics_.rx_frames++;
  MotorAck a;if(!decodeMotorAck(bytes,n,mac_,a)){diagnostics_.rx_rejected++;continue;}
  // A correctly addressed/CRC-checked old unconfigured ACK is reduce-only:
  // it can request physical ZERO configuration, never establish freshness or
  // authority. Firmware may keep its previous sequence while rejecting ARM.
  if(!a.configured){configuration_needed_=true;++diagnostics_.configuration_hints;}
  if(window_.accept(a,now)){if(a.configured&&!a.watchdog){configuration_needed_=false;configuration_verified_=true;}diagnostics_.ack_frames++;diagnostics_.last_ack_ms=now;if(a.protocol==2&&a.configured&&!a.watchdog)angle_supported_=true;if(a.protocol==3&&a.configured&&!a.watchdog)arc_supported_=true;if(a.protocol==4){velocity_supported_=a.configured&&!a.watchdog;softcore_ack_ms_=now;softcore_ready_=a.softcore_ready&&a.configured&&!a.watchdog;}}
  else {
   ++diagnostics_.rx_rejected;
   // A new RK process starts without the FPGA's sequence epoch. A decoded
   // old status may seed ONLY the next DISARM+ZERO configuration request.
   // It is never an ACK, capability, owner heartbeat or motion permission.
   // Once any fresh configuration was confirmed, no old status may reseed
   // the running process, even after a later network interruption.
   const uint32_t advance=a.sequence-sequence_;
   if(!configuration_verified_&&configuration_needed_&&!window_.fresh(now)&&
      (!sequence_seeded_||(advance&&!(advance&0x80000000u)))){
    sequence_=a.sequence;sequence_seeded_=true;window_.clear();
    angle_supported_=arc_supported_=velocity_supported_=false;softcore_ready_=false;softcore_ack_ms_=0;
    ++diagnostics_.configuration_seeds;diagnostics_.last_seed_sequence=a.sequence;
   }
  }
 }
 if(nav_fd_>=0)for(int count=0;count<64;count++){
  uint8_t bytes[2048];sockaddr_ll from={};socklen_t size=sizeof(from);
  const ssize_t n=recvfrom(nav_fd_,bytes,sizeof(bytes),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&from),&size);
  if(n<0&&errno==EINTR)continue;
  if(n<0){if(errno!=EAGAIN&&errno!=EWOULDBLOCK)softcore_.socketState(true,errno);break;}
  if(from.sll_pkttype!=PACKET_OUTGOING)softcore_.accept(bytes,size_t(n),mac_,now);
 }
}
bool MotorLink::fresh(uint64_t now)const{return fd_>=0&&!configuration_needed_&&window_.fresh(now);}
void MotorAckWindow::clear(){sent_.clear();ack_={};last_ack_=last_command_=0;have_ack_=false;}
void MotorAckWindow::noteSent(uint32_t sequence,uint64_t now,unsigned mask,unsigned invert,unsigned protocol){
 sent_.push_back({sequence,now,mask,invert,protocol});while(sent_.size()>32)sent_.pop_front();
}
bool MotorAckWindow::accept(const MotorAck &a,uint64_t now){
 if(a.protocol<1||a.protocol>4||(a.protocol!=4&&a.softcore_ready))return false;
 if(a.left < -600||a.left > 600||a.right < -600||a.right > 600||a.left_mask==0||a.left_mask>=15||a.invert_mask>15)return false;
 bool recent=false;uint64_t command_time=0;
 for(const auto &sent:sent_)if(sent.sequence==a.sequence&&now>=sent.time&&now-sent.time<=300&&
     a.left_mask==sent.left_mask&&a.invert_mask==sent.invert_mask&&a.protocol==sent.protocol){recent=true;command_time=sent.time;}
 const uint32_t delta=a.sequence-ack_.sequence;
 if(!recent||(have_ack_&&(!delta||(delta&0x80000000u))))return false;
 ack_=a;last_ack_=now;last_command_=command_time;have_ack_=true;
 return true;
}
bool MotorAckWindow::fresh(uint64_t now)const{
 return have_ack_&&now>=last_ack_&&now-last_ack_<=300&&now>=last_command_&&now-last_command_<=300&&ack_.configured&&!ack_.watchdog;
}
}
