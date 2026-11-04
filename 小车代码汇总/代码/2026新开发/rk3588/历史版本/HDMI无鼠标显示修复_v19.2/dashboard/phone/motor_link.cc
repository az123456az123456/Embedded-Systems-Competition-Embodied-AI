#include "motor_link.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <linux/if_packet.h>
#include <net/if.h>
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
bool decodeMotorAck(const uint8_t *b,size_t n,const uint8_t host[6],MotorAck &out){
 if(!b||!host||(n!=60&&n!=64))return false;
 static const uint8_t broadcast[]={255,255,255,255,255,255};
 if((std::memcmp(b,host,6)&&std::memcmp(b,broadcast,6))||std::memcmp(b+6,fpga_mac,6)||b[12]!=0x88||b[13]!=0xb6)return false;
 const uint8_t *p=b+14;
 const unsigned protocol=!std::memcmp(p,"ACK1",4)&&p[4]==1?1:!std::memcmp(p,"ACK2",4)&&p[4]==2?2:0;
 if(!protocol||(p[5]&~7u)||motorCrc16(p,16)!=u16(p+16))return false;
 for(size_t i=32;i<60;i++)if(b[i])return false;
 int left=signed16(p+10),right=signed16(p+12);
 if(std::abs(left)>600||std::abs(right)>600||p[14]==0||p[14]>=15||p[15]>15)return false;
 if(n==64){uint32_t crc=~0u;for(size_t i=0;i<60;i++){crc^=b[i];for(int bit=0;bit<8;bit++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}uint32_t f=b[60]|(uint32_t(b[61])<<8)|(uint32_t(b[62])<<16)|(uint32_t(b[63])<<24);if(~crc!=f)return false;}
 out.sequence=u32(p+6);out.armed=p[5]&1;out.watchdog=p[5]&2;out.configured=p[5]&4;out.left=left;out.right=right;out.left_mask=p[14];out.invert_mask=p[15];out.protocol=protocol;return true;
}
MotorLink::~MotorLink(){close();}
void MotorLink::close(){if(fd_>=0)::close(fd_);fd_=-1;window_.clear();angle_supported_=false;}
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
 return true;
}
bool MotorLink::send(bool armed,double left,double right,unsigned mask,unsigned invert,uint64_t now){
 if(fd_<0||!std::isfinite(left)||!std::isfinite(right)||mask==0||mask>=15||invert>15)return false;
 // Clamp before multiplying/rounding, including unusually large finite inputs.
 left=std::max(-.6,std::min(.6,left));right=std::max(-.6,std::min(.6,right));
 uint32_t seq=++sequence_;auto bytes=motorFrame(mac_,seq,armed,std::lround(left*1000),std::lround(right*1000),mask,invert);
 return transmit(bytes,seq,mask,invert,1,now);
}
bool MotorLink::sendAngle(bool armed,double angle,double forward,unsigned mask,unsigned invert,uint64_t now){
 if(fd_<0||!std::isfinite(angle)||!std::isfinite(forward)||mask==0||mask>=15||invert>15)return false;
 angle=std::max(-90.,std::min(90.,angle));forward=std::max(0.,std::min(.22,forward));
 uint32_t seq=++sequence_;auto bytes=angleMotorFrame(mac_,seq,armed,std::lround(angle*100),std::lround(forward*1000),mask,invert);
 return transmit(bytes,seq,mask,invert,2,now);
}
bool MotorLink::transmit(const std::vector<uint8_t>& bytes,uint32_t seq,unsigned mask,unsigned invert,unsigned protocol,uint64_t now){
 sockaddr_ll address={};address.sll_family=AF_PACKET;address.sll_protocol=htons(0x88b6);address.sll_ifindex=index_;address.sll_halen=6;std::memcpy(address.sll_addr,fpga_mac,6);
 if(sendto(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=static_cast<ssize_t>(bytes.size())){diagnostics_.tx_errors++;diagnostics_.last_errno=errno;return false;}
 diagnostics_.tx_frames++;diagnostics_.last_errno=0;
 window_.noteSent(seq,now,mask,invert,protocol);return true;
}
void MotorLink::receive(uint64_t now){
 if(fd_<0)return;
 for(int count=0;count<64;count++){
  uint8_t bytes[2048];sockaddr_ll from={};socklen_t size=sizeof(from);ssize_t n=recvfrom(fd_,bytes,sizeof(bytes),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&from),&size);
  if(n<0&&errno==EINTR)continue;
  if(n<0)break;
  if(from.sll_pkttype==PACKET_OUTGOING)continue;
  diagnostics_.rx_frames++;
  MotorAck a;if(!decodeMotorAck(bytes,n,mac_,a)){diagnostics_.rx_rejected++;continue;}
  if(window_.accept(a,now)){diagnostics_.ack_frames++;diagnostics_.last_ack_ms=now;if(a.protocol==2&&a.configured&&!a.watchdog)angle_supported_=true;}
  else diagnostics_.rx_rejected++;
 }
}
bool MotorLink::fresh(uint64_t now)const{return fd_>=0&&window_.fresh(now);}
void MotorAckWindow::clear(){sent_.clear();ack_={};last_ack_=last_command_=0;have_ack_=false;}
void MotorAckWindow::noteSent(uint32_t sequence,uint64_t now,unsigned mask,unsigned invert,unsigned protocol){
 sent_.push_back({sequence,now,mask,invert,protocol});while(sent_.size()>32)sent_.pop_front();
}
bool MotorAckWindow::accept(const MotorAck &a,uint64_t now){
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
