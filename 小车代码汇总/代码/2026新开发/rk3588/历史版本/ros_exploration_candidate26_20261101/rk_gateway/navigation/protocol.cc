#include "protocol.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>

namespace navwire {
static const uint8_t FPGA[6]={2,0xac,0x13,0x80,0,1};
static uint16_t u16(const uint8_t*p){return uint16_t(p[0])<<8|p[1];}
static uint32_t u32(const uint8_t*p){return uint32_t(p[0])<<24|uint32_t(p[1])<<16|uint32_t(p[2])<<8|p[3];}
static void w32(uint8_t*p,uint32_t n){p[0]=n>>24;p[1]=n>>16;p[2]=n>>8;p[3]=n;}
uint16_t crc16(const uint8_t*p,size_t n){uint16_t c=0xffff;while(n--){c^=uint16_t(*p++)<<8;for(int k=0;k<8;++k)c=(c&0x8000)?uint16_t((c<<1)^0x1021):uint16_t(c<<1);}return c;}
uint32_t crc32(const uint8_t*p,size_t n){uint32_t c=0xffffffff;while(n--){c^=*p++;for(int k=0;k<8;++k)c=(c&1)?(c>>1)^0xedb88320:c>>1;}return ~c;}
Bytes64 encode(const Message&m){Bytes64 b{};std::memcpy(b.data(),"NAV2",4);b[4]=1;b[5]=m.op;b[6]=m.status;b[7]=m.count;w32(&b[8],m.request);w32(&b[12],m.generation);w32(&b[16],m.arg0);w32(&b[20],m.arg1);w32(&b[24],m.arg2);std::copy(m.data.begin(),m.data.end(),b.begin()+32);return b;}
bool decode(const uint8_t*p,size_t n,Message&m){if(n!=64||std::memcmp(p,"NAV2",4)||p[4]!=1||u32(p+28)!=0)return false;m={};m.op=p[5];m.status=p[6];m.count=p[7];m.request=u32(p+8);m.generation=u32(p+12);m.arg0=u32(p+16);m.arg1=u32(p+20);m.arg2=u32(p+24);std::copy(p+32,p+64,m.data.begin());return true;}
std::vector<uint8_t> frame(const uint8_t source[6],const Message&m){std::vector<uint8_t>b(80);std::copy(FPGA,FPGA+6,b.begin());std::copy(source,source+6,b.begin()+6);b[12]=0x88;b[13]=0xb9;auto p=encode(m);std::copy(p.begin(),p.end(),b.begin()+14);auto crc=crc16(p.data(),p.size());b[78]=crc>>8;b[79]=crc;return b;}
static bool header(const uint8_t*p,size_t n,const uint8_t host[6],uint16_t type){if(n!=80&&n!=84)return false;bool broadcast=true;for(int i=0;i<6;++i)broadcast&=p[i]==0xff;if(!broadcast&&std::memcmp(p,host,6))return false;return !std::memcmp(p+6,FPGA,6)&&u16(p+12)==type&&crc16(p+14,64)==u16(p+78);}
bool reply(const uint8_t*p,size_t n,const uint8_t host[6],Message&m){return header(p,n,host,NAV_TYPE)&&decode(p+14,64,m)&&(m.op&0x80);}
bool sensorFrame(const uint8_t*p,size_t n,const uint8_t host[6],Sensors&s){if(!header(p,n,host,SENSOR_TYPE))return false;p+=14;if(std::memcmp(p,"SNS1",4)||p[4]!=1||p[63]||p[5]>7||p[6]>7||p[7]>15||p[62]>15)return false;Sensors v;v.valid=p[5];v.fault=p[6];v.invert=p[7];v.sequence=u32(p+8);v.uptime_ms=u32(p+12);for(int i=0;i<4;++i){v.counts[i]=int32_t(u32(p+16+i*4));v.delta[i]=int32_t(u32(p+32+i*4));}for(int i=0;i<3;++i){v.distance_mm[i]=u16(p+48+i*2);v.age_ms[i]=u16(p+54+i*2);}v.window_ms=u16(p+60);v.illegal=p[62];if(v.window_ms!=0&&v.window_ms!=50)return false;if(!v.window_ms)for(auto delta:v.delta)if(delta)return false;s=v;return true;}
double monotonicSeconds(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string findInterface(){DIR*d=opendir("/sys/class/net");if(!d)throw std::runtime_error("cannot inspect network interfaces");std::vector<std::string> names;while(auto*e=readdir(d)){if(e->d_name[0]=='.')continue;char buf[4096];auto path=std::string("/sys/class/net/")+e->d_name+"/device";auto n=readlink(path.c_str(),buf,sizeof(buf)-1);if(n>0){buf[n]=0;if(std::strstr(buf,"fe1c0000"))names.push_back(e->d_name);}}closedir(d);if(names.size()!=1)throw std::runtime_error("cannot uniquely find internal GMAC1; specify --interface");return names[0];}
RawSocket::~RawSocket(){if(fd_>=0)::close(fd_);}
void RawSocket::open(const std::string&name,uint16_t type){if(fd_>=0)throw std::runtime_error("socket already open");if(name.empty()||name.size()>=IFNAMSIZ)throw std::runtime_error("invalid interface");fd_=::socket(AF_PACKET,SOCK_RAW|SOCK_CLOEXEC,htons(type));if(fd_<0)throw std::runtime_error("raw socket requires root/CAP_NET_RAW");ifreq req{};std::strncpy(req.ifr_name,name.c_str(),IFNAMSIZ-1);if(ioctl(fd_,SIOCGIFHWADDR,&req)<0)throw std::runtime_error("cannot read interface MAC");std::copy(req.ifr_hwaddr.sa_data,req.ifr_hwaddr.sa_data+6,mac_.begin());sockaddr_ll addr{};addr.sll_family=AF_PACKET;addr.sll_protocol=htons(type);addr.sll_ifindex=if_nametoindex(name.c_str());if(!addr.sll_ifindex||bind(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0)throw std::runtime_error("cannot bind FPGA network interface");int size=128*1024;setsockopt(fd_,SOL_SOCKET,SO_RCVBUF,&size,sizeof(size));int timestamps=1;if(setsockopt(fd_,SOL_SOCKET,SO_TIMESTAMPNS,&timestamps,sizeof(timestamps)))throw std::runtime_error("kernel receive timestamps unavailable");}
void RawSocket::send(const std::vector<uint8_t>&b){ssize_t n=::send(fd_,b.data(),b.size(),MSG_DONTWAIT);if(n!=ssize_t(b.size()))throw std::runtime_error("NAV2 transmit failed");}
bool RawSocket::receive(std::vector<uint8_t>&b,int ms,double* stamp){
 pollfd p{fd_,POLLIN,0};int r=poll(&p,1,ms);
 if(r<0){if(errno==EINTR)return false;throw std::runtime_error("raw socket poll failed");}
 if(!r)return false;
 if(p.revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("raw socket closed");
 uint8_t buf[2048];sockaddr_ll from{};
 alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(timespec))]{};
 iovec vec{buf,sizeof(buf)};msghdr message{};
 message.msg_name=&from;message.msg_namelen=sizeof(from);message.msg_iov=&vec;message.msg_iovlen=1;
 message.msg_control=ancillary;message.msg_controllen=sizeof(ancillary);
 auto n=recvmsg(fd_,&message,MSG_DONTWAIT);
 if(n<0){if(errno==EAGAIN||errno==EINTR)return false;throw std::runtime_error("raw socket receive failed");}
 if(from.sll_pkttype==PACKET_OUTGOING||message.msg_flags&(MSG_TRUNC|MSG_CTRUNC))return false;
 if(stamp){
  bool have=false;timespec kernel{},real{};
  for(auto* c=CMSG_FIRSTHDR(&message);c;c=CMSG_NXTHDR(&message,c)){
   if(c->cmsg_level==SOL_SOCKET&&c->cmsg_type==SCM_TIMESTAMPNS&&c->cmsg_len>=CMSG_LEN(sizeof(timespec))){std::memcpy(&kernel,CMSG_DATA(c),sizeof(kernel));have=true;break;}
  }
  if(!have||clock_gettime(CLOCK_REALTIME,&real))return false;
  double age=double(real.tv_sec-kernel.tv_sec)+double(real.tv_nsec-kernel.tv_nsec)*1e-9;
  if(age<0||age>30)return false;
  // Preserve kernel queueing delay. Timestamping at read() would incorrectly
  // make old telemetry fresh after an overloaded CPU recovers.
  *stamp=monotonicSeconds()-age;
 }
 b.assign(buf,buf+n);return true;
}

Message Planner::exchange(Message m,int timeout_ms,const std::atomic<bool>*cancel){
#ifdef NAV_TEST_TRANSPORT
 if(test_transport){if(cancel&&cancel->load())throw std::runtime_error("planning cancelled");auto r=test_transport(m);if(r.op!=(m.op|0x80)||r.request!=m.request||r.generation!=m.generation)throw std::runtime_error("test transport correlation mismatch");if(r.status)throw std::runtime_error("FPGA NAV2 status="+std::to_string(r.status));return r;}
#endif
 auto bytes=frame(socket_.mac(),m);for(int attempt=0;attempt<3;++attempt){if(cancel&&cancel->load())throw std::runtime_error("planning cancelled");socket_.send(bytes);double until=monotonicSeconds()+timeout_ms/1000.;while(monotonicSeconds()<until){if(cancel&&cancel->load())throw std::runtime_error("planning cancelled");std::vector<uint8_t>b;if(!socket_.receive(b,50))continue;Message r;if(!reply(b.data(),b.size(),socket_.mac(),r)||r.request!=m.request||r.op!=(m.op|0x80)||r.generation!=m.generation)continue;if(r.status)throw std::runtime_error("FPGA NAV2 status="+std::to_string(r.status));return r;}}throw std::runtime_error("FPGA NAV2 timeout; check v16 bitstream");}
void Planner::open(const std::string&name){socket_.open(name,NAV_TYPE);}
std::vector<uint16_t> Planner::plan(const Bitmap&map,uint16_t start,uint16_t goal,const std::atomic<bool>*cancel){if(start>=4096||goal>=4096)throw std::runtime_error("plan coordinate outside grid");Message m;m.op=6;auto info=exchange(m,500,cancel);if(info.data[0]!=0||info.data[1]!=1||info.data[2]!=0||info.data[3]!=0)throw std::runtime_error("unsupported NAV2 capabilities");next_=info.arg1+1;if(!next_)next_=1;generation_=info.arg0+1;if(!generation_)generation_=1;
 auto request=[&](uint8_t op){Message q;q.op=op;q.request=next_++;if(!next_)next_=1;q.generation=generation_;return q;};
 m=request(1);m.arg0=crc32(map.data(),map.size());exchange(m,500,cancel);
 for(unsigned i=0;i<512;i+=32){m=request(2);m.arg0=i;m.count=32;std::copy(map.begin()+i,map.begin()+i+32,m.data.begin());exchange(m,500,cancel);}
 m=request(3);exchange(m,500,cancel);m=request(4);m.arg0=start;m.arg1=goal;const uint32_t plan_id=m.request;auto result=exchange(m,10000,cancel);if(result.arg0<1||result.arg0>4096)throw std::runtime_error("invalid FPGA path size");std::vector<uint16_t>path;const auto count=result.arg0;
 while(path.size()<count){m=request(5);m.arg0=path.size();m.arg1=plan_id;m.count=std::min<size_t>(16,count-path.size());auto page=exchange(m,500,cancel);if(page.arg0!=path.size()||page.arg1!=count||page.arg2!=plan_id||page.count!=m.count)throw std::runtime_error("mixed FPGA path pages");for(unsigned j=0;j<page.count;++j){auto cell=u16(page.data.data()+j*2);if(cell>=4096||(map[cell/8]&(1u<<(cell%8))))throw std::runtime_error("FPGA path intersects occupied cell");if(!path.empty()){auto previous=path.back();int dx=std::abs(int(cell%64)-int(previous%64)),dy=std::abs(int(cell/64)-int(previous/64));if(dx+dy!=1)throw std::runtime_error("noncontiguous FPGA path");}path.push_back(cell);}}
 if(path.front()!=start||path.back()!=goal)throw std::runtime_error("FPGA path endpoints mismatch");
 return path;
}
}
