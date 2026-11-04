#include "acg138_fpga_map_bridge/nav3_ethernet.hpp"
#include "acg138_fpga_map_bridge/nav2_wire.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace acg138::nav3 {
RawEthernet::RawEthernet(const std::string& name,const std::string& proc,bool execution_enabled):proc_(proc),execution_enabled_(execution_enabled) {
  if(name.empty() || name.size()>=IFNAMSIZ || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-")!=std::string::npos)
    throw std::invalid_argument("invalid interface name");
  if(wire::legacy_navigation_running(proc_))throw std::runtime_error("legacy robot-navigation active; no NAV3 frames sent");
  try {
    // Same inode as NAV2: mutually exclusive across protocols. Never unlink.
    const auto path="/run/lock/acg138-nav2-"+name+".lock";
    lock_=::open(path.c_str(),O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
    if(lock_<0 || flock(lock_,LOCK_EX|LOCK_NB)<0)throw std::runtime_error("another map client owns interface, or lock unavailable");
    fd_=::socket(AF_PACKET,SOCK_RAW|SOCK_CLOEXEC|SOCK_NONBLOCK,htons(kEtherType));
    if(fd_<0)throw std::runtime_error("NAV3 requires CAP_NET_RAW");
    ifreq req{};std::strncpy(req.ifr_name,name.c_str(),IFNAMSIZ-1);
    if(ioctl(fd_,SIOCGIFHWADDR,&req)<0)throw std::runtime_error("cannot read interface MAC");
    std::copy_n(req.ifr_hwaddr.sa_data,6,mac_.begin());
    if(ioctl(fd_,SIOCGIFMTU,&req)<0 || req.ifr_mtu<int(kWireBytes))throw std::runtime_error("MTU cannot hold NAV3");
    sockaddr_ll addr{};addr.sll_family=AF_PACKET;addr.sll_protocol=htons(kEtherType);
    addr.sll_ifindex=int(if_nametoindex(name.c_str()));
    if(!addr.sll_ifindex || bind(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0)throw std::runtime_error("cannot bind interface");
    int bytes=128*1024;setsockopt(fd_,SOL_SOCKET,SO_RCVBUF,&bytes,sizeof(bytes));
  } catch(...) {if(fd_>=0)::close(fd_);if(lock_>=0)::close(lock_);throw;}
}
RawEthernet::~RawEthernet(){if(fd_>=0)::close(fd_);if(lock_>=0)::close(lock_);}
void RawEthernet::send(const Wire& wire_bytes) {
  if(wire::legacy_navigation_running(proc_))throw std::runtime_error("legacy navigation appeared; refuse NAV3 send");
  Message m;if(decode(wire_bytes,m)!=Error::Ok)throw std::invalid_argument("invalid NAV3 frame");
  // No motion command can leave the ROS map bridge during line A.
  if((m.header.opcode&0x80u) || (!execution_enabled_&&m.header.opcode==request_opcode(Opcode::Start)))throw std::logic_error("line A forbids START/response transmission");
  std::array<std::uint8_t,kEthernetFrameBytes> bytes{};
  std::copy(kFpgaMac.begin(),kFpgaMac.end(),bytes.begin());std::copy(mac_.begin(),mac_.end(),bytes.begin()+6);
  bytes[12]=0x88;bytes[13]=0xb9;std::copy(wire_bytes.begin(),wire_bytes.end(),bytes.begin()+14);
  if(::send(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT)!=ssize_t(bytes.size()))
    throw std::runtime_error(std::string("NAV3 send failed: ")+std::strerror(errno));
}
bool RawEthernet::receive(Wire& out,unsigned timeout_ms) {
  pollfd poller{fd_,POLLIN,0};const int rc=poll(&poller,1,int(timeout_ms));
  if(rc<0){if(errno==EINTR)return false;throw std::runtime_error("NAV3 poll failed");}
  if(!rc)return false;
  if(poller.revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("NAV3 socket unavailable");
  std::array<std::uint8_t,4096> bytes{};sockaddr_ll source{};
  iovec vec{bytes.data(),bytes.size()};msghdr msg{};
  msg.msg_name=&source;msg.msg_namelen=sizeof(source);msg.msg_iov=&vec;msg.msg_iovlen=1;
  const auto n=recvmsg(fd_,&msg,MSG_DONTWAIT);
  if(n<0){if(errno==EINTR||errno==EAGAIN)return false;throw std::runtime_error("NAV3 receive failed");}
  if(source.sll_pkttype==PACKET_OUTGOING || (msg.msg_flags&MSG_TRUNC) || n!=ssize_t(kEthernetFrameBytes))return false;
  const std::array<std::uint8_t,6> broadcast{{255,255,255,255,255,255}};
  if(!std::equal(kFpgaMac.begin(),kFpgaMac.end(),bytes.begin()+6) ||
    (!std::equal(mac_.begin(),mac_.end(),bytes.begin()) && !std::equal(broadcast.begin(),broadcast.end(),bytes.begin())) ||
    bytes[12]!=0x88 || bytes[13]!=0xb9)return false;
  Wire w;std::copy_n(bytes.begin()+14,kWireBytes,w.begin());Message m;
  if(decode(w,m)!=Error::Ok)return false;
  out=w;return true;
}
std::uint64_t random_session() {
  const int fd=::open("/dev/urandom",O_RDONLY|O_CLOEXEC);
  if(fd<0)throw std::runtime_error("cannot obtain new NAV3 session randomness");
  std::uint64_t value=0;std::size_t done=0;
  while(done<sizeof(value)) {
    auto n=::read(fd,reinterpret_cast<char*>(&value)+done,sizeof(value)-done);
    if(n<0 && errno==EINTR)continue;
    if(n<=0){::close(fd);throw std::runtime_error("session random read failed");}
    done+=std::size_t(n);
  }
  ::close(fd);if(!value)throw std::runtime_error("random session zero; retry startup");return value;
}
} // namespace acg138::nav3
