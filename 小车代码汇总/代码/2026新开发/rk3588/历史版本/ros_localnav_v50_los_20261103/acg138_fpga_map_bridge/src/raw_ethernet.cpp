#include "acg138_fpga_map_bridge/nav2_wire.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace acg138::wire {
namespace {
enum class CommRead { Ok,Gone,Error };
CommRead read_comm(const std::string&path,std::string&name){
  // /proc/PID/comm is mode 0644 and does not use the exe symlink's ptrace
  // credential/capability check. Runtime intentionally retains NET_RAW only.
  const int fd=::open(path.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW);
  if(fd<0)return errno==ENOENT||errno==ESRCH?CommRead::Gone:CommRead::Error;
  char value[128];size_t used=0;bool failed=false;
  for(;;){
    const ssize_t n=::read(fd,value+used,sizeof(value)-used);
    if(n<0){if(errno==EINTR)continue;failed=true;break;}
    if(!n)break;
    used+=size_t(n);if(used==sizeof(value)){failed=true;break;}
  }
  ::close(fd);if(failed||!used)return CommRead::Error;
  name.assign(value,used);if(name.back()=='\n')name.pop_back();
  if(name.empty()||name.find('\n')!=std::string::npos||name.find('\0')!=std::string::npos)return CommRead::Error;
  return CommRead::Ok;
}
bool legacy_comm(const std::string&name){
  // Linux 6.1 TASK_COMM_LEN=16 includes the terminating NUL, hence the
  // deployed robot-navigation.aarch64 has this 15-character main-thread name.
  return name=="robot-navigatio"||name.rfind("robot-navigation",0)==0;
}
}
bool legacy_navigation_running(const std::string&proc){
  if(proc.empty()||proc.front()!='/')throw std::runtime_error("host_proc_root must be an absolute host procfs path");
  std::string self;
  if(read_comm(proc+"/self/comm",self)!=CommRead::Ok)throw std::runtime_error("host_proc_root/self/comm unavailable: cannot prove NAV2 ownership");
  DIR*d=opendir(proc.c_str());if(!d)throw std::runtime_error("cannot inspect host_proc_root: "+proc);
  bool found=false,unreadable=false;unsigned processes=0,inspected=0;
  for(;;){
    errno=0;auto*e=readdir(d);if(!e){if(errno)unreadable=true;break;}
    const std::string pid=e->d_name;if(pid.empty()||pid.find_first_not_of("0123456789")!=std::string::npos)continue;
    ++processes;std::string name;const auto result=read_comm(proc+"/"+pid+"/comm",name);
    if(result==CommRead::Error){unreadable=true;continue;}
    if(result==CommRead::Gone)continue; // Process exited during the directory snapshot.
    ++inspected;
    if(legacy_comm(name)){found=true;break;}
  }closedir(d);
  if(!found&&(!processes||!inspected||unreadable))throw std::runtime_error("host_proc_root process scan incomplete/unreadable; refusing live NAV2 upload");
  return found;
}
std::string find_interface(){
  DIR*d=opendir("/sys/class/net");if(!d)throw std::runtime_error("cannot inspect network interfaces");std::vector<std::string> matches;
  while(auto*e=readdir(d)){if(e->d_name[0]=='.')continue;char value[4096];const std::string link=std::string("/sys/class/net/")+e->d_name+"/device";const ssize_t n=readlink(link.c_str(),value,sizeof(value)-1);if(n>0){value[n]=0;if(std::strstr(value,"fe1c0000"))matches.emplace_back(e->d_name);}}
  closedir(d);if(matches.size()!=1)throw std::runtime_error("cannot uniquely identify internal GMAC1 fe1c0000; set interface explicitly");return matches.front();
}
RawEthernet::RawEthernet(const std::string&name,const std::string&proc):host_proc_root_(proc){
  if(name.empty()||name.size()>=IFNAMSIZ||name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-")!=std::string::npos)throw std::invalid_argument("invalid network interface");
  if(legacy_navigation_running(host_proc_root_))throw std::runtime_error("robot-navigation is active: stop the existing NAV2 planner before enabling ROS map upload; no process was killed");
  try{
    const std::string lockfile="/run/lock/acg138-nav2-"+name+".lock";
    lock_=::open(lockfile.c_str(),O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
    if(lock_<0)throw std::runtime_error("cannot open NAV2 lock (create /run/lock); no Ethernet frame sent");
    if(flock(lock_,LOCK_EX|LOCK_NB)<0)throw std::runtime_error("another ROS NAV2 client owns this interface");
    fd_=::socket(AF_PACKET,SOCK_RAW|SOCK_CLOEXEC|SOCK_NONBLOCK,htons(kEtherType));if(fd_<0)throw std::runtime_error("NAV2 AF_PACKET requires root/CAP_NET_RAW");
    ifreq req{};std::strncpy(req.ifr_name,name.c_str(),IFNAMSIZ-1);
    if(ioctl(fd_,SIOCGIFHWADDR,&req)<0)throw std::runtime_error("cannot read GMAC MAC address");
    std::copy(req.ifr_hwaddr.sa_data,req.ifr_hwaddr.sa_data+6,mac_.begin());
    if(ioctl(fd_,SIOCGIFMTU,&req)<0||req.ifr_mtu<66)throw std::runtime_error("GMAC MTU cannot hold NAV2 payload");
    sockaddr_ll address{};address.sll_family=AF_PACKET;address.sll_protocol=htons(kEtherType);address.sll_ifindex=int(if_nametoindex(name.c_str()));
    if(!address.sll_ifindex||bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0)throw std::runtime_error("cannot bind internal GMAC interface");
    const int bytes=128*1024;setsockopt(fd_,SOL_SOCKET,SO_RCVBUF,&bytes,sizeof(bytes));
  }catch(...){if(fd_>=0)::close(fd_);if(lock_>=0)::close(lock_);fd_=lock_=-1;throw;}
}
RawEthernet::~RawEthernet(){if(fd_>=0)::close(fd_);if(lock_>=0)::close(lock_);}
void RawEthernet::send(const Body&body){
  // Recheck before each command: a legacy planner may have started after this
  // bridge. Its old binary does not participate in our advisory flock.
  if(legacy_navigation_running(host_proc_root_))throw std::runtime_error("legacy robot-navigation appeared; refusing concurrent NAV2 map mutation");
  Message message;if(!decode(body,message))throw std::invalid_argument("refusing malformed NAV2 body");const auto bytes=ethernet(mac_,message);
  const ssize_t n=::send(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT);if(n!=ssize_t(bytes.size()))throw std::runtime_error("NAV2 nonblocking send failed: "+std::string(std::strerror(errno)));
}
bool RawEthernet::receive(Body&body,unsigned timeout_ms){
  pollfd descriptor{fd_,POLLIN,0};const int ready=poll(&descriptor,1,int(timeout_ms));
  if(ready<0){if(errno==EINTR)return false;throw std::runtime_error("NAV2 socket poll failed");}if(!ready)return false;
  if(descriptor.revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("NAV2 socket became unavailable");
  uint8_t bytes[2048];sockaddr_ll source{};iovec vec{bytes,sizeof(bytes)};msghdr message{};
  message.msg_name=&source;message.msg_namelen=sizeof(source);message.msg_iov=&vec;message.msg_iovlen=1;
  const ssize_t n=recvmsg(fd_,&message,MSG_DONTWAIT);if(n<0){if(errno==EINTR||errno==EAGAIN)return false;throw std::runtime_error("NAV2 receive failed");}
  if(source.sll_pkttype==PACKET_OUTGOING||(message.msg_flags&MSG_TRUNC))return false;
  Message decoded;if(!decode_reply(bytes,size_t(n),mac_,decoded))return false;body=encode(decoded);return true;
}
}  // namespace acg138::wire
