#include "nva_socket.h"
#include <algorithm>
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_packet.h>
#include <linux/filter.h>
#include <net/if.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
namespace robotnav {
NvaSocket::~NvaSocket(){if(fd_>=0)::close(fd_);if(lock_>=0)::close(lock_);}
bool NvaSocket::open(const std::string& name,std::string& error){
 if(fd_>=0||name.empty()||name.size()>=IFNAMSIZ||name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-")!=std::string::npos){error="invalid NVA interface";return false;}
 const auto file="/run/lock/acg138-nva-"+name+".lock";
 lock_=::open(file.c_str(),O_CREAT|O_RDWR|O_NOFOLLOW|O_CLOEXEC,0600);
 if(lock_<0||flock(lock_,LOCK_EX|LOCK_NB)){error="NVA sender already locked";return false;}
 fd_=::socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(NE_ETH_NVA1));
 if(fd_<0){error="NVA raw socket unavailable";return false;}
 // Keep map/POSE/STATE traffic out of this ACK-only socket, rather than letting
 // a busy map upload postpone the first permit ACK beyond its 100ms lifetime.
 sock_filter code[]={{0x20,0,0,14},{0x15,0,1,0x4e564131},{0x06,0,0,4096},{0x06,0,0,0}};
 sock_fprog filter{4,code};
 if(setsockopt(fd_,SOL_SOCKET,SO_ATTACH_FILTER,&filter,sizeof(filter))){error="NVA ACK filter unavailable";return false;}
 ifreq r{};std::strncpy(r.ifr_name,name.c_str(),IFNAMSIZ-1);
 if(ioctl(fd_,SIOCGIFHWADDR,&r)){error="NVA MAC unavailable";return false;}
 std::copy_n(r.ifr_hwaddr.sa_data,6,mac_.begin());index_=if_nametoindex(name.c_str());
 sockaddr_ll a{};a.sll_family=AF_PACKET;a.sll_protocol=htons(NE_ETH_NVA1);a.sll_ifindex=index_;
 if(!index_||bind(fd_,reinterpret_cast<sockaddr*>(&a),sizeof(a))){error="NVA bind failed";return false;}
 return true;
}
bool NvaSocket::send(const acg138::nav3::Wire&w){
 ne_message message{};if(fd_<0||ne_decode(w.data(),w.size(),&message)!=NE_CODEC_OK||message.h.family!=NE_NVA1||message.h.opcode!=NE_PERMIT)return false;
 std::array<uint8_t,272> bytes{};std::copy(acg138::nav3::kFpgaMac.begin(),acg138::nav3::kFpgaMac.end(),bytes.begin());std::copy(mac_.begin(),mac_.end(),bytes.begin()+6);bytes[12]=NE_ETH_NVA1>>8;bytes[13]=NE_ETH_NVA1&255;std::copy(w.begin(),w.end(),bytes.begin()+14);
 return ::send(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT|MSG_NOSIGNAL)==ssize_t(bytes.size());
}
bool NvaSocket::receive(acg138::nav3::Wire&w){
 uint8_t bytes[4096];sockaddr_ll a{};iovec io{bytes,sizeof(bytes)};msghdr m{};m.msg_name=&a;m.msg_namelen=sizeof(a);m.msg_iov=&io;m.msg_iovlen=1;
 auto n=recvmsg(fd_,&m,MSG_DONTWAIT);
 const std::array<uint8_t,6> broadcast{{255,255,255,255,255,255}};
 if(n!=272||(m.msg_flags&MSG_TRUNC)||a.sll_pkttype==PACKET_OUTGOING||
 !std::equal(acg138::nav3::kFpgaMac.begin(),acg138::nav3::kFpgaMac.end(),bytes+6)||
 (!std::equal(mac_.begin(),mac_.end(),bytes)&&!std::equal(broadcast.begin(),broadcast.end(),bytes))||bytes[12]!=(NE_ETH_NVA1>>8)||bytes[13]!=(NE_ETH_NVA1&255))return false;
 acg138::nav3::Wire candidate;std::copy_n(bytes+14,258,candidate.begin());ne_message packet{};
 if(ne_decode(candidate.data(),candidate.size(),&packet)!=NE_CODEC_OK||packet.h.family!=NE_NVA1||packet.h.opcode!=(NE_PERMIT|0x80))return false;
 w=candidate;return true;
}
}
