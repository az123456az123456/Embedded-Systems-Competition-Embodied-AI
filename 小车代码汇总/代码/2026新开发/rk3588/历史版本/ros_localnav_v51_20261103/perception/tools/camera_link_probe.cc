#include "camera_link_protocol.h"
#include "rgb_camera.h"
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <linux/if_packet.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
int main(int argc,char **argv){
 const char *interface="eth0";unsigned timeout=1800;
 for(int i=1;i<argc;++i){
  if(!std::strcmp(argv[i],"--interface")&&i+1<argc)interface=argv[++i];
  else if(!std::strcmp(argv[i],"--timeout-ms")&&i+1<argc){char *end=nullptr;const auto value=std::strtoul(argv[++i],&end,10);if(!end||*end||value<100||value>10000)return 2;timeout=value;}
  else{std::fprintf(stderr,"Usage: camera-link-probe --interface eth0 --timeout-ms 1800\n");return 2;}
 }
 if(!*interface||std::strlen(interface)>=IFNAMSIZ||std::strchr(interface,'/'))return 2;
 int fd=socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(3));
 if(fd<0){std::perror("camera-link-probe socket (root/CAP_NET_RAW)");return 1;}
 struct Guard{int fd;~Guard(){close(fd);}}guard{fd};
 ifreq info{};std::strncpy(info.ifr_name,interface,IFNAMSIZ-1);
 if(ioctl(fd,SIOCGIFINDEX,&info)<0){std::perror("camera-link-probe interface");return 1;}const auto index=info.ifr_ifindex;
 if(ioctl(fd,SIOCGIFMTU,&info)<0)return 1;
 if(info.ifr_mtu<4000){info.ifr_mtu=4000;if(ioctl(fd,SIOCSIFMTU,&info)<0){std::perror("camera-link-probe needs MTU4000 for RAW rows");return 1;}}
 if(ioctl(fd,SIOCGIFFLAGS,&info)<0)return 1;
 if(!(info.ifr_flags&IFF_UP)){info.ifr_flags|=IFF_UP;if(ioctl(fd,SIOCSIFFLAGS,&info)<0){std::perror("camera-link-probe bring interface up");return 1;}}
 sockaddr_ll bind_address{};bind_address.sll_family=AF_PACKET;bind_address.sll_ifindex=index;bind_address.sll_protocol=htons(3);
 if(bind(fd,reinterpret_cast<sockaddr*>(&bind_address),sizeof(bind_address))<0)return 1;
 // RGP1 images require a camera-only START request. This does not contain a
 // motor protocol, arm request or any networking configuration mutation.
 if(ioctl(fd,SIOCGIFHWADDR,&info)<0)return 1;
 uint8_t request[60]={};std::memcpy(request,fpga_rgb::DefaultPeer.data(),6);std::memcpy(request+6,info.ifr_hwaddr.sa_data,6);request[12]=0x88;request[13]=0xba;
 const auto command=fpga_rgb::start_command(0x3838);std::memcpy(request+14,command.data(),command.size());
 sockaddr_ll destination{};destination.sll_family=AF_PACKET;destination.sll_ifindex=index;destination.sll_halen=6;std::memcpy(destination.sll_addr,request,6);
 (void)sendto(fd,request,sizeof(request),0,reinterpret_cast<sockaddr*>(&destination),sizeof(destination));
 const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout);unsigned raw=0,rgb=0;
 while(std::chrono::steady_clock::now()<deadline){pollfd ready{fd,POLLIN,0};if(poll(&ready,1,50)<=0)continue;
  uint8_t packet[4096];const auto n=recv(fd,packet,sizeof(packet),0);if(n<=0)continue;
  const auto mode=camera_link::classify(packet,size_t(n));raw+=mode==camera_link::Mode::Raw;rgb+=mode==camera_link::Mode::PreparedRgb;
  if(raw>=8||rgb>=8){std::fprintf(stderr,"CAMERA_LINK interface=%s raw_packets=%u rgp_packets=%u\n",interface,raw,rgb);std::puts(raw>=8?"v4l2":"fpga-rgb");return 0;}
 }
 std::fprintf(stderr,"CAMERA_LINK interface=%s recognized_raw=%u recognized_rgp=%u no_complete_mode=1\n",interface,raw,rgb);return 1;
}
