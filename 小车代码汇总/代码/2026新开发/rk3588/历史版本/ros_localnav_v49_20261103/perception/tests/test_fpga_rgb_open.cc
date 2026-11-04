#include "rgb_camera.h"
#include <array>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace {
constexpr int FakeFd=945;
unsigned checks=0,force_calls=0,normal_calls=0,up_calls=0,sent=0,closed=0;
bool is_up=false,force_ok=true,up_ok=true;
std::array<uint8_t,6> local{{2,3,4,5,6,7}};
void ck(bool x,const char *s){++checks;if(!x)throw std::runtime_error(s);}
void reset(){force_calls=normal_calls=up_calls=sent=closed=0;is_up=false;force_ok=up_ok=true;}
}
extern "C" int __wrap_socket(int family,int type,int protocol){
    ck(family==AF_PACKET,"camera opened non-AF_PACKET socket");
    ck(type==(SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC),"socket flags changed");
    ck(protocol==htons(0x88ba),"camera used occupied NAV2 EtherType");return FakeFd;
}
extern "C" int __wrap_ioctl(int fd,unsigned long request,...){
    ck(fd==FakeFd,"unexpected ioctl fd");
    va_list args;va_start(args,request);auto *ifr=va_arg(args,ifreq *);va_end(args);
    ck(std::string(ifr->ifr_name)=="eth-fixture","camera changed other interface");
    switch(request){
    case SIOCGIFINDEX:ifr->ifr_ifindex=7;break;
    case SIOCGIFHWADDR:std::memcpy(ifr->ifr_hwaddr.sa_data,local.data(),6);break;
    case SIOCGIFFLAGS:ifr->ifr_flags=is_up?IFF_UP:0;break;
    case SIOCSIFFLAGS:++up_calls;ck(ifr->ifr_flags==IFF_UP,"camera changed flags other than IFF_UP");if(!up_ok){errno=EPERM;return -1;}break;
    default:throw std::runtime_error("camera attempted IP/route or unexpected ioctl");
    }return 0;
}
extern "C" int __wrap_bind(int fd,const sockaddr *addr,socklen_t size){
    const auto *a=reinterpret_cast<const sockaddr_ll *>(addr);
    ck(fd==FakeFd&&size==sizeof(sockaddr_ll)&&a->sll_family==AF_PACKET&&a->sll_ifindex==7&&a->sll_protocol==htons(0x88ba),"raw bind changed");return 0;
}
extern "C" int __wrap_setsockopt(int fd,int level,int option,const void *value,socklen_t size){
    ck(fd==FakeFd&&size==sizeof(int),"socket option malformed");const int v=*static_cast<const int *>(value);
    if(level==SOL_SOCKET&&option==SO_RCVBUFFORCE){++force_calls;ck(v==16*1024*1024,"receive force size changed");if(!force_ok){errno=EPERM;return -1;}return 0;}
    if(level==SOL_SOCKET&&option==SO_RCVBUF){++normal_calls;ck(v==16*1024*1024,"receive fallback size changed");return 0;}
#ifdef PACKET_IGNORE_OUTGOING
    if(level==SOL_PACKET&&option==PACKET_IGNORE_OUTGOING){ck(v==1,"outgoing frames not ignored");return 0;}
#endif
    throw std::runtime_error("unexpected/global socket option");
}
extern "C" int __wrap_getsockopt(int fd,int level,int option,void *value,socklen_t *size){
    ck(fd==FakeFd&&level==SOL_SOCKET&&option==SO_RCVBUF&&*size==sizeof(int),"actual buffer not queried");
    *static_cast<int *>(value)=force_ok?32*1024*1024:425984;return 0;
}
extern "C" ssize_t __wrap_sendto(int fd,const void *data,size_t size,int flags,const sockaddr *addr,socklen_t addrsize){
    ++sent;const auto *a=reinterpret_cast<const sockaddr_ll *>(addr);const auto *p=static_cast<const uint8_t *>(data);
    ck(fd==FakeFd&&size==60&&flags==MSG_DONTWAIT&&addrsize==sizeof(sockaddr_ll)&&a->sll_ifindex==7&&a->sll_protocol==htons(0x88ba),"START socket envelope wrong");
    ck(!std::memcmp(p,fpga_rgb::DefaultPeer.data(),6)&&!std::memcmp(p+6,local.data(),6),"START source/target wrong");
    ck(p[12]==0x88&&p[13]==0xba&&!std::memcmp(p+14,"RGC1",4)&&p[18]==1&&p[19]==1&&(p[20]||p[21]),"START header invalid");
    const auto command=fpga_rgb::start_command(static_cast<uint16_t>((p[20]<<8)|p[21]));
    ck(!std::memcmp(command.data(),p+14,16),"START CRC wrong");
    for(size_t i=30;i<60;++i)ck(p[i]==0,"START Ethernet padding was not zeroed");
    return 60;
}
extern "C" int __real_close(int);
extern "C" int __wrap_close(int fd){if(fd==FakeFd){++closed;return 0;}return __real_close(fd);}
int main(){try{
    unsetenv("PERCEPTION_RGB_SOURCE_MAC");std::string error;
    reset();{fpga_rgb::RgbCamera camera;ck(camera.open("eth-fixture",error),error.c_str());ck(up_calls==1&&force_calls==1&&!normal_calls&&sent==1,"up/forced receive setup omitted");}ck(closed==1,"socket not closed");
    reset();is_up=true;force_ok=false;{fpga_rgb::RgbCamera camera;ck(camera.open("eth-fixture",error),error.c_str());ck(!up_calls&&force_calls==1&&normal_calls==1&&sent==1,"receive fallback or existing-up handling failed");}
    reset();up_ok=false;{fpga_rgb::RgbCamera camera;ck(!camera.open("eth-fixture",error)&&error.find("CAP_NET_ADMIN")!=std::string::npos,"up failure not explicit");ck(!sent&&!force_calls,"sent START on down interface");}
    reset();{fpga_rgb::RgbCamera camera;ck(!camera.open("wlan0",error)&&!sent,"WLAN opened as FPGA link");ck(!camera.open("../../etc",error),"invalid interface accepted");}
    setenv("PERCEPTION_RGB_SOURCE_MAC","ff:ff:ff:ff:ff:ff",1);{fpga_rgb::RgbCamera camera;ck(!camera.open("eth-fixture",error)&&error.find("unicast")!=std::string::npos,"multicast peer accepted");}unsetenv("PERCEPTION_RGB_SOURCE_MAC");
    std::cout<<"PASS FPGA RGB raw-open "<<checks<<" checks: chosen IFF_UP only, forced/fallback actual buffer, 60-byte zero-padded START; syscalls mocked, no NIC access\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
