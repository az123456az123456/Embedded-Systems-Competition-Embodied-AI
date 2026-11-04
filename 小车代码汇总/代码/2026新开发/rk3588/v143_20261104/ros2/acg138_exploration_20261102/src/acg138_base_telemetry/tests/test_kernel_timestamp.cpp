#include "acg138_base_telemetry/rx_stamp.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
static void require(bool v,const char*m){if(!v)throw std::runtime_error(m);}
int main(){
 int fd[2];require(socketpair(AF_UNIX,SOCK_DGRAM,0,fd)==0,"local UNIX socketpair required");
 int on=1;require(setsockopt(fd[1],SOL_SOCKET,SO_TIMESTAMPNS,&on,sizeof(on))==0,"SO_TIMESTAMPNS unavailable");
 require(send(fd[0],"A",1,0)==1,"local send failed");
 std::this_thread::sleep_for(std::chrono::milliseconds(80));
 char data=0;alignas(cmsghdr) unsigned char ancillary[CMSG_SPACE(sizeof(timespec))]={};iovec iov{&data,1};msghdr message{};message.msg_iov=&iov;message.msg_iovlen=1;message.msg_control=ancillary;message.msg_controllen=sizeof(ancillary);
 require(recvmsg(fd[1],&message,0)==1&&data=='A',"recvmsg failed");
 auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
 auto stamp=acg138::receive_stamp(message,ns,10000);require(bool(stamp),"kernel timestamp was lost");
 const auto age_ms=(ns-stamp->realtime_ns)/1000000LL;require(age_ms>=60&&age_ms<250,"queued packet should preserve original arrival age");
 require(!acg138::receive_stamp(message,stamp->realtime_ns+300000000LL,10000),"300ms old packet must be rejected");
 close(fd[0]);close(fd[1]);std::cout<<"PASS real kernel SO_TIMESTAMPNS preserves queued age_ms="<<age_ms<<"; stale data not rejuvenated\n";
}
