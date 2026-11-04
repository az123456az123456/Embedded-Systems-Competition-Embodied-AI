/* Run the actual monitor main() against an in-memory raw-socket endpoint.
 * No hardware, socket, sleeping, privilege change or physical IO is used. */
#define main production_monitor_main
#include "softcore-monitor.c"
#undef main
#include <assert.h>

static uint64_t simulated_ms = 1000;
static unsigned supplied;
static const char *mode;
unsigned __wrap_if_nametoindex(const char *name) { assert(!strcmp(name,"fixture"));return 7; }
int __wrap_socket(int family,int type,int protocol) {
    (void)type;assert(family==AF_PACKET && ntohs((uint16_t)protocol)==NAV_ETHERTYPE);return 12345;
}
int __wrap_bind(int fd,const struct sockaddr *address,socklen_t n) {
    (void)n;const struct sockaddr_ll *a=(const struct sockaddr_ll *)address;
    assert(fd==12345 && a->sll_ifindex==7);return 0;
}
int __wrap_close(int fd) { assert(fd==12345);return 0; }
int __wrap_clock_gettime(clockid_t which,struct timespec *t) {
    assert(which==CLOCK_MONOTONIC);t->tv_sec=(time_t)(simulated_ms/1000);
    t->tv_nsec=(long)(simulated_ms%1000)*1000000;return 0;
}
int __wrap_poll(struct pollfd *fds,nfds_t count,int timeout) {
    assert(count==1 && fds[0].fd==12345);simulated_ms+=(uint64_t)timeout;
    fds[0].revents=supplied<100?POLLIN:0;return supplied<100?1:0;
}
ssize_t __wrap_recvfrom(int fd,void *buffer,size_t n,int flags,struct sockaddr *address,socklen_t *size) {
    (void)flags;(void)size;assert(fd==12345 && n>=60);
    if(supplied>=100 || !strcmp(mode,"none")){supplied=100;errno=EAGAIN;return -1;}
    uint8_t frame[60];fixture(frame);uint8_t *p=frame+14;
    p[5]=!strcmp(mode,"trap")?4:3;
    put32(p+8,!strcmp(mode,"healthy")?supplied:0);put32(p+20,0);
    put16(p+26,crc16(p,26));memcpy(buffer,frame,60);
    ((struct sockaddr_ll *)address)->sll_pkttype=PACKET_HOST;++supplied;return 60;
}
int main(int argc,char **argv) {
    assert(argc==2);mode=argv[1];
    char *args[]={"softcore-monitor","--interface","fixture","--count","100","--timeout","1",NULL};
    return production_monitor_main(7,args);
}
