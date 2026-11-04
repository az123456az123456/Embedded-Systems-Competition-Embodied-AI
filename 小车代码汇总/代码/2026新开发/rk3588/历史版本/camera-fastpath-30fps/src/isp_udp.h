/* RAW8 stream: LE row index + width bytes; 1080 modes require MTU >= 1950. */
#include "raw_rows.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
static int isp_udp_open(void) {
    int fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);
    if(fd<0)return -1;
    int bytes=8*1024*1024;
    if(setsockopt(fd,SOL_SOCKET,SO_RCVBUFFORCE,&bytes,sizeof(bytes))<0)
        setsockopt(fd,SOL_SOCKET,SO_RCVBUF,&bytes,sizeof(bytes));
    socklen_t length=sizeof(bytes);
    if(!getsockopt(fd,SOL_SOCKET,SO_RCVBUF,&bytes,&length))fprintf(stderr,"RAW UDP receive buffer=%d bytes\n",bytes);
    struct sockaddr_in a={.sin_family=AF_INET,.sin_port=htons(6102)};
    inet_pton(AF_INET,"192.168.0.3",&a.sin_addr);
    if(bind(fd,(struct sockaddr*)&a,sizeof(a))<0){perror("RAW UDP bind");close(fd);return -1;}
    return fd;
}
