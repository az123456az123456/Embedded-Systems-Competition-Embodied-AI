#define _GNU_SOURCE
#include "raw_receiver.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
static void row(int fd,unsigned y,unsigned char value) {
    unsigned char packet[1922];packet[0]=y;packet[1]=y>>8;
    memset(packet+2,value,1920);
    ssize_t sent;do sent=send(fd,packet,sizeof(packet),0);while(sent<0 && errno==EINTR);
    if(sent!=(ssize_t)sizeof(packet))perror("test row send");
    assert(sent==(ssize_t)sizeof(packet));
}
int main(void) {
    int fd[2];assert(socketpair(AF_UNIX,SOCK_DGRAM,0,fd)==0);
    struct raw_receiver r;assert(raw_receiver_open(&r,fd[1],2048,1920,1080,0)==0);
    // Reordered / missing rows never form a complete frame.
    row(fd[0],0,42);row(fd[0],2,42);
    for(unsigned y=3;y<1080;y++)row(fd[0],y,42);
    unsigned char invalid[3]={0};assert(send(fd[0],invalid,sizeof(invalid),0)==sizeof(invalid));
    for(unsigned frame=1;frame<=3;frame++) {
        for(unsigned y=0;y<1080;y++)row(fd[0],y,frame);
        for(int wait=0;wait<1000 && atomic_load(&r.complete)<frame;wait++)usleep(1000);
        assert(atomic_load(&r.complete)==frame);
        // Consumer intentionally does not run until three frames have arrived.
    }
    unsigned char *output=malloc(2048*1080);assert(output);uint64_t age=0;
    assert(raw_receiver_take(&r,output,&age)==0);
    for(unsigned y=0;y<1080;y++)for(unsigned x=0;x<2048;x++)assert(output[y*2048+x]==(x<1920?3:0));
    assert(atomic_load(&r.skipped)==2 && atomic_load(&r.rejected)==1 && atomic_load(&r.incomplete)==1);
    raw_receiver_close(&r);free(output);close(fd[0]);close(fd[1]);
    puts("PASS full 1920x1080 datagrams, continuous receive during slow consumer, latest complete frame, bounded buffers, missing/reordered/truncated rows, stride padding, stop/join");
}
