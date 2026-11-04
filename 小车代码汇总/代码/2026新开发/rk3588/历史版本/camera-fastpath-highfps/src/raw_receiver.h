#ifndef CAMERA_RAW_RECEIVER_H
#define CAMERA_RAW_RECEIVER_H
/* One producer, one consumer. The ISP never owns the receiver's write buffer.
 * No frame-ID exists on the legacy wire: only complete ordered rows qualify. */
#include "raw_rows.h"
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>

struct raw_receiver {
    int fd, started, ready, failed;
    unsigned width, height, stride;
    size_t size;
    unsigned char *writing, *latest;
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t changed;
    atomic_int stop;
    atomic_uint_fast64_t packets, complete, skipped, rejected, incomplete;
    uint64_t stamp_ms;
    int swap_pairs;
};
static uint64_t raw_clock_ms(void) {
    struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000+t.tv_nsec/1000000;
}
static void *raw_receive_loop(void *argument) {
    struct raw_receiver *r=argument;
    enum { BATCH=32, MAX_PACKET=1922 };
    unsigned char data[BATCH][MAX_PACKET];
    struct mmsghdr messages[BATCH];struct iovec iov[BATCH];
    struct raw_rows rows={.swap_pairs=r->swap_pairs};
    while(!atomic_load(&r->stop)) {
        memset(messages,0,sizeof(messages));
        for(unsigned i=0;i<BATCH;i++) {
            iov[i]=(struct iovec){.iov_base=data[i],.iov_len=MAX_PACKET};
            messages[i].msg_hdr.msg_iov=&iov[i];messages[i].msg_hdr.msg_iovlen=1;
        }
        int n=recvmmsg(r->fd,messages,BATCH,MSG_DONTWAIT|MSG_TRUNC,NULL);
        if(n<0) {
            if(errno==EINTR)continue;
            if(errno==EAGAIN || errno==EWOULDBLOCK) {
                struct pollfd p={.fd=r->fd,.events=POLLIN};
                int result=poll(&p,1,100);
                if(result>=0 && !(p.revents&(POLLERR|POLLHUP|POLLNVAL)))continue;
                if(result<0 && errno==EINTR)continue;
            }
            pthread_mutex_lock(&r->mutex);r->failed=1;
            pthread_cond_broadcast(&r->changed);pthread_mutex_unlock(&r->mutex);break;
        }
        for(int i=0;i<n;i++) {
            atomic_fetch_add(&r->packets,1);
            if(messages[i].msg_len!=r->width+2 || (messages[i].msg_hdr.msg_flags&MSG_TRUNC)) {
                rows.collecting=0;atomic_fetch_add(&r->rejected,1);continue;
            }
            unsigned row=data[i][0]|((unsigned)data[i][1]<<8);
            if(rows.collecting && (row!=rows.next || row>=r->height))atomic_fetch_add(&r->incomplete,1);
            if(raw_rows_push(&rows,r->writing,r->stride,r->width,r->height,data[i],messages[i].msg_len)) {
                pthread_mutex_lock(&r->mutex);
                unsigned char *old=r->latest;r->latest=r->writing;r->writing=old;
                if(r->ready)atomic_fetch_add(&r->skipped,1);
                r->ready=1;r->stamp_ms=raw_clock_ms();atomic_fetch_add(&r->complete,1);
                pthread_cond_signal(&r->changed);pthread_mutex_unlock(&r->mutex);
            }
        }
    }
    return NULL;
}
static void raw_receiver_close(struct raw_receiver *r) {
    atomic_store(&r->stop,1);
    if(r->started)pthread_join(r->thread,NULL);
    free(r->writing);free(r->latest);
    pthread_cond_destroy(&r->changed);pthread_mutex_destroy(&r->mutex);
}
static int raw_receiver_open(struct raw_receiver *r,int fd,unsigned stride,unsigned width,unsigned height,int swap) {
    memset(r,0,sizeof(*r));r->fd=fd;r->width=width;r->height=height;r->stride=stride;r->swap_pairs=swap;
    atomic_init(&r->stop,0);atomic_init(&r->packets,0);atomic_init(&r->complete,0);
    atomic_init(&r->skipped,0);atomic_init(&r->rejected,0);atomic_init(&r->incomplete,0);
    pthread_mutex_init(&r->mutex,NULL);pthread_cond_init(&r->changed,NULL);
    if(!width || width>1920 || !height || height>1080 || stride<width || stride>4096 || (swap && (width&1)))return -1;
    r->size=(size_t)stride*height;r->writing=calloc(1,r->size);r->latest=calloc(1,r->size);
    if(!r->writing || !r->latest)return -1;
    int rc=pthread_create(&r->thread,NULL,raw_receive_loop,r);
    if(rc){errno=rc;return -1;}r->started=1;return 0;
}
/* Copies a complete newest frame. Skipped complete frames are counted, never queued. */
static int raw_receiver_take(struct raw_receiver *r,unsigned char *dst,uint64_t *age_ms) {
    struct timespec deadline;clock_gettime(CLOCK_REALTIME,&deadline);deadline.tv_sec+=4;
    pthread_mutex_lock(&r->mutex);
    while(!r->ready && !r->failed && !atomic_load(&r->stop)) {
        if(pthread_cond_timedwait(&r->changed,&r->mutex,&deadline)==ETIMEDOUT)break;
    }
    int ok=r->ready && !r->failed;
    if(ok){memcpy(dst,r->latest,r->size);r->ready=0;*age_ms=raw_clock_ms()-r->stamp_ms;}
    pthread_mutex_unlock(&r->mutex);return ok?0:-1;
}
#endif
