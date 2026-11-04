#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include "sensor_data.h"
#include <arpa/inet.h>
#include <errno.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
static uint16_t u16(const unsigned char *p) { return ((uint16_t)p[0]<<8)|p[1]; }
static uint32_t u32(const unsigned char *p) { return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
static int32_t s32(const unsigned char *p) {
    uint32_t v=u32(p);
    return v<=INT32_MAX?(int32_t)v:(int32_t)((int64_t)v-INT64_C(4294967296));
}
static int after(uint32_t a,uint32_t b) { uint32_t d=a-b;return d&&d<UINT32_C(0x80000000); }
uint16_t sensor_crc16(const unsigned char *p,size_t n) {
    uint16_t crc=0xffff;
    while(n--) { crc^=(uint16_t)*p++<<8;for(unsigned k=0;k<8;k++)crc=(uint16_t)((crc<<1)^((crc&0x8000)?0x1021:0)); }
    return crc;
}
static int valid_sample(const struct sensor_sample *v) {
    if((v->valid_mask|v->fault_mask)!=7 || (v->valid_mask&v->fault_mask) ||
       (v->invert_mask&~15u) || (v->illegal_mask&~15u) || (v->window_ms!=0 && v->window_ms!=50))return 0;
    /* window=0 is an immediate sonar event. Counts are absolute snapshots;
       zero deltas must never masquerade as another 50 ms speed sample. */
    if(!v->window_ms)for(unsigned i=0;i<4;i++)if(v->delta[i])return 0;
    for(unsigned i=0;i<3;i++)if(v->valid_mask&(1u<<i))
        if(v->distance_mm[i]<30 || v->distance_mm[i]>SENSOR_RANGE_MAX_MM || v->age_ms[i]>SENSOR_MAX_AGE_MS)return 0;
    return 1;
}
int sensor_decode(const unsigned char *b,size_t n,struct sensor_sample *out) {
    static const unsigned char header[]={255,255,255,255,255,255,2,0xac,0x13,0x80,0,1,0x88,0xb8};
    if(!b||!out||(n!=SENSOR_FRAME_BYTES&&n!=SENSOR_FRAME_BYTES+4)||memcmp(b,header,14))return 0;
    const unsigned char *p=b+14;
    if(memcmp(p,"SNS1",4)||p[4]!=1||p[63]||sensor_crc16(p,64)!=u16(p+64))return 0;
    if(n==SENSOR_FRAME_BYTES+4) {
        uint32_t crc=UINT32_MAX;
        for(unsigned i=0;i<SENSOR_FRAME_BYTES;i++){crc^=b[i];for(unsigned j=0;j<8;j++)crc=(crc>>1)^((crc&1)?UINT32_C(0xedb88320):0);}
        uint32_t fcs=(uint32_t)b[80]|((uint32_t)b[81]<<8)|((uint32_t)b[82]<<16)|((uint32_t)b[83]<<24);
        if(~crc!=fcs)return 0;
    }
    struct sensor_sample v={0};
    v.valid_mask=p[5];v.fault_mask=p[6];v.invert_mask=p[7];v.sequence=u32(p+8);v.uptime_ms=u32(p+12);
    for(unsigned i=0;i<4;i++){v.position[i]=s32(p+16+i*4);v.delta[i]=s32(p+32+i*4);}
    for(unsigned i=0;i<3;i++){v.distance_mm[i]=u16(p+48+i*2);v.age_ms[i]=u16(p+54+i*2);}
    v.window_ms=u16(p+60);v.illegal_mask=p[62];
    if(!valid_sample(&v))return 0;
    *out=v;return 1;
}
void sensor_init(struct sensor_state *s) { memset(s,0,sizeof(*s)); }
int sensor_update(struct sensor_state *s,const struct sensor_sample *v,uint64_t now) {
    if(!valid_sample(v))return 0;
    uint64_t stamp=now;
    if(s->have_sample) {
        if(now<s->received_ms)return 0;
        uint64_t gap=now-s->received_ms;
        int sequence_new=after(v->sequence,s->sample.sequence),uptime_new=after(v->uptime_ms,s->sample.uptime_ms);
        if(sequence_new&&(uptime_new || (v->uptime_ms==s->sample.uptime_ms && gap<=100u))) {
            /* Multiple sonar channels can finish within one FPGA millisecond.
               Retain that millisecond's first receive time: extra packets must
               not make a frozen device clock or old observation look fresh. */
            if(!uptime_new)stamp=s->received_ms;
            /* Reject a delayed pre-restart packet jumping back to the old boot.
               The device cannot advance far faster than elapsed host time. */
            if((uint32_t)(v->uptime_ms-s->sample.uptime_ms)>gap+SENSOR_MAX_AGE_MS+100u)return 0;
            s->restart_pending=0;
        } else {
            /* There is no boot nonce in SNS1. Require two advancing packets with plausible reboot uptime
               after silence; one reordered/replayed packet cannot reset
               freshness. Natural 32-bit sequence/uptime wraps use after(). */
            int restart=(uint64_t)v->uptime_ms<=gap+2000u&&gap>=250u&&
                s->sample.uptime_ms>v->uptime_ms&&s->sample.uptime_ms-v->uptime_ms>=100u&&
                !sequence_new;
            if(!restart)return 0;
            if(!s->restart_pending||now<s->restart_seen_ms||now-s->restart_seen_ms>500u||
               !after(v->sequence,s->restart_sequence)||!after(v->uptime_ms,s->restart_uptime)) {
                s->restart_pending=1;s->restart_sequence=v->sequence;s->restart_uptime=v->uptime_ms;s->restart_seen_ms=now;
                return 0;
            }
            s->restarts++;s->restart_pending=0;
        }
    }
    s->sample=*v;s->received_ms=stamp;s->have_sample=1;return 1;
}
int sensor_link_fresh(const struct sensor_state *s,uint64_t now) {
    return s->have_sample&&now>=s->received_ms&&now-s->received_ms<=SENSOR_MAX_AGE_MS;
}
enum sensor_distance_status sensor_distance(const struct sensor_state *s,unsigned i,uint64_t now,double *metres) {
    if(metres)*metres=0;
    if(i>=3||!s->have_sample)return SENSOR_WAITING;
    if(!sensor_link_fresh(s,now))return SENSOR_STALE;
    if(s->sample.age_ms[i]==UINT16_MAX && (s->sample.fault_mask&(1u<<i)))return SENSOR_WAITING;
    uint64_t age=now-s->received_ms+s->sample.age_ms[i];
    if(age>SENSOR_MAX_AGE_MS)return SENSOR_STALE;
    if(!(s->sample.valid_mask&(1u<<i))||(s->sample.fault_mask&(1u<<i)))return SENSOR_FAULT;
    if(metres)*metres=s->sample.distance_mm[i]/1000.;
    return SENSOR_VALID;
}
enum sensor_distance_status sensor_distance_text(const struct sensor_state *s,unsigned i,uint64_t now,char *text,size_t size) {
    double metres=0;
    const enum sensor_distance_status status=sensor_distance(s,i,now,&metres);
    if(text&&size) {
        if(status!=SENSOR_VALID)snprintf(text,size,"-- m");
        else if(s->sample.distance_mm[i]>SENSOR_DISPLAY_LIMIT_MM)snprintf(text,size,">4 m");
        else snprintf(text,size,"%.2f m",metres);
    }
    return status;
}
int sensor_open_socket(const char *name) {
    if(!name||!*name||strlen(name)>=IFNAMSIZ){errno=EINVAL;return -1;}
    unsigned index=if_nametoindex(name);if(!index){errno=ENODEV;return -1;}
    int fd=socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(SENSOR_ETHERTYPE));
    if(fd<0)return -1;
    struct sockaddr_ll address={.sll_family=AF_PACKET,.sll_protocol=htons(SENSOR_ETHERTYPE),.sll_ifindex=(int)index};
    if(bind(fd,(struct sockaddr *)&address,sizeof(address))<0){int error=errno;close(fd);errno=error;return -1;}
    return fd;
}
