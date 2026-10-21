#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include "imu_data.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <dirent.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <sys/socket.h>
#include <unistd.h>
static uint16_t u16(const unsigned char *p) { return (p[0]<<8)|p[1]; }
static uint32_t u32(const unsigned char *p) { return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
static int s16(const unsigned char *p) { unsigned n=u16(p); return n<32768?(int)n:(int)n-65536; }
int imu_decode(const unsigned char *b, size_t n, struct imu_sample *s) {
    const unsigned char header[]={255,255,255,255,255,255,2,0xac,0x13,0x80,0,1,0x88,0xb5};
    if ((n!=78 && n!=82) || memcmp(b,header,14)) return 0;
    const unsigned char *p=b+14;
    if (memcmp(p,"IMU9",4) || p[4]!=1 || p[5]!=0x92 || (u16(p+6)&~63u) ||
        (p[41]!=0x68 && p[41]!=0x69) || u16(p+58)!=100) return 0;
    if(n==82) {
        uint32_t crc=~0u;
        for(int j=0;j<78;j++) { crc^=b[j]; for(int k=0;k<8;k++) crc=(crc>>1)^((crc&1)?0xedb88320u:0); }
        uint32_t f=b[78]|((uint32_t)b[79]<<8)|((uint32_t)b[80]<<16)|((uint32_t)b[81]<<24);
        if(~crc!=f) return 0;
    }
    memset(s,0,sizeof(*s));
    s->flags=u16(p+6);
    if(p[39]!=0x71 || !(s->flags&4)) s->flags&=~1u;
    s->ag_age=u16(p+56); s->mag_age=u16(p+42);
    s->sequence=u32(p+8); s->timestamp=u32(p+12); s->uptime=u32(p+52);
    s->errors=u32(p+44); s->drops=u32(p+48);
    for(int i=0;i<3;i++) { s->accel[i]=s16(p+16+i*2)/16384.0; s->gyro[i]=s16(p+24+i*2)/131.0; }
    s->temperature=s16(p+22)/333.87+21.0;
    return 1;
}
static double norm3(const double *v) { return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
static double wrap(double a) { return atan2(sin(a),cos(a)); }
void imu_calibrate(struct imu_state *s) {
    s->calibration=0; s->calibrating=1;
    memset(s->sum,0,sizeof(s->sum)); memset(s->bias,0,sizeof(s->bias));
}
void imu_init(struct imu_state *s) { memset(s,0,sizeof(*s)); s->q[0]=1; imu_calibrate(s); }
void imu_zero_yaw(struct imu_state *s) { s->yaw_zero=wrap(s->yaw*IMU_PI/180+s->yaw_zero); s->yaw=0; }
int imu_fresh(const struct imu_state *s, uint64_t now) {
    return s->have_packet && (s->sample.flags&1) && now>=s->received_ms &&
           now-s->received_ms+s->sample.ag_age<500;
}
static void angles(struct imu_state *s) {
    double w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3];
    s->roll=atan2(2*(w*x+y*z),1-2*(x*x+y*y))*180/IMU_PI;
    s->pitch=asin(fmax(-1,fmin(1,2*(w*y-z*x))))*180/IMU_PI;
    s->yaw=wrap(atan2(2*(w*z+x*y),1-2*(y*y+z*z))-s->yaw_zero)*180/IMU_PI;
}
int imu_update(struct imu_state *s, const struct imu_sample *v, uint64_t now) {
    // Modular counters support natural 32-bit wrap. A backward uptime indicates
    // FPGA reconfiguration; discard the previous orientation/time reference.
    if(s->have_packet && (int32_t)(v->uptime-s->previous_uptime)<0) imu_init(s);
    s->sample=*v; s->received_ms=now; s->have_packet=1; s->previous_uptime=v->uptime;
    if (!(v->flags&1) || v->ag_age>=500) {
        if(s->calibrating) { s->calibration=0; memset(s->sum,0,sizeof(s->sum)); }
        return 0;
    }
    if (s->have_sequence && (int32_t)(v->sequence-s->previous_sequence)<=0) return 0;
    uint32_t dtms=v->timestamp-s->previous_timestamp;
    double n=norm3(v->accel);
    if(s->have_sequence && (!dtms || dtms>200)) s->initialized=0;
    if(s->calibrating && s->have_sequence && dtms>50) {
        s->calibration=0; memset(s->sum,0,sizeof(s->sum));
    }
    s->previous_sequence=v->sequence; s->previous_timestamp=v->timestamp; s->have_sequence=1;
    if(s->calibrating) {
        // Keep the module still for 200 consecutive new samples (~2 seconds).
        if(fabs(n-1)<0.08 && norm3(v->gyro)<3) {
            for(int i=0;i<3;i++) s->sum[i]+=v->gyro[i];
            if(++s->calibration>=200) {
                for(int i=0;i<3;i++) s->bias[i]=s->sum[i]/s->calibration;
                s->calibrating=0;
            }
        } else { s->calibration=0; memset(s->sum,0,sizeof(s->sum)); }
    }
    if(!s->initialized) {
        if(n<0.85 || n>1.15) return 0;
        double r=atan2(v->accel[1],v->accel[2])/2;
        double p=atan2(-v->accel[0],hypot(v->accel[1],v->accel[2]))/2;
        s->q[0]=cos(r)*cos(p); s->q[1]=sin(r)*cos(p);
        s->q[2]=cos(r)*sin(p); s->q[3]=-sin(r)*sin(p);
        s->yaw_zero=0; s->initialized=1; angles(s); return 1;
    }
    // Six-axis quaternion complementary filter (Mahony proportional feedback).
    // Acceleration outside the gravity band does not correct tilt.
    double w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3];
    double g[3]={2*(x*z-w*y),2*(w*x+y*z),1-2*(x*x+y*y)};
    double omega[3];
    for(int i=0;i<3;i++) omega[i]=(v->gyro[i]-s->bias[i])*IMU_PI/180;
    if(n>0.85 && n<1.15) {
        double a[3]; for(int i=0;i<3;i++) a[i]=v->accel[i]/n;
        omega[0]+=2.0*(a[1]*g[2]-a[2]*g[1]);
        omega[1]+=2.0*(a[2]*g[0]-a[0]*g[2]);
        omega[2]+=2.0*(a[0]*g[1]-a[1]*g[0]);
    }
    double t=dtms/2000.0, gx=omega[0],gy=omega[1],gz=omega[2];
    s->q[0]+=(-x*gx-y*gy-z*gz)*t; s->q[1]+=(w*gx+y*gz-z*gy)*t;
    s->q[2]+=(w*gy-x*gz+z*gx)*t; s->q[3]+=(w*gz+x*gy-y*gx)*t;
    double length=sqrt(s->q[0]*s->q[0]+s->q[1]*s->q[1]+s->q[2]*s->q[2]+s->q[3]*s->q[3]);
    for(int i=0;i<4;i++) s->q[i]/=length;
    angles(s); return 1;
}
uint64_t imu_clock_ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return (uint64_t)t.tv_sec*1000+t.tv_nsec/1000000; }
int imu_open_socket(const char *requested, char name[16]) {
    memset(name,0,16);
    if(requested) {
        if(strlen(requested)>=IFNAMSIZ) { errno=EINVAL; return -1; }
        strcpy(name,requested);
    } else {
        DIR *d=opendir("/sys/class/net");
        if(!d) return -1;
        struct dirent *e;
        while((e=readdir(d))) {
            char path[512],target[4096];
            snprintf(path,sizeof(path),"/sys/class/net/%s/device",e->d_name);
            if(!realpath(path,target)) continue;
            const char *base=strrchr(target,'/');
            if(base && !strcmp(base+1,"fe1c0000.ethernet") && strlen(e->d_name)<16) strcpy(name,e->d_name);
        }
        closedir(d);
    }
    unsigned index=if_nametoindex(name);
    if(!index) { errno=ENODEV; return -1; }
    int fd=socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(0x88b5));
    if(fd<0) return -1;
    struct sockaddr_ll a={0}; a.sll_family=AF_PACKET; a.sll_protocol=htons(0x88b5); a.sll_ifindex=(int)index;
    if(bind(fd,(struct sockaddr *)&a,sizeof(a))) { int err=errno; close(fd); errno=err; return -1; }
    return fd;
}
