#include "imu_data.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static struct imu_sample level(void) {
    struct imu_sample v={.flags=15,.sequence=1,.timestamp=10,.uptime=10}; v.accel[2]=1; return v;
}
static void advance(struct imu_sample *v) { v->sequence++; v->timestamp+=10; v->uptime+=10; }
static void near(double a,double b,double eps) { if(fabs(a-b)>eps) { fprintf(stderr,"%f != %f\n",a,b); assert(0); } }
int main(void) {
    struct imu_state s; struct imu_sample v=level(); imu_init(&s);
    for(int i=0;i<220;i++) { v.gyro[0]=.25; v.gyro[1]=-.1; v.gyro[2]=.5; imu_update(&s,&v,v.uptime); advance(&v); }
    assert(!s.calibrating && s.calibration==200); near(s.bias[0],.25,1e-9); near(s.bias[2],.5,1e-9);
    double q[4]; memcpy(q,s.q,sizeof(q)); imu_update(&s,&s.sample,v.uptime); assert(!memcmp(q,s.q,sizeof(q)));
    assert(imu_fresh(&s,v.uptime)); assert(!imu_fresh(&s,v.uptime+500));
    imu_init(&s); v=level(); v.accel[1]=.5; v.accel[2]=sqrt(.75);
    imu_update(&s,&v,10); near(s.roll,30,1e-6); near(s.pitch,0,1e-6);
    imu_init(&s); v=level(); v.accel[0]=-.5; v.accel[2]=sqrt(.75);
    imu_update(&s,&v,10); near(s.pitch,30,1e-6); near(s.roll,0,1e-6);
    // Cross-axis integration: body angular velocity rotates quaternion, not
    // independently added Euler angles. A level 90deg/s yaw should turn 90deg.
    imu_init(&s); s.calibrating=0; v=level(); imu_update(&s,&v,10); v.gyro[2]=90;
    for(int i=0;i<100;i++) { advance(&v); imu_update(&s,&v,v.uptime); }
    near(s.yaw,90,.01); near(s.roll,0,1e-6); near(s.pitch,0,1e-6);
    imu_zero_yaw(&s); near(s.yaw,0,1e-6);
    advance(&v); imu_update(&s,&v,v.uptime); near(s.yaw,.9,.01);
    // Feedback sign / convergence after a tilt step.
    v.gyro[2]=0; v.accel[1]=.5; v.accel[2]=sqrt(.75);
    for(int i=0;i<600;i++) { advance(&v); imu_update(&s,&v,v.uptime); }
    near(s.roll,30,.01);
    // Startup in free-fall must not invent an attitude.
    imu_init(&s); v=level(); v.accel[2]=0; imu_update(&s,&v,10); assert(!s.initialized);
    // Out of gravity range must not pull tilt toward translational acceleration.
    imu_init(&s); s.calibrating=0; v=level(); imu_update(&s,&v,10); v.accel[0]=1;
    for(int i=0;i<100;i++) { advance(&v); imu_update(&s,&v,v.uptime); }
    near(s.pitch,0,1e-6);
    // Invalid/heartbeat packets do not integrate stale gyro.
    v.flags=12; advance(&v); memcpy(q,s.q,sizeof(q)); imu_update(&s,&v,v.uptime);
    assert(!memcmp(q,s.q,sizeof(q)) && !imu_fresh(&s,v.uptime));
    // A long sample gap restarts tilt instead of integrating across downtime.
    v=level(); v.timestamp+=1000; v.uptime+=2000; v.sequence=1000; v.gyro[2]=90;
    imu_update(&s,&v,v.uptime); near(s.yaw,0,1e-6);
    // Counter wrap is a 10ms step, FPGA reboot starts a new session.
    imu_init(&s); s.calibrating=0; v=level(); v.timestamp=v.uptime=UINT32_MAX-5; v.sequence=UINT32_MAX;
    imu_update(&s,&v,10); advance(&v); v.gyro[2]=90; imu_update(&s,&v,20); near(s.yaw,.9,.01);
    advance(&v); v.uptime=500; imu_update(&s,&v,25);
    v=level(); imu_update(&s,&v,30); assert(s.calibrating); near(s.yaw,0,1e-6);
    unsigned char b[82]={255,255,255,255,255,255,2,0xac,0x13,0x80,0,1,0x88,0xb5};
    unsigned char *p=b+14; memcpy(p,"IMU9",4); p[4]=1;p[5]=0x92;p[7]=15;
    p[16]=0xc0;p[24]=0xff;p[25]=0x7d;p[39]=0x71;p[40]=0x48;p[41]=0x68;p[59]=100;
    assert(imu_decode(b,78,&v)); near(v.accel[0],-1,1e-9); near(v.gyro[0],-1,1e-9);
    assert(!imu_decode(b,77,&v)); assert(!imu_decode(b,82,&v));
    p[39]=0x70; assert(imu_decode(b,78,&v)); assert(!(v.flags&1));
    p[4]=2; assert(!imu_decode(b,78,&v));
    puts("PASS: protocol, signed units, quaternion tilt/yaw, bias calibration, duplicate packets, staleness, gravity gate, gaps, wrap and reboot");
}
