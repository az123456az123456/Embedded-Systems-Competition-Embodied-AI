#include "imu_data.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near(double a,double b,double tolerance) {
    if(fabs(remainder(a-b,360))>tolerance) {
        fprintf(stderr,"%.9f != %.9f (tol %.6f)\n",a,b,tolerance); assert(0);
    }
}
static struct imu_sample level(void) {
    struct imu_sample v={.flags=15,.temperature=25}; v.accel[2]=1; return v;
}
static void step(struct imu_state *s,struct imu_sample *v,unsigned ms) {
    v->sequence++;v->timestamp+=ms;v->uptime+=ms;
    assert(imu_update(s,v,v->uptime));
}
static void calibrated(struct imu_state *s,struct imu_sample *v) {
    imu_init(s);*v=level();
    for(int i=0;i<250;i++) step(s,v,10);
    assert(!s->calibrating && s->calibration==200);
}
int main(void) {
    struct imu_state s;struct imu_sample v;
    // Regression: v4 reset calibration on EVERY >50ms interval. At 60ms
    // cadence it stayed at 0% forever despite a perfectly still sensor.
    imu_init(&s);v=level();v.gyro[2]=13/131.;
    for(int i=0;i<60;i++) step(&s,&v,60);
    assert(!s.calibrating);near(s.bias[2],13/131.,1e-12);
    double start=s.yaw;
    for(int i=0;i<1000;i++) step(&s,&v,60);
    near(s.yaw,start,1e-9);
    printf("PASS 60ms cadence: calibration finished; 60s drift %.9fdeg (synthetic 0.09924deg/s bias)\n",s.yaw-start);

    // Jitter, lost packets and replayed magnetometer packets cannot reset or
    // advance the quiet window incorrectly. Use sensor time for progress.
    imu_init(&s);v=level();v.gyro[2]=.1;
    unsigned intervals[]={10,60,140,30,80};
    for(int i=0;i<80;i++) {
        v.sequence+=2;step(&s,&v,intervals[i%5]);
        unsigned elapsed=s.calibration_ms,count=s.calibration_samples;
        uint64_t received=s.received_ms;
        assert(!imu_update(&s,&v,v.uptime+999));
        assert(s.received_ms==received && s.calibration_ms==elapsed && s.calibration_samples==count);
    }
    assert(!s.calibrating);near(s.bias[2],.1,1e-12);
    imu_init(&s);v=level();
    for(int i=0;i<35;i++) step(&s,&v,200);
    assert(!s.calibrating && s.calibration_samples>=32);
    puts("PASS irregular 10-200ms cadence, minimum samples, packet loss and replay freshness");

    // Bias calibration must NEVER gate orientation, even if it cannot finish.
    imu_init(&s);v=level();step(&s,&v,10);v.gyro[2]=10;
    for(int i=0;i<900;i++) step(&s,&v,10);
    assert(s.calibrating && s.calibration==0);near(s.yaw,90,.001);near(s.bias[2],0,1e-12);
    puts("PASS unfinished calibration: a 90deg real turn still reports 90deg");

    // Once calibrated, no stationary detector/online learner is allowed to
    // absorb even a continuous 0.05 or 0.1deg/s yaw into the learned bias.
    double rates[]={.05,-.05,.1,-.1,.5,45};
    for(unsigned r=0;r<sizeof(rates)/sizeof(*rates);r++) {
        calibrated(&s,&v);v.gyro[2]=rates[r];
        int count=fabs(rates[r])>1?200:2000;
        for(int i=0;i<count;i++) step(&s,&v,10);
        near(s.yaw,rates[r]*count/100,.002);near(s.bias[2],0,1e-12);
    }
    puts("PASS +/-0.05, +/-0.1, 0.5 and 45deg/s turns retain full gain; bias stays frozen");

    calibrated(&s,&v);v.gyro[2]=.02;
    for(int i=0;i<6000;i++) step(&s,&v,10);
    near(s.yaw,0,1e-12);
    s.gyro_deadband=0;
    for(int i=0;i<1000;i++) step(&s,&v,10);
    near(s.yaw,.2,.001);
    puts("PASS 0.03deg/s narrow residual band and explicit deadband disable");

    // Reproducible quantized gyro noise plus constant bias. This verifies the
    // calibration and residual band without an unrealistic exact-zero source.
    imu_init(&s);v=level();
    for(int i=0;i<250;i++) { v.gyro[2]=(13+(i%3)-1)/131.;step(&s,&v,10); }
    assert(!s.calibrating);start=s.yaw;
    for(int i=0;i<6000;i++) { v.gyro[2]=(13+(i%3)-1)/131.;step(&s,&v,10); }
    near(s.yaw,start,1e-9);
    printf("PASS 60s quantized stationary noise: yaw change %.9fdeg\n",s.yaw-start);

    // Unstable readings must not be treated as a successful still calibration.
    imu_init(&s);v=level();
    for(int i=0;i<600;i++) { v.gyro[2]=(i&1)?.3:-.3;step(&s,&v,10); }
    assert(s.calibrating);near(s.bias[2],0,1e-12);
    imu_init(&s);v=level();
    for(int i=0;i<600;i++) { v.accel[0]=(i&1)?.02:-.02;step(&s,&v,10); }
    assert(s.calibrating);near(s.bias[2],0,1e-12);
    puts("PASS gyro and acceleration vibration rejected for calibration");

    calibrated(&s,&v);s.bias[2]=.1;imu_calibrate(&s);near(s.bias[2],.1,1e-12);
    v.gyro[2]=45.1;
    for(int i=0;i<100;i++) step(&s,&v,10);
    near(s.yaw,45,.001);near(s.bias[2],.1,1e-12);assert(s.calibrating);
    puts("PASS pending manual recalibration preserves last bias and real motion");

    calibrated(&s,&v);uint64_t received=s.received_ms;
    struct imu_sample old=v;old.sequence--;old.timestamp-=10;old.uptime-=10;
    // Old packets can resemble a reboot. A full counter rollback is treated as
    // reboot by this protocol; a newer sequence with older uptime is rejected.
    old.sequence=v.sequence+1;
    assert(!imu_update(&s,&old,received+1000));assert(s.received_ms==received);
    v.gyro[0]=NAN;v.sequence++;v.timestamp+=10;v.uptime+=10;
    assert(!imu_update(&s,&v,v.uptime));assert(!imu_fresh(&s,v.uptime));
    for(int i=0;i<4;i++) assert(isfinite(s.q[i]));
    puts("PASS malformed timing, nonfinite input and finite attitude");
}
