#include "../backend/perception/imu_data.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int checks;
static void check(int value,const char *message) {
    ++checks;
    if(!value){fprintf(stderr,"FAIL: %s\n",message);exit(1);}
}
static struct imu_sample sample(uint32_t seq,uint32_t ms) {
    struct imu_sample v={0};
    v.flags=15;v.sequence=seq;v.timestamp=ms;v.uptime=ms;v.accel[2]=1;
    return v;
}
int main(void) {
    struct imu_state state;
    imu_init(&state);
    struct imu_sample first=sample(10,1000);
    check(imu_update(&state,&first,10000)==1&&imu_fresh(&state,10000),"initial valid IMU sample accepted");
    struct imu_sample duplicate=first;duplicate.accel[0]=.4;duplicate.ag_age=0;
    check(!imu_update(&state,&duplicate,10490),"duplicate IMU sequence rejected");
    check(state.received_ms==10000&&state.sample.accel[0]==0,"duplicate cannot replace sample or reception time");
    check(!imu_fresh(&state,10500),"replayed data expires at original freshness deadline");
    struct imu_sample older=first;older.sequence=9;older.uptime=1000;
    check(!imu_update(&state,&older,11000)&&state.received_ms==10000,"out of order sequence cannot renew reception time");
    struct imu_sample next=sample(11,1010);next.ag_age=200;
    check(imu_update(&state,&next,11010)&&imu_fresh(&state,11309)&&!imu_fresh(&state,11310),"source sample age is included in freshness");
    struct imu_sample invalid=sample(12,1020);invalid.flags=0;
    check(!imu_update(&state,&invalid,11020)&&!imu_fresh(&state,11020),"new invalid sample immediately clears freshness");
    invalid.flags=15;
    check(!imu_update(&state,&invalid,11030)&&!imu_fresh(&state,11030),"same invalid sequence cannot become valid by replaying modified flags");
    struct imu_sample stale=sample(13,1030);stale.ag_age=500;
    check(!imu_update(&state,&stale,11040)&&!imu_fresh(&state,11040),"new source-expired sample never appears fresh");
    next=sample(14,1040);
    check(imu_update(&state,&next,11050)&&imu_fresh(&state,11050),"later genuinely new valid data restores freshness");

    imu_init(&state);
    first=sample(UINT32_C(0xfffffffe),UINT32_C(0xfffffff0));
    imu_update(&state,&first,20000);
    next=sample(1,20);
    check(imu_update(&state,&next,20036)&&state.sample.sequence==1&&state.calibration==2,"natural sequence and uptime wrap remain continuous");
    first=sample(2,30);imu_update(&state,&first,20046);
    struct imu_sample reset=sample(1,5);
    check(imu_update(&state,&reset,20100)&&state.calibrating&&state.calibration==1&&state.sample.sequence==1,"backward uptime restarts orientation and stationary calibration");

    imu_init(&state);
    for(uint32_t i=1;i<=200;i++){next=sample(i,i*10);imu_update(&state,&next,30000+i*10);}
    check(!state.calibrating&&state.calibration==200&&state.initialized,"two hundred consecutive still samples complete gyro calibration");
    check(fabs(state.roll)<1e-9&&fabs(state.pitch)<1e-9&&fabs(state.yaw)<1e-9,"stationary gravity produces finite zero attitude");
    next=sample(201,2010);next.gyro[2]=90;
    imu_update(&state,&next,32010);
    check(state.yaw>.8&&state.yaw<1.0,"new positive Z gyro produces positive CCW yaw");
    imu_zero_yaw(&state);
    check(fabs(state.yaw)<1e-9,"yaw reference zero works");
    imu_calibrate(&state);
    check(state.calibrating&&state.calibration==0&&state.bias[0]==0,"recalibration resets accumulation and biases");
    puts("PASS IMU: duplicate/old sequence freshness, invalid samples, source age, counter wrap, restart, calibration, yaw");
    printf("%d checks\n",checks);
    return 0;
}
