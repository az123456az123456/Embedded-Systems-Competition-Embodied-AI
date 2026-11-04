#include "imu_data.h"
#include <math.h>
#include <stdio.h>
int main(void){
 struct imu_state s;imu_init(&s);struct imu_sample v={.flags=15,.temperature=25};
 v.accel[1]=sin(-2*IMU_PI/180);v.accel[2]=cos(-2*IMU_PI/180);
 v.gyro[0]=-303/131.;v.gyro[1]=-303/131.;v.gyro[2]=3/131.;
 double start=0,at_cal=0;
 for(int i=1;i<=7000;i++){
  v.sequence++;v.timestamp+=10;v.uptime+=10;int before=s.calibrating;imu_update(&s,&v,v.uptime);
  if(before && !s.calibrating)at_cal=s.yaw;
  if(i==1000)start=s.yaw;
 }
 printf("calibrating=%u progress=%u%% bias=(%.6f,%.6f,%.6f) yaw_at_cal=%.6f yaw_at_10s=%.6f yaw_at_70s=%.6f drift_60s=%.6f\n",s.calibrating,s.calibration/2,s.bias[0],s.bias[1],s.bias[2],at_cal,start,s.yaw,s.yaw-start);
}
