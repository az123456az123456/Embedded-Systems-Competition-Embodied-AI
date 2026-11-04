#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/layered_control.h"
static struct lc_state s;static struct lc_input in;static struct lc_output out;
static unsigned pair(int a,int b){return ((unsigned)(unsigned short)a<<16)|(unsigned short)b;}
static void setup(int ax,int ay,int az){
 memset(&s,0,sizeof s);memset(&in,0,sizeof in);memset(&out,0,sizeof out);lc_init(&s);
 in.left_mask=10;in.flags=2;in.sonar_online=7;in.imu_words[0]=0x494d5539;in.imu_words[1]=0x0192000f;
 in.imu_words[4]=pair(ax,ay);in.imu_words[5]=pair(az,0);in.imu_words[6]=pair(-300,-304);in.imu_words[7]=pair(30,0);in.imu_words[14]=100;
}
static void step(void){
 unsigned n;if(!(in.now_ms%10)){in.imu_words[2]++;in.imu_words[3]=in.now_ms;in.imu_arrival_ms=in.now_ms;}
 if(!(in.now_ms%100))for(n=0;n<3;n++){in.sonar_sequence[n]++;in.sonar_stamp_ms[n]=in.now_ms;in.sonar_mm[n]=1500;}
 lc_update(&s,&in,&out);in.now_ms+=2;
}
static void run(unsigned ms){unsigned end=in.now_ms+ms;while(in.now_ms<end)step();}
static int reason(void){return (int)((out.flags>>LC_REASON_SHIFT)&255u);}
static void mounting(int ax,int ay,int az){
 double norm=sqrt((double)ax*ax+(double)ay*ay+(double)az*az);int g[3],a[3]={ax,ay,az},i;
 setup(ax,ay,az);run(2200);assert(s.calibrated&&reason()==LC_IMU_READY);assert(out.flags&LC_MOUNT_VALID);
 assert(!!(out.flags&LC_MOUNT_Z_DOWN)==(az<0));
 for(i=0;i<3;i++)g[i]=(i==0?-300:i==1?-304:30)+(int)lround(11790.0*a[i]/norm);
 in.imu_words[6]=pair(g[0],g[1]);in.imu_words[7]=pair(g[2],0);run(1000);
 assert(s.yaw_mdeg>89700&&s.yaw_mdeg<90300);
 printf("PASS mount(%d,%d,%d) gravity_q12=(%ld,%ld,%ld) positive90deg yaw=%ld bias=%ld\n",ax,ay,az,(long)s.gravity_q12[0],(long)s.gravity_q12[1],(long)s.gravity_q12[2],(long)s.yaw_mdeg,(long)s.gyro_bias_mdps);
}
static void reasons(void){
 setup(0,0,16384);in.imu_words[0]=0;step();assert(reason()==LC_IMU_WAIT_PACKET);
 in.imu_words[0]=1;step();assert(reason()==LC_IMU_BAD_FORMAT);
 in.imu_words[0]=0x494d5539;in.imu_words[1]=0x01920001;step();assert(reason()==LC_IMU_SENSOR_NOT_INITIALIZED);
 in.imu_words[1]=0x0192000f;in.imu_words[14]=101u<<16;step();assert(reason()==LC_IMU_SOURCE_STALE);
 in.imu_words[14]=100;in.now_ms=209;in.imu_arrival_ms=0;step();assert(reason()==LC_IMU_RECEIVE_STALE);
 setup(0,0,0);run(40);assert(reason()==LC_CAL_GRAVITY_RANGE);
 setup(0,0,16384);in.flags=7;in.linear_mmps=100;run(40);assert(reason()==LC_CAL_REQUESTED_MOTION);
 setup(0,0,16384);step();in.counts[0]=100;run(20);assert(reason()==LC_CAL_ENCODER_MOVING);
 setup(0,0,16384);in.imu_words[6]=pair(900,-304);run(40);assert(reason()==LC_CAL_GYRO_OFFSET_RANGE);
 setup(0,0,16384);run(100);assert(reason()==LC_CAL_COLLECTING);in.imu_words[6]=pair(-100,-304);step();
 assert(reason()==LC_CAL_GYRO_UNSTABLE);
 setup(0,0,16384);run(2200);assert(reason()==0);in.imu_words[2]=1;in.imu_words[3]=1;in.now_ms+=2;lc_update(&s,&in,&out);
 assert(!s.calibrated&&(out.flags&LC_SOURCE_RESTART_SEEN));
 puts("PASS separate packet/format/init/source-age/receive-age/motion/encoder/gravity/offset/variance/calibration/reset readiness reasons");
}
int main(void){mounting(0,0,16384);mounting(0,0,-16384);mounting(16384,0,0);mounting(1561,497,-16384);reasons();puts("PASS v28.1 installation projection and explicit readiness diagnostics; no physical hardware");return 0;}
