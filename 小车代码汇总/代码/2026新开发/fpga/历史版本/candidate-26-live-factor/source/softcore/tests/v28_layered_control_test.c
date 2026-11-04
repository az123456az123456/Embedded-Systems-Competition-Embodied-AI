#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "../firmware/layered_control.h"
static struct lc_state s;
static struct lc_input in;
static struct lc_output out;
static unsigned commits;
static int abs_i(int x) {return x<0?-x:x;}
static void reset(void) {
 memset(&s,0,sizeof s);memset(&in,0,sizeof in);memset(&out,0,sizeof out);
 lc_init(&s); in.left_mask=10;in.flags=LC_CONFIGURED;in.sonar_online=7;commits=0;
 in.imu_words[0]=0x494d5539;in.imu_words[1]=0x0192000f;in.imu_words[5]=16384u<<16;
}
static void imu(int gz) {
 ++in.imu_words[2];in.imu_words[3]=in.now_ms;in.imu_arrival_ms=in.now_ms;
 in.imu_words[7]=(uint32_t)(uint16_t)gz<<16;in.imu_words[14]=100;
}
static void sonar(unsigned i,unsigned mm) {
 ++in.sonar_sequence[i];in.sonar_mm[i]=(uint16_t)mm;in.sonar_stamp_ms[i]=in.now_ms;
}
static void update(void){commits+=(unsigned)lc_update(&s,&in,&out);}
static void stationary(unsigned duration_ms,int gz) {
 unsigned end=in.now_ms+duration_ms;
 for(;in.now_ms<end;in.now_ms+=2) {
  if(!(in.now_ms%10))imu(gz);
  if(!(in.now_ms%100)) {sonar(0,1500);sonar(1,1600);sonar(2,1700);}
  update();
 }
}
static void test_bias_and_yaw(void){
 reset();stationary(2200,26);assert(s.calibrated);assert(out.calibration_progress==100);
 assert(s.gyro_bias_mdps==198);assert(s.yaw_mdeg==0);
 /* Duplicate source frames do not advance calibration or yaw. */
 {int count=s.calibration_count;int yaw=s.yaw_mdeg;update();update();assert(s.calibration_count==count);assert(s.yaw_mdeg==yaw);}
 stationary(1000,26+11790);assert(s.yaw_mdeg>89000&&s.yaw_mdeg<91000);
 stationary(1000,26-11790);assert(abs_i(s.yaw_mdeg)<1000);
 /* ±180 crossing is represented continuously by wrapped relative heading. */
 stationary(2000,26+11790);assert(abs_i(abs_i(s.yaw_mdeg)-180000)<1000);
 /* Invalid age does not freeze freshness at a previous good commit. */
 in.now_ms+=110;update();assert(!(out.flags&2u));
 printf("PASS calibration fixed offset, signed 90deg turns, yaw wrap, duplicate and stale IMU\n");
}
static void test_calibration_motion_and_slow_turn(void){
 reset();stationary(1000,26);assert(!s.calibrated);
 in.flags=7;in.linear_mmps=100;stationary(1500,26);assert(!s.calibrated);
 in.linear_mmps=0;stationary(2200,26);assert(s.calibrated);
 {int before=s.yaw_mdeg;stationary(10000,33);assert(s.yaw_mdeg-before>400&&s.yaw_mdeg-before<650);}
 printf("PASS calibration blocks requested motion; 0.053deg/s real slow turn retained\n");
}
static void test_pi_and_immediate_zero(void){
 unsigned t,i;int64_t pos[4]={0};int speed[4]={150,180,120,170};
 reset();stationary(2200,0);in.flags=7;in.linear_mmps=200;in.epoch=1;
 for(t=0;t<2000;t+=2,in.now_ms+=2) {
  if(!(in.now_ms%10))imu(0);
  if(!(in.now_ms%100)){sonar(0,2000);sonar(1,2000);sonar(2,2000);}
  for(i=0;i<4;++i) {
   int measured=t<500?speed[i]:200;
   pos[i]+=(int64_t)measured*2000000/LC_TICK_NM;
   in.counts[i]=(int32_t)(pos[i]/1000);if(i==0||i==2)in.counts[i]=-in.counts[i];
  }
  update();
  for(i=0;i<4;++i)assert(out.duty[i]>=0&&out.duty[i]<=600);
 }
 assert(out.duty[2]>out.duty[1]); /* slower M3 accumulated more compensation */
 assert(abs_i(s.wheel_mmps[0]-200)<=1);
 in.linear_mmps=0;in.now_ms+=2;imu(0);update();
 for(i=0;i<4;++i){assert(!out.duty[i]);assert(!s.integral_q8[i]);}
 assert(!s.active_previous);
 /* Very large modulo count jump is rejected without signed-overflow UB. */
 in.linear_mmps=200;in.counts[0]=INT_MIN;in.now_ms+=20;imu(0);update();
 for(i=0;i<4;++i)assert(!out.duty[i]);
 in.flags=LC_CONFIGURED;in.now_ms+=2;update();for(i=0;i<4;++i)assert(!out.duty[i]);
 printf("PASS four-wheel PI physical signs, unequal-wheel corrections, hard zero and overflow-safe bad counts\n");
}
static void test_heading_and_clipping(void){
 unsigned i; reset();stationary(2200,0);in.flags=7;in.linear_mmps=300;in.yaw_rate_mradps=1000;in.epoch=1;
 in.now_ms+=20;imu(0);update();
 assert(out.duty[1]<out.duty[0]);assert(out.duty[3]<out.duty[2]);
 for(i=0;i<4;++i)assert(abs_i(out.duty[i])<=600);
 /* Positive yaw disturbance during straight travel must slow right, accelerate left. */
 in.yaw_rate_mradps=0;in.linear_mmps=150;
 for(i=0;i<20;++i){in.now_ms+=10;imu(1310);update();}
 assert(out.duty[1]>out.duty[0]);
 printf("PASS CCW heading convention, derivative damping and proportional wheel speed clipping\n");
}
static void test_kalman(void){
 unsigned n;int total_raw=0,total_filtered=0,previous=1000,prev_filtered=1000;
 reset();stationary(2200,0);
 for(n=0;n<100;++n) {
  int z=1000+(n&1?20:-20);in.now_ms+=100;imu(0);sonar(0,(unsigned)z);update();
  if(n>10){total_raw+=abs_i(z-previous);total_filtered+=abs_i((int)out.sonar_mm[0]-prev_filtered);}
  previous=z;prev_filtered=out.sonar_mm[0];
 }
 assert(total_filtered<total_raw/2);
 in.now_ms+=100;imu(0);sonar(0,200);update();assert(out.sonar_mm[0]==200);assert(out.flags&(1u<<8));
 /* Two far frames 500ms apart can differ arbitrarily: no adjacency trap. */
 in.now_ms+=500;imu(0);sonar(0,1500);update();assert(!(out.flags&(1u<<8)));
 in.now_ms+=500;imu(0);sonar(0,4200);update();assert(out.sonar_mm[0]==4200);assert(out.flags&(1u<<8));
 assert(out.sonar_sequence[0]==in.sonar_sequence[0]);
 {uint32_t stamp=s.sonar[0].stamp_ms;in.now_ms+=752;imu(0);update();assert(!(out.flags&(1u<<8)));assert(s.sonar[0].stamp_ms==stamp);}
 in.now_ms+=100;imu(0);sonar(0,65535);update();assert(!(out.flags&(1u<<8)));assert(out.sonar_mm[0]!=4500);
 /* A turning scene bypasses old-wall averaging on its first new frame. */
 in.now_ms+=100;imu(3000);sonar(0,800);update();assert(out.sonar_mm[0]==800);
 printf("PASS scalar KF jitter suppression, near bypass, finite far recovery, real source age, invalid sentinel\n");
}
static void test_obstacle_windup(void){
 unsigned i;reset();stationary(2200,0);in.flags=7;in.linear_mmps=150;in.epoch=1;
 in.now_ms+=20;imu(0);update();assert(out.duty[0]>0);
 in.now_ms+=2;imu(0);sonar(0,200);update();
 for(i=0;i<4;++i){assert(!out.duty[i]);assert(!s.integral_q8[i]);}
 assert((out.flags&3u)==3u);assert(!s.active_previous);
 /* Loss of sonar freshness must not be reported as lost IMU calibration. */
 in.now_ms+=20;imu(0);in.sonar_online=6;update();
 assert((out.flags&3u)==3u);for(i=0;i<4;++i)assert(!out.duty[i]);
 puts("PASS near/invalid sonar clears PI and trajectory without clearing CPU/IMU readiness");
}
int main(void){
 assert(LC_TICK_NM==3388&&LC_TRACK_UM==180635&&LC_CPR_X10==605277);
 test_bias_and_yaw();test_calibration_motion_and_slow_turn();test_pi_and_immediate_zero();test_heading_and_clipping();test_kalman();test_obstacle_windup();
 puts("PASS v28 production fixed-point control unit tests (host sanitizers)");return 0;
}
