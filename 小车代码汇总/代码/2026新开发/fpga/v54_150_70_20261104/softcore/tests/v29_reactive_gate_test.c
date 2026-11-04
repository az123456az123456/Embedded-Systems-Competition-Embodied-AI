#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/layered_control.h"
static struct lc_state s;
static struct lc_input in;
static struct lc_output out;
static void refresh(void) {
 unsigned i;in.imu_words[2]++;in.imu_words[3]=in.now_ms;
 in.imu_arrival_ms=in.now_ms;
 for(i=0;i<3;i++){in.sonar_sequence[i]++;in.sonar_stamp_ms[i]=in.now_ms;}
}
static void tick(unsigned ms){in.now_ms+=ms;refresh();(void)lc_update(&s,&in,&out);}
static void zero(void){unsigned i;for(i=0;i<4;i++){assert(out.duty[i]==0);assert(s.integral_q8[i]==0);}}
static void ready(void){assert(s.calibrated && (out.flags&3u)==3u);}
static void setup(void){
 unsigned i;memset(&s,0,sizeof s);memset(&in,0,sizeof in);memset(&out,0,sizeof out);lc_init(&s);
 in.left_mask=10;in.flags=LC_CONFIGURED;in.sonar_online=7;
 in.imu_words[0]=0x494d5539;in.imu_words[1]=0x0192000f;in.imu_words[5]=16384u<<16;
 for(i=0;i<3;i++)in.sonar_mm[i]=250;
 for(i=0;i<220;i++)tick(10);
 ready();zero();in.flags=7;in.epoch=1;
}
int main(void){unsigned i;
 assert(LC_CPR_X10==605277u && LC_TICK_NM==3388 && LC_TRACK_UM==180635u);
 assert(lc_front_stop_mm(0,0)==200);
 assert(lc_front_stop_mm(20,0)==224);
 assert(lc_front_stop_mm(300,0)==658);
 assert(lc_front_stop_mm(300,100)==688);
 assert(lc_front_stop_mm(300,751)==65535);
 assert(lc_front_stop_mm(INT32_MAX,0)==65535);
 setup();in.linear_mmps=20;tick(20);
 for(i=0;i<4;i++)assert(out.duty[i]>0); /*250mm supports20mm/s, not300.*/
 in.linear_mmps=300;tick(2);zero();ready();
 in.linear_mmps=0;in.yaw_rate_mradps=600;tick(20);
 assert(out.duty[0]>0 && out.duty[2]>0 && out.duty[1]<0 && out.duty[3]<0);ready();
 /* Pureturn uses centre-line component, not abs(singlewheel velocity). */
 s.wheel_mmps[0]=90;s.wheel_mmps[1]=-90;s.wheel_mmps[2]=90;s.wheel_mmps[3]=-90;
 assert(lc_forward_component_mmps(&s)==0);tick(2);
 assert(out.duty[0]>0 && out.duty[1]<0);ready();
 in.sonar_mm[1]=190;tick(2);zero();ready();
 in.yaw_rate_mradps=-600;tick(20);
 assert(out.duty[0]<0 && out.duty[2]<0 && out.duty[1]>0 && out.duty[3]>0);ready();
 in.sonar_online=6;tick(2);zero();ready();
 for(i=0;i<10;i++){tick(10);}
 zero();ready();
 in.sonar_online=7;tick(20);assert(out.duty[0]<0 && out.duty[1]>0);ready();
 /* A real old source cannot be rejuvenated by fresh IMU or loop commits. */
 in.now_ms+=2;refresh();in.sonar_stamp_ms[0]=in.now_ms-751;
 lc_update(&s,&in,&out);zero();ready();assert(in.sonar_stamp_ms[0]==in.now_ms-751);
 tick(20);assert(out.duty[0]<0 && out.duty[1]>0);ready();
 in.yaw_rate_mradps=0;in.linear_mmps=20;in.sonar_mm[1]=250;tick(20);
 assert(out.duty[0]>0);ready();
 /* Real forward coast must dominate a reduced requested speed. */
 for(i=0;i<4;i++)s.wheel_mmps[i]=300;
 assert(lc_forward_component_mmps(&s)==300);tick(2);zero();ready();
 /* True impossible count jumps still stop the fast loop. */
 in.counts[0]=INT32_MIN;tick(20);zero();
 in.flags=LC_WATCHDOG;tick(2);zero();assert(!(in.flags&LC_ARMED));ready();
 puts("PASS v29 production C reactive gates:200mm floor + sourceage/850msreaction/300mslatency/400mmps2brake;250mm lowdrive/pureturn,190mm directionalside,coast,sonarWAIT/calretained,realencoderjump/leasepreserved");
 return 0;
}
