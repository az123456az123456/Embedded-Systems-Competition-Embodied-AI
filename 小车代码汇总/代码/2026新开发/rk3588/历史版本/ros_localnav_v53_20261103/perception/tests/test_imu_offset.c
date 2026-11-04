#include "imu_data.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static const double observed[3]={-303/131.,-303/131.,3/131.};
static void near(double a,double b,double tolerance){
 if(fabs(remainder(a-b,360))>tolerance){fprintf(stderr,"%.9f != %.9f tolerance %.9f\n",a,b,tolerance);assert(0);}
}
static struct imu_sample sample(double roll,double pitch){
 struct imu_sample v={.flags=15,.temperature=25};double r=roll*IMU_PI/180,p=pitch*IMU_PI/180;
 v.accel[0]=-sin(p);v.accel[1]=sin(r)*cos(p);v.accel[2]=cos(r)*cos(p);
 memcpy(v.gyro,observed,sizeof(observed));return v;
}
static void step(struct imu_state *s,struct imu_sample *v,unsigned dt){
 v->sequence++;v->timestamp+=dt;v->uptime+=dt;assert(imu_update(s,v,v->uptime));
}
static void calibrate(struct imu_state *s,struct imu_sample *v,unsigned dt){
 imu_init(s);for(int i=0;i<400;i++)step(s,v,dt);
 assert(!s->calibrating && s->bias_valid && s->calibration==200);
 for(int i=0;i<3;i++)near(s->bias[i],observed[i],1e-10);
}
int main(void){
 struct imu_state s;struct imu_sample v;
 // Observed negative signed gyro words from the FPGA packet must decode to the
 // exact values used here (no axis swapping, hardcoded yaw offset or unit change).
 unsigned char bytes[78]={255,255,255,255,255,255,2,0xac,0x13,0x80,0,1,0x88,0xb5};
 unsigned char *p=bytes+14;memcpy(p,"IMU9",4);p[4]=1;p[5]=0x92;p[7]=15;p[20]=0x40;
 p[24]=p[26]=0xfe;p[25]=p[27]=0xd1;p[29]=3;p[39]=0x71;p[40]=0x48;p[41]=0x68;p[59]=100;
 assert(imu_decode(bytes,sizeof(bytes),&v));for(int a=0;a<3;a++)near(v.gyro[a],observed[a],1e-10);

 // The initial offsets have norm 3.271 dps: old norm<3 gate never progressed.
 const double poses[][2]={{0,0},{-2,0},{15,-10},{-30,20}};
 for(unsigned pose=0;pose<sizeof(poses)/sizeof(*poses);pose++){
  v=sample(poses[pose][0],poses[pose][1]);calibrate(&s,&v,10);
  for(int i=0;i<600;i++)step(&s,&v,10); // permit original complementary tilt feedback to settle
  double start=s.yaw;
  for(int i=0;i<6000;i++)step(&s,&v,10);
  near(s.yaw,start,.00001);near(s.roll,poses[pose][0],.00001);near(s.pitch,poses[pose][1],.00001);
  printf("PASS observed offsets, roll=%.0f pitch=%.0f: cal=100%% bias_valid=1 stationary_60s_drift=%.9fdeg\n",poses[pose][0],poses[pose][1],s.yaw-start);
 }
 // Quantized filtered-sensor noise and irregular sample delivery.
 imu_init(&s);v=sample(-2,0);unsigned gaps[]={10,60,140,30,80};
 for(int i=0;i<1600;i++){
  for(int a=0;a<3;a++)v.gyro[a]=observed[a]+((i+a)%3-1)/131.;
  step(&s,&v,gaps[i%5]);
 }
 assert(!s.calibrating && s.bias_valid);for(int a=0;a<3;a++)near(s.bias[a],observed[a],.001);
 double start=s.yaw;for(int i=0;i<6000;i++){
  for(int a=0;a<3;a++)v.gyro[a]=observed[a]+((i+a)%3-1)/131.;
  step(&s,&v,10);
 }
 near(s.yaw,start,.00001);puts("PASS observed offsets with quantized noise and irregular 10-140ms delivery");

 // No online learning: real slow and fast yaw remains full scale after large bias calibration.
 const double rates[]={.05,-.05,.1,-.1,.5,10,45,-45};
 for(unsigned r=0;r<sizeof(rates)/sizeof(*rates);r++){
  v=sample(0,0);calibrate(&s,&v,10);for(int i=0;i<600;i++)step(&s,&v,10);imu_zero_yaw(&s);
  v.gyro[2]+=rates[r];int count=fabs(rates[r])>=10?200:2000;
  for(int i=0;i<count;i++)step(&s,&v,10);
  near(s.yaw,rates[r]*count/100.,.003);
  for(int a=0;a<3;a++)near(s.bias[a],observed[a],1e-10);
 }
 puts("PASS calibrated +/-0.05, +/-0.1, 0.5, 10 and +/-45dps turns; fixed bias, no angle suppression");
 // Variance and gravity checks must continue to reject moving/vibrating data.
 imu_init(&s);v=sample(0,0);
 for(int i=0;i<600;i++){v.gyro[0]=observed[0]+((i&1)?.3:-.3);step(&s,&v,10);}
 assert(s.calibrating && !s.bias_valid);
 imu_init(&s);v=sample(0,0);
 for(int i=0;i<600;i++){v.accel[0]=(i&1)?.03:-.03;step(&s,&v,10);}
 assert(s.calibrating && !s.bias_valid);
 imu_init(&s);v=sample(0,0);v.gyro[2]=10;
 for(int i=0;i<600;i++)step(&s,&v,10);
 assert(s.calibrating && !s.bias_valid);
 puts("PASS gyro noise, acceleration movement and excessive rotation remain rejected");
 // Signed initial offsets near the specified +/-5 dps bounds are valid PER AXIS.
 imu_init(&s);v=sample(0,0);v.gyro[0]=5;v.gyro[1]=-5;v.gyro[2]=4.8;
 for(int i=0;i<250;i++)step(&s,&v,10);
 assert(!s.calibrating && s.bias_valid);for(int a=0;a<3;a++)near(s.bias[a],v.gyro[a],1e-10);
 imu_calibrate(&s);assert(s.bias_valid);near(s.bias[0],5,1e-10);
 puts("PASS per-axis acquisition range and previous bias preserved during requested recalibration");
}
