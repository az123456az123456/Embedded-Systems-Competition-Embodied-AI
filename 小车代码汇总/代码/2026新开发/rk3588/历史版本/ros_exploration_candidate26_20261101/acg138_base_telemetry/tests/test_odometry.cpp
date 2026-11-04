#include "acg138_base_telemetry/odometry.hpp"
#include "acg138_base_telemetry/rx_stamp.hpp"
extern "C" {
#include "sensor_data.h"
#include "imu_data.h"
}
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
static int checks=0;
static void need(bool good,const char* text){++checks;if(!good){std::cerr<<"FAIL "<<text<<'\n';std::exit(1);}}
static void close_to(double a,double b,double tol,const char* text){need(std::abs(a-b)<tol,text);}
static void put32(unsigned char*p,std::uint32_t v){for(int i=0;i<4;i++)p[i]=v>>(24-8*i);}
int main(){
 constexpr double pi=3.14159265358979323846;acg138::OdometryConfig c;
 acg138::WheelOdometry o(c);std::array<std::int32_t,4> zero{};
 need(o.update(zero,100,1,0,0),"first encoder baseline");
 std::array<std::int32_t,4> counts{{-60528,60528,-60528,60528}};
 need(o.update(counts,600,2,0,0),"forward with actual M1..4 signs and leftmask10");
 close_to(o.state().x,c.circumference_m*60528/c.cpr,1e-10,"one wheel revolution distance");close_to(o.state().y,0,1e-12,"forward no lateral");close_to(o.state().yaw,0,1e-12,"forward no yaw");
 need(!o.update(counts,600,2,0,0),"duplicates cannot create new odometry");
 need(!o.update(counts,600,3,0,0),"same device millisecond cannot fake velocity");
 need(o.update(zero,1100,4,0,0),"reverse");close_to(o.state().x,0,1e-10,"reverse returns origin");
 o.reset();o.update(zero,100,1,0,0);
 const int quarter=static_cast<int>(std::lround(c.track_m*pi*.25*c.cpr/c.circumference_m));
 std::array<std::int32_t,4> turn{{-quarter,-quarter,-quarter,-quarter}};
 need(o.update(turn,600,2,0,0),"CCW turn");close_to(o.state().yaw,pi/2,5e-5,"left backwards/right forward is positive CCW");close_to(o.state().x,0,1e-12,"pure rotation no x");close_to(o.state().y,0,1e-12,"pure rotation no y");
 // Unequal wheel speeds during skid steering are not a false disagreement fault.
 o.reset();o.update(zero,100,1,0,0);need(o.update({{-1000,6000,-5000,10000}},200,2,0,0),"skid-steer unequal wheels permitted");
 o.reset();const std::int32_t high=std::numeric_limits<std::int32_t>::max()-100;
 std::array<std::int32_t,4> wrap{{high,high,high,high}};o.update(wrap,0xffffffe0u,0xffffffffu,0,0);
 for(int i=0;i<4;i++)wrap[i]=static_cast<std::int32_t>(static_cast<std::uint32_t>(high)+c.signs[i]*1000u);
 need(o.update(wrap,18u,0u,0,0),"counter, sequence and uptime wrap");close_to(o.state().x,c.circumference_m*1000/c.cpr,1e-10,"signed32 count wrap gives correct distance");
 need(!o.update(zero,0xffffffd0u,0xffffffffu,0,0),"old packet cannot rewind");
 o.reset();o.update(zero,0,0,0,0);need(!o.update(counts,2000,1,0,0)&&o.state().frozen,"large gap cannot teleport odom");
 o.reset();o.update(zero,1000,1,0,0);need(!o.update(counts,1050,2,0,0)&&o.state().frozen,"implausible speed frozen");
 o.reset();need(!o.update(zero,1,1,1,0)&&o.state().frozen,"unexpected inversion not double-corrected");
 o.reset();need(!o.update(zero,1,1,0,1)&&o.state().frozen,"illegal quadrature not accepted");
 bool bad=false;try{c.left_mask=1;acg138::WheelOdometry x(c);}catch(const std::invalid_argument&){bad=true;}need(bad,"two wheels per side required");
 const unsigned char reference[]="123456789";need(sensor_crc16(reference,9)==0x29b1,"CRC16/CCITT-FALSE known external check vector");
 unsigned char frame[80]={255,255,255,255,255,255,2,0xac,0x13,0x80,0,1,0x88,0xb8};auto*p=frame+14;std::memcpy(p,"SNS1",4);p[4]=1;p[5]=7;
 put32(p+8,42);put32(p+12,100);put32(p+16,0xffffffffu);p[60]=0;p[61]=50;
 for(int i=0;i<3;i++){p[48+2*i]=3;p[49+2*i]=0xe8;}
 auto checksum=sensor_crc16(p,64);p[64]=checksum>>8;p[65]=checksum;
 sensor_sample sample{};need(sensor_decode(frame,80,&sample)&&sample.position[0]==-1&&sample.sequence==42,"real SNS1 frame and signed count decode");
 frame[31]^=1;need(!sensor_decode(frame,80,&sample),"corrupted packet CRC rejected");frame[31]^=1;
 need(!sensor_decode(frame,79,&sample),"truncated SNS1 rejected");
 // Same production stationary bias estimator used by the current RK dashboard.
 imu_state imu{};imu_init(&imu);imu_sample is{};is.flags=1;is.accel[2]=1;is.gyro[0]=-2;is.gyro[1]=-2;is.gyro[2]=.1;
 for(unsigned i=1;i<=220;i++){is.sequence=i;is.timestamp=i*10;is.uptime=i*10;imu_update(&imu,&is,i*10);}
 need(imu.bias_valid&&!imu.calibrating,"real stationary calibration completes");close_to(imu.bias[2],.1,1e-12,"gyro bias removed, no forced zero heading");
 const double calibrated_yaw=imu.yaw;is.gyro[2]=90.1;
 for(unsigned i=221;i<=320;i++){is.sequence=i;is.timestamp=i*10;is.uptime=i*10;imu_update(&imu,&is,i*10);}
 close_to(imu.yaw-calibrated_yaw,90.,.01,"production quaternion relative yaw follows bias-corrected gyro rotation");
 alignas(cmsghdr) unsigned char ancillary[CMSG_SPACE(sizeof(timespec))]={};msghdr message{};message.msg_control=ancillary;message.msg_controllen=sizeof(ancillary);
 auto*control=CMSG_FIRSTHDR(&message);control->cmsg_level=SOL_SOCKET;control->cmsg_type=SCM_TIMESTAMPNS;control->cmsg_len=CMSG_LEN(sizeof(timespec));
 timespec stamp{100,500000000};std::memcpy(CMSG_DATA(control),&stamp,sizeof(stamp));
 auto st=acg138::receive_stamp(message,100600000000LL,5000);need(st&&st->realtime_ns==100500000000LL&&st->mono_ms==4900,"kernel timestamp retains100ms arrival age");
 need(!acg138::receive_stamp(message,100900000000LL,5000),"400ms backlog rejected rather than stamped now");
 need(!acg138::receive_stamp(message,100400000000LL,5000),"future clock timestamp rejected");
 message.msg_flags=MSG_CTRUNC;need(!acg138::receive_stamp(message,100600000000LL,5000),"truncated ancillary timestamp rejected");message.msg_flags=0;
 message.msg_controllen=0;need(!acg138::receive_stamp(message,100600000000LL,5000),"missing kernel timestamp cannot become current");
 std::cout<<"PASS base odometry/real telemetry decoder "<<checks<<" checks\n";
}
