#include "../protocol.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
using namespace navwire;
static uint8_t host[6]={2,4,6,8,10,12};
static void seal(std::vector<uint8_t>&b){auto c=crc16(b.data()+14,64);b[78]=c>>8;b[79]=c;}
int main(){
 const uint8_t text[]="123456789";
 assert(crc16(text,9)==0x29b1);assert(crc32(text,9)==0xcbf43926);
 Message m;m.op=0x85;m.request=0x12345678;m.generation=0xabcdef01;m.arg0=4090;m.arg1=4096;m.arg2=55;m.count=6;m.data[0]=0xf;m.data[1]=0xfa;
 auto b=frame(host,m);std::swap_ranges(b.begin(),b.begin()+6,b.begin()+6);Message out;
 assert(reply(b.data(),b.size(),host,out));assert(out.request==m.request&&out.generation==m.generation&&out.count==6&&out.arg1==4096);
 auto original=b;b[44]^=1;assert(!reply(b.data(),b.size(),host,out));b=original;b[6]^=1;assert(!reply(b.data(),b.size(),host,out));b=original;b[0]^=1;assert(!reply(b.data(),b.size(),host,out));
 b=original;std::fill(b.begin(),b.begin()+6,255);assert(reply(b.data(),b.size(),host,out));assert(!reply(b.data(),79,host,out));
 b=original;b[14+28]=1;seal(b);assert(!reply(b.data(),b.size(),host,out));
 b=original;b[13]=0xb8;std::fill(b.begin()+14,b.end(),0);std::memcpy(b.data()+14,"SNS1",4);b[18]=1;b[19]=7;b[21]=0;b[14+60]=0;b[14+61]=50;
 b[14+16]=0x80;b[14+19]=1;b[14+32]=0xff;b[14+33]=0xff;b[14+34]=0xff;b[14+35]=0xfe;b[14+48]=3;b[14+49]=0xe8;b[14+54]=1;b[14+55]=0xf4;seal(b);
 Sensors s;assert(sensorFrame(b.data(),b.size(),host,s));assert(s.counts[0]==int32_t(0x80000001u)&&s.delta[0]==-2&&s.distance_mm[0]==1000&&s.age_ms[0]==500);
 b[14+62]=16;seal(b);assert(!sensorFrame(b.data(),b.size(),host,s));
 b[14+62]=0;b[14+61]=0;seal(b);assert(!sensorFrame(b.data(),b.size(),host,s));
 std::fill(b.begin()+14+32,b.begin()+14+48,0);seal(b);assert(sensorFrame(b.data(),b.size(),host,s)&&s.window_ms==0&&s.counts[0]==int32_t(0x80000001u));
 b[14+61]=1;seal(b);assert(!sensorFrame(b.data(),b.size(),host,s));
 std::cout<<"PASS CRC reference, NAV2 roundtrip/corruption/MAC/length/reserved, SNS1 signed counts/range\n";
}
