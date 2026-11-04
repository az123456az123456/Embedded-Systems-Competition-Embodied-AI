// Use the real RK wire encoder, not a second copy of the MOT1 implementation.
#include "motor_link.h"
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace robotphone;
static const uint8_t host[]={2,1,2,3,4,5};
struct Move{bool armed;int left,right;};
static const Move moves[]={
 {false,0,0},{true,0,0},{true,300,300},{true,-300,-300},
 {true,100,300},{true,300,100},{true,350,-350},{true,-350,350},
 {true,0,0},{false,0,0},{true,300,300}};
int main(int argc,char **argv){
 assert(argc==3);std::string operation=argv[1];
 if(operation=="generate"){
  std::ofstream out(argv[2]);assert(out.good());
  for(unsigned index=0;index<sizeof(moves)/sizeof(*moves);++index){
   auto m=moves[index];auto bytes=motorFrame(host,index+1,m.armed,m.left,m.right,10,0);
   uint32_t crc=~0u;for(auto byte:bytes){crc^=byte;for(int i=0;i<8;i++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}crc=~crc;
   for(int i=0;i<4;i++)bytes.push_back(crc>>(i*8));
   for(auto byte:bytes){char line[8];snprintf(line,sizeof(line),"%02x\n",byte);out<<line;}
  }
  std::cout<<"PASS actual RK motorFrame generated11 STOP/config/arm/forward/reverse/left/right/CW/CCW/zero/watchdog vectors\n";
 }else{
  std::ifstream in(argv[2]);assert(in.good());std::vector<uint8_t> bytes;std::string word;
  int counts[12]={};bool seen[12]={};bool timeout_seen=false;unsigned total=0;
  auto verify=[&]{if(bytes.empty())return;assert(bytes.size()==72);MotorAck a;assert(decodeMotorAck(bytes.data()+8,64,host,a));assert(a.left_mask==10&&a.invert_mask==0);assert(a.sequence>=1&&a.sequence<=11);++total;
   if(a.sequence==11&&a.watchdog){assert(!a.configured&&!a.armed&&a.left==0&&a.right==0);timeout_seen=true;return;}
   if(!a.configured||a.watchdog)return;
   auto expected=moves[a.sequence-1];assert(a.armed==expected.armed);++counts[a.sequence];
   if(a.left==expected.left&&a.right==expected.right)seen[a.sequence]=true;
  };
  while(in>>word){if(word=="END"){verify();bytes.clear();}else bytes.push_back(std::stoul(word,nullptr,16));}verify();
  for(unsigned n=1;n<=11;n++){assert(counts[n]>0);assert(seen[n]);}assert(timeout_seen);
  std::cout<<"PASS "<<total<<" actual RTL ACK1 frames decoded by C++, every movement sequence reached requested output, watchdog cleared arm/config and PWM\n";
 }
}
