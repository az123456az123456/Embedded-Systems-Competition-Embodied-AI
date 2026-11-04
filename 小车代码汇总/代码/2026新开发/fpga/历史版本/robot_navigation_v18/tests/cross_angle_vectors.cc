// Exercise the actual RK encoder/decoder against the FPGA IDDR/parser/CDC/PWM/ACK path.
#include "motor_link.h"
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace robotphone;
static const uint8_t host[]={2,1,2,3,4,5};
struct Move {unsigned mode;bool armed;int a,b,left,right;};
static const Move moves[]={
 {1,false,0,0,0,0},{2,false,0,0,0,0},{2,true,0,0,0,0},
 {2,true,0,220,220,220},{2,true,200,220,220,220},{2,true,-200,220,220,220},
 {2,true,201,220,228,212},{2,true,-201,220,212,228},
 {2,true,2499,220,319,121},{2,true,-2499,220,121,319},
 {2,true,2500,220,100,-100},{2,true,-2500,220,-100,100},
 {2,true,9000,220,220,-220},{2,true,-9000,220,-220,220},
 {2,true,0,0,0,0},{1,true,300,300,300,300},{1,true,-300,-300,-300,-300},
 {2,false,0,0,0,0},{2,true,0,220,220,220}};
int main(int argc,char**argv){
 assert(argc==3);std::string op=argv[1];const auto count=sizeof(moves)/sizeof(*moves);
 if(op=="generate"){
  std::ofstream out(argv[2]);assert(out.good());
  for(unsigned n=0;n<count;n++){
   const auto&m=moves[n];auto bytes=m.mode==2 ? angleMotorFrame(host,n+1,m.armed,m.a,m.b,10,15) : motorFrame(host,n+1,m.armed,m.a,m.b,10,15);
   uint32_t crc=~0u;for(auto b:bytes){crc^=b;for(int i=0;i<8;i++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}crc=~crc;
   for(int i=0;i<4;i++)bytes.push_back(crc>>(i*8));
   assert(bytes.size()==64);
   for(auto b:bytes){char line[8];snprintf(line,sizeof(line),"%02x\n",b);out<<line;}
  }
  std::cout<<"PASS actual RK MOT1/MOT2 encoder: "<<count<<" command fixtures\n";
 }else{
  std::ifstream in(argv[2]);assert(in.good());std::vector<uint8_t> bytes;std::string word;
  std::vector<bool> seen(count,false);bool timeout=false;unsigned total=0;
  auto verify=[&]{if(bytes.empty())return;assert(bytes.size()==72);
   uint32_t crc=~0u;for(size_t i=8;i<bytes.size();i++){crc^=bytes[i];for(int b=0;b<8;b++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}assert(crc==0xdebb20e3u);
   MotorAck ack;assert(decodeMotorAck(bytes.data()+8,64,host,ack));assert(ack.sequence>0&&ack.sequence<=count);assert(ack.left_mask==10&&ack.invert_mask==15);
   const auto&m=moves[ack.sequence-1];assert(ack.protocol==m.mode);total++;
   if(ack.watchdog){assert(!ack.armed&&!ack.configured&&ack.left==0&&ack.right==0);if(ack.sequence==count)timeout=true;return;}
   assert(ack.configured&&ack.armed==m.armed);
   if(ack.left==m.left&&ack.right==m.right)seen[ack.sequence-1]=true;
  };
  while(in>>word){if(word=="END"){verify();bytes.clear();}else bytes.push_back(std::stoul(word,nullptr,16));}verify();
  for(size_t i=0;i<count;i++){if(!seen[i])std::cerr<<"Missing stable sequence "<<i+1<<"\n";assert(seen[i]);}assert(timeout);
  std::cout<<"PASS "<<total<<" RTL ACK1/ACK2 frames decoded by actual RK, every command reached expected sides, CRC16/FCS and timeout verified\n";
 }
}
