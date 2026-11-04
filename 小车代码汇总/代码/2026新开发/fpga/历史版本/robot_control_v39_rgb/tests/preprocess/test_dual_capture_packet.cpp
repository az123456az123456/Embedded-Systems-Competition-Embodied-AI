#include "Vdual_capture_packet_top.h"
#include "verilated.h"
#include "capture_golden.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
static uint64_t checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"FAIL %s line=%d\n",#x,__LINE__);std::abort();}}while(0)
static constexpr uint64_t FT=1880ull*1120,NUM=1000000000000ull,DEN=84135000;
static constexpr double PERIOD_MS=double(FT)*1000./DEN;
static uint32_t crc32(const uint8_t *data,size_t n){uint32_t crc=~0u;for(size_t i=0;i<n;i++){crc^=data[i];for(int bit=0;bit<8;bit++)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}return ~crc;}
static unsigned be16(const uint8_t *p){return unsigned(p[0])*256+p[1];}
static uint32_t be32(const uint8_t *p){return(uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3];}
using SourceKey=std::tuple<unsigned,uint32_t,unsigned>;
struct Source{int pattern;uint64_t capture_ps;};
struct Pair{
 std::array<std::array<bool,1280>,2> seen{};unsigned count[2]={};uint32_t stamp=0;bool have_stamp=false;
 uint64_t completed_ps=0;
};
struct Simulation{
 std::unique_ptr<Vdual_capture_packet_top> d{new Vdual_capture_packet_top};
 std::string name;int frames;uint64_t phase_cycles;bool recover_phase,change_epoch;
 uint64_t tx_next=0,cam_next[2]={},cam_remainder[2]={},cam_cycles[2]={},tx_cycles=0,now_ps=0;
 uint64_t next_control=5000,blocked=0,control_events=0;unsigned remaining_control=0;
 uint64_t last_packet_end_tx=0,min_ifg=1000000;std::vector<uint8_t> packet;
 std::map<SourceKey,Source> sources;std::map<int,std::vector<uint32_t>> golden;
 std::map<std::pair<unsigned,uint32_t>,Pair> pairs;
 unsigned max_used[2]={};uint64_t packets=0,payload_bytes=0;unsigned requested=1;
 bool prev_valid=false;unsigned received_epoch=0;
 Simulation(std::string n,int count,uint64_t phase=0,bool recover=false,bool epoch=false):name(n),frames(count),phase_cycles(phase),recover_phase(recover),change_epoch(epoch){
  d->rst_n=0;d->tx_clk=d->cam0_clk=d->cam1_clk=0;d->href0=d->href1=d->vsync0=d->vsync1=0;
  d->raw0=d->raw1=0;d->session_commit=0;d->requested_epoch=1;d->tx_allow=1;
 }
 int physical_frame(int eye,int64_t &pos){
  int64_t cycles=cam_cycles[eye];
  if(eye==1&&phase_cycles&&(!recover_phase||cycles<int64_t(FT)))cycles-=phase_cycles;
  if(cycles<0){pos=-1;return -1;}
  pos=cycles%FT;return int(cycles/FT);
 }
 void drive_camera(int eye){
  int64_t pos;int frame=physical_frame(eye,pos);bool h=false,v=false;unsigned raw=0;
  if(frame>=0&&frame<frames){int row=int(pos/1880),x=int(pos%1880);v=row==0;
   h=row>=2&&row<1082&&x<960;if(h)raw=capture_golden::raw_pixel(x,row-2,frame+eye*7);
  }
  if(eye==0){d->href0=h;d->vsync0=v;d->raw0=raw;}else{d->href1=h;d->vsync1=v;d->raw1=raw;}
 }
 void record_source(int eye){
  bool sof=eye?d->source_sof1:d->source_sof0;if(!sof||!d->rst_n)return;
  unsigned ep=eye?d->bridge_epoch1:d->bridge_epoch0;uint32_t seq=(eye?d->source_seq1:d->source_seq0)+1;
  int64_t pos;int frame=physical_frame(eye,pos);CHECK(frame>=0&&frame<frames);
  int pattern=frame+eye*7;sources[{ep,seq,unsigned(eye)}]={pattern,now_ps};
  if(!golden.count(pattern))golden[pattern]=capture_golden::reference(pattern);
 }
 void finish_packet(){
  CHECK(packet.size()==1018);auto *p=packet.data();
  for(int i=0;i<7;i++){CHECK(p[i]==0x55);}
  CHECK(p[7]==0xd5);
  CHECK(be16(p+20)==0x88ba);CHECK(crc32(p+8,1006)==(uint32_t(p[1014])|(uint32_t(p[1015])<<8)|(uint32_t(p[1016])<<16)|(uint32_t(p[1017])<<24)));
  const uint8_t *h=p+22,*payload=h+32;
  CHECK(h[0]=='R'&&h[1]=='G'&&h[2]=='P'&&h[3]=='1'&&h[4]==1&&h[5]<2&&h[6]==1);
  unsigned eye=h[5],row=be16(h+16),part=h[18],ep=be16(h+26);uint32_t seq=be32(h+8),stamp=be32(h+12);
  CHECK(h[7]==unsigned(seq==1)&&row<640&&part<2&&h[19]==2&&be16(h+20)==640&&be16(h+22)==640&&be16(h+24)==960);
  CHECK(ep>=received_epoch);received_epoch=ep;
  std::array<uint8_t,988> app{};std::copy_n(h,28,app.begin());std::copy_n(payload,960,app.begin()+28);
  CHECK(crc32(app.data(),app.size())==be32(h+28));
  SourceKey key{ep,seq,eye};CHECK(sources.count(key));
  const auto &source=sources.at(key);const auto &ref=golden.at(source.pattern);size_t start=row*640+part*320;
  for(size_t i=0;i<320;i++){uint32_t value=(uint32_t(payload[i*3])<<16)|(uint32_t(payload[i*3+1])<<8)|payload[i*3+2];
   if(value!=ref[start+i]){std::fprintf(stderr,"%s payload ep=%u seq=%u eye=%u row=%u part=%u p=%zu pattern=%d got=%06x want=%06x\n",name.c_str(),ep,seq,eye,row,part,i,source.pattern,value,ref[start+i]);CHECK(false);}++checks;
  }
  auto &pair=pairs[{ep,seq}];CHECK(!pair.seen[eye][row*2+part]);pair.seen[eye][row*2+part]=true;++pair.count[eye];
  if(pair.have_stamp)CHECK(pair.stamp==stamp);else{pair.have_stamp=true;pair.stamp=stamp;}
  if(pair.count[0]==1280&&pair.count[1]==1280){
   pair.completed_ps=now_ps;uint64_t source_time=std::min(sources.at({ep,seq,0}).capture_ps,sources.at({ep,seq,1}).capture_ps);
   uint64_t expected_ms=1000+source_time/1000000000ull;CHECK(stamp==expected_ms||stamp+1==expected_ms);
   std::printf("PAIR case=%s epoch=%u seq=%u complete_age_ms=%.4f fifo_peak=(%u,%u)\n",name.c_str(),ep,seq,double(now_ps-source_time)/1e9,max_used[0],max_used[1]);
  }
  ++packets;payload_bytes+=960;packet.clear();
 }
 void sample_wire(){
  bool valid=d->gmii_valid;
  if(valid){
   if(!prev_valid&&packets){uint64_t gap=tx_cycles-last_packet_end_tx-1;min_ifg=std::min(min_ifg,gap);CHECK(gap>=12);}
   packet.push_back(d->gmii_data);CHECK(packet.size()<=1018);
  }else if(prev_valid){last_packet_end_tx=tx_cycles-1;finish_packet();}
  prev_valid=valid;
 }
 void step(){
  now_ps=std::min(tx_next,std::min(cam_next[0],cam_next[1]));
  bool tx=tx_next==now_ps,c0=cam_next[0]==now_ps,c1=cam_next[1]==now_ps;
  if(tx){d->tx_clk=0;} if(c0){d->cam0_clk=0;} if(c1){d->cam1_clk=0;}
  d->rst_n=now_ps>=100000;
  if(c0){drive_camera(0);} if(c1){drive_camera(1);}
  if(change_epoch&&now_ps>uint64_t(PERIOD_MS*1e9)+5000000000ull)requested=2;
  d->requested_epoch=requested;d->session_commit=d->rst_n&&d->epoch!=requested;
  if(tx){
   if(tx_cycles>=next_control&&!remaining_control&&!d->tx_busy){remaining_control=160;next_control=tx_cycles+5000;++control_events;}
   d->tx_allow=remaining_control==0;if(remaining_control){--remaining_control;++blocked;}
  }
  d->eval();if(c0){record_source(0);} if(c1){record_source(1);}
  if(tx){d->tx_clk=1;} if(c0){d->cam0_clk=1;} if(c1){d->cam1_clk=1;} d->eval();
  max_used[0]=std::max(max_used[0],unsigned(d->used0));max_used[1]=std::max(max_used[1],unsigned(d->used1));
  CHECK(d->used0<=2047&&d->used1<=2047);
  if(tx){sample_wire();++tx_cycles;tx_next+=8000;}
  for(int eye=0;eye<2;eye++)if(eye==0?c0:c1){++cam_cycles[eye];cam_next[eye]+=NUM/DEN;cam_remainder[eye]+=NUM%DEN;if(cam_remainder[eye]>=DEN){cam_next[eye]++;cam_remainder[eye]-=DEN;}}
 }
 void run(){
  uint64_t stop=uint64_t((frames+0.20)*FT*double(NUM)/DEN);
  while(now_ps<stop){step();}
  CHECK(packet.empty()&&!prev_valid);
  CHECK(packets==d->packets);
  unsigned complete=0,after_epoch=0;double max_age=0;std::vector<uint64_t> times;
  uint64_t max_source_skew_ps=0;
  for(const auto &item:sources){
   unsigned ep=std::get<0>(item.first),eye=std::get<2>(item.first);uint32_t seq=std::get<1>(item.first);
   if(eye==0&&sources.count({ep,seq,1})){
    auto a=item.second.capture_ps,b=sources.at({ep,seq,1}).capture_ps;
    max_source_skew_ps=std::max(max_source_skew_ps,a>b?a-b:b-a);
   }
  }
  if(phase_cycles){
   uint64_t wanted=phase_cycles*NUM/DEN;
   CHECK(max_source_skew_ps>=wanted-1&&max_source_skew_ps<=wanted+1);
  }else CHECK(max_source_skew_ps==0);
  for(auto &item:pairs)if(item.second.completed_ps){
   ++complete;if(item.first.first==2)++after_epoch;times.push_back(item.second.completed_ps);
   auto key=item.first;uint64_t start=std::min(sources.at({key.first,key.second,0}).capture_ps,sources.at({key.first,key.second,1}).capture_ps);
   max_age=std::max(max_age,double(item.second.completed_ps-start)/1e9);
  }
  if(recover_phase){CHECK(d->dropped0+d->dropped1>0&&d->pair_fault_count>0);CHECK(complete>=3);CHECK(!pairs.count({1,1})||!pairs.at({1,1}).completed_ps);}
  else if(change_epoch){CHECK(after_epoch>=3&&complete>=4);}
  else {CHECK(complete==unsigned(frames));CHECK(d->dropped0==0&&d->dropped1==0&&d->pair_fault_count==0);CHECK(max_age<PERIOD_MS);}
  std::printf("RESULT case=%s source_fps=%.6f complete_pairs=%u epoch2_pairs=%u packets=%llu fifo_peak=(%u,%u) source_drops=(%u,%u) pair_faults=%u max_pair_age_ms=%.5f max_source_skew_ps=%llu ifg_min=%llu control_events=%llu control_reserved_cycles=%llu tx_cycles=%llu checks=%llu\n",
   name.c_str(),1000./PERIOD_MS,complete,after_epoch,(unsigned long long)packets,max_used[0],max_used[1],d->dropped0,d->dropped1,d->pair_fault_count,max_age,
   (unsigned long long)max_source_skew_ps,(unsigned long long)min_ifg,
   (unsigned long long)control_events,(unsigned long long)blocked,(unsigned long long)tx_cycles,(unsigned long long)checks);
 }
};
int main(int argc,char **argv){
 Verilated::commandArgs(argc,argv);std::setvbuf(stdout,nullptr,_IONBF,0);
 std::string selected=argc>1?argv[1]:"all";
 if(selected=="all"||selected=="sync")Simulation("sync",3).run();
 if(selected=="all"||selected=="skew")Simulation("skew-200-pclk",3,200).run();
 if(selected=="all"||selected=="overflow")Simulation("overflow-30000-pclk-recover",5,30000,true).run();
 if(selected=="all"||selected=="epoch")Simulation("epoch-midstream",5,0,false,true).run();
 std::printf("PASS dual DVP->real bounded FIFO->GMII integration; checks=%llu; simulation only\n",(unsigned long long)checks);
}
