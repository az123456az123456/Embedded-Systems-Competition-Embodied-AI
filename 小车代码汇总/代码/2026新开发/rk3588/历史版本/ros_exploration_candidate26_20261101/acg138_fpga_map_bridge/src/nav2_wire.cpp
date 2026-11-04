#include "acg138_fpga_map_bridge/nav2_wire.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <stdexcept>

namespace acg138::wire {
namespace {
uint16_t u16(const uint8_t*p){return uint16_t(p[0])<<8|p[1];}
uint32_t u32(const uint8_t*p){return uint32_t(p[0])<<24|uint32_t(p[1])<<16|uint32_t(p[2])<<8|p[3];}
void put32(uint8_t*p,uint32_t v){p[0]=uint8_t(v>>24);p[1]=uint8_t(v>>16);p[2]=uint8_t(v>>8);p[3]=uint8_t(v);}
bool blocked(const Bitmap&b,uint16_t i){return i>=kCells||(b[i>>3]&(1u<<(i&7)));}
}
uint16_t crc16(const uint8_t*p,size_t n){uint16_t c=0xffff;while(n--){c^=uint16_t(*p++)<<8;for(unsigned k=0;k<8;++k)c=(c&0x8000)?uint16_t((c<<1)^0x1021):uint16_t(c<<1);}return c;}
uint32_t crc32(const uint8_t*p,size_t n){uint32_t c=0xffffffff;while(n--){c^=*p++;for(unsigned k=0;k<8;++k)c=(c&1)?(c>>1)^0xedb88320:c>>1;}return ~c;}
Body encode(const Message&m){Body b{};std::memcpy(b.data(),"NAV2",4);b[4]=1;b[5]=m.op;b[6]=m.status;b[7]=m.count;put32(b.data()+8,m.request);put32(b.data()+12,m.generation);put32(b.data()+16,m.arg0);put32(b.data()+20,m.arg1);put32(b.data()+24,m.arg2);std::copy(m.data.begin(),m.data.end(),b.begin()+32);return b;}
bool decode(const Body&b,Message&m){if(std::memcmp(b.data(),"NAV2",4)||b[4]!=1||u32(b.data()+28)!=0)return false;m={};m.op=b[5];m.status=b[6];m.count=b[7];m.request=u32(b.data()+8);m.generation=u32(b.data()+12);m.arg0=u32(b.data()+16);m.arg1=u32(b.data()+20);m.arg2=u32(b.data()+24);std::copy(b.begin()+32,b.end(),m.data.begin());return true;}
std::vector<uint8_t> ethernet(const Mac&source,const Message&m){std::vector<uint8_t> f(80);std::copy(kFpgaMac.begin(),kFpgaMac.end(),f.begin());std::copy(source.begin(),source.end(),f.begin()+6);f[12]=0x88;f[13]=0xb9;const Body b=encode(m);std::copy(b.begin(),b.end(),f.begin()+14);const uint16_t crc=crc16(b.data(),b.size());f[78]=uint8_t(crc>>8);f[79]=uint8_t(crc);return f;}
bool decode_reply(const uint8_t*p,size_t n,const Mac&host,Message&m){
  if((n!=80&&n!=84)||!std::equal(kFpgaMac.begin(),kFpgaMac.end(),p+6)||u16(p+12)!=kEtherType)return false;
  bool broadcast=true;for(unsigned i=0;i<6;++i)broadcast&=p[i]==0xff;
  if(!broadcast&&!std::equal(host.begin(),host.end(),p))return false;
  if(crc16(p+14,64)!=u16(p+78))return false;
  Body b;std::copy(p+14,p+78,b.begin());return decode(b,m)&&(m.op&0x80);
}
Message Client::exchange(const Message&q,unsigned timeout_ms){
  const Body bytes=encode(q);
  for(unsigned attempt=0;attempt<3;++attempt){
    if(attempt)++stats_.retries;
    channel_.send(bytes);++stats_.sent;
    const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
    while(std::chrono::steady_clock::now()<until){
      const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(until-std::chrono::steady_clock::now()).count();
      Body body;if(!channel_.receive(body,unsigned(std::max<int64_t>(1,std::min<int64_t>(left,50)))))continue;
      Message r;if(!decode(body,r)||r.op!=(q.op|0x80)||r.request!=q.request||r.generation!=q.generation){++stats_.ignored;continue;}
      if(r.status)throw std::runtime_error("FPGA NAV2 status="+std::to_string(r.status)+" op="+std::to_string(q.op));
      return r;
    }
  }
  throw std::runtime_error("FPGA NAV2 ACK timeout op="+std::to_string(q.op)+"; map not committed/usable until acknowledged");
}
Message Client::request(uint8_t op){Message q;q.op=op;q.request=next_++;if(!next_)next_=1;q.generation=generation_;return q;}
uint32_t Client::upload(const Bitmap&bitmap){
  committed_=false;
  Message q;q.op=Info;const Message info=exchange(q,500);
  if(info.data[0]!=0||info.data[1]!=1||info.data[2]!=0||info.data[3]!=0)throw std::runtime_error("FPGA NAV2 capabilities do not match 64x64 ABI");
  next_=info.arg1+1;if(!next_)next_=1;generation_=info.arg0+1;if(!generation_)generation_=1;
  q=request(Begin);q.arg0=crc32(bitmap.data(),bitmap.size());exchange(q,500);
  for(unsigned offset=0;offset<bitmap.size();offset+=32){q=request(Chunk);q.arg0=offset;q.count=32;std::copy(bitmap.begin()+offset,bitmap.begin()+offset+32,q.data.begin());const Message r=exchange(q,500);if(r.arg0!=offset)throw std::runtime_error("FPGA chunk ACK offset mismatch");}
  q=request(Commit);exchange(q,500);
  committed_=true;
  // Only this success return authorizes the RK metadata generation to be used.
  return generation_;
}
std::vector<uint16_t> Client::plan(const Bitmap&bitmap,uint16_t start,uint16_t goal){
  if(!committed_)throw std::runtime_error("upload and acknowledge a complete map before PLAN");
  if(blocked(bitmap,start)||blocked(bitmap,goal))throw std::runtime_error("FPGA PLAN endpoint is blocked/unknown; do not clear it blindly");
  Message q=request(Plan);q.arg0=start;q.arg1=goal;const uint32_t plan_id=q.request;const Message result=exchange(q,10000);
  if(result.arg0<1||result.arg0>kCells)throw std::runtime_error("FPGA path length outside 1..4096");
  std::vector<uint16_t> path;std::set<uint16_t> seen;
  while(path.size()<result.arg0){
    q=request(Page);q.arg0=uint32_t(path.size());q.arg1=plan_id;q.count=uint8_t(std::min<size_t>(16,result.arg0-path.size()));const Message page=exchange(q,500);
    if(page.arg0!=path.size()||page.arg1!=result.arg0||page.arg2!=plan_id||page.count!=q.count)throw std::runtime_error("mixed/stale FPGA path page");
    for(unsigned j=0;j<page.count;++j){const uint16_t id=u16(page.data.data()+j*2);
      if(blocked(bitmap,id)||!seen.insert(id).second)throw std::runtime_error("FPGA path intersects blocked cell or loops");
      if(!path.empty()){const uint16_t p=path.back();if(std::abs(int(id%64)-int(p%64))+std::abs(int(id/64)-int(p/64))!=1)throw std::runtime_error("FPGA path is not four-connected");}
      path.push_back(id);
    }
  }
  if(path.front()!=start||path.back()!=goal)throw std::runtime_error("FPGA path endpoints do not match requested cells");
  return path;
}
}  // namespace acg138::wire
