#include "acg138_fpga_map_bridge/nav3_wire.hpp"
#include <algorithm>

namespace acg138::nav3 {
namespace {
using U8=std::uint8_t; using U16=std::uint16_t;
using U32=std::uint32_t; using U64=std::uint64_t;
U16 get16(const U8* p) { return U16((U16(p[0])<<8)|p[1]); }
U32 get32(const U8* p) {
  return (U32(p[0])<<24)|(U32(p[1])<<16)|(U32(p[2])<<8)|p[3];
}
U64 get64(const U8* p) { return (U64(get32(p))<<32)|get32(p+4); }
void put16(U8* p,U16 n) { p[0]=U8(n>>8); p[1]=U8(n); }
void put32(U8* p,U32 n) { for(unsigned i=0;i<4;++i) p[i]=U8(n>>(24-8*i)); }
void put64(U8* p,U64 n) { put32(p,U32(n>>32)); put32(p+4,U32(n)); }
bool zero(const U8* p,std::size_t n) {
  return std::all_of(p,p+n,[](U8 b){ return b==0; });
}
bool task(const Header& h) {
  return h.run_generation && !zero(h.goal_id.data(),h.goal_id.size()) &&
    h.map_epoch && h.map_revision && h.localization_epoch;
}
bool known(U8 op) {
  switch(static_cast<Opcode>(op)) {
    case Opcode::Info: case Opcode::FrameBind: case Opcode::SessionBind:
    case Opcode::ClockAnchor: case Opcode::MapBegin: case Opcode::MapChunk:
    case Opcode::MapCommit: case Opcode::GoalPlan: case Opcode::Start:
    case Opcode::Cancel: case Opcode::Pose: case Opcode::State:
    case Opcode::PathPage: case Opcode::Event: case Opcode::PlanResult: return true;
  }
  return false;
}
Error shape(const Header& h,const Payload& payload) {
  const auto* p=payload.data(); const auto n=h.payload_bytes;
  if(n>kPayloadBytes) return Error::Length;
  if(h.flags) return Error::Flags;
  const U8 raw_op=h.opcode&0x7fu;
  if(!known(raw_op)) return Error::Opcode;
  const auto op=static_cast<Opcode>(raw_op);
  if(h.opcode&0x80u) {
    if(op==Opcode::Pose || op==Opcode::State || op==Opcode::Event ||
       op==Opcode::PlanResult) return Error::Opcode;
    if(n<(op==Opcode::Info?64:kAckBytes)) return Error::Shape;
    if(get16(p)>static_cast<U16>(Status::Internal)) return Error::Range;
    // Firmware always emits p[2:4]==0 but its C decoder does not check it.
    // Bridge enforces the documented reserved field; golden valid frames agree.
    if(!zero(p+2,2) || !zero(p+28,4)) return Error::Reserved;
    if(op==Opcode::Info) return n==64?Error::Ok:Error::Shape;
    if(op==Opcode::PathPage) {
      const unsigned offset=get16(p+24), count=get16(p+26);
      if(count>kPageCells || n!=kAckBytes+2*count) return Error::Shape;
      if(get16(p)!=0 && count) return Error::Shape;
      if(offset>4096 || offset+count>4096) return Error::Range;
      for(unsigned i=0;i<count;++i) if(get16(p+32+2*i)>=4096) return Error::Range;
      return Error::Ok;
    }
    return n==kAckBytes?Error::Ok:Error::Shape;
  }
  if(op==Opcode::Info) return n==0?Error::Ok:Error::Shape;
  if(!h.transport_seq) return Error::Range;
  if(op!=Opcode::Pose && op!=Opcode::State && op!=Opcode::Event &&
     op!=Opcode::PlanResult && !h.request_id) return Error::Range;
  if(op==Opcode::SessionBind) {
    if(n!=16) return Error::Shape;
    return !h.session_id && get64(p) && get64(p+8)?Error::Ok:Error::Range;
  }
  if(!h.session_id) return Error::Range;
  switch(op) {
    case Opcode::FrameBind:
      if(n!=68) return Error::Shape;
      if(!get32(p) || !p[4] || p[4]>63) return Error::Range;
      for(unsigned i=0;i<p[4];++i) if(!p[5+i]) return Error::Range;
      return zero(p+5+p[4],63-p[4])?Error::Ok:Error::Reserved;
    case Opcode::ClockAnchor:
      if(n!=24) return Error::Shape;
      if(!zero(p+16,8)) return Error::Reserved;
      return get64(p) && get32(p+12)<=100?Error::Ok:Error::Range;
    case Opcode::MapBegin:
      if(n!=32) return Error::Shape;
      if(!h.map_epoch || !h.map_revision || !get32(p) || get16(p+12)!=100 ||
         p[14]!=64 || p[15]!=64 || !get64(p+20)) return Error::Range;
      return zero(p+28,4)?Error::Ok:Error::Reserved;
    case Opcode::MapChunk:
      if(n!=48) return Error::Shape;
      if(!h.map_epoch || !h.map_revision || !get64(p) || get16(p+8)>=kMapBytes ||
         (get16(p+8)&31u) || get16(p+10)!=kMapChunkBytes) return Error::Range;
      return zero(p+12,4)?Error::Ok:Error::Reserved;
    case Opcode::MapCommit:
      if(n!=16) return Error::Shape;
      if(!h.map_epoch || !h.map_revision || !get64(p)) return Error::Range;
      return zero(p+12,4)?Error::Ok:Error::Reserved;
    case Opcode::GoalPlan:
      if(n!=24) return Error::Shape;
      if(!task(h) || !get32(p+8) || !get64(p+12)) return Error::Range;
      return zero(p+20,4)?Error::Ok:Error::Reserved;
    case Opcode::Start:
      if(n!=24) return Error::Shape;
      return task(h) && h.plan_id && get64(p) && get64(p+8) && get64(p+16)
        ?Error::Ok:Error::Range;
    case Opcode::Cancel:
      if(n!=8) return Error::Shape;
      return h.run_generation && !zero(h.goal_id.data(),16) && zero(p+4,4)
        ?Error::Ok:Error::Range;
    case Opcode::Pose: {
      if(n!=60) return Error::Shape;
      if(!h.map_epoch || !h.map_revision || !h.localization_epoch || !get32(p+12) ||
         !get64(p+16) || !get64(p+48) || get64(p+36)>250000000ULL) return Error::Range;
      // Explicit signed conversion, no implementation-defined unsigned->signed cast.
      const U32 raw=get32(p+8);
      const std::int64_t yaw=raw<=0x7fffffffU?raw:std::int64_t(raw)-0x100000000LL;
      if(yaw < -3141593 || yaw >= 3141593) return Error::Range;
      return (get32(p+56)&~3u)?Error::Reserved:Error::Ok;
    }
    case Opcode::PathPage:
      if(n!=8) return Error::Shape;
      return task(h) && h.plan_id && get16(p)<4096 && get16(p+2) && get16(p+2)<=kPageCells
        ?Error::Ok:Error::Range;
    case Opcode::State:
      if(n!=160) return Error::Shape;
      if(!zero(p+130,2) || !zero(p+152,8) || (get32(p+4)&~255u)) return Error::Reserved;
      return p[0]<=5 && p[1]<=3 && get32(p+124)<=1?Error::Ok:Error::Range;
    case Opcode::Event:
      if(n!=64) return Error::Shape;
      return zero(p+28,4) && zero(p+60,4)?Error::Ok:Error::Reserved;
    case Opcode::PlanResult:
      if(n!=64) return Error::Shape;
      if(!zero(p+60,4) || (get16(p+46)&~1u)) return Error::Reserved;
      return task(h) && get16(p)<=16 && get16(p+40)<=4096 && get16(p+42)<4096 &&
        get16(p+44)<4096?Error::Ok:Error::Range;
    default: return Error::Opcode;
  }
}
} // namespace

std::uint16_t crc16_ccitt_false(const U8* bytes,std::size_t n) noexcept {
  U16 crc=0xffff;
  for(std::size_t i=0;i<n;++i) {
    crc^=U16(U16(bytes[i])<<8);
    for(unsigned bit=0;bit<8;++bit) crc=U16((crc<<1)^((crc&0x8000u)?0x1021u:0u));
  }
  return crc;
}
Error encode(const Header& h,const Payload& payload,Wire& output) noexcept {
  const auto e=shape(h,payload); if(e!=Error::Ok) return e;
  Wire w{}; auto* p=w.data();
  p[0]='N';p[1]='A';p[2]='V';p[3]='3';p[4]=kVersion;p[5]=h.opcode;
  put16(p+6,h.flags); put64(p+8,h.transport_seq); put64(p+16,h.session_id);
  put64(p+24,h.run_generation); std::copy(h.goal_id.begin(),h.goal_id.end(),p+32);
  put64(p+48,h.map_epoch); put64(p+56,h.map_revision); put64(p+64,h.localization_epoch);
  put64(p+72,h.plan_id); put64(p+80,h.request_id); put16(p+88,h.payload_bytes);
  std::copy_n(payload.begin(),h.payload_bytes,p+96);
  put16(p+256,crc16_ccitt_false(p,256)); output=w; return Error::Ok;
}
Error decode(const U8* p,std::size_t n,Message& output) noexcept {
  if(!p) return Error::Pointer;
  if(n!=kWireBytes) return Error::Length;
  if(p[0]!='N'||p[1]!='A'||p[2]!='V'||p[3]!='3') return Error::Magic;
  if(p[4]!=kVersion) return Error::Version;
  if(crc16_ccitt_false(p,256)!=get16(p+256)) return Error::Crc;
  if(!zero(p+90,6)) return Error::Reserved;
  Message m; auto& h=m.header;
  h.opcode=p[5]; h.flags=get16(p+6); h.transport_seq=get64(p+8);
  h.session_id=get64(p+16); h.run_generation=get64(p+24);
  std::copy_n(p+32,16,h.goal_id.begin()); h.map_epoch=get64(p+48);
  h.map_revision=get64(p+56); h.localization_epoch=get64(p+64);
  h.plan_id=get64(p+72); h.request_id=get64(p+80); h.payload_bytes=get16(p+88);
  if(h.payload_bytes>kPayloadBytes) return Error::Length;
  if(!zero(p+96+h.payload_bytes,kPayloadBytes-h.payload_bytes)) return Error::Reserved;
  std::copy_n(p+96,kPayloadBytes,m.payload.begin());
  const auto e=shape(h,m.payload); if(e!=Error::Ok) return e;
  output=m; return Error::Ok;
}
Error parse_ack_prefix(const Message& m,AckPrefix& output) noexcept {
  if(!(m.header.opcode&0x80u)) return Error::Opcode;
  const auto e=shape(m.header,m.payload); if(e!=Error::Ok) return e;
  const auto* p=m.payload.data(); AckPrefix ack;
  ack.status=static_cast<Status>(get16(p)); ack.device_ms=get32(p+4);
  ack.nav_epoch=get64(p+8); ack.state_seq=get64(p+16); ack.value=get32(p+24);
  output=ack; return Error::Ok;
}
Error parse_info_extra(const Message& m,InfoExtra& output) noexcept {
  if(m.header.opcode!=reply_opcode(Opcode::Info)) return Error::Opcode;
  AckPrefix ack; const auto e=parse_ack_prefix(m,ack); if(e!=Error::Ok) return e;
  const auto* p=m.payload.data();
  if(get32(p+60)>1) return Error::Range;
  InfoExtra info;
  info.capabilities=get32(p+32); info.cpu_hz=get32(p+36);
  info.info_cookie=get64(p+40); info.current_session=get64(p+48);
  info.active_map_crc32=get32(p+56); info.ready=get32(p+60)!=0;
  output=info; return Error::Ok;
}
const char* error_name(Error e) noexcept {
  switch(e) {
    case Error::Ok:return "ok"; case Error::Length:return "length";
    case Error::Magic:return "magic"; case Error::Version:return "version";
    case Error::Crc:return "crc"; case Error::Flags:return "flags";
    case Error::Reserved:return "reserved"; case Error::Opcode:return "opcode";
    case Error::Shape:return "shape"; case Error::Range:return "range";
    case Error::Pointer:return "pointer";
  }
  return "unknown";
}
} // namespace acg138::nav3
