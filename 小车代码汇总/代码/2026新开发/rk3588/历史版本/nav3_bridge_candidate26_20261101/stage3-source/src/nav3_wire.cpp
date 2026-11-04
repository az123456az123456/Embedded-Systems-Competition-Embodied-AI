#include "acg138_fpga_map_bridge/nav3_wire.hpp"
#include <algorithm>
#include <limits>
#include <sstream>
#include <utility>

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

Client::Client(Channel& channel,unsigned timeout_ms,std::function<void(const std::string&)> log)
 :channel_(channel),timeout_ms_(timeout_ms),log_(std::move(log)) {
  if(timeout_ms==0 || timeout_ms>1000)throw std::invalid_argument("NAV3 ACK timeout must be 1..1000ms");
}
Header Client::request(Opcode op,unsigned bytes,Lane lane) {
  auto& seq=sequences_[static_cast<std::size_t>(lane)];
  if(seq==std::numeric_limits<U64>::max() || next_request_==std::numeric_limits<U64>::max())
    throw std::overflow_error("NAV3 sequence exhausted; bind a new session");
  Header h;h.opcode=request_opcode(op);h.payload_bytes=U16(bytes);
  h.transport_seq=++seq;h.request_id=++next_request_;h.session_id=session_;
  h.map_epoch=1;h.localization_epoch=1;return h;
}
Message Client::exchange(const Header& h,const Payload& payload) {
  Wire w;const auto e=encode(h,payload,w);
  if(e!=Error::Ok)throw std::invalid_argument(std::string("NAV3 encode: ")+error_name(e));
  // Line A has no motion permission or START API, even if an upstream caller errs.
  if(h.opcode==request_opcode(Opcode::Start))throw std::logic_error("NAV3 line A cannot START");
  channel_.send(w);++stats_.sent;
  if(log_) {
    std::ostringstream s;s<<"NAV3 TX op="<<unsigned(h.opcode)<<" seq="<<h.transport_seq
      <<" request="<<h.request_id<<" session="<<h.session_id<<" payload_bytes="<<h.payload_bytes;
    log_(s.str());
  }
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms_);
  while(std::chrono::steady_clock::now()<deadline) {
    const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();
    Wire reply;
    if(!channel_.receive(reply,unsigned(std::max<std::int64_t>(1,left))))continue;
    Message m;if(decode(reply,m)!=Error::Ok){++stats_.malformed;continue;}
    ++stats_.received;
    const auto& r=m.header;
    if(!(r.opcode&0x80u)) {
      receive_async(m);
      ++stats_.ignored;continue; // No unbounded queue; async is never a request ACK.
    }
    if(r.opcode!=(h.opcode|0x80u) || r.transport_seq!=h.transport_seq ||
       r.request_id!=h.request_id || r.session_id!=h.session_id || r.run_generation!=h.run_generation ||
       r.goal_id!=h.goal_id || r.map_epoch!=h.map_epoch || r.map_revision!=h.map_revision ||
       r.localization_epoch!=h.localization_epoch || r.plan_id!=h.plan_id) {++stats_.ignored;continue;}
    AckPrefix ack;if(parse_ack_prefix(m,ack)!=Error::Ok){++stats_.malformed;continue;}
    ack_=ack;
    if(log_) {
      std::ostringstream s;s<<"NAV3 RX op="<<unsigned(r.opcode)<<" seq="<<r.transport_seq
       <<" request="<<r.request_id<<" session="<<r.session_id<<" payload_bytes="<<r.payload_bytes
       <<" status="<<unsigned(ack.status)<<" nav_epoch="<<ack.nav_epoch<<" device_ms="<<ack.device_ms;
      log_(s.str());
    }
    if(ack.status!=Status::Ok)throw Rejected(ack.status,"NAV3 opcode "+std::to_string(h.opcode)+" rejected status="+std::to_string(unsigned(ack.status)));
    return m;
  }
  throw Timeout("NAV3 ACK timeout opcode="+std::to_string(h.opcode)+" request="+std::to_string(h.request_id));
}
InfoReply Client::info() {
  auto h=request(Opcode::Info,0);Payload p{};InfoReply i;
  i.sent_at=std::chrono::steady_clock::now();const auto m=exchange(h,p);
  i.received_at=std::chrono::steady_clock::now();i.header=m.header;
  if(parse_ack_prefix(m,i.ack)!=Error::Ok || parse_info_extra(m,i.extra)!=Error::Ok)
    throw std::runtime_error("NAV3 malformed INFO fields");
  info_=i;
  if(log_) {
    std::ostringstream s;s<<"NAV3 INFO session="<<i.extra.current_session<<" info_cookie="<<i.extra.info_cookie
      <<" ready="<<i.extra.ready<<" capabilities="<<i.extra.capabilities<<" cpu_hz="<<i.extra.cpu_hz;
    log_(s.str());
  }
  return i;
}
AckPrefix Client::session_bind(U64 new_session) {
  if(!new_session || new_session==info_.extra.current_session || !info_.extra.info_cookie)
    throw std::invalid_argument("NAV3 requires a fresh nonzero session and INFO cookie");
  if(!info_.extra.ready)throw std::runtime_error("FPGA INFO ready=0; no binding attempted");
  const auto age=std::chrono::steady_clock::now()-info_.sent_at;
  if(age>=std::chrono::milliseconds(1000))throw std::runtime_error("INFO cookie age >=1000ms; obtain new INFO");
  auto h=request(Opcode::SessionBind,16);h.session_id=0;
  Payload p{};put64(p.data(),new_session);put64(p.data()+8,info_.extra.info_cookie);
  try { exchange(h,p); }
  catch(const Timeout&) {
    // Never retransmit SESSION_BIND with a consumed cookie. Query actual session.
    const auto recovered=info();
    if(recovered.extra.current_session!=new_session)throw Timeout("SESSION_BIND ACK lost and INFO did not confirm requested session");
    if(log_)log_("NAV3 SESSION_BIND recovered by fresh INFO; no bind replay");
  }
  session_=new_session;anchor_=0;next_pose_=0;next_run_=0;previous_pose_stamp_=-1;plan_request_.reset();pending_plan_result_.reset();
  // All firmware floors were reset by this successful new-session transaction.
  sequences_.fill(0);info_.extra.info_cookie=0;latest_state_.reset();map_={};
  next_map_revision_=0;next_transaction_=0;return ack_;
}
AckPrefix Client::frame_bind(U32 token,const std::string& frame) {
  if(!session_ || !token || frame.empty() || frame.size()>63 || frame.find('\0')!=std::string::npos)
    throw std::invalid_argument("NAV3 frame needs session, nonzero token and 1..63 non-NUL bytes");
  auto h=request(Opcode::FrameBind,68);Payload p{};put32(p.data(),token);p[4]=U8(frame.size());
  std::copy(frame.begin(),frame.end(),p.begin()+5);exchange(h,p);return ack_;
}
AckPrefix Client::clock_anchor(U64 anchor_id) {
  if(!session_ || !anchor_id || anchor_id<=anchor_)throw std::invalid_argument("NAV3 anchor must increase within session");
  const auto now=std::chrono::steady_clock::now();
  const auto rtt=std::chrono::duration_cast<std::chrono::nanoseconds>(info_.received_at-info_.sent_at).count();
  const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(now-info_.sent_at).count();
  const auto rtt_ms=(rtt+999999)/1000000;
  if(elapsed<0 || elapsed>=1000 || rtt_ms<0 || rtt_ms>100 || !info_.extra.info_cookie)
    throw std::runtime_error("NAV3 CLOCK_ANCHOR requires fresh INFO, RTT<=100ms");
  auto h=request(Opcode::ClockAnchor,24);Payload p{};
  put64(p.data(),anchor_id);put32(p.data()+8,info_.ack.device_ms);put32(p.data()+12,U32(rtt_ms));
  exchange(h,p);anchor_=anchor_id;anchor_info_=info_;return ack_;
}
void Client::handshake(U64 new_session,U32 token,const std::string& frame) {
  info();session_bind(new_session);frame_bind(token,frame);info();clock_anchor(1);
}
U32 crc32_ieee(const U8* bytes,std::size_t n) noexcept {
  U32 crc=0xffffffffu;
  for(std::size_t i=0;i<n;++i) {
    crc^=bytes[i];
    for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^((crc&1)?0xedb88320u:0u);
  }
  return crc^0xffffffffu;
}
void Client::accept_state(const Message& m) {
  if(m.header.opcode!=request_opcode(Opcode::State) || !session_ || m.header.session_id!=session_)return;
  if(latest_state_ && m.header.transport_seq<=latest_state_->header.transport_seq)return;
  latest_state_=m;
}
bool Client::wait_map_state(U32 revision,U32 crc,unsigned timeout_ms) {
  // STATE localization_epoch comes from the current task, hence is zero after
  // MAP_COMMIT clears that task. Map proof binds session + active epoch/revision/CRC.
  const auto matches=[&]{return latest_state_ && latest_state_->header.map_epoch==1 &&
    latest_state_->header.map_revision==revision &&
    get32(latest_state_->payload.data()+64)==crc;};
  if(matches())return true;
  const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
  while(std::chrono::steady_clock::now()<end) {
    Wire w;const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(end-std::chrono::steady_clock::now()).count();
    if(!channel_.receive(w,unsigned(std::max<std::int64_t>(1,left))))continue;
    Message m;if(decode(w,m)!=Error::Ok){++stats_.malformed;continue;}
    ++stats_.received;receive_async(m);if(matches())return true;
    ++stats_.ignored;
  }
  return false;
}
MapReceipt Client::upload(const Bitmap& bitmap,const MapGeometry& geometry) {
  if(!session_ || !anchor_)throw std::logic_error("NAV3 upload requires handshake");
  if(!geometry.frame_token || geometry.origin_x_mm>std::numeric_limits<std::int32_t>::max()-6400 ||
     geometry.origin_y_mm>std::numeric_limits<std::int32_t>::max()-6400)throw std::invalid_argument("invalid map geometry");
  const U32 crc=crc32_ieee(bitmap.data(),bitmap.size());
  // No re-encoding/re-upload for unchanged geometry and bytes. CRC alone is insufficient.
  if(map_.committed && bitmap==bitmap_ && geometry.origin_x_mm==geometry_.origin_x_mm &&
     geometry.origin_y_mm==geometry_.origin_y_mm && geometry.frame_token==geometry_.frame_token) {
    if(!map_.state_verified)map_.state_verified=wait_map_state(map_.revision,map_.crc32,250);
    return map_;
  }
  if(next_map_revision_==std::numeric_limits<U32>::max() || next_transaction_==std::numeric_limits<U64>::max())
    throw std::overflow_error("map revision/transaction exhausted; new session required");
  MapReceipt pending;pending.revision=++next_map_revision_;pending.transaction=++next_transaction_;pending.crc32=crc;
  const auto send_map=[&](Opcode op,const Payload& p,unsigned bytes){
    auto h=request(op,bytes,Lane::Map);h.map_revision=pending.revision;
    // At most one retry, only for these firmware-defined idempotent map operations.
    // Same transaction and payload, same request ID; no generic ACK cache assumed.
    try{exchange(h,p);}catch(const Timeout&){if(log_)log_("NAV3 map ACK retry same immutable transaction");exchange(h,p);}
  };
  Payload p{};put32(p.data(),geometry.frame_token);put32(p.data()+4,U32(geometry.origin_x_mm));
  put32(p.data()+8,U32(geometry.origin_y_mm));put16(p.data()+12,100);p[14]=64;p[15]=64;
  put32(p.data()+16,crc);put64(p.data()+20,pending.transaction);send_map(Opcode::MapBegin,p,32);
  for(unsigned i=0;i<kMapChunks;++i) {
    p={};put64(p.data(),pending.transaction);put16(p.data()+8,U16(i*kMapChunkBytes));
    put16(p.data()+10,U16(kMapChunkBytes));std::copy_n(bitmap.begin()+i*kMapChunkBytes,kMapChunkBytes,p.begin()+16);
    send_map(Opcode::MapChunk,p,48);
  }
  p={};put64(p.data(),pending.transaction);put32(p.data()+8,crc);send_map(Opcode::MapCommit,p,16);
  // ACK OK is the only commit point; STATE verification is recorded separately.
  pending.committed=true;plan_request_.reset();pending_plan_result_.reset();pending.state_verified=wait_map_state(pending.revision,crc,250);
  map_=pending;bitmap_=bitmap;geometry_=geometry;
  if(log_) {
    std::ostringstream s;s<<"NAV3 MAP committed="<<map_.committed<<" revision="<<map_.revision
      <<" crc32="<<map_.crc32<<" state_verified="<<map_.state_verified;
    log_(s.str());
  }
  return map_;
}
void Client::receive_async(const Message& m) {
  accept_state(m);
  if(m.header.opcode!=request_opcode(Opcode::PlanResult) || !plan_request_)return;
  const auto &a=m.header,&b=*plan_request_;
  if(a.session_id!=b.session_id || a.run_generation!=b.run_generation || a.goal_id!=b.goal_id ||
     a.map_epoch!=b.map_epoch || a.map_revision!=b.map_revision || a.localization_epoch!=b.localization_epoch)return;
  if(pending_plan_result_ && a.transport_seq<=pending_plan_result_->header.transport_seq)return;
  pending_plan_result_=m;
}
U16 Client::cell(std::int32_t x,std::int32_t y)const {
  const auto dx=std::int64_t(x)-geometry_.origin_x_mm,dy=std::int64_t(y)-geometry_.origin_y_mm;
  if(dx<0 || dy<0 || dx>=6400 || dy>=6400)throw Rejected(Status::GoalOutsideWindow,"NAV3 coordinate outside active 6.4m window");
  return U16((dy/100)*64+dx/100);
}
U64 Client::send_pose(const PoseSample& pose) {
  const auto now=std::chrono::steady_clock::now();
  const auto queued=std::chrono::duration_cast<std::chrono::nanoseconds>(now-pose.sampled_at).count();
  if(queued<0 || pose.source_stamp_ns<0 || pose.source_stamp_ns<=previous_pose_stamp_ ||
     pose.source_age_ns>250000000u || U64(queued)>250000000u-pose.source_age_ns)
    throw Rejected(Status::PoseStale,"NAV3 pose is old, repeated, or from changed host time");
  const U64 age=pose.source_age_ns+U64(queued);
  const auto lower=std::chrono::duration_cast<std::chrono::milliseconds>(now-anchor_info_.received_at).count();
  const auto upper=std::chrono::duration_cast<std::chrono::nanoseconds>(now-anchor_info_.sent_at).count();
  const auto budget=(250000000u-age)/1000000u;
  // INFO gives an interval, not an exact shared clock. The lower endpoint is
  // conservative; reserve 2ms quantisation/drift margin, never renew old TF.
  if(lower<0 || upper<0 || upper>=1000000000LL || budget<3 ||
     U64((upper+999999)/1000000+1-lower)>=budget-2)
    throw Rejected(Status::PoseStale,"NAV3 pose budget insufficient for clock uncertainty");
  if(next_pose_==std::numeric_limits<U64>::max())throw std::overflow_error("pose sequence exhausted");
  cell(pose.x_mm,pose.y_mm);
  auto h=request(Opcode::Pose,60,Lane::Pose);h.map_revision=map_.revision;
  Payload p{};put32(p.data(),U32(pose.x_mm));put32(p.data()+4,U32(pose.y_mm));put32(p.data()+8,U32(pose.yaw_urad));
  put32(p.data()+12,geometry_.frame_token);put64(p.data()+16,++next_pose_);put64(p.data()+24,U64(pose.source_stamp_ns));
  put32(p.data()+32,1);put64(p.data()+36,age);
  put32(p.data()+44,anchor_info_.ack.device_ms+U32(lower)+U32(budget)-2);
  put64(p.data()+48,anchor_);put32(p.data()+56,3);
  Wire w;if(encode(h,p,w)!=Error::Ok)throw std::invalid_argument("invalid NAV3 pose");
  channel_.send(w);++stats_.sent;previous_pose_stamp_=pose.source_stamp_ns;
  if(log_)log_("NAV3 POSE sent seq="+std::to_string(next_pose_)+" source_age_ns="+std::to_string(age));
  // POSE has no ACK. Stage3 waits for its exact sequence in real STATE before PLAN.
  const auto end=now+std::chrono::milliseconds(std::min<unsigned>(timeout_ms_,unsigned(budget)));
  while(std::chrono::steady_clock::now()<end) {
    if(latest_state_ && latest_state_->header.session_id==session_ && latest_state_->header.map_epoch==1 &&
       latest_state_->header.map_revision==map_.revision && get64(latest_state_->payload.data()+24)==next_pose_ &&
       (get32(latest_state_->payload.data()+4)&32u) && get32(latest_state_->payload.data()+44)<=250)return next_pose_;
    Wire incoming;if(!channel_.receive(incoming,5))continue;
    Message m;if(decode(incoming,m)!=Error::Ok){++stats_.malformed;continue;}++stats_.received;receive_async(m);
  }
  throw Timeout("NAV3 POSE not confirmed by STATE; GOAL not sent");
}
Message Client::wait_plan(unsigned timeout_ms) {
  const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
  while(std::chrono::steady_clock::now()<end) {
    if(pending_plan_result_)return *pending_plan_result_;
    Wire w;if(!channel_.receive(w,5))continue;
    Message m;if(decode(w,m)!=Error::Ok){++stats_.malformed;continue;}++stats_.received;receive_async(m);
  }
  throw Timeout("NAV3 PLAN_RESULT timeout; no cached path substituted");
}
PlanResult Client::plan(const std::array<U8,16>& goal_id,const PoseSample& pose,std::int32_t gx,std::int32_t gy) {
  if(!map_.committed || !map_.state_verified)throw std::logic_error("PLAN needs a STATE-verified committed map");
  if(std::all_of(goal_id.begin(),goal_id.end(),[](U8 b){return b==0;}))throw std::invalid_argument("zero goal ID");
  const U16 start=cell(pose.x_mm,pose.y_mm),goal=cell(gx,gy);
  if(next_run_==std::numeric_limits<U64>::max() || anchor_==std::numeric_limits<U64>::max())throw std::overflow_error("run/anchor exhausted");
  // Stage3 one-shot pose: re-anchor before sending, including this elapsed time
  // in source_age. No POSE or GOAL is sent if the source expired while anchoring.
  info();clock_anchor(anchor_+1);const U64 pose_seq=send_pose(pose);
  auto h=request(Opcode::GoalPlan,24);h.run_generation=++next_run_;h.goal_id=goal_id;h.map_revision=map_.revision;
  Payload p{};put32(p.data(),U32(gx));put32(p.data()+4,U32(gy));put32(p.data()+8,geometry_.frame_token);put64(p.data()+12,pose_seq);
  plan_request_=h;pending_plan_result_.reset();exchange(h,p);
  const auto event=wait_plan(1500);const auto* q=event.payload.data();PlanResult result;
  result.identity=event.header;result.status=Status(get16(q));result.start_pose_seq=get64(q+24);result.nav_epoch=get64(q+8);
  result.start_cell=get16(q+42);result.goal_cell=get16(q+44);result.planned_revision=get32(q+48);
  result.expanded=get32(q+32);result.cycles=get32(q+36);result.cpu_hz=get32(q+56);
  const U16 count=get16(q+40),flags=get16(q+46);
  if(result.start_pose_seq!=pose_seq || result.start_cell!=start || result.goal_cell!=goal ||
     get32(q+52)!=geometry_.frame_token || result.nav_epoch!=ack_.nav_epoch)
    throw std::runtime_error("PLAN_RESULT source/geometry/nav_epoch mismatch");
  if(result.status!=Status::Ok) {
    if(count || flags)throw std::runtime_error("failed PLAN_RESULT carried a valid path");
    return result;
  }
  if(!result.identity.plan_id || !result.planned_revision || !count || count>4096 || flags!=1)
    throw std::runtime_error("PLAN_RESULT invalid path descriptor");
  result.cells.reserve(count);
  for(unsigned off=0;off<count;) {
    auto page=request(Opcode::PathPage,8);page.run_generation=h.run_generation;page.goal_id=h.goal_id;
    page.map_revision=h.map_revision;page.plan_id=result.identity.plan_id;p={};
    const unsigned n=std::min<unsigned>(kPageCells,count-off);
    put16(p.data(),U16(off));put16(p.data()+2,U16(n));put32(p.data()+4,result.planned_revision);
    const auto reply=exchange(page,p);const auto* data=reply.payload.data();
    if(get16(data+24)!=off || get16(data+26)!=n || reply.header.payload_bytes!=32+2*n)
      throw std::runtime_error("PATH_PAGE offset/count mismatch");
    for(unsigned i=0;i<n;++i)result.cells.push_back(get16(data+32+2*i));
    off+=n;
  }
  std::array<bool,4096> visited{};
  for(unsigned i=0;i<result.cells.size();++i) {
    const unsigned id=result.cells[i];
    if(id>=4096 || (bitmap_[id/8]&(1u<<(id%8))) || visited[id])throw std::runtime_error("PATH_PAGE blocked/repeated/out-of-range cell");
    visited[id]=true;
    if(i) {
      const int prev=result.cells[i-1];
      if(std::abs(int(id/64)-prev/64)+std::abs(int(id%64)-prev%64)!=1)throw std::runtime_error("PATH_PAGE not four-connected");
    }
  }
  if(result.cells.front()!=start || result.cells.back()!=goal)throw std::runtime_error("PATH_PAGE endpoints mismatch");
  if(log_)log_("NAV3 PLAN_RESULT plan="+std::to_string(result.identity.plan_id)+" cells="+std::to_string(result.cells.size())+" cycles="+std::to_string(result.cycles));
  return result;
}
} // namespace acg138::nav3
