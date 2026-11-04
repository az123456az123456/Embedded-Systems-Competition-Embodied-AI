#include "acg138_fpga_map_bridge/nav3_wire.hpp"
extern "C" {
#include "nav_exec_wire.h"
}
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace n=acg138::nav3;
struct Fixture { std::string name; n::Message message; };
static std::ofstream cases;
static std::filesystem::path output;
static unsigned total=0, failed=0, goldens=0;
static std::string hex(const n::Wire& w) {
  std::ostringstream out; out<<std::hex<<std::setfill('0');
  for(auto b:w) out<<std::setw(2)<<unsigned(b);
  return out.str();
}
static void check(const std::string& name,bool ok,const n::Wire* wire=nullptr,
                  const std::string& detail="") {
  ++total; if(!ok) ++failed;
  cases<<"{\"case\":\""<<name<<"\",\"pass\":"<<(ok?"true":"false")
       <<",\"detail\":\""<<detail<<"\"";
  if(wire) cases<<",\"input_hex\":\""<<hex(*wire)<<"\"";
  cases<<"}\n";
  if(!ok) {
    std::cerr<<"FAIL "<<name<<" "<<detail<<"\n";
    if(wire) {
      std::ofstream bin(output/("failure-"+std::to_string(total)+".bin"),std::ios::binary);
      bin.write(reinterpret_cast<const char*>(wire->data()),wire->size());
    }
  }
}
static n::Message base(n::Opcode op,unsigned bytes) {
  n::Message m; auto& h=m.header; h.opcode=n::request_opcode(op);
  h.payload_bytes=std::uint16_t(bytes);
  h.transport_seq=0x0102030405060708ULL; h.session_id=0x1112131415161718ULL;
  h.run_generation=0x2122232425262728ULL;
  for(unsigned i=0;i<16;++i) h.goal_id[i]=std::uint8_t(0x30+i);
  h.map_epoch=0x4142434445464748ULL; h.map_revision=0x5152535455565758ULL;
  h.localization_epoch=0x6162636465666768ULL; h.plan_id=0x7172737475767778ULL;
  h.request_id=0x8182838485868788ULL; return m;
}
static ne_header golden_header(const n::Header& h) {
  ne_header g{};g.family=NE_NAV3;g.opcode=h.opcode;g.flags=h.flags;
  g.transport_seq=h.transport_seq;g.session_id=h.session_id;g.run_generation=h.run_generation;
  std::copy(h.goal_id.begin(),h.goal_id.end(),g.goal_id);
  g.map_epoch=h.map_epoch;g.map_revision=h.map_revision;g.localization_epoch=h.localization_epoch;
  g.plan_id=h.plan_id;g.request_id=h.request_id;g.payload_bytes=h.payload_bytes;return g;
}
static bool equal(const n::Header& a,const n::Header& b) {
  return a.opcode==b.opcode && a.flags==b.flags && a.transport_seq==b.transport_seq &&
    a.session_id==b.session_id && a.run_generation==b.run_generation && a.goal_id==b.goal_id &&
    a.map_epoch==b.map_epoch && a.map_revision==b.map_revision &&
    a.localization_epoch==b.localization_epoch && a.plan_id==b.plan_id &&
    a.request_id==b.request_id && a.payload_bytes==b.payload_bytes;
}
static bool equal(const n::Message& a,const n::Message& b) {
  return equal(a.header,b.header) && a.payload==b.payload;
}
static n::Wire golden(const n::Message& m) {
  auto g=golden_header(m.header); n::Wire w{};
  const auto e=ne_encode(&g,m.payload.data(),w.data());
  if(e) throw std::runtime_error("invalid test fixture: golden error "+std::to_string(e));
  return w;
}
static void fix(n::Wire& w) { ne_put16(w.data()+256,ne_crc16(w.data(),256)); }
static void reject(const std::string& name,const n::Wire& w,n::Error expected,
                   std::size_t size=n::kWireBytes) {
  auto out=base(n::Opcode::Event,64);out.payload.fill(0xa5);const auto before=out;
  const auto e=n::decode(w.data(),size,out);
  check(name,e==expected && equal(out,before),&w,
        std::string("expected=")+n::error_name(expected)+" actual="+n::error_name(e));
}
static std::vector<Fixture> fixtures() {
  std::vector<Fixture> v; n::Message m;
  auto add=[&](const std::string& name){v.push_back({name,m});};
  m=base(n::Opcode::Info,0);m.header.session_id=0;add("info_unbound");
  m=base(n::Opcode::SessionBind,16);m.header.session_id=0;
  ne_put64(m.payload.data(),0xfedcba9876543210ULL);ne_put64(m.payload.data()+8,0x8070605040302010ULL);add("session_bind");
  m=base(n::Opcode::FrameBind,68);ne_put32(m.payload.data(),1);m.payload[4]=3;
  std::copy_n("map",3,m.payload.begin()+5);add("frame_bind");
  m.payload[4]=63;std::fill(m.payload.begin()+5,m.payload.begin()+68,'m');add("frame_63");
  m=base(n::Opcode::ClockAnchor,24);ne_put64(m.payload.data(),8);
  ne_put32(m.payload.data()+8,0xfffffff0);ne_put32(m.payload.data()+12,100);add("anchor_rtt100");
  m=base(n::Opcode::MapBegin,32);ne_put32(m.payload.data(),7);
  ne_put32(m.payload.data()+4,std::uint32_t(-3200));ne_put32(m.payload.data()+8,std::uint32_t(-3200));
  ne_put16(m.payload.data()+12,100);m.payload[14]=64;m.payload[15]=64;
  ne_put32(m.payload.data()+16,0x12345678);ne_put64(m.payload.data()+20,0x1020304050607080ULL);add("map_begin");
  for(unsigned j=0;j<16;++j) {
    m=base(n::Opcode::MapChunk,48);ne_put64(m.payload.data(),0x1020304050607080ULL);
    ne_put16(m.payload.data()+8,std::uint16_t(j*32));ne_put16(m.payload.data()+10,32);
    for(unsigned i=0;i<32;++i)m.payload[16+i]=std::uint8_t(j*32+i);
    add("map_chunk_"+std::to_string(j));
  }
  m=base(n::Opcode::MapCommit,16);ne_put64(m.payload.data(),0x1020304050607080ULL);
  ne_put32(m.payload.data()+8,0x12345678);add("map_commit");
  m=base(n::Opcode::GoalPlan,24);ne_put32(m.payload.data(),3200);
  ne_put32(m.payload.data()+4,std::uint32_t(-100));ne_put32(m.payload.data()+8,7);
  ne_put64(m.payload.data()+12,0xfedcba9876543210ULL);add("goal_mm_signed");
  m=base(n::Opcode::Start,24);ne_put64(m.payload.data(),1);ne_put64(m.payload.data()+8,2);
  ne_put64(m.payload.data()+16,3);add("start_codec_only_no_transport");
  m=base(n::Opcode::Cancel,8);ne_put32(m.payload.data(),6);add("cancel_codec_only");
  for(auto yaw:{-3141593,0,3141592}) {
    m=base(n::Opcode::Pose,60);m.header.request_id=0;
    ne_put32(m.payload.data(),42);ne_put32(m.payload.data()+4,std::uint32_t(-42));
    ne_put32(m.payload.data()+8,std::uint32_t(yaw));ne_put32(m.payload.data()+12,7);
    ne_put64(m.payload.data()+16,0xfedcba9876543210ULL);ne_put64(m.payload.data()+24,10000000000ULL);
    ne_put32(m.payload.data()+32,1);ne_put64(m.payload.data()+36,250000000);
    ne_put32(m.payload.data()+44,0xfffffff0);ne_put64(m.payload.data()+48,8);
    ne_put32(m.payload.data()+56,3);add("pose_yaw_"+std::to_string(yaw));
  }
  m=base(n::Opcode::PathPage,8);ne_put16(m.payload.data(),4095);ne_put16(m.payload.data()+2,64);
  ne_put32(m.payload.data()+4,0x12345678);add("page_request_last");
  m=base(n::Opcode::State,160);m.header.request_id=0;m.payload[0]=5;m.payload[1]=3;
  ne_put32(m.payload.data()+4,255);ne_put32(m.payload.data()+64,0xabcdef01);
  ne_put32(m.payload.data()+124,1);add("unsolicited_state");
  m=base(n::Opcode::Event,64);m.header.request_id=0;add("unsolicited_event");
  m=base(n::Opcode::PlanResult,64);m.header.request_id=0;
  ne_put16(m.payload.data()+40,4096);ne_put16(m.payload.data()+42,0);
  ne_put16(m.payload.data()+44,4095);ne_put16(m.payload.data()+46,1);
  ne_put32(m.payload.data()+48,0x12345678);add("unsolicited_plan_result");
  for(auto op:{n::Opcode::Info,n::Opcode::SessionBind,n::Opcode::FrameBind,
              n::Opcode::ClockAnchor,n::Opcode::MapBegin,n::Opcode::MapChunk,
              n::Opcode::MapCommit,n::Opcode::GoalPlan,n::Opcode::Start,n::Opcode::Cancel}) {
    m=base(op,op==n::Opcode::Info?64:32);m.header.opcode=n::reply_opcode(op);
    ne_put32(m.payload.data()+4,0xfffffff0);ne_put64(m.payload.data()+8,0x1122334455667788ULL);
    ne_put64(m.payload.data()+16,0x8877665544332211ULL);
    if(op==n::Opcode::Info) {
      ne_put32(m.payload.data()+32,63);ne_put32(m.payload.data()+36,50000000);
      ne_put64(m.payload.data()+40,0x9080706050403020ULL);ne_put64(m.payload.data()+48,0xfedcba9876543210ULL);
      ne_put32(m.payload.data()+56,0xabcdef01);ne_put32(m.payload.data()+60,1);
    }
    add("ack_op_"+std::to_string(m.header.opcode));
  }
  for(unsigned status=0;status<=16;++status) {
    m=base(n::Opcode::MapCommit,32);m.header.opcode=n::reply_opcode(n::Opcode::MapCommit);
    ne_put16(m.payload.data(),std::uint16_t(status));add("ack_status_"+std::to_string(status));
  }
  for(unsigned count=0;count<=64;++count) {
    m=base(n::Opcode::PathPage,32+2*count);m.header.opcode=n::reply_opcode(n::Opcode::PathPage);
    ne_put16(m.payload.data()+24,std::uint16_t(4096-count));ne_put16(m.payload.data()+26,std::uint16_t(count));
    for(unsigned i=0;i<count;++i)ne_put16(m.payload.data()+32+2*i,std::uint16_t(4095-i));
    add("page_ack_count_"+std::to_string(count));
  }
  return v;
}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  output=argv[1];std::filesystem::create_directories(output);
  cases.open(output/"cases.jsonl");if(!cases)return 2;
  try {
    check("crc_known_123456789",n::crc16_ccitt_false(reinterpret_cast<const std::uint8_t*>("123456789"),9)==0x29b1);
    check("crc_empty",n::crc16_ccitt_false(nullptr,0)==0xffff);
    const auto all=fixtures();
    for(const auto& f:all) {
      auto expected=golden(f.message);n::Wire w{};w.fill(0xa5);
      auto e=n::encode(f.message.header,f.message.payload,w);
      check(f.name+"_golden",e==n::Error::Ok && w==expected,&expected);++goldens;
      n::Message decoded; e=n::decode(expected,decoded);
      check(f.name+"_decode",e==n::Error::Ok && equal(decoded,f.message),&expected);
      n::Wire roundtrip{};e=n::encode(decoded.header,decoded.payload,roundtrip);
      check(f.name+"_roundtrip",e==n::Error::Ok && roundtrip==expected,&expected);
      std::ofstream frame(output/(f.name+".bin"),std::ios::binary);
      frame.write(reinterpret_cast<const char*>(expected.data()),expected.size());
      // Mutation oracle: each byte independently flipped with a recomputed CRC.
      // The sole allowed stricter decoder policy is ACK's reserved p[2:4].
      for(unsigned i=5;i<256;++i) {
        auto mutated=expected;mutated[i]^=0x01;fix(mutated);
        ne_message oracle{};const auto g=ne_decode(mutated.data(),mutated.size(),&oracle);
        n::Message got;const auto c=n::decode(mutated,got);
        const bool strict_ack=(f.message.header.opcode&0x80u) && (i==98 || i==99);
        check(f.name+"_mutate_"+std::to_string(i),
              strict_ack ? c==n::Error::Reserved : int(c)==g,&mutated,
              "golden="+std::to_string(g)+" cpp="+std::to_string(int(c)));
      }
    }
    const auto basic=golden(all.front().message);
    for(std::size_t size=0;size<=300;++size)if(size!=258)
      reject("length_"+std::to_string(size),basic,n::Error::Length,size);
    for(unsigned version=0;version<256;++version)if(version!=1) {
      auto w=basic;w[4]=std::uint8_t(version);fix(w);reject("version_"+std::to_string(version),w,n::Error::Version);
    }
    for(unsigned i=0;i<258;++i) {
      auto w=basic;w[i]^=1;
      reject("corrupt_byte_"+std::to_string(i),w,i<4?n::Error::Magic:i==4?n::Error::Version:n::Error::Crc);
    }
    for(unsigned i=90;i<256;++i) {
      auto w=basic;w[i]=1;fix(w);reject("reserved_or_padding_"+std::to_string(i),w,n::Error::Reserved);
    }
    for(unsigned bit=0;bit<16;++bit) {
      auto w=basic;ne_put16(w.data()+6,std::uint16_t(1u<<bit));fix(w);reject("flags_bit_"+std::to_string(bit),w,n::Error::Flags);
    }
    for(unsigned count:{161u,65535u}) {
      auto w=basic;ne_put16(w.data()+88,std::uint16_t(count));fix(w);reject("payload_overflow_"+std::to_string(count),w,n::Error::Length);
    }
    auto w=basic;std::copy_n("NAV2",4,w.begin());fix(w);reject("NAV2_not_NAV3",w,n::Error::Magic);
    w=basic;std::copy_n("NVA1",4,w.begin());fix(w);reject("NVA1_not_ROS_NAV3_channel",w,n::Error::Magic);
    n::Message out=base(n::Opcode::Info,0), before=out;
    check("null_decode",n::decode(nullptr,258,out)==n::Error::Pointer && equal(out,before));
    auto invalid=all.front().message;invalid.header.flags=1;w.fill(0xa5);const auto unchanged=w;
    check("encode_failure_atomic",n::encode(invalid.header,invalid.payload,w)==n::Error::Flags && w==unchanged);
    auto padding=all.front().message;padding.payload.fill(0xaa);w.fill(0);
    check("encode_zero_padding",n::encode(padding.header,padding.payload,w)==n::Error::Ok && w==basic,&w);
    for(const auto& f:all)if(f.message.header.opcode==n::reply_opcode(n::Opcode::Info)) {
      n::Message m;n::decode(golden(f.message),m);n::AckPrefix ack;n::InfoExtra info;
      check("info_ack_fields",n::parse_ack_prefix(m,ack)==n::Error::Ok && ack.status==n::Status::Ok &&
        ack.device_ms==0xfffffff0 && ack.nav_epoch==0x1122334455667788ULL && ack.state_seq==0x8877665544332211ULL && !ack.value);
      check("info_extra_fields",n::parse_info_extra(m,info)==n::Error::Ok && info.capabilities==63 &&
        info.cpu_hz==50000000 && info.info_cookie==0x9080706050403020ULL &&
        info.current_session==0xfedcba9876543210ULL && info.active_map_crc32==0xabcdef01 && info.ready);
      m.header.payload_bytes=32;auto old=info;
      check("info_must_be_64",n::parse_info_extra(m,info)==n::Error::Shape && info.info_cookie==old.info_cookie);
      m=f.message;ne_put32(m.payload.data()+60,2);
      check("info_ready_not_boolean",n::parse_info_extra(m,info)==n::Error::Range && info.info_cookie==old.info_cookie);
    }
    for(auto op:{n::Opcode::State,n::Opcode::PlanResult,n::Opcode::Event}) {
      const auto it=std::find_if(all.begin(),all.end(),[&](const Fixture& f){return f.message.header.opcode==n::request_opcode(op);});
      const auto w2=golden(it->message);n::Message m;
      check("async_accept_"+std::to_string(int(op)),n::decode(w2,m)==n::Error::Ok,&w2);
      n::AckPrefix a;a.device_ms=123;
      check("async_not_ack_"+std::to_string(int(op)),n::parse_ack_prefix(m,a)==n::Error::Opcode && a.device_ms==123);
      auto bad=w2;bad[5]|=0x80;fix(bad);reject("async_bit7_forbidden_"+std::to_string(int(op)),bad,n::Error::Opcode);
    }
    // Explicit maxima/overflows with correctly recomputed CRC, not just CRC rejection.
    for(const auto& f:all)if(f.name=="page_ack_count_64") {
      auto bad=golden(f.message);ne_put16(bad.data()+96+26,65);fix(bad);reject("page65_reject",bad,n::Error::Shape);
      bad=golden(f.message);ne_put16(bad.data()+96+24,4095);fix(bad);reject("page_end_overflow",bad,n::Error::Range);
      bad=golden(f.message);ne_put16(bad.data()+96,9);fix(bad);reject("error_page_has_cells",bad,n::Error::Shape);
      bad=golden(f.message);ne_put16(bad.data()+128,4096);fix(bad);reject("cell4096_reject",bad,n::Error::Range);
      n::AckPrefix a;check("page_prefix_offset_count",n::parse_ack_prefix(f.message,a)==n::Error::Ok && (a.value>>16)==4032 && (a.value&65535)==64);
    }
    for(const auto& f:all)if(f.name=="pose_yaw_0") {
      for(std::uint64_t age:{249999999ULL,250000000ULL,250000001ULL,0xffffffffffffffffULL}) {
        auto bad=golden(f.message);ne_put64(bad.data()+96+36,age);fix(bad);n::Message m;
        check("pose_age_"+std::to_string(age),n::decode(bad,m)==(age<=250000000?n::Error::Ok:n::Error::Range),&bad);
      }
      for(auto yaw:{-3141594,3141593}) {
        auto bad=golden(f.message);ne_put32(bad.data()+96+8,std::uint32_t(yaw));fix(bad);
        reject("yaw_outside_"+std::to_string(yaw),bad,n::Error::Range);
      }
    }
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';check("test_harness_exception",false);
  }
  std::ofstream summary(output/"summary.json");
  summary<<"{\"stage\":0,\"cases\":"<<total<<",\"failed\":"<<failed
    <<",\"golden_vectors\":"<<goldens<<",\"pass\":"<<(failed?"false":"true")
    <<",\"network_frames_sent\":0,\"golden_linkage\":\"test_only\"}\n";
  std::cout<<"NAV3 stage0 cases="<<total<<" failed="<<failed<<" golden="<<goldens<<'\n';
  return failed?1:0;
}
