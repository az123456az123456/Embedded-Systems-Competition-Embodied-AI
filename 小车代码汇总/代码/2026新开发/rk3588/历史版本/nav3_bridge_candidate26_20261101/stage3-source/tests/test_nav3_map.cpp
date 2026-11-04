#include "acg138_fpga_map_bridge/nav3_wire.hpp"
extern "C" {
#include "reference/nav3/candidate26/softcore/firmware/nav_exec.h"
}
#include <algorithm>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
namespace n=acg138::nav3;
struct Device : n::Channel {
  nav_server server{};nav_exec exec{};std::deque<n::Wire> rx;
  std::vector<unsigned> ops;unsigned chunks=0,commits=0,begins=0;
  std::uint32_t now=100;std::uint64_t event_seq=0;
  bool drop_chunk_once=false,bad_commit=false,bad_state_crc=false,atomic=true;
  Device(){nav_server_init(&server);nav_exec_init(&exec,&server);}
  void send(const n::Wire& input) override {
    auto w=input;ne_message m{};
    if(ne_decode(w.data(),w.size(),&m))throw std::runtime_error("wire rejected by frozen decoder");
    ops.push_back(m.h.opcode);++now;
    if(m.h.opcode==NE_MAP_CHUNK) {++chunks; if(exec.active.valid && exec.active.crc32!=nav_crc32(server.planner.occupied,512))atomic=false;}
    if(m.h.opcode==NE_MAP_BEGIN)++begins;
    if(m.h.opcode==NE_MAP_COMMIT){
      ++commits;
      if(bad_commit){w[96+8]^=1;ne_put16(w.data()+256,ne_crc16(w.data(),256));
        if(ne_decode(w.data(),w.size(),&m))throw std::runtime_error("bad test injection");}
    }
    nav_exec_set_external_quiet(&exec,1);nav_exec_set_map_quiet(&exec,1);
    const auto status=nav_exec_handle(&exec,&m,now,10000);
    if(m.h.opcode==NE_MAP_CHUNK && drop_chunk_once){drop_chunk_once=false;return;}
    auto h=m.h;h.opcode|=0x80;h.payload_bytes=m.h.opcode==NE_INFO?64:32;
    n::Payload p{};ne_put16(p.data(),std::uint16_t(status));ne_put32(p.data()+4,now);
    ne_put64(p.data()+8,exec.nav_epoch);ne_put64(p.data()+16,exec.state_seq);
    if(m.h.opcode==NE_INFO){ne_put32(p.data()+32,63);ne_put32(p.data()+36,50000000);
      ne_put64(p.data()+40,exec.guard.info_cookie);ne_put64(p.data()+48,exec.guard.session);ne_put32(p.data()+60,1);}
    n::Wire reply;if(ne_encode(&h,p.data(),reply.data()))throw std::runtime_error("ACK encode");rx.push_back(reply);
    if(m.h.opcode==NE_MAP_COMMIT && status==0) {
      h.opcode=NE_STATE;h.payload_bytes=160;h.session_id=exec.guard.session;h.transport_seq=++event_seq;h.request_id=0;
      h.map_epoch=1;h.map_revision=exec.active.revision;h.localization_epoch=exec.task.localization_epoch;p={};
      p[0]=exec.state;ne_put32(p.data()+64,exec.active.crc32^(bad_state_crc?1u:0u));
      if(ne_encode(&h,p.data(),reply.data()))throw std::runtime_error("STATE encode");
      rx.push_back(reply);
    }
  }
  bool receive(n::Wire& w,unsigned ms) override {
    if(rx.empty()){std::this_thread::sleep_for(std::chrono::milliseconds(ms));return false;}
    w=rx.front();rx.pop_front();return true;
  }
};
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  std::filesystem::create_directories(argv[1]);std::ofstream log(std::filesystem::path(argv[1])/"cases.jsonl");
  unsigned total=0,failed=0;
  const auto check=[&](const char* id,bool pass){++total;if(!pass){++failed;std::cerr<<"FAIL "<<id<<'\n';}
    log<<"{\"case\":\""<<id<<"\",\"pass\":"<<(pass?"true":"false")<<"}\n";};
  try {
    check("crc32_known",n::crc32_ieee(reinterpret_cast<const std::uint8_t*>("123456789"),9)==0xcbf43926);
    Device d;n::Client c(d,5);c.handshake(123,1,"map");n::Bitmap b{};
    for(unsigned i=0;i<b.size();++i)b[i]=std::uint8_t(i*17);
    auto receipt=c.upload(b,{-3200,-3200,1});
    check("complete_sequence",d.begins==1 && d.chunks==16 && d.commits==1);
    check("atomic_commit_and_crc",receipt.committed && receipt.state_verified && d.exec.active.crc32==receipt.crc32 && !std::memcmp(b.data(),d.server.planner.occupied,512));
    check("actual_guard_no_enable",d.exec.owner==0 && !d.exec.resume && d.exec.guard.live==0 && !d.exec.intent.linear_mmps && !d.exec.intent.angular_mradps);
    const auto sent=c.statistics().sent;c.upload(b,{-3200,-3200,1});
    check("unchanged_map_no_upload",c.statistics().sent==sent);
    b[17]^=1;receipt=c.upload(b,{-3200,-3200,1});
    check("changed_map_new_revision",receipt.revision==2 && receipt.committed && receipt.state_verified);
    check("old_active_immutable_during_upload",d.atomic);
    d.drop_chunk_once=true;b[18]^=1;receipt=c.upload(b,{-3200,-3200,1});
    check("chunk_ack_loss_idempotent",receipt.committed && receipt.state_verified && d.chunks==49);
    const auto active=d.exec.active.crc32;d.bad_commit=true;b[19]^=1;bool rejected=false;
    try{c.upload(b,{-3200,-3200,1});}catch(const n::Rejected& e){rejected=e.status==n::Status::MapCrc;}
    check("bad_commit_crc_rejected",rejected && d.exec.active.crc32==active);
    check("failed_commit_not_claimed",c.active_map().revision==3);
    d.bad_commit=false;d.exec.state=NV_FOLLOWING;const auto nc=d.chunks;rejected=false;
    try{c.upload(b,{-3200,-3200,1});}catch(const n::Rejected& e){rejected=e.status==n::Status::Busy;}
    check("busy_no_chunks_no_reset",rejected && d.chunks==nc && d.exec.active.crc32==active && c.session()==123);
    Device corrupt;n::Client other(corrupt,5);other.handshake(456,1,"map");corrupt.bad_state_crc=true;
    receipt=other.upload(b,{-3200,-3200,1});
    check("state_crc_mismatch_not_verified",receipt.committed && !receipt.state_verified);
    check("no_start_no_permit",std::find(d.ops.begin(),d.ops.end(),unsigned(NE_START))==d.ops.end());
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';check("exception",false);}
  std::ofstream summary(std::filesystem::path(argv[1])/"summary.json");
  summary<<"{\"stage\":2,\"cases\":"<<total<<",\"failed\":"<<failed<<",\"pass\":"<<(failed?"false":"true")<<",\"oracle\":\"candidate26 nav_exec_handle + real map CRC32\"}\n";
  std::cout<<"stage2 cases="<<total<<" failed="<<failed<<'\n';return failed?1:0;
}
