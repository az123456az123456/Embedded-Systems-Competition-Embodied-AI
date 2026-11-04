#pragma once
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
struct MapDevice : n::Channel {
  nav_server server{};nav_exec exec{};std::deque<n::Wire> rx;
  std::vector<unsigned> ops;unsigned chunks=0,commits=0,begins=0;
  std::uint32_t now=100;std::uint64_t event_seq=0;
  bool drop_chunk_once=false,bad_commit=false,bad_state_crc=false,atomic=true;
  MapDevice(){nav_server_init(&server);nav_exec_init(&exec,&server);}
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
