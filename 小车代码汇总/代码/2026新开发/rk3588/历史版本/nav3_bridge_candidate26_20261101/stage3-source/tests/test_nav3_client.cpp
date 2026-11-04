#include "acg138_fpga_map_bridge/nav3_wire.hpp"
extern "C" {
#include "reference/nav3/candidate26/softcore/firmware/nav_exec_protocol.h"
}
#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
namespace n=acg138::nav3;
// Only link transport and monotonic device tick are simulated. The admission
// functions are the frozen candidate26 C implementation, compiled unmodified.
struct Oracle : n::Channel {
  nx_guard guard{};
  std::deque<n::Wire> rx;
  std::vector<ne_header> requests;
  bool ready=true,quiet=true,drop_bind=false,drop_all=false,inject=false;
  unsigned bind_count=0;std::uint32_t now=100;
  Oracle(){nx_guard_init(&guard);}
  void send(const n::Wire& w) override {
    ne_message m{};if(ne_decode(w.data(),w.size(),&m))throw std::runtime_error("reference decode failed");
    requests.push_back(m.h);unsigned status=0;++now;
    switch(m.h.opcode) {
      case NE_INFO:nx_info_cookie(&guard,m.h.request_id,now,10000);break;
      case NE_SESSION_BIND:++bind_count;status=nx_bind_session(&guard,&m,now,quiet);break;
      case NE_FRAME_BIND:{auto g=guard;status=nx_bind_frame(&g,&m);if(!status)status=nx_accept_transport(&g,&m.h);if(!status)guard=g;break;}
      case NE_CLOCK_ANCHOR:{auto g=guard;status=nx_bind_anchor(&g,&m,now);if(!status)status=nx_accept_transport(&g,&m.h);if(!status)guard=g;break;}
      default:throw std::runtime_error("unexpected stage1 opcode");
    }
    if(drop_all || (drop_bind && m.h.opcode==NE_SESSION_BIND))return;
    ne_header h=m.h;h.opcode|=0x80;h.payload_bytes=m.h.opcode==NE_INFO?64:32;
    n::Payload p{};ne_put16(p.data(),std::uint16_t(status));ne_put32(p.data()+4,now);
    ne_put64(p.data()+8,1);ne_put64(p.data()+16,2);
    if(m.h.opcode==NE_INFO) {
      ne_put32(p.data()+32,63);ne_put32(p.data()+36,50000000);
      ne_put64(p.data()+40,guard.info_cookie);ne_put64(p.data()+48,guard.session);
      ne_put32(p.data()+56,0x12345678);ne_put32(p.data()+60,ready?1:0);
    }
    n::Wire reply;if(ne_encode(&h,p.data(),reply.data()))throw std::runtime_error("reference reply failed");
    if(inject) {
      auto wrong=reply;ne_put64(wrong.data()+80,h.request_id+1);ne_put16(wrong.data()+256,ne_crc16(wrong.data(),256));rx.push_back(wrong);
      wrong=reply;wrong[257]^=1;rx.push_back(wrong);
      auto event=h;event.opcode=NE_STATE;event.payload_bytes=160;event.session_id=guard.session?guard.session:777;
      event.transport_seq=99;event.request_id=0;n::Payload ep{};
      if(ne_encode(&event,ep.data(),wrong.data()))throw std::runtime_error("reference state failed");
      rx.push_back(wrong);
    }
    rx.push_back(reply);
  }
  bool receive(n::Wire& w,unsigned wait) override {
    if(rx.empty()){std::this_thread::sleep_for(std::chrono::milliseconds(wait));return false;}
    w=rx.front();rx.pop_front();return true;
  }
};
int main(int argc,char** argv){
  if(argc!=2)return 2;
  std::filesystem::create_directories(argv[1]);
  std::ofstream f(std::filesystem::path(argv[1])/"cases.jsonl");unsigned count=0,failed=0;
  auto check=[&](const char* name,bool ok){++count;if(!ok){++failed;std::cerr<<"FAIL "<<name<<'\n';}f<<"{\"case\":\""<<name<<"\",\"pass\":"<<(ok?"true":"false")<<"}\n";};
  try {
    Oracle o;n::Client c(o,10);c.handshake(0xabcdef0123456789ULL,7,"map");
    check("handshake_actual_guard",o.guard.session==c.session() && o.guard.frame_token==7 && o.guard.anchors[0].id==1);
    std::vector<unsigned> ops;for(auto h:o.requests)ops.push_back(h.opcode);
    check("required_sequence",ops==std::vector<unsigned>({NE_INFO,NE_SESSION_BIND,NE_FRAME_BIND,NE_INFO,NE_CLOCK_ANCHOR}));
    check("zero_motion_permissions",o.guard.live==0 && o.guard.nav_epoch==0 && !o.guard.permit_deadline && !o.guard.auth_deadline);
    check("singlemap_epochs",std::all_of(o.requests.begin(),o.requests.end(),[](const ne_header& h){return h.map_epoch==1 && h.localization_epoch==1;}));
    check("frame_and_anchor_seq_monotonic",o.requests[2].transport_seq<o.requests[4].transport_seq);
    check("session_bind_header_zero",o.requests[1].session_id==0);
    check("new_session_headers",o.requests[2].session_id==c.session() && o.requests[4].session_id==c.session());
    check("info64_parsed",c.last_info().extra.cpu_hz==50000000 && c.last_info().extra.capabilities==63);
    check("no_transmitted_start",std::none_of(o.requests.begin(),o.requests.end(),[](const ne_header& h){return h.opcode==NE_START || h.family==NE_NVA1;}));
    Oracle lost;lost.drop_bind=true;n::Client recovery(lost,5);recovery.handshake(55,1,"map");
    check("lost_bind_ack_recovered",recovery.session()==55 && lost.guard.anchors[0].id==1);
    check("bind_not_replayed",lost.bind_count==1 && lost.requests.size()==6 && lost.requests[2].opcode==NE_INFO);
    Oracle noisy;noisy.inject=true;n::Client correlated(noisy,10);correlated.handshake(66,1,"map");
    check("wrong_request_async_ignored",correlated.statistics().ignored==10);
    check("bad_crc_ignored",correlated.statistics().malformed==5);
    Oracle unready;unready.ready=false;n::Client unavailable(unready,10);bool rejected=false;
    try{unavailable.handshake(77,1,"map");}catch(const std::runtime_error&){rejected=true;}
    check("ready0_no_bind",rejected && unready.bind_count==0 && unready.requests.size()==1);
    Oracle busy;busy.quiet=false;n::Client busy_client(busy,10);rejected=false;
    try{busy_client.handshake(88,1,"map");}catch(const n::Rejected& e){rejected=e.status==n::Status::Busy;}
    check("busy_stops_handshake",rejected && !busy_client.session() && busy.requests.size()==2);
    Oracle silent;silent.drop_all=true;n::Client timed(silent,2);rejected=false;
    try{timed.info();}catch(const n::Timeout&){rejected=true;}
    check("timeout_no_generic_retries",rejected && silent.requests.size()==1);
    const auto sent=o.requests.size();rejected=false;
    try{c.clock_anchor(1);}catch(const std::invalid_argument&){rejected=true;}
    check("anchor_id_no_reuse",rejected && o.requests.size()==sent);
    rejected=false;try{c.frame_bind(1,std::string(64,'x'));}catch(const std::invalid_argument&){rejected=true;}
    check("frame64_no_send",rejected && o.requests.size()==sent);
    c.info();rejected=false;try{c.session_bind(c.session());}catch(const std::invalid_argument&){rejected=true;}
    check("cannot_rebind_same_session",rejected);
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';check("exception",false);}
  std::ofstream s(std::filesystem::path(argv[1])/"summary.json");
  s<<"{\"stage\":1,\"offline_cases\":"<<count<<",\"failed\":"<<failed<<",\"pass\":"<<(failed?"false":"true")<<",\"oracle\":\"candidate26 nx_guard\",\"real_network_sent\":0}\n";
  std::cout<<"stage1 client cases="<<count<<" failed="<<failed<<'\n';return failed?1:0;
}
