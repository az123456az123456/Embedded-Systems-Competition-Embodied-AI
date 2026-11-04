#include "ros_bridge_session.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <filesystem>
using namespace arb1;
unsigned checks=0;
void test(bool x){assert(x);++checks;}
std::vector<uint8_t> wire(const Message&m){std::vector<uint8_t>b;DecodeError e;test(encode(m,b,&e));return b;}
Message msg(const Payload&p,uint16_t type=ARB1_MSG_MISSION){Message m;m.header.type=type;m.header.boot_id[0]=1;m.header.bridge_epoch=42;m.header.message_sequence=2;m.blocks.push_back({Metadata{},p});return m;}
int main(int argc,char**argv){
 std::filesystem::path dest=argc>1?argv[1]:"/tmp/arb1-nav3";std::filesystem::create_directories(dest);
 Mission t;t.command_id=UINT64_MAX;t.gateway_generation=(1ull<<60)+1;t.operation=ARB1_MAPPING_START;t.valid_for_ms=500;t.produced_mono_ns=1000000000;
 t.identity.session=(1ull<<61)+3;t.identity.frame_token=1;
 for(unsigned op=1;op<=5;++op){t.operation=op;auto b=wire(msg(t));Message d;test(decode(b.data(),b.size(),d));auto a=std::get<Mission>(d.blocks[0].payload);test(a.command_id==t.command_id&&a.gateway_generation==t.gateway_generation&&a.identity.session==t.identity.session&&a.operation==op);test(executable(a,t.produced_mono_ns+499999999));test(!executable(a,t.produced_mono_ns+500000000));test(!executable(a,t.produced_mono_ns-1));
  for(unsigned cut=0;cut<b.size();++cut){Message unchanged;unchanged.header.bridge_epoch=9;test(!decode(b.data(),cut,unchanged));test(unchanged.header.bridge_epoch==9);}
  for(unsigned off:{76u,168u,175u}){auto broken=b;broken[112+off]=1;test(!decode(broken.data(),broken.size(),d));std::ofstream out(dest/("reserved-"+std::to_string(op)+"-"+std::to_string(off)+".bin"),std::ios::binary);out.write((char*)broken.data(),broken.size());}
 }
 for(unsigned ttl:{0u,499u,500u}){t.valid_for_ms=ttl;auto b=wire(msg(t));Message d;test(decode(b.data(),b.size(),d));test(executable(t,t.produced_mono_ns)==(ttl!=0));}
 t.valid_for_ms=501;std::vector<uint8_t>b;test(!encode(msg(t),b));t.valid_for_ms=500;
 t.flags=1;test(!executable(t,t.produced_mono_ns));test(!wire(msg(t)).empty());t.flags=2;test(!encode(msg(t),b));t.flags=0;
 t.operation=ARB1_MAPPING_START;t.goal_x_mm=1;test(!encode(msg(t),b));t.goal_x_mm=0;
 t.operation=ARB1_MAPPING_START|0x8000;t.status=4;test(!wire(msg(t)).empty());test(!executable(t,t.produced_mono_ns));t.operation=ARB1_MAPPING_START;t.status=0;
 PlanResult plan;plan.identity=t.identity;plan.cells.resize(4096);for(unsigned i=0;i<4096;++i)plan.cells[i]=i;
 b=wire(msg(plan,ARB1_MSG_SNAPSHOT));Message decoded;test(decode(b.data(),b.size(),decoded));test(std::get<PlanResult>(decoded.blocks[0].payload).cells==plan.cells);
 plan.cells.push_back(0);test(!encode(msg(plan,ARB1_MSG_SNAPSHOT),b));plan.cells.resize(4096);plan.cells[1]=4096;test(!encode(msg(plan,ARB1_MSG_SNAPSHOT),b));
 FpgaState state;state.frame_id="map";state.count=4096;state.cursor=4096;state.identity=t.identity;state.x_mm=-123;
 b=wire(msg(state,ARB1_MSG_SNAPSHOT));test(decode(b.data(),b.size(),decoded));test(std::get<FpgaState>(decoded.blocks[0].payload).x_mm==-123);
 state.cursor=4097;test(!encode(msg(state,ARB1_MSG_SNAPSHOT),b));state.cursor=0;state.state=6;test(!encode(msg(state,ARB1_MSG_SNAPSHOT),b));
 ClockAnchor clock;clock.session=t.identity.session;clock.host_send_ns=123;clock.host_receive_ns=456;clock.flags=1;clock.anchor_id=7;
 b=wire(msg(clock,ARB1_MSG_SNAPSHOT));test(decode(b.data(),b.size(),decoded));clock.host_send_ns=500;test(!encode(msg(clock,ARB1_MSG_SNAPSHOT),b));
 SessionConfig cfg;cfg.local_endpoint=LocalEndpoint::Bridge;cfg.channel=Channel::Control;cfg.expected_peer_role=PeerRole::Gateway;cfg.boot_id[0]=1;cfg.bridge_epoch=42;
 cfg.capabilities={{ARB1_KIND_HELLO,1},{ARB1_KIND_ERROR,1},{ARB1_KIND_MISSION,1}};cfg.allowed_message_types={ARB1_MSG_MISSION};cfg.required_capabilities=cfg.capabilities;
 unsigned admissions=0;ProtocolSession session(cfg,[&](const AcceptedMessage&){++admissions;});b=wire(msg(t));test(!session.receive(b.data(),b.size()));test(admissions==0&&session.counters().queue_admissions==0&&session.counters().claim_admissions==0);
 ProtocolSession good(cfg,[&](const AcceptedMessage&){++admissions;});Hello h;h.boot_id=cfg.boot_id;h.bridge_epoch=42;h.capabilities=cfg.capabilities;auto hello=msg(h,ARB1_MSG_HELLO);hello.header.message_sequence=1;b=wire(hello);test(good.receive(b.data(),b.size()));test(good.ready());b=wire(msg(t));test(good.receive(b.data(),b.size()));test(admissions==1);
 auto unsupported=msg(t);unsupported.header.minor=1;test(!encode(unsupported,b));
 std::ofstream(dest/"summary.json")<<"{\"checks\":"<<checks<<",\"failed\":0,\"same_frame_integer_identity\":true}\n";
 std::cout<<"checks="<<checks<<" failed=0\n";
}
