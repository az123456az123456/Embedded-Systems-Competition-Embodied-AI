#include "acg138_fpga_map_bridge/map_core.hpp"
#include "acg138_fpga_map_bridge/nav2_wire.hpp"
extern "C" {
#include "reference/nav_protocol.h"
}
#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>
#include <iostream>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>

using namespace acg138;
using namespace acg138::wire;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x))throw std::runtime_error(std::string("check failed line ")+std::to_string(__LINE__)+": "+#x);}while(0)
template<typename F>static void rejects(F f){bool threw=false;try{f();}catch(const std::exception&){threw=true;}CHECK(threw);}
class FirmwareChannel final:public Channel {
public:
  nav_server server{};
  std::deque<Body> replies;
  unsigned sends=0,duplicates=0;
  bool drop_one_chunk_ack=false,reject_next_chunk=false,inject_other_request=false;
  FirmwareChannel(){nav_server_init(&server);}
  void send(const Body&q)override{
    ++sends;Body r{};CHECK(nav_process(&server,q.data(),r.data(),nullptr)==1);Message request;CHECK(decode(q,request));
    if(request.op==Chunk&&drop_one_chunk_ack){drop_one_chunk_ack=false;++duplicates;return;}
    if(request.op==Chunk&&reject_next_chunk){reject_next_chunk=false;r[6]=NAV_BAD_REQUEST;}
    if(inject_other_request){Body other=r;other[11]^=0x40;replies.push_back(other);inject_other_request=false;}
    replies.push_back(r);
  }
  bool receive(Body&b,unsigned timeout_ms)override{
    if(replies.empty()){std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));return false;}
    b=replies.front();replies.pop_front();return true;
  }
};
static Grid free_grid(){Grid g;g.width=g.height=200;g.resolution=.1;g.origin={-10,-10,0};g.cells.assign(40000,0);return g;}
static void test_geometry(){
  CropConfig c;Grid g=free_grid();LocalGrid l=crop_and_inflate(g,{0,0,0},c);
  CHECK(l.origin_x==-3.2&&l.origin_y==-3.2);CHECK(l.free_cells>2500);CHECK(!l.is_blocked(l.cell(0,0)));
  for(unsigned x=0;x<64;++x){CHECK(l.is_blocked(x));CHECK(l.is_blocked(63*64+x));}
  for(unsigned y=0;y<64;++y){CHECK(l.is_blocked(y*64));CHECK(l.is_blocked(y*64+63));}
  for(unsigned i=0;i<kCells;++i){const Pose p=l.center(uint16_t(i));CHECK(l.cell(p.x,p.y)==i);}
  rejects([&]{l.cell(std::numeric_limits<double>::quiet_NaN(),0);});rejects([&]{l.cell(100,100);});
  // Unknown beneath the robot is not cleared merely to make A* succeed.
  g.cells[100*200+100]=-1;l=crop_and_inflate(g,{0,0,0},c);CHECK(l.is_blocked(l.cell(0,0)));
  // Fine-grid thin obstacle survives area-overlap downsampling and padding.
  g=free_grid();g.width=g.height=1600;g.resolution=.01;g.origin={-8,-8,0};g.cells.assign(2560000,0);
  g.cells[800*1600+809]=100;l=crop_and_inflate(g,{0,0,0},c);CHECK(l.is_blocked(l.cell(.05,.05)));CHECK(l.is_blocked(l.cell(.25,.05)));CHECK(!l.is_blocked(l.cell(1.05,.05)));
  // Source origin rotation is respected; all-free source stays all-free in center.
  g=free_grid();g.origin={0,-14.14213562373095,.7853981633974483};l=crop_and_inflate(g,{0,0,0},c);CHECK(!l.is_blocked(l.cell(0,0)));
  // Outside coverage and uncertain cells are blocked.
  g=free_grid();g.origin={0,0,0};l=crop_and_inflate(g,{0,0,0},c);CHECK(l.is_blocked(l.cell(-.5,-.5)));
  g=free_grid();g.cells[100*200+100]=26;l=crop_and_inflate(g,{0,0,0},c);CHECK(l.is_blocked(l.cell(0,0)));
  g=free_grid();g.cells[0]=-2;rejects([&]{crop_and_inflate(g,{0,0,0},c);});
  g=free_grid();g.origin.yaw=std::numeric_limits<double>::infinity();rejects([&]{crop_and_inflate(g,{0,0,0},c);});
  g=free_grid();c.resolution=0;rejects([&]{crop_and_inflate(g,{0,0,0},c);});
  CHECK(json_escape("a\n\"\\")=="a\\u000a\\\"\\\\");
}
static void test_wire(){
  const uint8_t *text=reinterpret_cast<const uint8_t*>("123456789");CHECK(crc16(text,9)==0x29b1);CHECK(crc32(text,9)==0xcbf43926);CHECK(crc32(text,9)==nav_crc32(text,9));
  Message m;m.op=Chunk;m.count=32;m.request=0x12345678;m.generation=0x10203040;m.arg0=0x120;m.data[0]=1;
  const Body b=encode(m);CHECK(b[0]=='N'&&b[3]=='2'&&b[4]==1);CHECK(nav_get32(b.data()+8)==m.request);CHECK(nav_get32(b.data()+12)==m.generation);
  CHECK(b[32]==1&&b[63]==0);Message decoded;CHECK(decode(b,decoded));CHECK(decoded.arg0==m.arg0);
  Body corrupt=b;corrupt[31]=1;CHECK(!decode(corrupt,decoded));
  const Mac host{{0x02,1,2,3,4,5}};m.op|=0x80;auto frame=ethernet(host,m);
  // Turn a request envelope into a synthetic FPGA reply envelope.
  std::copy(host.begin(),host.end(),frame.begin());std::copy(kFpgaMac.begin(),kFpgaMac.end(),frame.begin()+6);
  CHECK(frame.size()==80);CHECK(decode_reply(frame.data(),frame.size(),host,decoded));
  auto damaged=frame;damaged[49]^=1;CHECK(!decode_reply(damaged.data(),damaged.size(),host,decoded));
  damaged=frame;damaged[6]^=1;CHECK(!decode_reply(damaged.data(),damaged.size(),host,decoded));
  damaged=frame;damaged[0]^=1;CHECK(!decode_reply(damaged.data(),damaged.size(),host,decoded));
  damaged=frame;damaged[13]=0xb6;CHECK(!decode_reply(damaged.data(),damaged.size(),host,decoded));
  CHECK(!decode_reply(frame.data(),79,host,decoded));
}
static void test_host_process_visibility(){
  rejects([]{legacy_navigation_running("/definitely-no-acg138-proc");});
  rejects([]{legacy_navigation_running("relative/proc");});
  char directory[]="/tmp/acg138-host-proc-XXXXXX";char*made=mkdtemp(directory);CHECK(made!=nullptr);
  const std::string root=made;
  rejects([&]{legacy_navigation_running(root);});
  CHECK(mkdir((root+"/self").c_str(),0700)==0);{std::ofstream own(root+"/self/comm");own<<"fpga_map_bridge\n";}
  rejects([&]{legacy_navigation_running(root);}); // self but no numeric PID is not procfs evidence.
  CHECK(mkdir((root+"/1").c_str(),0700)==0);{std::ofstream other(root+"/1/comm");other<<"other\n";}
  CHECK(!legacy_navigation_running(root));
  CHECK(unlink((root+"/1/comm").c_str())==0);rejects([&]{legacy_navigation_running(root);});
  {std::ofstream other(root+"/1/comm");other<<"other\n";}
  CHECK(mkdir((root+"/2").c_str(),0700)==0);
  for(const auto&name:{"robot-navigatio","robot-navigation","robot-navigation.aarch64"}){
    {std::ofstream legacy(root+"/2/comm");legacy<<name<<'\n';}CHECK(legacy_navigation_running(root));
  }
  CHECK(unlink((root+"/2/comm").c_str())==0);CHECK(mkdir((root+"/2/comm").c_str(),0700)==0);
  rejects([&]{legacy_navigation_running(root);}); // read() fails, even for a root test process.
  CHECK(rmdir((root+"/2/comm").c_str())==0);
  {std::ofstream empty(root+"/2/comm");}rejects([&]{legacy_navigation_running(root);});
  {std::ofstream oversized(root+"/2/comm");oversized<<std::string(200,'x');}rejects([&]{legacy_navigation_running(root);});
  CHECK(unlink((root+"/2/comm").c_str())==0);CHECK(rmdir((root+"/2").c_str())==0);
  CHECK(unlink((root+"/1/comm").c_str())==0);CHECK(rmdir((root+"/1").c_str())==0);
  CHECK(unlink((root+"/self/comm").c_str())==0);CHECK(rmdir((root+"/self").c_str())==0);CHECK(rmdir(root.c_str())==0);
  // Exercise the real Linux procfs without inspecting exe links. This test
  // also works as an unprivileged host user with CapEff=0; a live legacy
  // planner may legitimately be present, so either boolean result is valid.
  const bool active=legacy_navigation_running("/proc");++checks;
  std::ifstream status("/proc/self/status");std::string line,caps="unavailable";
  while(std::getline(status,line))if(line.rfind("CapEff:",0)==0)caps=line.substr(7);
  std::cout<<"host_proc_comm_scan=passed uid="<<getuid()<<" effective_caps="<<caps<<" legacy_active="<<active<<'\n';
}
static void test_transaction_and_astar(){
  Bitmap map{};for(unsigned y=0;y<64;++y)for(unsigned x=0;x<64;++x)if(x==0||y==0||x==63||y==63){const unsigned i=y*64+x;map[i>>3]|=uint8_t(1u<<(i&7));}
  FirmwareChannel link;Client client(link);rejects([&]{client.plan(map,65,4030);});
  link.drop_one_chunk_ack=true;link.inject_other_request=true;CHECK(client.upload(map)==1);
  CHECK(link.server.map_valid&&link.server.generation==1);CHECK(std::equal(map.begin(),map.end(),link.server.planner.occupied));
  CHECK(client.statistics().retries==1&&client.statistics().ignored==1);CHECK(link.duplicates==1);CHECK(link.sends==20);
  const auto path=client.plan(map,65,4030);CHECK(path.front()==65&&path.back()==4030);CHECK(path.size()==123);
  // Fresh client synchronizes IDs/generation using INFO; no hard-coded epoch.
  Client restarted(link);CHECK(restarted.upload(map)==2);CHECK(link.server.generation==2);
  CHECK(restarted.plan(map,65,65).size()==1);
  rejects([&]{restarted.plan(map,0,65);});
  // Failed staging must preserve the previously committed map and disable PLAN
  // on this failed client until a full acknowledged upload succeeds again.
  const uint32_t active=link.server.generation;link.reject_next_chunk=true;rejects([&]{restarted.upload(map);});
  CHECK(link.server.generation==active);CHECK(std::equal(map.begin(),map.end(),link.server.planner.occupied));
  rejects([&]{restarted.plan(map,65,4030);});CHECK(restarted.upload(map)==3);
  // Firmware rejects partial commit and bad bitmap CRC, preserving active map.
  Message q;q.op=Info;Body info{};Body reply{};info=encode(q);CHECK(nav_process(&link.server,info.data(),reply.data(),nullptr)==1);
  Message state;CHECK(decode(reply,state));uint32_t seq=state.arg1+1;const uint32_t gen=state.arg0+1;
  q={};q.op=Begin;q.request=seq++;q.generation=gen;q.arg0=0xdeadbeef;auto body=encode(q);CHECK(nav_process(&link.server,body.data(),reply.data(),nullptr)==1&&reply[6]==0);
  q={};q.op=Commit;q.request=seq++;q.generation=gen;body=encode(q);CHECK(nav_process(&link.server,body.data(),reply.data(),nullptr)==1&&reply[6]==NAV_INCOMPLETE);CHECK(link.server.generation==3);
  for(unsigned offset=0;offset<512;offset+=32){q={};q.op=Chunk;q.request=seq++;q.generation=gen;q.count=32;q.arg0=offset;body=encode(q);CHECK(nav_process(&link.server,body.data(),reply.data(),nullptr)==1&&reply[6]==0);}
  q={};q.op=Commit;q.request=seq++;q.generation=gen;body=encode(q);CHECK(nav_process(&link.server,body.data(),reply.data(),nullptr)==1&&reply[6]==NAV_MAP_CRC);CHECK(link.server.generation==3);
  std::cout<<"firmware_context_bytes="<<sizeof(nav_server)<<" map_bytes="<<map.size()<<"\n";
}
int main(){try{test_geometry();test_wire();test_host_process_visibility();test_transaction_and_astar();std::cout<<"PASS map_bridge checks="<<checks<<" real_firmware_oracle=yes movement_messages=0\n";return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
