#include "ros_bridge_transport.h"
#include <cassert>
#include <chrono>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <thread>
using namespace arb1;
std::atomic<unsigned> checks{0};
void check(bool value){assert(value);++checks;}
template<class F>void wait(F f,unsigned ms=1000){auto end=mono_ns()+uint64_t(ms)*1000000;while(!f()&&mono_ns()<end)std::this_thread::sleep_for(std::chrono::milliseconds(1));check(f());}
int main(int argc,char**argv){
 std::filesystem::path output=argc>1?argv[1]:"/tmp/arb1-transport-report";std::filesystem::create_directories(output);
 const auto directory=(output/"ipc").string();std::atomic<unsigned> calls{0},responses{0},maps{0};std::atomic<uint64_t> response_ns{0};
 std::unique_ptr<LocalBridge> server;
 server=std::make_unique<LocalBridge>(directory,true,[&](Channel channel,const AcceptedMessage&m){check(channel==Channel::Control);++calls;auto reply=m.message;auto& task=std::get<Mission>(reply.blocks[0].payload);task.operation|=0x8000;check(server->send(Channel::Control,reply));});
 LocalBridge client(directory,false,[&](Channel channel,const AcceptedMessage&m){if(channel==Channel::Control){++responses;response_ns=mono_ns();}else{check(m.message.header.type==ARB1_MSG_SNAPSHOT);++maps;}});
 wait([&]{return client.ready(Channel::Control)&&client.ready(Channel::State)&&server->ready(Channel::Control)&&server->ready(Channel::State);});
 struct stat before{},after{};check(stat((directory+"/bridge.lock").c_str(),&before)==0);
 bool rejected=false;try{LocalBridge second(directory,true,[](Channel,const AcceptedMessage&){});}catch(const std::exception&){rejected=true;}check(rejected);check(stat((directory+"/bridge.lock").c_str(),&after)==0&&before.st_ino==after.st_ino);
 Map map;map.width=1024;map.height=1024;map.resolution=.1;map.frame_id="map";map.cells.resize(1048576);map.prefer_rle=false;
 for(unsigned i=0;i<map.cells.size();++i)map.cells[i]=int8_t(i%101);
 Unknown extra;extra.kind=500;extra.schema=1;extra.bytes.resize(1048288,0x55);
 Message state;state.blocks={{Metadata{},map},{Metadata{},extra}};state.blocks[1].metadata.flags=0;
 std::vector<uint8_t> bytes;check(encode(state,bytes));check(bytes.size()==2097152);check(server->send(Channel::State,state));
 check(server->queued_bytes(Channel::State)<=2097152);
 Mission task;task.operation=ARB1_MAPPING_START;task.command_id=1;task.gateway_generation=7;task.valid_for_ms=500;task.produced_mono_ns=mono_ns();
 Message command;command.header.type=ARB1_MSG_MISSION;command.blocks.push_back({Metadata{},task});
 auto began=mono_ns();check(client.send(Channel::Control,command));wait([&]{return responses==1;});auto latency=response_ns-began;check(latency<=50000000);
 wait([&]{return maps==1;},2000);check(calls==1);
 struct stat socket{};check(lstat((directory+"/state.sock").c_str(),&socket)==0&&(socket.st_mode&0777)==0600&&socket.st_uid==0);
 // Hold the state receiver unread while a different lane performs its very
 // first HELLO. This tests handshake latency, not merely an existing session.
 const auto hd=(output/"hello-ipc").string();
 LocalBridge hello_server(hd,true,[](Channel,const AcceptedMessage&){});
 int fd=::socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);check(fd>=0);
 sockaddr_un address{};address.sun_family=AF_UNIX;std::string path=hd+"/state.sock";
 check(path.size()<sizeof(address.sun_path));std::copy(path.begin(),path.end(),address.sun_path);
 check(connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0);
 FrameStream stream;std::vector<std::vector<uint8_t>> frames;const auto deadline=mono_ns()+100000000;
 while(frames.empty()&&mono_ns()<deadline){uint8_t b[4096];auto n=recv(fd,b,sizeof(b),MSG_DONTWAIT);if(n>0)check(stream.feed(b,size_t(n),frames));else std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 check(frames.size()==1);Message hello;check(decode(frames[0].data(),frames[0].size(),hello));
 hello.header.message_sequence=1;hello.header.sent_mono_ns=mono_ns();check(encode(hello,bytes));
 check(send(fd,bytes.data(),bytes.size(),MSG_NOSIGNAL)==ssize_t(bytes.size()));
 wait([&]{return hello_server.ready(Channel::State);});check(hello_server.send(Channel::State,state));
 began=mono_ns();uint64_t hello_ns=0;
 {
  LocalBridge late(hd,false,[](Channel,const AcceptedMessage&){});
  wait([&]{return late.ready(Channel::Control)&&hello_server.ready(Channel::Control);},100);
  hello_ns=mono_ns()-began;check(hello_ns<=50000000);
  check(hello_server.queued_bytes(Channel::State)<=2097152);
 }
 close(fd);
 std::ofstream(output/"summary.json")<<"{\"checks\":"<<checks<<",\"failed\":0,\"control_round_trip_ns_during_2MiB_state\":"<<latency<<",\"control_HELLO_ns_during_unread_2MiB_state\":"<<hello_ns<<",\"lock_inode_stable\":true,\"root_test\":\"real SO_PEERCRED in user namespace; board root confirmation pending\",\"control_HELLO_during_state\":\"PASS\"}\n";
 std::cout<<"checks="<<checks<<" control_ns="<<latency<<" failed=0\n";
}
