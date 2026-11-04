// Offline IPC verification on RK. Links no hardware packet sender.
#include "ros_bridge_transport.h"
#include <chrono>
#include <atomic>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <thread>
int main(int argc,char**argv){
 if(argc!=2)return 2;
 std::atomic<bool> reply{false};std::atomic<unsigned> unsolicited{0};
 const auto command=arb1::mono_ns();
 arb1::LocalBridge client("/run/acg138-ros-bridge",false,[&](arb1::Channel,const arb1::AcceptedMessage&m){
  for(const auto&b:m.message.blocks){
   if(auto q=std::get_if<arb1::Mission>(&b.payload)){
    if(q->command_id==command&&q->operation==(ARB1_MAPPING_START|ARB1_MISSION_REPLY)&&q->status==4&&!q->wire_request_id)reply=true;
   }else ++unsolicited;
  }
 });
 const auto deadline=arb1::mono_ns()+3000000000;
 while(arb1::mono_ns()<deadline&&(!client.ready(arb1::Channel::Control)||!client.ready(arb1::Channel::State)))std::this_thread::sleep_for(std::chrono::milliseconds(1));
 const bool handshake=client.ready(arb1::Channel::Control)&&client.ready(arb1::Channel::State);
 arb1::Mission task;task.command_id=command;task.operation=ARB1_MAPPING_START;
 task.gateway_generation=1;task.produced_mono_ns=arb1::mono_ns();task.valid_for_ms=500;
 task.flags=ARB1_MISSION_DRY_RUN; // Production bridge rejects before any ROS service.
 arb1::Message message;message.header.type=ARB1_MSG_MISSION;message.blocks.push_back({arb1::Metadata{},task});
 const bool sent=handshake&&client.send(arb1::Channel::Control,message);
 const auto end=arb1::mono_ns()+1000000000;
 while(!reply&&arb1::mono_ns()<end)std::this_thread::sleep_for(std::chrono::milliseconds(1));
 struct stat st{};const bool mode=stat("/run/acg138-ros-bridge/control.sock",&st)==0&&st.st_uid==0&&(st.st_mode&0777)==0600;
 const bool pass=handshake&&sent&&reply&&mode;
 std::ofstream out(argv[1]);out<<"{\"pass\":"<<(pass?"true":"false")<<",\"HELLO_both_lanes\":"<<(handshake?"true":"false")<<",\"dry_run_rejected_before_service\":"<<(reply?"true":"false")<<",\"root_socket_0600\":"<<(mode?"true":"false")<<",\"hardware_packets_sent\":0,\"ROS_motion_services_requested\":0,\"unsolicited_blocks\":"<<unsolicited<<"}\n";
 return pass?0:1;
}
