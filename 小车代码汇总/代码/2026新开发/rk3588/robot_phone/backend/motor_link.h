#ifndef ROBOT_PHONE_MOTOR_LINK_H
#define ROBOT_PHONE_MOTOR_LINK_H
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
namespace robotphone {
uint16_t motorCrc16(const uint8_t *,size_t);
std::vector<uint8_t> motorFrame(const uint8_t source[6],uint32_t sequence,bool armed,int left,int right,unsigned left_mask,unsigned invert_mask);
struct MotorAck {uint32_t sequence=0;bool armed=false,watchdog=false,configured=false;int left=0,right=0;unsigned left_mask=0,invert_mask=0;};
bool decodeMotorAck(const uint8_t *,size_t,const uint8_t host_mac[6],MotorAck &);
// Pure acknowledgement window, shared by the raw socket link and its tests.
// Only an actually transmitted, recent command can establish liveness. Replayed
// acknowledgements never extend that liveness or change the current status.
class MotorAckWindow {
public:
 void noteSent(uint32_t sequence,uint64_t now,unsigned left_mask,unsigned invert_mask);
 bool accept(const MotorAck &,uint64_t now);
 bool fresh(uint64_t now)const;
 const MotorAck &ack()const{return ack_;}
 void clear();
private:
 struct Sent {uint32_t sequence;uint64_t time;unsigned left_mask,invert_mask;};
 std::deque<Sent> sent_;
 MotorAck ack_;uint64_t last_ack_=0,last_command_=0;bool have_ack_=false;
};
class MotorLink {
public:
 ~MotorLink();
 bool open(const std::string &interface,std::string &error);
 bool send(bool armed,double left,double right,unsigned left_mask,unsigned invert_mask,uint64_t now);
 void receive(uint64_t now);
 bool fresh(uint64_t now)const;
 const MotorAck &ack()const{return window_.ack();}
 void close();
private:
 int fd_=-1,index_=0;uint8_t mac_[6]={};uint32_t sequence_=0;
 MotorAckWindow window_;
};
}
#endif
