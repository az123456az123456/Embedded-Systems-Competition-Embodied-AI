#ifndef ROBOT_PHONE_MOTOR_LINK_H
#define ROBOT_PHONE_MOTOR_LINK_H
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
namespace robotphone {
uint16_t motorCrc16(const uint8_t *,size_t);
std::vector<uint8_t> motorFrame(const uint8_t source[6],uint32_t sequence,bool armed,int left,int right,unsigned left_mask,unsigned invert_mask);
std::vector<uint8_t> angleMotorFrame(const uint8_t source[6],uint32_t sequence,bool armed,int angle_cdeg,int forward_permille,unsigned left_mask,unsigned invert_mask);
std::vector<uint8_t> arcMotorFrame(const uint8_t source[6],uint32_t sequence,bool armed,int outer_permille,int curve_q8,unsigned left_mask,unsigned invert_mask);
std::vector<uint8_t> velocityMotorFrame(const uint8_t source[6],uint32_t sequence,bool armed,int linear_mmps,int yaw_rate_mradps,unsigned left_mask,unsigned invert_mask);
struct MotorAck {uint32_t sequence=0;bool armed=false,watchdog=false,configured=false,softcore_ready=false;int left=0,right=0;unsigned left_mask=0,invert_mask=0,protocol=1;};
struct MotorDiagnostics {
 uint64_t tx_frames=0,tx_errors=0,rx_frames=0,rx_rejected=0,ack_frames=0,last_ack_ms=0,configuration_hints=0,configuration_seeds=0;
 uint32_t last_seed_sequence=0;
 int last_errno=0;
};
bool decodeMotorAck(const uint8_t *,size_t,const uint8_t host_mac[6],MotorAck &);
// Passive NAV1 status is an independent observation, never a motor ACK or a
// source of command authority. All fields use the RK monotonic clock for age.
struct SoftcoreStatus {
 uint8_t flags=0;
 uint16_t path_length=0,firmware_error=0;
 uint32_t heartbeat=0,astar_cycles=0,expanded=0,imu_sequence=0;
 uint64_t received_ms=0,last_advance_ms=0,advances=0,resets=0,frames=0,rejected=0;
 bool seen=false,socket_open=false;int last_errno=0;
};
bool decodeSoftcoreStatus(const uint8_t *,size_t,const uint8_t host_mac[6],SoftcoreStatus &);
class SoftcoreMonitor {
public:
 bool accept(const uint8_t *,size_t,const uint8_t host_mac[6],uint64_t now);
 bool heartbeatLive(uint64_t now)const;
 bool packetFresh(uint64_t now)const;
 std::string reason(uint64_t now,bool matching_ack4_ready,bool autonomous_requested=true)const;
 std::string json(uint64_t now,bool matching_ack4_ready,bool autonomous_requested=true)const;
 const SoftcoreStatus &status()const{return status_;}
 void socketState(bool open,int error=0){status_.socket_open=open;status_.last_errno=error;}
 void clear(){status_={};}
private:
 SoftcoreStatus status_;
};
// Pure acknowledgement window, shared by the raw socket link and its tests.
// Only an actually transmitted, recent command can establish liveness. Replayed
// acknowledgements never extend that liveness or change the current status.
class MotorAckWindow {
public:
 void noteSent(uint32_t sequence,uint64_t now,unsigned left_mask,unsigned invert_mask,unsigned protocol=1);
 bool accept(const MotorAck &,uint64_t now);
 bool fresh(uint64_t now)const;
 const MotorAck &ack()const{return ack_;}
 void clear();
private:
 struct Sent {uint32_t sequence;uint64_t time;unsigned left_mask,invert_mask,protocol;};
 std::deque<Sent> sent_;
 MotorAck ack_;uint64_t last_ack_=0,last_command_=0;bool have_ack_=false;
};
class MotorLink {
public:
 ~MotorLink();
 bool open(const std::string &interface,std::string &error);
 bool send(bool armed,double left,double right,unsigned left_mask,unsigned invert_mask,uint64_t now);
 bool sendAngle(bool armed,double angle_deg,double forward,unsigned left_mask,unsigned invert_mask,uint64_t now);
 bool angleSupported()const{return angle_supported_;}
 bool sendArc(bool armed,int outer_permille,int curve_q8,unsigned left_mask,unsigned invert_mask,uint64_t now);
 bool arcSupported()const{return arc_supported_;}
 bool sendVelocity(bool armed,double linear_mps,double yaw_rate_rps,unsigned left_mask,unsigned invert_mask,uint64_t now);
 bool velocitySupported()const{return velocity_supported_;}
 bool softcoreReady(uint64_t now)const;
 const SoftcoreMonitor &softcore()const{return softcore_;}
 std::string softcoreReason(uint64_t now,bool autonomous_requested=true)const{return softcore_.reason(now,softcoreReady(now),autonomous_requested);}
 void receive(uint64_t now);
 bool fresh(uint64_t now)const;
 bool configurationNeeded()const{return configuration_needed_;}
 const MotorAck &ack()const{return window_.ack();}
 const MotorDiagnostics &diagnostics()const{return diagnostics_;}
 void close();
private:
 int fd_=-1,nav_fd_=-1,index_=0;uint8_t mac_[6]={};uint32_t sequence_=0;
 MotorAckWindow window_;
 MotorDiagnostics diagnostics_;
 SoftcoreMonitor softcore_;
 bool angle_supported_=false,arc_supported_=false,velocity_supported_=false,configuration_needed_=true,configuration_verified_=false,sequence_seeded_=false;
 uint64_t softcore_ack_ms_=0;bool softcore_ready_=false;
 bool transmit(const std::vector<uint8_t>&,uint32_t,unsigned,unsigned,unsigned,uint64_t);
};
}
#endif
