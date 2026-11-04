#pragma once
#include "../acg138_fpga_map_bridge/include/acg138_fpga_map_bridge/nav3_wire.hpp"
extern "C" {
#include "wire/nav_exec_wire.h"
}
#include <optional>
#include <string>
namespace robotnav {
struct DeviceClock {
 uint64_t session=0,anchor=0,host_send_ms=0,host_receive_ms=0;
 uint32_t device_ms=0;
};
// Called ONLY by the dashboard/Gateway authority owner; ROS never constructs NVA.
// Millisecond arguments are original CLOCK_MONOTONIC values on the same RK.
class NavAuthority {
 using Plan=acg138::nav3::PlanResult;
 using Wire=acg138::nav3::Wire;
 Plan plan_{};DeviceClock clock_{};
 std::string phone_owner_;
 std::string stop_reason_="NVA_not_claimed";
 uint64_t phone_sequence_=0,phone_source_ms_=0,auth_epoch_=0,sequence_=0;
 uint64_t generation_=0,last_send_ms_=0,first_send_ms_=0,last_auth_phone_=0;
 uint32_t auth_deadline_=0;
 bool armed_=false,enabled_=false,permit_started_=false,navigation_started_=false;
 // Bounded replan reconciliation: no permit/START while awaiting fresh STATE.
 uint64_t replan_wait_until_ms_=0;
 std::optional<ne_header> pending_;
 std::optional<acg138::nav3::StartTicket> ticket_;
public:
 void phone(const std::string& owner,uint64_t sequence,uint64_t original_ms,bool armed,uint64_t now_ms);
 bool begin(const Plan&,const DeviceClock&,uint64_t generation,uint64_t now_ms);
 bool clock(const DeviceClock&);
 bool state(const acg138::nav3::StateView&,uint64_t now_ms);
 std::optional<Wire> permit(uint64_t now_ms);
 bool accept_ack(const Wire&,uint64_t now_ms);
 std::optional<acg138::nav3::StartTicket> ticket(uint64_t now_ms)const;
 bool started(std::uint64_t generation,std::uint64_t permit_sequence);
 void stop(const std::string& reason="NVA_stopped");
 const std::string& stop_reason()const{return stop_reason_;}
 bool enabled()const{return enabled_;}
 uint64_t generation()const{return generation_;}
};
}
