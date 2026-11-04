#pragma once
#include "nav_authority.h"
#include "../common/ros_bridge_codec.h"
#include <functional>
#include <optional>

namespace robotnav {
// A thin dashboard adapter. It never computes a route, speed or steering angle.
class FpgaDelegation {
public:
 using Send=std::function<bool(const acg138::nav3::Wire&)>;
 explicit FpgaDelegation(Send);
 bool context(const arb1::Message&,uint64_t now_ns);
 bool prepare(const std::string& phone_owner,uint64_t generation,bool idle);
 bool claim(uint64_t plan_command,uint64_t generation,uint64_t now_ms);
 void phone(const std::string&,uint64_t seq,uint64_t source_ms,bool armed,uint64_t now_ms);
 void tick(uint64_t now_ms);
 bool ack(const acg138::nav3::Wire&,uint64_t now_ms);
 bool started(uint64_t generation,uint64_t permit_sequence);
 void takeover();
 bool quiet()const{return quiet_;}
 bool healthy(uint64_t now_ms)const;
 bool enabled()const{return authority_.enabled();}
 uint64_t generation()const{return generation_;}
 uint64_t plan_command()const{return plan_?plan_->command_id:0;}
 std::optional<acg138::nav3::StartTicket> ticket(uint64_t now_ms)const{return authority_.ticket(now_ms);}
 const std::string& reason()const{return !authority_.enabled()&&reason_=="waiting_real_NVA_ACK"?authority_.stop_reason():reason_;}
private:
 Send send_;NavAuthority authority_;
 std::optional<arb1::PlanResult> plan_;
 std::optional<arb1::Identity> authorized_identity_;
 std::optional<arb1::FpgaState> state_;
 std::optional<DeviceClock> clock_;
 uint64_t state_sample_ns_=0,plan_received_ns_=0,generation_=0;
 std::string owner_,reason_="not_prepared";bool quiet_=false;
};
}
