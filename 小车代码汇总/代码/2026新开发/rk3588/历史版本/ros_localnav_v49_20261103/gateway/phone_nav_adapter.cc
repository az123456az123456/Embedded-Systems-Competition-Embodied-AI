#include "phone_nav_adapter.h"
#include "../vendor/json.h"
#include <charconv>
#include <limits>
namespace robotnav {
using Json=nlohmann::json;
namespace {
uint64_t number(const Json&j,const char*name){
 const auto&i=j.at(name);if(!i.is_string())throw std::runtime_error("64bit identifiers must be decimal strings");
 const auto s=i.get<std::string>();uint64_t n=0;auto r=std::from_chars(s.data(),s.data()+s.size(),n);
 if(s.empty()||r.ec!=std::errc{}||r.ptr!=s.data()+s.size())throw std::runtime_error("invalid identifier");
 return n;
}
std::string hex(const std::array<uint8_t,16>&id){const char*s="0123456789abcdef";std::string result;for(auto b:id){result+=s[b>>4];result+=s[b&15];}return result;}
}
PhoneNavAdapter::PhoneNavAdapter():delegation_([this](const acg138::nav3::Wire&w){return socket_.send(w);}){}
bool PhoneNavAdapter::open(const std::string& name){active_=socket_.open(name,error_);return active_;}
void PhoneNavAdapter::owner(const std::string&o,uint64_t seq,uint64_t source,bool armed,uint64_t now){delegation_.phone(o,seq,source,armed,now);}
void PhoneNavAdapter::tick(uint64_t now){if(!active_)return;acg138::nav3::Wire w;for(unsigned i=0;i<32&&socket_.receive(w);++i)delegation_.ack(w,now);delegation_.tick(now);}
void PhoneNavAdapter::takeover(){delegation_.takeover();}
bool PhoneNavAdapter::request(const std::string&method,const std::string&path,const std::string&body,
 const std::string&owner,bool idle,uint64_t now,int&code,std::string&out){
 if(path.compare(0,21,"/api/navigation/fpga/")!=0)return false;
 code=409;Json response={{"ok",false},{"active",active_},{"quiet",quiet()},{"reason",error_.empty()?delegation_.reason():error_}};
 try{
  if(!active_)throw std::runtime_error("FPGA adapter not enabled");
  const auto op=path.substr(21);
  if(method=="POST"&&op=="context"){
   arb1::Message m;if(body.size()>16384||!arb1::decode(reinterpret_cast<const uint8_t*>(body.data()),body.size(),m)||!delegation_.context(m,now*1000000))throw std::runtime_error("invalid or stale FPGA context");
  }else if(method=="POST"&&(op=="prepare"||op=="claim"||op=="started"||op=="release")){
   auto j=Json::parse(body);if(owner.empty()||j.at("session").get<std::string>()!=owner)throw std::runtime_error("not current phone owner");
   const auto produced=number(j,"produced_mono_ns"),ttl=number(j,"valid_for_ms");
   if(!ttl||ttl>500||produced>now*1000000||now*1000000-produced>=ttl*1000000)throw std::runtime_error("expired dashboard task");
   const auto generation=number(j,"generation");bool ok=false;
   if(op=="prepare")ok=number(j,"expected_generation")==delegation_.generation()&&delegation_.prepare(owner,generation,idle);
   if(op=="claim")ok=delegation_.claim(number(j,"command_id"),generation,now);
   if(op=="started")ok=delegation_.ticket(now).has_value()&&delegation_.started(generation,number(j,"permit_seq"));
   if(op=="release"){
    // A stale ROS stop cannot interrupt a newer manual owner or generation.
    ok=delegation_.quiet()&&generation==delegation_.generation();
    if(ok)delegation_.takeover();
   }
   if(!ok)throw std::runtime_error(delegation_.reason());
  }else if(method!="GET"||op!="status"){code=404;throw std::runtime_error("unknown FPGA adapter endpoint");}
  code=200;response["ok"]=true;response["quiet"]=quiet();response["healthy"]=ready(now);
  response["enabled"]=delegation_.enabled();response["generation"]=std::to_string(delegation_.generation());response["reason"]=delegation_.reason();
  response["plan_command_id"]=std::to_string(delegation_.plan_command());
  if(auto t=delegation_.ticket(now)){
   response["ticket"]={{"session_id",std::to_string(t->identity.session_id)},{"run_generation",std::to_string(t->identity.run_generation)},
    {"goal_id",hex(t->identity.goal_id)},{"map_epoch",std::to_string(t->identity.map_epoch)},{"map_revision",std::to_string(t->identity.map_revision)},
    {"localization_epoch",std::to_string(t->identity.localization_epoch)},{"plan_id",std::to_string(t->identity.plan_id)},
    {"nav_epoch",std::to_string(t->nav_epoch)},{"permit_seq",std::to_string(t->permit_seq)},{"auth_epoch",std::to_string(t->auth_epoch)},
    {"expires_mono_ns",std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(t->expires_at.time_since_epoch()).count())}};
  }
 }catch(const std::exception&e){response["error"]=e.what();}
 out=response.dump();return true;
}
}
