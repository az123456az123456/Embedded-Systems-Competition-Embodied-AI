from pathlib import Path
r=Path(__file__).resolve().parents[1]
def edit(name,old,new):
 p=r/name;s=p.read_text();assert old in s,(name,old[:80]);p.write_text(s.replace(old,new))
edit('rk_gateway/navigation/gateway.h','std::string operation,phone_session,map_session;','std::string operation,phone_session,map_session,semantic_target;')
edit('rk_gateway/navigation/gateway.h','struct GatewayCallbacks {','struct GatewayCallbacks {\n std::function<bool(std::string&)> semantic_search_ready;')
edit('rk_gateway/navigation/gateway.cc','#include "gateway.h"','#include "gateway.h"\n#include "semantic_target.h"')
edit('rk_gateway/navigation/gateway.cc','const bool begin=op=="start"||op=="mapping-start"||op=="explore";',
 '''const bool semantic=op=="semantic-start";
    const bool mapping_start=op=="mapping-start"||semantic;
    std::string text,target;
    if(semantic&&(!jsonString(input,"text",text)||(target=semantic_target(text)).empty()))return errorReply(400,"unsupported_semantic_command");
    const bool begin=op=="start"||mapping_start||op=="explore";''')
edit('rk_gateway/navigation/gateway.cc','if(op=="mapping-start"&&(!callbacks.exploration_mapping_ready ||','if(mapping_start&&(!callbacks.exploration_mapping_ready ||')
edit('rk_gateway/navigation/gateway.cc','if(begin&&op=="mapping-start"){',
 '''if(semantic&&(!callbacks.semantic_search_ready||!callbacks.semantic_search_ready(error)))return errorReply(409,error.empty()?"semantic_service_not_ready":error);
    if(begin&&mapping_start){''')
edit('rk_gateway/navigation/gateway.cc','if(begin&&op!="mapping-start"&&','if(begin&&!mapping_start&&')
edit('rk_gateway/navigation/gateway.cc','intent.x_m=x;intent.y_m=y;intent.gateway_generation=',
 'intent.semantic_target=target;intent.x_m=x;intent.y_m=y;intent.gateway_generation=')
edit('rk_gateway/navigation/gateway.cc','exploration_manual=op=="mapping-start";','exploration_manual=mapping_start;')
edit('gateway/runtime.cc','Json map_metadata_=Json::object(),mapping_=Json::object(),exploration_=',
 'Json semantic_=Json::object();bool semantic_active_=false;std::string semantic_task_;\n Json map_metadata_=Json::object(),mapping_=Json::object(),exploration_=')
edit('gateway/runtime.cc','else if(kind=="mapping")mapping_=j.at("value");','else if(kind=="semantic")semantic_=j.at("value");\n     else if(kind=="mapping")mapping_=j.at("value");')
edit('gateway/runtime.cc','if(i.operation=="mapping-start"){',
 '''if(i.operation=="mapping-start"||i.operation=="semantic-start"){
   semantic_active_=i.operation=="semantic-start";
   if(semantic_active_){
    semantic_task_="semantic-"+std::to_string(public_generation_)+"-"+std::to_string(mono_ns());
    Json request={{"task",semantic_task_},{"target",i.semantic_target},{"gateway_generation",std::to_string(public_generation_)},
                  {"produced_mono_ns",std::to_string(mono_ns())},{"valid_for_ms",500}};
    auto reply=dashboard_request(8088,token_,"POST","/task",request.dump(),500);
    ensure_current();
    if(!reply.transport_ok||reply.status!=200||!Json::parse(reply.body).value("ok",false))throw std::runtime_error("semantic_service_start_rejected");
   }''')
edit('gateway/runtime.cc',' void advance_exploration(){',
 ''' void advance_exploration(){
  if(semantic_active_){
   Json s;{std::lock_guard<std::mutex>lock(mutex_);s=semantic_;}
   const auto now=mono_ns(),stamp=s.value("published_mono_ns",uint64_t(0));
   const bool matching=s.value("task",std::string())==semantic_task_&&s.value("gateway_generation",std::string())==std::to_string(public_generation_);
   if(!matching||!stamp||stamp>now||now-stamp>350000000){
    if(ms()-mapping_started_ms_>500)gateway_->cancelNavigation("semantic_worker_stale",public_generation_);
    return;
   }
   const auto phase=s.value("state",std::string());
   if(phase=="STOPPING"||phase=="FOUND"||phase=="NOT_FOUND"||phase=="ERROR"||phase=="CANCELLED"){
    gateway_->cancelNavigation("semantic_"+phase,public_generation_);return;
   }
  }''')
edit('gateway/runtime.cc','  if(state_){const auto&s=std::get<FpgaState>(state_->payload);const char*names[]=',
 '''  nav["semantic"]=semantic_;
  if(state_){const auto&s=std::get<FpgaState>(state_->payload);const char*names[]=''')
edit('gateway/runtime.cc','  callbacks.fpga_navigation_ready=',
 '''  callbacks.semantic_search_ready=[this](std::string&e){std::lock_guard<std::mutex>lock(mutex_);const auto now=mono_ns(),stamp=semantic_.value("published_mono_ns",uint64_t(0));const bool ready=stamp&&now>=stamp&&now-stamp<=500000000&&semantic_.value("ready",false)&&!semantic_.value("active",false);if(!ready)e="semantic_service_not_ready";return ready;};
  callbacks.fpga_navigation_ready=''')
