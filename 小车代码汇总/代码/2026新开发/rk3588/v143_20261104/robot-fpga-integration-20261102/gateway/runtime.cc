// Public phone Gateway: FPGA point navigation; RK Frontier through manual backend.
#include "../rk_gateway/navigation/gateway.h"
#include "http_client.h"
#include "display_pose.h"
#include "display_grid.h"
#include "../common/ros_bridge_transport.h"
#include "../common/ros_json_adapter.h"
#include <atomic>
#include <csignal>
#include <deque>
#include <fstream>
#include <mutex>
#include <optional>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>
namespace {
using namespace arb1;using namespace robotnav;
std::atomic<bool> stop_requested{false};
uint64_t ms(){return mono_ns()/1000000;}
bool same(const Identity&a,const Identity&b){return a.session==b.session&&a.run==b.run&&a.goal==b.goal&&a.nav_epoch==b.nav_epoch&&a.map_epoch==b.map_epoch&&a.map_revision==b.map_revision&&a.localization_epoch==b.localization_epoch&&a.plan_id==b.plan_id;}
struct Cached {Payload payload;Metadata metadata;};
class Runtime {
 GatewayOptions options_;std::string token_,map_path_,ipc_path_;
 std::unique_ptr<Gateway> gateway_;std::unique_ptr<LocalBridge> ipc_;
 struct Job { FpgaNavigationIntent intent; uint64_t epoch; };
 std::mutex mutex_;std::deque<Job> jobs_;
 std::atomic<uint64_t> intent_epoch_{1};uint64_t executing_epoch_=1;
 std::optional<Cached> state_,plan_,anchor_,map_,pose_;
 Json semantic_=Json::object();bool semantic_active_=false,semantic_stop_pending_=false;std::string semantic_task_;uint64_t semantic_stop_epoch_=0;
 Json map_metadata_=Json::object(),mapping_=Json::object(),exploration_=Json::object(),frontier_request_=Json::object();
 std::string render_map_json_,display_map_json_;uint64_t render_revision_=0;
 bool map_display_dirty_=true;std::string display_cells_;double display_ox_=0,display_oy_=0,display_res_=0;
 std::deque<Mission> responses_;
 std::string map_json_="{\"schema\":1,\"available\":false}",state_json_="{}",reason_="waiting_for_ROS_bridge";
 uint64_t command_=mono_ns(),authority_generation_=mono_ns(),pending_cancel_=0,goal_command_=0,start_command_=0;
 uint64_t public_generation_=0,ticket_expiry_=0,permit_seq_=0,last_context_=0;
 std::string phone_owner_,mode_="IDLE",phase_="IDLE";
 std::optional<std::pair<double,double>> selected_;
 std::atomic<bool> preview_ready_{false},mapping_authorized_{false},cancel_requested_{false};
 std::atomic<bool> manual_mapping_requested_{false};
 bool needs_prepare_=false;std::atomic<bool> backend_ready_{false};std::string cancellation_reason_;
 std::atomic<bool> execution_pending_{false};
 uint64_t nva_report_ms_=0;std::string nva_report_reason_;
 uint64_t drive_sequence_=0,drive_generation_=0,drive_ack_ms_=0;bool drive_requested_=false;
 uint64_t mapping_started_ms_=0;std::string drive_log_phase_;
 std::thread worker_,poller_;
 std::atomic<bool> stopped_{true};
 Metadata metadata(uint64_t now){Metadata m;m.received_mono_ns=now;return m;}
 void receive(Channel lane,const AcceptedMessage&m){
  (void)lane;
  // Decode/HELLO admission already occurred in LocalBridge. Preserve source time.
  std::lock_guard<std::mutex>lock(mutex_);
  for(const auto&b:m.message.blocks){
   if(std::holds_alternative<FpgaState>(b.payload))state_=Cached{b.payload,b.metadata};
   else if(std::holds_alternative<PlanResult>(b.payload))plan_=Cached{b.payload,b.metadata};
   else if(std::holds_alternative<ClockAnchor>(b.payload))anchor_=Cached{b.payload,b.metadata};
   else if(std::holds_alternative<Map>(b.payload)){map_=Cached{b.payload,b.metadata};map_display_dirty_=true;}
   else if(std::holds_alternative<Tf>(b.payload))pose_=Cached{b.payload,b.metadata};
   else if(auto d=std::get_if<Diagnostic>(&b.payload)){
    try{const auto j=Json::parse(d->json);if(!j.is_object()||j.find("value")==j.end()||!j.at("value").is_object())continue;const auto kind=j.value("kind",std::string());
     if(kind=="map_metadata")map_metadata_=j.at("value");
     else if(kind=="semantic")semantic_=j.at("value");
     else if(kind=="mapping")mapping_=j.at("value");
     else if(kind=="exploration")exploration_=j.at("value");
     else if(kind=="exploration_start_request")frontier_request_=j.at("value");
    }catch(...){}
   }else if(auto response=std::get_if<Mission>(&b.payload)){
    if(response->operation&ARB1_MISSION_REPLY){if(responses_.size()<8)responses_.push_back(*response);}
   }
  }
 }
 bool state_fresh(uint64_t now_ns,bool require_pose=false)const{
  if(!state_)return false;
  const auto&m=state_->metadata;const auto&s=std::get<FpgaState>(state_->payload);
  if(m.received_mono_ns>now_ns||m.source_age_ns==ARB1_UNKNOWN_SOURCE_AGE||m.source_age_ns>250000000||now_ns-m.received_mono_ns>250000000-m.source_age_ns)return false;
  if(require_pose&&(!(s.state_flags&32)||s.pose_age_ms>250||now_ns-m.received_mono_ns+m.source_age_ns>250000000-uint64_t(s.pose_age_ms)*1000000))return false;
  return true;
 }
 void ensure_current()const{if(executing_epoch_!=intent_epoch_.load())throw std::runtime_error("superseded_phone_intent");}
 void prepare(){
  execution_pending_=false;
  auto status=backend("status");ensure_current();auto j=private_task("prepare");
  authority_generation_=next_dashboard_generation(authority_generation_,exact_u64(status.at("generation")));
  j["generation"]=std::to_string(authority_generation_);
  j["expected_generation"]=status.at("generation");const auto prepared=backend("prepare",j);
  if(!prepared.value("quiet",false)||prepared.value("enabled",true))
   throw std::runtime_error("FPGA_prepare_not_quiet_or_unexpected_motion_authority");
  std::fprintf(stderr,"FPGA_PREPARE generation=%llu MOT_quiet=1 NVA_enabled=0\n",(unsigned long long)authority_generation_);
 }
 Json private_task(const std::string&operation){
  return {{"session",phone_owner_},{"generation",std::to_string(authority_generation_)},{"produced_mono_ns",std::to_string(dashboard_request_timestamp(mono_ns()))},{"valid_for_ms","500"},{"operation",operation}};
 }
 Json backend(const std::string&operation,Json value=Json::object()){
  const auto reply=dashboard_request(options_.backend_port,token_,operation=="status"?"GET":"POST","/api/navigation/fpga/"+operation,operation=="status"?std::string():value.dump());
  if(!reply.transport_ok||reply.status!=200)throw std::runtime_error("dashboard_FPGA_"+operation+"_rejected:"+reply.body);
  return Json::parse(reply.body);
 }
 bool context(){
  Message message;message.header.type=ARB1_MSG_SNAPSHOT;std::vector<uint8_t> bytes;
  {std::lock_guard<std::mutex>lock(mutex_);for(const auto*c:{&state_,&plan_,&anchor_})if(*c)message.blocks.push_back({(*c)->metadata,(*c)->payload});}
  if(message.blocks.empty()||!encode(message,bytes)||bytes.size()>16384)return false;
  const auto result=dashboard_request(options_.backend_port,token_,"POST","/api/navigation/fpga/context",std::string(reinterpret_cast<const char*>(bytes.data()),bytes.size()));
  if(execution_pending_){
   try{const auto j=Json::parse(result.body);const auto reason=j.value("reason",std::string());
    if(ms()-nva_report_ms_>=100||reason!=nva_report_reason_){
     Json record={{"mono_ms",ms()},{"http_status",result.status},{"enabled",j.value("enabled",false)},{"healthy",j.value("healthy",false)},{"reason",reason}};
     for(const auto&b:message.blocks)if(const auto*s=std::get_if<FpgaState>(&b.payload)){
      record["run"]=std::to_string(s->identity.run);record["pose_seq"]=std::to_string(s->pose_seq);record["pose_age_ms"]=s->pose_age_ms;
      const auto now=mono_ns();record["state_age_ms"]=b.metadata.source_age_ns!=ARB1_UNKNOWN_SOURCE_AGE&&now>=b.metadata.received_mono_ns?Json((now-b.metadata.received_mono_ns+b.metadata.source_age_ns)/1000000):Json(nullptr);
     }
     std::fprintf(stderr,"NVA_RENEWAL %s\n",record.dump().c_str());nva_report_ms_=ms();nva_report_reason_=reason;
    }
   }catch(...){/* Diagnostics never alter authorization or retry a command. */}
  }
  return result.transport_ok&&result.status==200;
 }
 uint64_t mission(Mission q){
  ensure_current();q.command_id=++command_;q.gateway_generation=public_generation_;
  if(!q.produced_mono_ns){q.produced_mono_ns=mono_ns();q.valid_for_ms=500;}
  Message m;m.header.type=ARB1_MSG_MISSION;m.blocks.push_back({metadata(mono_ns()),q});
  if(!ipc_->send(Channel::Control,m))throw std::runtime_error("ROS_control_queue_unavailable");
  return q.command_id;
 }
 void cancel_current(bool exploration_only=false){
  execution_pending_=false;
  pending_cancel_=0;
  if(exploration_only){Mission q;q.operation=ARB1_CANCEL;q.reason=4;pending_cancel_=mission(q);return;}
  std::optional<Cached> state;{std::lock_guard<std::mutex>lock(mutex_);state=state_;}
  if(state){const auto&s=std::get<FpgaState>(state->payload);if(s.identity.run){Mission q;q.operation=ARB1_CANCEL;q.identity=s.identity;q.reason=4;pending_cancel_=mission(q);}}
 }
 void process(const FpgaNavigationIntent&i){
  ensure_current();
  semantic_stop_pending_=false;
  const bool was_mapping=mode_=="MAPPING"||manual_mapping_requested_.load();
  phone_owner_=i.phone_session;public_generation_=i.gateway_generation;
  if(i.operation=="stop"||i.operation=="mapping-stop"){
   manual_mapping_requested_=false;
   if(i.operation=="mapping-stop")semantic_active_=false;
   try{backend("release",private_task("release"));}catch(...){}
   preview_ready_=false;selected_.reset();mapping_authorized_=false;++authority_generation_;
   mode_="IDLE";phase_="CANCELLED";if(!was_mapping)cancel_current();
   else if(i.operation!="mapping-stop")cancel_current(true);
   if(i.operation=="mapping-stop"){
    // Phone STOP already revoked motion. Silence subsequent disarmed MOT probes
    // before saved-map rebind/upload; this grants no NVA or motor authorization.
    prepare();Mission q;q.operation=ARB1_MAPPING_STOP;mission(q);
   }return;
  }
  if(i.operation=="goal"){
   semantic_active_=false;
   mapping_authorized_=false;selected_={{i.x_m,i.y_m}};preview_ready_=false;mode_="POINT";phase_="WAIT_CANCEL";needs_prepare_=true;
   ++authority_generation_;cancel_current();if(!pending_cancel_)phase_="PREPARE_GOAL";return;
  }
  if(i.operation=="manual-mapping-start"){
   semantic_active_=false;preview_ready_=false;selected_.reset();mapping_authorized_=false;
   mode_="MANUAL_MAPPING";phase_="STARTING";reason_="starting_phone_joystick_mapping";
   mapping_started_ms_=ms();
   Mission q;q.operation=ARB1_MANUAL_MAPPING_START;mission(q);return;
  }
  if(i.operation=="mapping-start"||i.operation=="semantic-start"){
   semantic_active_=i.operation=="semantic-start";
   if(semantic_active_){
    semantic_task_="semantic-"+std::to_string(public_generation_)+"-"+std::to_string(mono_ns());
    Json request={{"task",semantic_task_},{"target",i.semantic_target},{"gateway_generation",std::to_string(public_generation_)},
                  {"produced_mono_ns",std::to_string(mono_ns())},{"valid_for_ms",500}};
    auto reply=dashboard_request(8088,token_,"POST","/task",request.dump(),500);
    ensure_current();
    if(!reply.transport_ok||reply.status!=200||!Json::parse(reply.body).value("ok",false))throw std::runtime_error("semantic_service_start_rejected");
   }
   preview_ready_=false;selected_.reset();mapping_authorized_=true;mode_="MAPPING";phase_="MAPPING";
   ++authority_generation_;
   // Release any earlier navigation quiet mode; DRIVE itself also takes over.
   try{auto status=backend("status");ensure_current();auto release=private_task("release");release["generation"]=status.at("generation");backend("release",release);}catch(...){}
   drive_sequence_=drive_generation_=drive_ack_ms_=0;drive_requested_=false;mapping_started_ms_=ms();
   Mission q;q.operation=semantic_active_?ARB1_SEMANTIC_START:ARB1_MAPPING_START;mission(q);return;
  }
  if(i.operation=="start"){if(!preview_ready_)throw std::runtime_error("select_endpoint_first");mode_="POINT";phase_="CLAIM";return;}
  throw std::runtime_error("select_endpoint_first");
 }
 void prepare_goal(){
  if(needs_prepare_){prepare();needs_prepare_=false;}
  Mission q;q.operation=ARB1_GOAL;q.goal_x_mm=int32_t(std::llround(selected_->first*1000));q.goal_y_mm=int32_t(std::llround(selected_->second*1000));
  goal_command_=mission(q);phase_="WAIT_PLAN";reason_="planning_preview_only";
 }
 void handle_responses(){
  std::deque<Mission> responses;{std::lock_guard<std::mutex>lock(mutex_);responses.swap(responses_);}
  for(const auto&r:responses){
   if(r.command_id==pending_cancel_){
    if(r.status){pending_cancel_=0;if(phase_=="WAIT_CANCEL")throw std::runtime_error("FPGA_CANCEL_rejected");}
    else if(r.wire_request_id){pending_cancel_=0;if(phase_=="WAIT_CANCEL")phase_="PREPARE_GOAL";}
   }
   if(r.command_id==start_command_&&phase_=="WAIT_START_ACK"){
    if(r.status)throw std::runtime_error("FPGA_START_rejected");
    if(r.wire_request_id){
     {std::lock_guard<std::mutex>lock(mutex_);if(!plan_||!same(r.identity,std::get<PlanResult>(plan_->payload).identity))throw std::runtime_error("START_ACK_task_identity_mismatch");}
     if(mono_ns()>=ticket_expiry_)throw std::runtime_error("late_START_ACK");
     bool already_arrived=false;
     {std::lock_guard<std::mutex>lock(mutex_);if(state_&&state_fresh(mono_ns())){const auto&s=std::get<FpgaState>(state_->payload);auto id=s.identity;id.nav_epoch=r.identity.nav_epoch;id.plan_id=r.identity.plan_id;
      already_arrived=same(id,r.identity)&&s.state==4&&s.reason==13&&s.owner==0&&!(s.state_flags&1);}}
     if(already_arrived){phase_="ARRIVED";reason_="FPGA_arrived";preview_ready_=false;gateway_->finishFpgaNavigation(reason_,public_generation_);continue;}
     ensure_current();auto j=private_task("started");j["permit_seq"]=std::to_string(permit_seq_);backend("started",j);phase_="RUNNING";reason_="FPGA_autonomous_navigation";preview_ready_=false;
    }
   }
   if(r.command_id==goal_command_&&r.status)throw std::runtime_error("FPGA_GOAL_rejected");
   if(r.status&&((r.operation&0x7fff)==ARB1_MAPPING_START||(r.operation&0x7fff)==ARB1_SEMANTIC_START||(r.operation&0x7fff)==ARB1_MANUAL_MAPPING_START))throw std::runtime_error("mapping_start_rejected");
  }
 }
 void advance_exploration(){
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
    semantic_stop_pending_=true;
    semantic_stop_epoch_=intent_epoch_.load()+1;
    gateway_->cancelNavigation("semantic_"+phase,public_generation_);return;
   }
  }
  Json e;{std::lock_guard<std::mutex>lock(mutex_);e=exploration_;}
  if(!ipc_->ready(Channel::Control)){gateway_->cancelNavigation("exploration_bridge_disconnected",public_generation_);return;}
  if(semantic_active_){
   Json s;{std::lock_guard<std::mutex>lock(mutex_);s=semantic_;}
   if(s.value("state",std::string())!="SEARCHING")return;
   if(!e.value("semantic_external",false)){gateway_->cancelNavigation("semantic_executor_mode_mismatch",public_generation_);return;}
  }
  if(!e.is_object()||e.find("drive")==e.end()||exact_u64(e.value("gateway_generation",Json("0")))!=public_generation_){
   if(ms()-mapping_started_ms_>30000)gateway_->cancelNavigation("exploration_start_timeout",public_generation_);
   return;
  }
  const auto&d=e.at("drive");const auto now=mono_ns();
  const auto source=exact_u64(d.at("produced_mono_ns"));const auto ttl=exact_u64(d.at("valid_for_ms"));
  const auto gen=exact_u64(d.at("generation"));const auto seq=exact_u64(d.at("sequence"));
  if(!ttl||ttl>500||source>now||now-source>=ttl*1000000){gateway_->cancelNavigation("exploration_source_expired",public_generation_);return;}
  if(!drive_generation_)drive_generation_=gen;
  if(!gen||gen!=drive_generation_||exact_u64(d.at("gateway_generation"))!=public_generation_)return;
  if(seq<=drive_sequence_)return;
  const bool moving=d.value("motion_requested",false);
  if(moving){
   for(const auto*key:{"pose_age_ns","scan_age_ns"}){
    const auto age=exact_u64(d.at(key));if(age>250000000||now-source>250000000-age){gateway_->cancelNavigation("exploration_sensor_stale",public_generation_);return;}
   }
  }
  std::string error;
  if(!gateway_->applyExplorationDrive(d.at("op").get<std::string>(),d.at("throttle").get<double>(),d.at("steer").get<double>(),
       d.at("speed").get<double>(),d.at("direction")=="cw",public_generation_,seq,source/1000000,unsigned(ttl),error)){
   std::fprintf(stderr,"FRONTIER_REJECT generation=%llu sequence=%llu op=%s throttle=%.6f steer=%.6f speed=%.6f reason=%s\n",
       (unsigned long long)public_generation_,(unsigned long long)seq,d.at("op").get<std::string>().c_str(),
       d.at("throttle").get<double>(),d.at("steer").get<double>(),d.at("speed").get<double>(),error.c_str());
   if(gateway_->navigationActive())gateway_->cancelNavigation(error,public_generation_);
   return;
  }
  drive_sequence_=seq;drive_ack_ms_=ms();drive_requested_=moving;
  ensure_current();phase_=e.value("state",std::string("WAITING"));reason_=e.value("reason",std::string("waiting_for_exploration"));
  const auto observed=gateway_->telemetry();
  if(moving&&observed.reason!="manual"&&observed.reason!="rotate_cw"&&observed.reason!="rotate_ccw"){
   phase_="WAITING";reason_="manual_backend:"+observed.reason;
  }
  const auto log_phase=phase_+":"+reason_;
  if(log_phase!=drive_log_phase_||!(seq%100)){
   std::fprintf(stderr,"FRONTIER_DRIVE generation=%llu sequence=%llu phase=%s requested=%d backend=%s ack=yes\n",
       (unsigned long long)public_generation_,(unsigned long long)seq,phase_.c_str(),int(moving),observed.reason.c_str());drive_log_phase_=log_phase;
  }
  if(!e.value("active",false)){gateway_->cancelNavigation("exploration_"+phase_,public_generation_);}
 }
 void observe_semantic_stop(){
  // Motion authority is revoked immediately on STOPPING. Keep observing this
  // exact task until the coordinator confirms zero output + stopped odometry;
  // the cancellation callback must not replace a successful outcome with the
  // generic CANCELLED navigation phase.
  if(!semantic_active_||!semantic_stop_pending_)return;
  if(intent_epoch_.load()!=semantic_stop_epoch_){semantic_stop_pending_=false;return;}
  // Let the queued cancellation finish first, even if STOPPING was skipped and
  // the first received terminal state is already FOUND.
  if(cancel_requested_.load())return;
  Json s;{std::lock_guard<std::mutex>lock(mutex_);s=semantic_;}
  const auto now=mono_ns(),stamp=s.value("published_mono_ns",uint64_t(0));
  if(s.value("task",std::string())!=semantic_task_||s.value("gateway_generation",std::string())!=std::to_string(public_generation_)
     ||!stamp||stamp>now||now-stamp>350000000)return;
  const auto phase=s.value("state",std::string());
  if(phase!="STOPPING"&&phase!="FOUND"&&phase!="NOT_FOUND"&&phase!="ERROR"&&phase!="CANCELLED")return;
  phase_=phase;reason_=s.value("reason",phase);
  if(phase!="STOPPING")semantic_stop_pending_=false;
 }
 void advance(){
  if(phase_=="PREPARE_GOAL")prepare_goal();
  std::optional<Cached> plan,state;
  {std::lock_guard<std::mutex>lock(mutex_);plan=plan_;state=state_;}
  if(phase_=="WAIT_PLAN"&&plan){const auto&p=std::get<PlanResult>(plan->payload);if(p.command_id==goal_command_){if(p.status||p.cells.empty())throw std::runtime_error("FPGA_no_route");preview_ready_=true;phase_="PREVIEW_READY";reason_="preview_only_press_start";}}
  if(mode_=="MAPPING"&&mapping_authorized_)advance_exploration();
  if(mode_=="MANUAL_MAPPING"){
   Json mapping;{std::lock_guard<std::mutex>lock(mutex_);mapping=mapping_;}
   const auto stamp=mapping.value("published_mono_ns",uint64_t(0)),now=mono_ns();
   if(exact_u64(mapping.value("gateway_generation",Json("0")))==public_generation_&&stamp&&now>=stamp&&now-stamp<=500000000){
    phase_=mapping.value("state",std::string("STARTING"));reason_=mapping.value("reason",phase_);
    if(phase_=="ERROR"||phase_=="LOADED"||phase_=="IDLE"){manual_mapping_requested_=false;mode_="IDLE";}
   }else if(ms()-mapping_started_ms_>30000)throw std::runtime_error("manual_mapping_worker_stale");
  }
  observe_semantic_stop();
  if(mode_=="POINT"&&phase_=="RUNNING"&&plan&&state){
   const auto&p=std::get<PlanResult>(plan->payload);const auto&s=std::get<FpgaState>(state->payload);
   auto id=s.identity;id.nav_epoch=p.identity.nav_epoch;id.plan_id=p.identity.plan_id;
   if(same(p.identity,id)&&s.identity.plan_id>=p.identity.plan_id&&
      ((s.state==4&&s.reason==13)||(s.state==5)||(s.state_flags&8u))){
    phase_=s.state==4?"ARRIVED":"CANCELLED";reason_=s.state==4?"FPGA_arrived":(s.state_flags&8u)?"FPGA_no_route":"FPGA_cancelled_reason_"+std::to_string(s.reason);preview_ready_=false;
    gateway_->finishFpgaNavigation(reason_,public_generation_);
   }
  }
  if(mode_=="MAPPING"&&!semantic_active_){
   Json mapping;{std::lock_guard<std::mutex>lock(mutex_);mapping=mapping_;}
   if(exact_u64(mapping.value("gateway_generation",Json("0")))==public_generation_&&(mapping.value("state",std::string())=="LOADED"||mapping.value("state",std::string())=="ERROR")){
    mapping_authorized_=false;phase_=mapping.value("state",std::string());reason_=mapping.value("reason",phase_);
    gateway_->cancelNavigation(reason_,public_generation_);
   }
  }
  if(phase_=="CLAIM"){
   if(!plan||!state)throw std::runtime_error("complete_plan_and_state_required");
   const auto&p=std::get<PlanResult>(plan->payload);const auto&s=std::get<FpgaState>(state->payload);
   if(!same(p.identity,s.identity))throw std::runtime_error("plan_state_identity_mismatch");
   if(!context())return;
   ensure_current();execution_pending_=true;auto j=private_task("claim");j["command_id"]=std::to_string(p.command_id);backend("claim",j);phase_="WAIT_PERMIT";reason_="waiting_NVA_ACK";
  }
  if(phase_=="WAIT_PERMIT"){
   const auto status=backend("status");const auto t=status.find("ticket");
   if(t!=status.end()){
    ticket_expiry_=exact_u64(t->at("expires_mono_ns"));if(mono_ns()>=ticket_expiry_)throw std::runtime_error("expired_NVA_ticket");
    Mission q;q.operation=ARB1_START;q.identity=identity_from_json(*t);q.permit_seq=exact_u64(t->at("permit_seq"));q.auth_epoch=exact_u64(t->at("auth_epoch"));
    q.produced_mono_ns=ticket_expiry_-350000000;q.valid_for_ms=350;permit_seq_=q.permit_seq;
    start_command_=mission(q);phase_="WAIT_START_ACK";
   }else if(!status.value("enabled",false))throw std::runtime_error("NVA_authority_not_enabled:"+status.value("reason",std::string("missing_backend_reason")));
  }
 }
 void display(){
  std::lock_guard<std::mutex>lock(mutex_);
  Json mapping=mapping_;
  if(mode_=="MANUAL_MAPPING"&&exact_u64(mapping.value("gateway_generation",Json("0")))!=public_generation_){
   mapping={{"state","STARTING"},{"strategy","manual"},{"active",true},{"gateway_generation",std::to_string(public_generation_)}};
  }
  if(mode_=="MAPPING"&&!semantic_active_){
   mapping["state"]=phase_;mapping["reason"]=reason_;mapping["active"]=mapping_authorized_.load();
   mapping["drive_command_acknowledged"]=drive_ack_ms_&&ms()-drive_ack_ms_<250;
   mapping["drive_motion_requested"]=drive_requested_;mapping["drive_sequence"]=std::to_string(drive_sequence_);
  }
  Json nav={{"state",phase_},{"reason",reason_},{"can_start",preview_ready_.load()},{"readiness_reason",preview_ready_?"ready":"select_endpoint_first"},
    {"supports_goal_preview",true},{"navigation_enabled",phase_=="RUNNING"},{"mapping",mapping},{"execution_source",mode_=="MAPPING"?"RK_frontier_manual_channel":"FPGA_candidate26"},{"sends_velocity",mode_=="MAPPING"}};
  nav["semantic"]=semantic_active_?semantic_:Json::object();
  if(state_){const auto&s=std::get<FpgaState>(state_->payload);const char*names[]={"IDLE","PLANNING","FOLLOWING","AVOIDING","ARRIVED","CANCELLED"};nav["fpga_state"]=names[s.state];nav["avoid_stage"]=s.avoid_phase;nav["remaining_m"]=s.remaining_mm/1000.;nav["fpga_reason"]=s.reason;nav["plan_id"]=std::to_string(s.identity.plan_id);nav["pose_age_ms"]=s.pose_age_ms;nav["permit_remaining_ms"]=s.permit_ms;}
  state_json_=nav.dump();
  Json view={{"schema",1},{"available",false},{"state",phase_},{"reason",reason_},{"session_id","waiting"},{"map_revision",1}};
  const auto now=mono_ns();robotnav::DisplayPose display_pose;
  const bool local_nav_display=map_metadata_.value("local_odometry",false)&&phase_=="RUNNING";
  if(state_&&state_fresh(now,true)){const auto&s=std::get<FpgaState>(state_->payload);const auto&m=state_->metadata;
   try{double display_yaw;
    if(robotnav::fpga_display_heading(s.yaw_urad/1e6,map_metadata_.value("navigation_heading_offset_rad",0.),s.identity.session,exact_u64(map_metadata_.at("session_id")),display_yaw))
     display_pose.offer(s.x_mm/1000.,s.y_mm/1000.,display_yaw,now-m.received_mono_ns+m.source_age_ns+uint64_t(s.pose_age_ms)*1000000,"FPGA_report");
   }catch(...){/* Missing calibration metadata: use an honest ROS pose below. */}}
  if(pose_&&!local_nav_display){const auto&m=pose_->metadata;const auto&p=std::get<Tf>(pose_->payload);
   if(p.frame_id=="map"&&p.child_frame_id=="base_link"&&m.source_age_ns!=ARB1_UNKNOWN_SOURCE_AGE&&now>=m.received_mono_ns&&m.source_age_ns<=250000000&&now-m.received_mono_ns<=250000000-m.source_age_ns)
    display_pose.offer(p.x,p.y,p.yaw,now-m.received_mono_ns+m.source_age_ns,"ROS_scan_match");}
  if(!local_nav_display&&mapping_.find("live_pose")!=mapping_.end()&&mapping_["live_pose"].is_object()){
   try{const auto&live=mapping_["live_pose"];const auto source=exact_u64(live.at("source_monotonic_ns"));const auto&p=live.at("xyyaw");
    if(now>=source&&now-source<=250000000&&p.size()==3)display_pose.offer(p[0].get<double>(),p[1].get<double>(),p[2].get<double>(),now-source,"ROS_live_TF_IMU");
   }catch(...){}
  }
  const double x=display_pose.x,y=display_pose.y,yaw=display_pose.yaw;const bool valid=display_pose.valid;
  if(map_){
   const auto&g=std::get<Map>(map_->payload);
   // RK display keeps the original SLAM grid. Serialize only when the map changes;
   // the legacy phone/FPGA selection bitmap remains a separate 64x64 contract.
   if(map_display_dirty_||render_map_json_.empty()){
    std::string native(g.cells.size(),'?');
    for(size_t i=0;i<native.size();++i)native[i]=g.cells[i]<0?'?':g.cells[i]<=25?'.':'#';
    render_map_json_=Json({{"width",g.width},{"height",g.height},{"resolution_m",g.resolution},
      {"origin",{g.origin_x,g.origin_y,g.origin_yaw}},{"cells",native},{"revision",++render_revision_},
      {"session",mapping_.value("slam_session_start_ns",std::string())}}).dump();
   }
   double ox=std::floor((x-3.2)*10)/10,oy=std::floor((y-3.2)*10)/10,res=.1;
   // Overview is display-only: derive an axis-aligned bound from the full map.
   const double gc=std::cos(g.origin_yaw),gs=std::sin(g.origin_yaw);
   double minx=g.origin_x,maxx=minx,miny=g.origin_y,maxy=miny;
   for(int ix=0;ix<=1;++ix)for(int iy=0;iy<=1;++iy){double a=ix*g.width*g.resolution,b=iy*g.height*g.resolution;
    double wx=g.origin_x+gc*a-gs*b,wy=g.origin_y+gs*a+gc*b;minx=std::min(minx,wx);maxx=std::max(maxx,wx);miny=std::min(miny,wy);maxy=std::max(maxy,wy);}
   res=std::max(.1,std::max(maxx-minx,maxy-miny)/64.);ox=minx;oy=miny;
   std::string session="preview",navigable(4096,'#');uint32_t revision=1;
   bool committed=false;
   if(mapping_.value("state",std::string())=="LOADED"&&!map_metadata_.empty()&&map_metadata_.value("uploaded",false)&&map_metadata_.value("state_crc_verified",false)&&state_){
    const auto&s=std::get<FpgaState>(state_->payload);if(s.active_crc==map_metadata_.value("bitmap_crc32",0u)&&s.identity.map_revision==exact_u64(map_metadata_.at("map_revision"))&&s.identity.map_epoch==exact_u64(map_metadata_.at("map_epoch"))){
     committed=true;res=.1;
     ox=map_metadata_.at("origin_m")[0].get<double>();oy=map_metadata_.at("origin_m")[1].get<double>();session=std::to_string(s.identity.session);revision=uint32_t(s.identity.map_revision);
    }
   }
   if(map_display_dirty_||display_cells_.size()!=4096||display_ox_!=ox||display_oy_!=oy||display_res_!=res){
    display_cells_=robotnav::display_grid(g.width,g.height,g.resolution,g.origin_x,g.origin_y,g.origin_yaw,g.cells,ox,oy,res);
    display_ox_=ox;display_oy_=oy;display_res_=res;map_display_dirty_=false;
   }
   const auto&cells=display_cells_;
   // Selection remains fail-closed until the exact acknowledged local bitmap arrives.
   if(committed&&map_metadata_.find("local_navigable")!=map_metadata_.end())navigable=map_metadata_.at("local_navigable").get<std::string>();
   view={{"schema",1},{"available",true},{"width",64},{"height",64},{"resolution_m",res},{"origin",{ox,oy}},
    {"cells",cells},{"navigable",navigable},{"pose",{x,y,yaw}},{"pose_valid",valid},{"goal",nullptr},
    {"path",Json::array()},{"trajectory",Json::array()},{"session_id",session},{"map_revision",revision},
    {"state",phase_},{"reason",reason_},{"updated_monotonic_ms",ms()},{"snapshot_age_ms",0},{"sensor_age_ms",valid?display_pose.age_ns/1000000:251},{"pose_source",display_pose.source},
    {"map_source","ROS_slam_toolbox"},{"map_static",mapping_.value("state",std::string())=="LOADED"},{"view_scope",committed?"6.4m_local_window":"global_overview"},{"robot_radius_m",.20},{"map_commit_confirmed",committed}};
   if(selected_)view["goal"]={selected_->first,selected_->second};
   if(plan_&&phase_!="CANCELLED"&&phase_!="ERROR"&&phase_!="ARRIVED"){const auto&p=std::get<PlanResult>(plan_->payload);const auto*st=state_?std::get_if<FpgaState>(&state_->payload):nullptr;if(phase_!="RUNNING"||(st&&same(p.identity,st->identity)&&(st->state_flags&2u)))for(auto cell:p.cells)view["path"].push_back({(p.origin_x_mm+(cell%64)*100+50)/1000.,(p.origin_y_mm+(cell/64)*100+50)/1000.});}
  }
  map_json_=view.dump();
  display_map_json_=map_json_;
  if(map_&&!render_map_json_.empty()){display_map_json_.pop_back();display_map_json_+=",\"render_map\":"+render_map_json_+"}";}
  if(!map_path_.empty()){const auto tmp=map_path_+".tmp";std::ofstream file(tmp);file<<display_map_json_<<'\n';file.close();if(file)rename(tmp.c_str(),map_path_.c_str());}
 }
 void loop(){uint64_t display_at=0;while(!stopped_){try{
  std::optional<Job> job;std::string cancel_reason;
  {std::lock_guard<std::mutex>lock(mutex_);if(cancel_requested_.exchange(false))cancel_reason=cancellation_reason_;if(!jobs_.empty()){job=jobs_.front();jobs_.pop_front();}}
  // End-mapping owns stop/save; do not pre-empt it with a takeover notice.
  if(job&&job->epoch==intent_epoch_.load()&&job->intent.operation=="mapping-stop")cancel_reason.clear();
  if(!cancel_reason.empty()){executing_epoch_=intent_epoch_.load();const bool was_mapping=mode_=="MAPPING";reason_=cancel_reason;mode_="IDLE";phase_="CANCELLED";preview_ready_=false;mapping_authorized_=false;cancel_current(was_mapping);}
  if(job&&job->epoch==intent_epoch_.load()){executing_epoch_=job->epoch;process(job->intent);}handle_responses();
  // Keep source timestamps intact; reduce forwarding wait rather than re-age
  // a cached STATE or relax the 250 ms admission guard.
  if(mode_!="MAPPING"&&mode_!="MANUAL_MAPPING"&&ms()-last_context_>=20){last_context_=ms();const bool ok=context();backend_ready_=ok;}
  advance();
  if(ms()-display_at>=50){display();display_at=ms();}
 }catch(const std::exception&e){if(executing_epoch_!=intent_epoch_.load())continue;reason_=e.what();phase_="ERROR";preview_ready_=false;mapping_authorized_=false;manual_mapping_requested_=false;try{cancel_current(mode_=="MAPPING");}catch(...){}gateway_->cancelNavigation(reason_,public_generation_);display();}
 std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
public:
 Runtime(GatewayOptions options,std::string directory,std::string map_path):options_(options),map_path_(std::move(map_path)),ipc_path_(std::move(directory)){
  std::ifstream file(options.token_file);std::getline(file,token_);if(!file||token_.size()<6)throw std::runtime_error("pair token missing");
  GatewayCallbacks callbacks;
  callbacks.fpga_navigation_command=[this](const FpgaNavigationIntent&i,std::string&error){std::lock_guard<std::mutex>lock(mutex_);
   if(manual_mapping_requested_&&i.operation!="mapping-stop"&&i.operation!="stop"){error="finish_manual_mapping_first";return false;}
   if(jobs_.size()>=8){error="navigation_queue_full";return false;}
   if(i.operation=="start"&&!preview_ready_){error="select_endpoint_first";return false;}
   try{if(i.operation=="goal"){
    Json v=Json::parse(map_json_);if(!v.value("available",false)||v.value("session_id",std::string())!=i.map_session||v.value("map_revision",0u)!=i.map_revision){error="map_changed_reselect_goal";return false;}
    const auto&o=v.at("origin");const auto cx=int(std::floor((i.x_m-o[0].get<double>())*10)),cy=int(std::floor((i.y_m-o[1].get<double>())*10));
    if(cx<0||cy<0||cx>=64||cy>=64){error="GOAL_OUTSIDE_WINDOW";return false;}
    const auto cells=v.at("navigable").get<std::string>();if(cells.size()!=4096||cells[cy*64+cx]!='.'){error="BLOCKED_ENDPOINT";return false;}
   }
   }catch(const std::exception&){error="map_not_available";return false;}
   jobs_.push_back(Job{i,intent_epoch_.load()});if(i.operation=="manual-mapping-start")manual_mapping_requested_=true;return true;};
  callbacks.semantic_search_ready=[this](std::string&e){std::lock_guard<std::mutex>lock(mutex_);const auto now=mono_ns(),stamp=semantic_.value("published_mono_ns",uint64_t(0));const bool ready=stamp&&now>=stamp&&now-stamp<=500000000&&semantic_.value("ready",false)&&!semantic_.value("active",false);if(!ready)e="semantic_service_not_ready";return ready;};
  callbacks.fpga_navigation_ready=[this](std::string&e){std::lock_guard<std::mutex>lock(mutex_);const bool ready=ipc_&&ipc_->ready(Channel::Control)&&state_fresh(mono_ns())&&backend_ready_;if(!ready)e="ROS_FPGA_state_unavailable";return ready;};
  callbacks.fpga_execution_pending=[this]{return execution_pending_.load();};
  callbacks.exploration_mapping_ready=[this](std::string&e){std::lock_guard<std::mutex>lock(mutex_);const auto stamp=mapping_.value("published_mono_ns",uint64_t(0));const auto now=mono_ns();const bool ready=ipc_&&ipc_->ready(Channel::Control)&&stamp&&now>=stamp&&now-stamp<=500000000&&mapping_.value("ready",false);if(!ready)e="exploration_adapter_not_ready";return ready;};
  callbacks.manual_mapping_ready=[this](std::string&e){std::lock_guard<std::mutex>lock(mutex_);const auto stamp=mapping_.value("published_mono_ns",uint64_t(0)),now=mono_ns();const bool ready=!manual_mapping_requested_&&ipc_&&ipc_->ready(Channel::Control)&&stamp&&now>=stamp&&now-stamp<=500000000&&mapping_.value("manual_ready",false);if(!ready)e="manual_mapping_not_ready";return ready;};
  callbacks.navigation_state=[this]{std::lock_guard<std::mutex>lock(mutex_);return state_json_;};callbacks.navigation_map=[this]{std::lock_guard<std::mutex>lock(mutex_);return display_map_json_.empty()?map_json_:display_map_json_;};
  callbacks.navigation_cancel=[this](const std::string&reason){std::lock_guard<std::mutex>lock(mutex_);
   if(manual_mapping_requested_&&reason!="navigation_mapping-stop"&&reason!="gateway_shutdown")return;
   preview_ready_=false;mapping_authorized_=false;jobs_.clear();++intent_epoch_;cancellation_reason_=reason;cancel_requested_=true;std::fprintf(stderr,"NAVIGATION_CANCEL reason=%s\n",reason.c_str());};
  gateway_=std::make_unique<Gateway>(options_,callbacks);
 }
 bool start(std::string&error){
  ipc_=std::make_unique<LocalBridge>(ipc_path_,false,[this](Channel c,const AcceptedMessage&m){receive(c,m);});
  if(!gateway_->start(error))return false;
  stopped_=false;
  poller_=std::thread([this]{while(!stopped_){gateway_->pollState();std::this_thread::sleep_for(std::chrono::milliseconds(50));}});
  worker_=std::thread([this]{loop();});return true;
 }
 ~Runtime(){stopped_=true;if(worker_.joinable())worker_.join();if(poller_.joinable())poller_.join();if(gateway_)gateway_->stop();ipc_.reset();}
};
}
int main(int argc,char**argv){
 GatewayOptions options;options.fpga_navigation_source=true;std::string ipc="/run/acg138-ros-bridge",map="/tmp/robot-navigation/map.json";
 for(int i=1;i<argc;++i){std::string key=argv[i];if(key=="--help"){std::puts("robot-fpga-gateway --ipc DIRECTORY --map-file FILE --token-file FILE --port 8080 --backend-port 8082; no Navigator or velocity output");return 0;}if(++i==argc)return 2;const std::string value=argv[i];if(key=="--ipc")ipc=value;else if(key=="--map-file")map=value;else if(key=="--token-file")options.token_file=value;else if(key=="--port")options.port=std::stoul(value);else if(key=="--backend-port")options.backend_port=std::stoul(value);else return 2;}
 std::signal(SIGINT,[](int){stop_requested=true;});std::signal(SIGTERM,[](int){stop_requested=true;});
 try{Runtime runtime(options,ipc,map);std::string error;if(!runtime.start(error))throw std::runtime_error(error);while(!stop_requested)std::this_thread::sleep_for(std::chrono::milliseconds(100));}
 catch(const std::exception&e){std::fprintf(stderr,"FPGA Gateway: %s\n",e.what());return 1;}return 0;
}
