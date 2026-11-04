#include "ros_bridge_transport.h"
#include "ros_json_adapter.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <acg138_nav_interfaces/srv/execution.hpp>
#include <deque>
#include <algorithm>
#include <map>
#include <mutex>
#include <optional>

using namespace std::chrono_literals;
namespace {
using Json=arb1::Json;using Text=std_msgs::msg::String;using Trigger=std_srvs::srv::Trigger;
using Execution=acg138_nav_interfaces::srv::Execution;
class Bridge final:public rclcpp::Node {
 std::unique_ptr<arb1::LocalBridge> ipc_;std::mutex mutex_;
 std::deque<arb1::Mission> queue_;std::map<uint64_t,arb1::Mission> inflight_,completed_;
 uint64_t received_=0,rejected_=0,last_command_=0,map_revision_=0,last_map_hash_=0;
 struct Goal {arb1::Mission request;int64_t ros_stamp=0;};std::optional<Goal> goal_;
 std::optional<arb1::FpgaState> fpga_;Json metadata_;
 std::optional<arb1::PlanResult> complete_plan_;std::optional<arb1::ClockAnchor> clock_;
 arb1::Metadata fpga_meta_,plan_meta_,clock_meta_;
 std::optional<arb1::Map> latest_map_;arb1::Metadata map_meta_;
 bool map_dirty_=false,was_state_ready_=false;uint64_t context_sent_=0;
 std::string local_navigable_;uint32_t local_crc_=0;double local_x_=0,local_y_=0;
 Json frontier_intent_;
 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goals_;
 rclcpp::Publisher<Text>::SharedPtr authorized_,manual_,acks_,frontier_execution_;
 rclcpp::Client<Execution>::SharedPtr start_,cancel_;
 rclcpp::Client<Execution>::SharedPtr mapping_start_,mapping_stop_;
 std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
 rclcpp::TimerBase::SharedPtr timer_;
 static arb1::Metadata meta(uint64_t receipt,int64_t source=0,uint64_t age=ARB1_UNKNOWN_SOURCE_AGE){arb1::Metadata m;m.received_mono_ns=receipt;m.source_stamp_ns=source;m.source_age_ns=age;return m;}
 void send(arb1::Channel lane,const arb1::Payload&p,const arb1::Metadata&m){arb1::Message message;message.blocks.push_back({m,p});ipc_->send(lane,message);}
 void diagnostic(const Json&j){arb1::Diagnostic d;d.json=j.dump();if(d.json.size()<=16384)send(arb1::Channel::Control,d,meta(arb1::mono_ns()));}
 void reply(arb1::Mission m,uint16_t status,uint64_t wire=0){
  m.operation|=0x8000;m.status=status;m.wire_request_id=wire;
  arb1::Message message;message.header.type=ARB1_MSG_MISSION;message.blocks.push_back({meta(arb1::mono_ns()),m});
  ipc_->send(arb1::Channel::Control,message);
  completed_[m.command_id]=m;while(completed_.size()>8)completed_.erase(completed_.begin());
 }
 void receive(arb1::Channel channel,const arb1::AcceptedMessage&m){
  if(channel!=arb1::Channel::Control)return;
  // Socket callbacks enqueue only. ROS calls and serialization stay off this lane.
  std::lock_guard<std::mutex> lock(mutex_);
  if(m.message.blocks.size()!=1||!std::holds_alternative<arb1::Mission>(m.message.blocks[0].payload)){++rejected_;return;}
  const auto q=std::get<arb1::Mission>(m.message.blocks[0].payload);
  if(!arb1::executable(q,arb1::mono_ns())){
   ++rejected_;
   // Fail before queue admission or ROS services, but complete the bounded RPC.
   // Do not touch the ROS-thread-owned idempotency cache from this socket thread.
   auto denied=q;denied.operation|=ARB1_MISSION_REPLY;denied.status=4;denied.wire_request_id=0;
   arb1::Message response;response.header.type=ARB1_MSG_MISSION;
   response.blocks.push_back({meta(arb1::mono_ns()),denied});ipc_->send(arb1::Channel::Control,response);
   return;
  }
  if(q.operation==ARB1_CANCEL){
   queue_.erase(std::remove_if(queue_.begin(),queue_.end(),[](const auto&old){return old.operation==ARB1_START||old.operation==ARB1_GOAL;}),queue_.end());
  }
  if(queue_.size()>=8){++rejected_;arb1::Message error;error.header.type=ARB1_MSG_ERROR;arb1::Error e;e.code=ARB1_E_QUEUE_FULL;e.offending_sequence=m.message.header.message_sequence;e.detail="mission queue capacity 8";error.blocks.push_back({meta(arb1::mono_ns()),e});ipc_->send(arb1::Channel::Control,error);return;}
  if(q.operation==ARB1_CANCEL)queue_.push_front(q);else queue_.push_back(q);
  ++received_;
 }
 template<class F> void text(const char*topic,F fn,bool latched=false){
  auto qos=rclcpp::QoS(8).reliable();if(latched)qos.transient_local();
  subscriptions_.push_back(create_subscription<Text>(topic,qos,[this,fn](Text::ConstSharedPtr m){try{if(m->data.size()>32768)throw std::runtime_error("oversized ROS JSON");fn(Json::parse(m->data));}catch(const std::exception&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"ROS IPC rejected: %s",e.what());}}));
 }
 void process(arb1::Mission q){
  if(auto i=completed_.find(q.command_id);i!=completed_.end()){
   // Idempotent response repeats the original source time and never renews NVA.
   auto old=i->second;arb1::Message m;m.header.type=ARB1_MSG_MISSION;m.blocks.push_back({meta(arb1::mono_ns()),old});ipc_->send(arb1::Channel::Control,m);return;
  }
  if(!arb1::executable(q,arb1::mono_ns())||q.command_id<=last_command_){reply(q,4);return;}
  last_command_=q.command_id;
  if(q.flags&1){reply(q,4);return;}
  if(q.operation==ARB1_GOAL){
   if(inflight_.size()>=8){reply(q,10);return;}
   geometry_msgs::msg::PoseStamped g;g.header.frame_id="map";g.header.stamp=now();g.pose.position.x=q.goal_x_mm/1000.;g.pose.position.y=q.goal_y_mm/1000.;g.pose.orientation.w=1;
   goal_=Goal{q,rclcpp::Time(g.header.stamp).nanoseconds()};goals_->publish(g);reply(q,0);return;
  }
  if(q.operation==ARB1_MAPPING_START||q.operation==ARB1_MAPPING_STOP){
   if(inflight_.size()>=8){reply(q,10);return;}
   auto c=q.operation==ARB1_MAPPING_START?mapping_start_:mapping_stop_;
   if(!c->service_is_ready()){reply(q,10);return;}
   inflight_[q.command_id]=q;
   auto request=std::make_shared<Execution::Request>();request->request_id=q.command_id;
   request->created_monotonic_ns=q.produced_mono_ns;request->valid_for_ms=q.valid_for_ms;
   request->run_generation=q.gateway_generation;
   c->async_send_request(request,[this,q](rclcpp::Client<Execution>::SharedFuture f){
    inflight_.erase(q.command_id);try{const auto r=f.get();reply(q,r->success?0:10);diagnostic({{"kind","mapping_request"},{"command_id",std::to_string(q.command_id)},{"message",r->message},{"success",r->success}});}catch(...){reply(q,16);}});return;
  }
  if(q.operation!=ARB1_START&&q.operation!=ARB1_CANCEL){reply(q,1);return;}
  if(q.operation==ARB1_CANCEL){
   goal_.reset();complete_plan_.reset();
   Text notice;notice.data=Json({{"gateway_generation",std::to_string(q.gateway_generation)},{"reason","manual_takeover"}}).dump();manual_->publish(notice);
   if(!q.identity.run){reply(q,0,q.command_id);return;}
  }
  auto client=q.operation==ARB1_START?start_:cancel_;
  if(!client->service_is_ready()||inflight_.size()>=8){reply(q,10);return;}
  auto r=std::make_shared<Execution::Request>();const auto&i=q.identity;
  r->request_id=q.command_id;r->session_id=i.session;r->run_generation=i.run;r->goal_id=i.goal;
  r->map_epoch=i.map_epoch;r->map_revision=i.map_revision;r->localization_epoch=i.localization_epoch;r->plan_id=i.plan_id;r->nav_epoch=i.nav_epoch;
  r->permit_seq=q.permit_seq;r->auth_epoch=q.auth_epoch;r->created_monotonic_ns=q.produced_mono_ns;r->valid_for_ms=q.valid_for_ms;r->reason=q.reason;
  inflight_[q.command_id]=q;
  if(q.operation==ARB1_START&&!frontier_intent_.empty()){
   try{const auto expected=arb1::identity_from_json(frontier_intent_.at("identity"));
    if(expected.session==i.session&&expected.run==i.run&&expected.goal==i.goal&&expected.plan_id==i.plan_id){
     auto notice=arb1::identity_json(i);notice["generation"]=frontier_intent_.at("generation");notice["request_id"]=q.command_id;notice["produced_mono_ns"]=q.produced_mono_ns;notice["valid_for_ms"]=q.valid_for_ms;
     Text m;m.data=notice.dump();frontier_execution_->publish(m);
    }
   }catch(...){}
  }
  client->async_send_request(r,[this,q](rclcpp::Client<Execution>::SharedFuture f){
   try{const auto result=f.get();if(!result->success){inflight_.erase(q.command_id);reply(q,result->status?result->status:16);}else reply(q,0,0);}catch(...){inflight_.erase(q.command_id);reply(q,16);}});
 }
 void tick(){
  std::optional<arb1::Mission> next;
  {std::lock_guard<std::mutex>lock(mutex_);if(!ipc_->ready(arb1::Channel::Control)){queue_.clear();return;}if(!queue_.empty()){next=queue_.front();queue_.pop_front();}}
  if(next)process(*next);
  for(auto i=inflight_.begin();i!=inflight_.end();){if(!arb1::executable(i->second,arb1::mono_ns())){auto q=i->second;i=inflight_.erase(i);reply(q,4);}else ++i;}
  const auto now=arb1::mono_ns();
  if(ipc_->ready(arb1::Channel::Control)&&now-context_sent_>=50000000){
   arb1::Message context;
   if(fpga_)context.blocks.push_back({fpga_meta_,*fpga_});if(clock_)context.blocks.push_back({clock_meta_,*clock_});if(complete_plan_)context.blocks.push_back({plan_meta_,*complete_plan_});
   if(!context.blocks.empty()&&ipc_->send(arb1::Channel::Control,context))context_sent_=now;
  }
  const bool ready=ipc_->ready(arb1::Channel::State);
  if(ready&&!was_state_ready_)map_dirty_=true;was_state_ready_=ready;
  if(map_dirty_&&latest_map_&&ready&&ipc_->queued_bytes(arb1::Channel::State)==0){
   arb1::Message m;m.blocks.push_back({map_meta_,*latest_map_});if(ipc_->send(arb1::Channel::State,m))map_dirty_=false;
  }
 }
 void publish_metadata(){
  if(metadata_.empty())return;
  auto value=metadata_;
  try{const auto&o=value.at("origin_m");
   if(value.value("uploaded",false)&&value.value("state_crc_verified",false)&&value.at("bitmap_crc32").get<uint32_t>()==local_crc_&&
      local_navigable_.size()==4096&&std::abs(o[0].get<double>()-local_x_)<1e-6&&std::abs(o[1].get<double>()-local_y_)<1e-6)
      value["local_navigable"]=local_navigable_;
  }catch(...){}
  diagnostic({{"kind","map_metadata"},{"value",value}});
 }
 void state(const Json&j){
  auto s=arb1::fpga_state_from_json(j);
  if(!metadata_.empty()&&metadata_.value("uploaded",false)){
   const auto&o=metadata_.at("origin_m");s.origin_x_mm=std::lround(o[0].get<double>()*1000);s.origin_y_mm=std::lround(o[1].get<double>()*1000);s.frame_id=metadata_.at("frame_id").get<std::string>();
  }
  const auto receipt=arb1::exact_u64(j.at("published_mono_ns"));const auto age=arb1::exact_u64(j.at("host_receipt_age_ms"))*1000000;
  if(receipt>arb1::mono_ns()||arb1::mono_ns()-receipt>250000000||age>250000000)return;
  fpga_=s;fpga_meta_=meta(receipt,0,age);send(arb1::Channel::Control,s,fpga_meta_);
 }
 void plan(const Json&j){
  uint64_t command=0;
  if(goal_&&arb1::exact_u64(j.at("goal_source_stamp_ns"))==uint64_t(goal_->ros_stamp))command=goal_->request.command_id;
  auto p=arb1::plan_from_json(j,command);complete_plan_=p;plan_meta_=meta(arb1::mono_ns());send(arb1::Channel::Control,p,plan_meta_);
 }
 void command_result(const Json&j){
  const auto request=arb1::exact_u64(j.at("request_id"));auto i=inflight_.find(request);if(i==inflight_.end())return;
  auto q=i->second;const auto id=arb1::identity_from_json(j);
  if(id.session!=q.identity.session||id.run!=q.identity.run||id.goal!=q.identity.goal||id.map_epoch!=q.identity.map_epoch||id.map_revision!=q.identity.map_revision||id.localization_epoch!=q.identity.localization_epoch||id.nav_epoch!=q.identity.nav_epoch||id.plan_id!=q.identity.plan_id)return;
  inflight_.erase(i);reply(q,j.at("status").get<uint16_t>(),j.at("fpga_ack").get<bool>()?request:0);
 }
 void map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
  if(!m->info.width||!m->info.height||uint64_t(m->info.width)*m->info.height>1000000||m->data.size()!=size_t(m->info.width)*m->info.height)return;
  // Only map callbacks encode pixels; odom/state callbacks never re-encode a map.
  uint64_t hash=1469598103934665603ull;
  for(auto b:m->data){hash^=uint8_t(b);hash*=1099511628211ull;}
  const auto&q=m->info.origin.orientation;const double yaw=2*std::atan2(q.z,q.w);
  auto geometry=std::to_string(m->info.width)+":"+std::to_string(m->info.height)+":"+std::to_string(m->info.resolution)+":"+std::to_string(m->info.origin.position.x)+":"+std::to_string(m->info.origin.position.y)+":"+std::to_string(yaw)+":"+m->header.frame_id;
  for(char c:geometry){hash^=uint8_t(c);hash*=1099511628211ull;}
  if(hash==last_map_hash_)return;last_map_hash_=hash;
  arb1::Map map;map.width=m->info.width;map.height=m->info.height;map.resolution=m->info.resolution;map.origin_x=m->info.origin.position.x;map.origin_y=m->info.origin.position.y;map.origin_yaw=yaw;map.frame_id=m->header.frame_id;map.map_epoch=1;map.map_revision=++map_revision_;map.cells=m->data;
  latest_map_=std::move(map);map_meta_=meta(arb1::mono_ns(),rclcpp::Time(m->header.stamp).nanoseconds());map_dirty_=true;
 }
 void scan(sensor_msgs::msg::LaserScan::ConstSharedPtr m){
  if(m->ranges.empty())return;
  arb1::Scan s;s.frame_id=m->header.frame_id;s.angle_min=m->angle_min;s.range_min=m->range_min;s.range_max=m->range_max;
  const auto stride=std::max<size_t>(1,(m->ranges.size()+359)/360);s.angle_increment=m->angle_increment*stride;
  for(size_t i=0;i<m->ranges.size()&&s.ranges.size()<360;i+=stride)s.ranges.push_back(m->ranges[i]);
  const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds(),age=now().nanoseconds()-stamp;
  send(arb1::Channel::State,s,meta(arb1::mono_ns(),stamp,age>=0?uint64_t(age):ARB1_UNKNOWN_SOURCE_AGE));
 }
public:
 Bridge():Node("acg138_ros_bridge"){
  const auto directory=declare_parameter<std::string>("ipc_directory","/run/acg138-ros-bridge");
  goals_=create_publisher<geometry_msgs::msg::PoseStamped>("/fpga/goal",1);
  authorized_=create_publisher<Text>("/exploration/authorized_start",1);manual_=create_publisher<Text>("/exploration/manual_takeover",1);acks_=create_publisher<Text>("/navigation/diagnostics",1);frontier_execution_=create_publisher<Text>("/exploration/gateway_start",8);
  start_=create_client<Execution>("/fpga/start");cancel_=create_client<Execution>("/fpga/cancel");mapping_start_=create_client<Execution>("/mapping/start");mapping_stop_=create_client<Execution>("/mapping/stop");
  ipc_=std::make_unique<arb1::LocalBridge>(directory,true,[this](arb1::Channel c,const arb1::AcceptedMessage&m){receive(c,m);});
  text("/fpga/state",[this](const Json&j){state(j);});
  text("/fpga/map_metadata",[this](const Json&j){metadata_=j;publish_metadata();},true);
  text("/fpga/plan_result",[this](const Json&j){plan(j);},true);
  text("/fpga/command_result",[this](const Json&j){command_result(j);});
  text("/fpga/clock_anchor",[this](const Json&j){clock_=arb1::anchor_from_json(j);clock_meta_=meta(arb1::mono_ns());send(arb1::Channel::Control,*clock_,clock_meta_);});
  text("/mapping/state",[this](const Json&j){diagnostic({{"kind","mapping"},{"value",j}});},true);
  text("/exploration/state",[this](const Json&j){diagnostic({{"kind","exploration"},{"value",j}});});
  text("/exploration/start_request",[this](const Json&j){frontier_intent_=j;diagnostic({{"kind","exploration_start_request"},{"value",j}});});
  const auto qos=rclcpp::QoS(1).reliable().transient_local();
  subscriptions_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>("/map",qos,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){map(m);}));
  subscriptions_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>("/fpga/local_costmap",qos,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
   if(m->info.width!=64||m->info.height!=64||m->data.size()!=4096)return;
   std::array<uint8_t,512> bitmap{};std::string text;text.reserve(4096);
   for(unsigned i=0;i<4096;++i){if(m->data[i]!=0&&m->data[i]!=100)return;text+=m->data[i]?'#':'.';if(m->data[i])bitmap[i/8]|=1u<<(i%8);}
   uint32_t crc=0xffffffff;for(auto b:bitmap){crc^=b;for(unsigned i=0;i<8;++i)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}
   local_crc_=~crc;local_navigable_=std::move(text);local_x_=m->info.origin.position.x;local_y_=m->info.origin.position.y;publish_metadata();}));
  subscriptions_.push_back(create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose",10,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
   if(m->header.frame_id!="map")return;const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds(),age=now().nanoseconds()-stamp;if(age<0||age>250000000)return;
   const auto&p=m->pose.pose;arb1::Tf tf;tf.frame_id="map";tf.child_frame_id="base_link";tf.x=p.position.x;tf.y=p.position.y;tf.yaw=2*std::atan2(p.orientation.z,p.orientation.w);tf.localization_epoch=1;
   send(arb1::Channel::State,tf,meta(arb1::mono_ns(),stamp,uint64_t(age)));}));
  subscriptions_.push_back(create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan(m);}));
  subscriptions_.push_back(create_subscription<nav_msgs::msg::Path>("/fpga/planned_path",qos,[this](nav_msgs::msg::Path::ConstSharedPtr m){
   if(m->poses.size()>4096)return;arb1::Path p;p.frame_id=m->header.frame_id;if(fpga_){p.goal_id=fpga_->identity.goal;p.map_epoch=fpga_->identity.map_epoch;p.map_revision=fpga_->identity.map_revision;p.route_revision=fpga_->identity.plan_id;}
   for(const auto&point:m->poses)p.points.push_back({point.pose.position.x,point.pose.position.y});send(arb1::Channel::State,p,meta(arb1::mono_ns(),rclcpp::Time(m->header.stamp).nanoseconds()));}));
  timer_=create_wall_timer(5ms,[this]{tick();});
 }
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Bridge>());}catch(const std::exception&e){std::fprintf(stderr,"ROS IPC stopped: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
