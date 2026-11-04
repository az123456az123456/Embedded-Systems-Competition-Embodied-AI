#include "acg138_fpga_map_bridge/map_core.hpp"
#include "acg138_fpga_map_bridge/nav2_wire.hpp"
#include "acg138_fpga_map_bridge/nav3_ethernet.hpp"
#include "acg138_fpga_map_bridge/nav3_map_latch.hpp"
#include "acg138_fpga_map_bridge/nav3_pose_schedule.hpp"
#include "acg138_fpga_map_bridge/navigation_heading.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "acg138_fpga_map_bridge/rear_clearance.hpp"
#include <acg138_nav_interfaces/srv/execution.hpp>
#include <time.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace acg138 {
namespace {
double planar_yaw(double x,double y,double z,double w){
  const double norm=x*x+y*y+z*z+w*w;
  if(!std::isfinite(norm)||std::abs(norm-1)>.001||std::abs(x)>.001||std::abs(y)>.001)
    throw std::runtime_error("map/base quaternion must be normalized and planar");
  return std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
}
void write_atomic(const std::string&path,const std::string&text){
  if(path.empty())return;
  const std::string tmp=path+".tmp."+std::to_string(getpid());
  {std::ofstream output(tmp,std::ios::binary|std::ios::trunc);if(!output||!(output<<text<<'\n'))throw std::runtime_error("cannot write map metadata: "+tmp);}
  if(std::rename(tmp.c_str(),path.c_str())){std::remove(tmp.c_str());throw std::runtime_error("cannot atomically replace map metadata: "+path);}
}
}
class MapBridge final : public rclcpp::Node {
  struct Job {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    Pose pose;
    std::optional<Pose> goal;
    uint64_t snapshot=0,goal_version=0,map_load_generation=0;int64_t goal_source_stamp_ns=0;
    bool nav3_load_authorized=false;
    rclcpp::Time pose_stamp{0,0,RCL_ROS_TIME};
  };
  using Execution=acg138_nav_interfaces::srv::Execution;
  struct Control {Execution::Request req;bool cancel=false;uint64_t generation=0;};
  std::optional<Control> control_; // one bounded slot, CANCEL replaces queued START
  std::atomic<uint64_t> control_generation_{0};
  std::atomic<bool> cancel_pending_{false},stopped_snapshot_{false};
  uint64_t last_start_request_=0,last_cancel_request_=0;
  bool execution_enabled_=false,reset_frozen_=false;
  rclcpp::Service<Execution>::SharedPtr start_service_,cancel_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_result_pub_,command_result_pub_,anchor_pub_;
  static uint64_t monotonic_ns(){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return uint64_t(t.tv_sec)*1000000000ull+t.tv_nsec;}
  static nav3::Header identity(const Execution::Request& r){nav3::Header h;h.session_id=r.session_id;h.run_generation=r.run_generation;h.goal_id=r.goal_id;h.map_epoch=r.map_epoch;h.map_revision=r.map_revision;h.localization_epoch=r.localization_epoch;h.plan_id=r.plan_id;return h;}
  static std::string goal_hex(const nav3::Header& h){std::ostringstream o;o<<std::hex<<std::setfill('0');for(auto b:h.goal_id)o<<std::setw(2)<<unsigned(b);return o.str();}
  static std::string identity_json(const nav3::Header& h){std::ostringstream o;o<<"\"session_id\":"<<h.session_id<<",\"run_generation\":"<<h.run_generation<<",\"goal_id\":\""<<goal_hex(h)<<"\",\"map_epoch\":"<<h.map_epoch<<",\"map_revision\":"<<h.map_revision<<",\"localization_epoch\":"<<h.localization_epoch<<",\"plan_id\":"<<h.plan_id;return o.str();}
  void enqueue(const std::shared_ptr<Execution::Request> q,std::shared_ptr<Execution::Response> response,bool cancel){
    std::lock_guard<std::mutex> lock(mutex_);const auto now=monotonic_ns();
    const bool fresh=q->valid_for_ms>0&&q->valid_for_ms<=500&&q->created_monotonic_ns<=now&&now-q->created_monotonic_ns<uint64_t(q->valid_for_ms)*1000000ull;
    if(dry_run_||!execution_enabled_||!fresh||!q->request_id||!q->session_id||!q->run_generation){response->status=4;response->message="disabled/dry-run/expired/invalid request";return;}
    auto& floor=cancel?last_cancel_request_:last_start_request_;
    if(q->request_id<=floor){response->status=4;response->message="duplicate or out-of-order local request";return;}
    if(control_&&!cancel){response->status=10;response->message="bounded control slot busy";return;}
    floor=q->request_id;
    if(cancel){++control_generation_;cancel_pending_=true;pending_.reset();goal_.reset();++goal_version_;}
    control_=Control{*q,cancel,control_generation_.load()};response->success=true;response->message="queued, NOT FPGA ACK; see /fpga/command_result";cv_.notify_one();
  }
  void process_control(nav3::Client& client,const Control& c){
    unsigned status=0;std::string error;
    try{const auto now=monotonic_ns();
      if(c.generation!=control_generation_||now<c.req.created_monotonic_ns||now-c.req.created_monotonic_ns>=uint64_t(c.req.valid_for_ms)*1000000ull)throw nav3::Rejected(nav3::Status::Stale,"queued command expired/cancelled");
      if(c.cancel){client.cancel(identity(c.req),c.req.reason);last_plan_.cells.clear();last_plan_error_="cancelled";nav_msgs::msg::Path empty;empty.header.frame_id=nav3_frame_;path_pub_->publish(empty);}
      else {nav3::StartTicket t;t.identity=identity(c.req);t.nav_epoch=c.req.nav_epoch;t.permit_seq=c.req.permit_seq;t.auth_epoch=c.req.auth_epoch;
        t.expires_at=std::chrono::steady_clock::now()+std::chrono::nanoseconds(uint64_t(c.req.valid_for_ms)*1000000ull-(now-c.req.created_monotonic_ns));client.start(t);}
    }catch(const nav3::Rejected& e){status=unsigned(e.status);error=e.what();}catch(const std::exception& e){status=16;error=e.what();}
    std_msgs::msg::String m;std::ostringstream j;j<<"{\"request_id\":"<<c.req.request_id<<",\"operation\":\""<<(c.cancel?"cancel":"start")<<"\",\"status\":"<<status<<",\"fpga_ack\":"<<(status==0?"true":"false")<<",\"error\":\""<<json_escape(error)<<"\","<<identity_json(identity(c.req))<<",\"nav_epoch\":"<<c.req.nav_epoch<<"}";m.data=j.str();command_result_pub_->publish(m);
  }
  CropConfig crop_;
  NavigationHeading navigation_heading_;
  std::string base_frame_,interface_,metadata_path_,host_proc_root_;
  std::string protocol_,nav3_frame_;
  bool dry_run_=true,enable_plan_=false;
  double map_timeout_=10,pose_timeout_=.25;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_sub_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr rear_scan_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_map_service_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_,state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::condition_variable cv_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr selected_map_;
  Nav3MapLatch map_latch_; // guarded by mutex_, including worker transitions
  uint64_t source_updates_after_request_=0,map_load_generation_=0;
  std::optional<LocalGrid> frozen_local_; // owner worker only
  std::string map_load_error_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_;
  std::optional<Job> pending_;
  bool stop_=false;
  uint64_t snapshots_=0,superseded_=0,goal_version_=0;
  uint64_t attempted_goal_=0;uint32_t attempted_map_=0;
  nav3::PlanResult last_plan_{};std::string last_plan_error_;
  std::thread worker_;
  uint64_t pose_stream_sent_=0,pose_stream_duplicates_=0,pose_stream_rejected_=0;
  bool nav3_stream_fault_=false;
  std::string pose_stream_error_;
  nav3::PoseSample current_pose(){
    const auto tf=tf_->lookupTransform(nav3_frame_,base_frame_,tf2::TimePointZero);
    const auto stamp=rclcpp::Time(tf.header.stamp,get_clock()->get_clock_type());
    const auto age=(now()-stamp).nanoseconds();
    if(age<0||age>250000000LL)throw std::runtime_error("TF source stale/future; no POSE");
    const auto mm=[](double x)->int32_t {const auto v=std::round(x*1000.0);if(!std::isfinite(v)||v<INT32_MIN||v>INT32_MAX)throw std::runtime_error("pose outside int32 mm");return int32_t(v);};
    nav3::PoseSample p;p.x_mm=mm(tf.transform.translation.x);p.y_mm=mm(tf.transform.translation.y);
    p.yaw_urad=navigation_heading_.microradians(planar_yaw(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w));
    p.source_stamp_ns=stamp.nanoseconds();p.source_age_ns=uint64_t(age);p.sampled_at=std::chrono::steady_clock::now();return p;
  }
  uint64_t plan_goal_stamp_=0,plan_goal_version_=0;
  double plan_goal_x_=0,plan_goal_y_=0;
  uint64_t logged_state_seq_=UINT64_MAX;
  void publish_complete_plan(const LocalGrid& local){
    std_msgs::msg::String result;std::ostringstream j;
    j<<"{"<<identity_json(last_plan_.identity)<<",\"nav_epoch\":"<<last_plan_.nav_epoch<<",\"status\":"<<unsigned(last_plan_.status)
     <<",\"path_count\":"<<last_plan_.cells.size()<<",\"goal_source_stamp_ns\":"<<plan_goal_stamp_<<",\"goal_version\":"<<plan_goal_version_
     <<",\"goal_x\":"<<plan_goal_x_<<",\"goal_y\":"<<plan_goal_y_<<",\"start_pose_seq\":"<<last_plan_.start_pose_seq
     <<",\"cycles\":"<<last_plan_.cycles<<",\"expanded\":"<<last_plan_.expanded
     <<",\"origin_mm\":["<<std::llround(local.origin_x*1000)<<','<<std::llround(local.origin_y*1000)
     <<"],\"bitmap_crc32\":"<<wire::crc32(local.blocked.data(),local.blocked.size())<<",\"cells\":[";
    for(size_t i=0;i<last_plan_.cells.size();++i){if(i)j<<',';j<<last_plan_.cells[i];}j<<"]}";
    result.data=j.str();plan_result_pub_->publish(result);write_atomic(metadata_path_+".plan.json",result.data);
    nav_msgs::msg::Path path;path.header.frame_id=nav3_frame_;path.header.stamp=now();
    for(auto id:last_plan_.cells){auto xy=local.center(id);geometry_msgs::msg::PoseStamped q;q.header=path.header;q.pose.position.x=xy.x;q.pose.position.y=xy.y;q.pose.orientation.w=1;path.poses.push_back(q);}
    path_pub_->publish(path);
  }
  uint64_t published_state_transport_=0;
  void publish_new_state(nav3::Client& client){
    if(client.state_transport_sequence()!=published_state_transport_)publish_state(client);
  }
  void publish_state(nav3::Client& client){
    published_state_transport_=client.state_transport_sequence();
    const auto v=client.state();std::ostringstream j;
    j<<"{\"protocol\":\"NAV3-v1\",\"available\":"<<(v?"true":"false")
      <<",\"navigation_heading_offset_rad\":"<<std::setprecision(17)<<navigation_heading_.offset()
      <<",\"stream_fault\":"<<(nav3_stream_fault_?"true":"false")<<",\"pose_target_hz\":50"
      <<",\"pose_sent\":"<<pose_stream_sent_<<",\"duplicate_tf_skipped\":"<<pose_stream_duplicates_
      <<",\"pose_rejected\":"<<pose_stream_rejected_<<",\"pose_error\":\""<<json_escape(pose_stream_error_)<<"\"";
    if(v&&(v->state_seq!=logged_state_seq_||((client.last_info().extra.capabilities&64u)&&(v->state==2||v->state==3)))){logged_state_seq_=v->state_seq;
      RCLCPP_INFO(get_logger(),"NAV3_STATE run=%llu plan=%llu epoch=%llu state=%u avoid=%u reason=%u flags=%u pose=(%d,%d,%d) age_ms=%u cursor=%u/%u remain_mm=%u permit_ms=%u auth_ms=%u",
        (unsigned long long)v->identity.run_generation,(unsigned long long)v->identity.plan_id,(unsigned long long)v->nav_epoch,unsigned(v->state),unsigned(v->avoid_stage),unsigned(v->reason),v->flags,v->x_mm,v->y_mm,v->yaw_urad,v->pose_age_ms,unsigned(v->cursor),unsigned(v->path_count),v->remaining_mm,v->permit_ms,v->authorization_ms);}
    if(v){const auto &x=*v;std::string goal;static const char hex[]="0123456789abcdef";for(auto b:x.identity.goal_id){goal+=hex[b>>4];goal+=hex[b&15];}static const char* names[]={"IDLE","PLANNING","FOLLOWING","AVOIDING","ARRIVED","CANCELLED"};
      j<<",\"state\":\""<<names[x.state]<<"\",\"avoid_stage\":"<<unsigned(x.avoid_stage)<<",\"reason\":"<<x.reason
        <<",\"session_id\":"<<x.identity.session_id<<",\"run_generation\":"<<x.identity.run_generation<<",\"goal_id\":\""<<goal<<"\""
        <<",\"map_epoch\":"<<x.identity.map_epoch<<",\"map_revision\":"<<x.identity.map_revision
        <<",\"localization_epoch\":"<<x.identity.localization_epoch<<",\"plan_id\":"<<x.identity.plan_id
        <<",\"nav_epoch\":"<<x.nav_epoch<<",\"state_seq\":"<<x.state_seq<<",\"pose_seq\":"<<x.pose_seq
        <<",\"pose_mm_urad\":["<<x.x_mm<<','<<x.y_mm<<','<<x.yaw_urad<<"],\"pose_age_ms\":"<<x.pose_age_ms
        <<",\"host_receipt_age_ms\":"<<client.state_receipt_age_ms()<<",\"state_stale\":"<<(client.state_receipt_age_ms()>250?"true":"false")<<",\"flags\":"<<x.flags
        <<",\"waypoint_index\":"<<x.cursor<<",\"path_count\":"<<x.path_count<<",\"remaining_mm\":"<<x.remaining_mm
        <<",\"permit_remaining_ms\":"<<x.permit_ms<<",\"authorization_remaining_ms\":"<<x.authorization_ms
        <<",\"active_crc32\":"<<x.active_crc32<<",\"device_ms\":"<<x.device_ms<<",\"owner\":"<<x.owner<<",\"frame_token\":"<<x.frame_token;
    }
    j<<",\"published_mono_ns\":"<<monotonic_ns()<<",\"sends_motor_commands\":false}";std_msgs::msg::String message;message.data=j.str();state_pub_->publish(message);
  }
  void tick(){
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;geometry_msgs::msg::PoseStamped::ConstSharedPtr goal;uint64_t goal_version;
    bool selected=false,frozen=false;uint64_t map_generation=0;
    {std::lock_guard<std::mutex>guard(mutex_);selected=protocol_=="nav3"&&bool(selected_map_);
      map=selected?selected_map_:latest_map_;goal=goal_;goal_version=goal_version_;
      frozen=map_latch_.state()==Nav3MapLatch::State::Frozen;map_generation=map_load_generation_;}
    if(!map)return;
    try{
      if(map->header.frame_id.empty())throw std::runtime_error("map frame_id is empty");
      const auto now=get_clock()->now();
      const double map_age=(now-rclcpp::Time(map->header.stamp,get_clock()->get_clock_type())).seconds();
      if(map_age<-.05||(!frozen&&map_timeout_>0&&map_age>map_timeout_))throw std::runtime_error("map timestamp is stale/future; no upload");
      const auto transform=tf_->lookupTransform(map->header.frame_id,base_frame_,tf2::TimePointZero);
      const double pose_age=(now-rclcpp::Time(transform.header.stamp,get_clock()->get_clock_type())).seconds();
      if(pose_age<-.05||pose_age>pose_timeout_)throw std::runtime_error("map->base_link TF is stale/future; no upload");
      const auto &t=transform.transform;Job job;job.map_load_generation=map_generation;job.map=map;job.nav3_load_authorized=selected;
      job.pose_stamp=rclcpp::Time(transform.header.stamp,get_clock()->get_clock_type());
      job.pose={t.translation.x,t.translation.y,planar_yaw(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)};
      if(goal){
        if(goal->header.frame_id!=map->header.frame_id)throw std::runtime_error("goal frame must equal OccupancyGrid frame");
        job.goal_source_stamp_ns=rclcpp::Time(goal->header.stamp,get_clock()->get_clock_type()).nanoseconds();job.goal=Pose{goal->pose.position.x,goal->pose.position.y,0};job.goal_version=goal_version;
      }
      {std::lock_guard<std::mutex>guard(mutex_);job.snapshot=++snapshots_;if(pending_)++superseded_;pending_=std::move(job);}
      cv_.notify_one();
    }catch(const std::exception&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"MAP_BRIDGE waiting: %s",e.what());}
  }
  void process(const Job&job,std::unique_ptr<wire::RawEthernet>&channel,std::unique_ptr<wire::Client>&client,nav3::Client* client3=nullptr){
    const auto began=std::chrono::steady_clock::now();
    {std::lock_guard<std::mutex> lock(mutex_);if(job.map_load_generation!=map_load_generation_)return;if(reset_frozen_){frozen_local_.reset();map_load_error_.clear();reset_frozen_=false;}}
    const auto &source=*job.map;const auto &q=source.info.origin.orientation;
    const double queued_pose_age=(now()-job.pose_stamp).seconds();
    const double queued_map_age=(now()-rclcpp::Time(source.header.stamp,get_clock()->get_clock_type())).seconds();
    if(queued_pose_age<-.05||queued_pose_age>pose_timeout_||queued_map_age<-.05||
       (!frozen_local_&&map_timeout_>0&&queued_map_age>map_timeout_))
      throw std::runtime_error("queued map/pose snapshot expired; no upload");
    Grid grid;grid.width=source.info.width;grid.height=source.info.height;grid.resolution=source.info.resolution;
    grid.origin={source.info.origin.position.x,source.info.origin.position.y,planar_yaw(q.x,q.y,q.z,q.w)};grid.cells=source.data;
    Pose footprint_pose=job.pose;
    if(protocol_=="nav3")footprint_pose.yaw=navigation_heading_.radians(job.pose.yaw);
    const LocalGrid local=frozen_local_?*frozen_local_:crop_and_inflate(grid,footprint_pose,crop_);
    nav_msgs::msg::OccupancyGrid message;message.header.frame_id=source.header.frame_id;message.header.stamp=now();
    message.info.resolution=float(local.resolution);message.info.width=kSide;message.info.height=kSide;
    message.info.origin.position.x=local.origin_x;message.info.origin.position.y=local.origin_y;message.info.origin.orientation.w=1;
    message.info.map_load_time=message.header.stamp;message.data=local.occupancy();local_pub_->publish(message);
    uint32_t generation=0;bool uploaded=false,state_verified=false;std::vector<uint16_t> path;std::string upload_error,plan_error;
    try{
      if(!dry_run_){
        if(client3 && job.nav3_load_authorized) {
          if(source.header.frame_id!=nav3_frame_)throw std::runtime_error("NAV3 map frame differs from bound frame");
          const auto mm=[](double m)->int32_t{
            const double value=std::round(m*1000.0);
            if(!std::isfinite(value)||value<INT32_MIN||value>INT32_MAX-6400)throw std::runtime_error("NAV3 map origin outside int32 millimetres");
            if(std::abs(value-m*1000.0)>0.001)throw std::runtime_error("NAV3 map origin must be exact millimetres");
            return int32_t(value);
          };
          bool first_upload=false;
          {std::lock_guard<std::mutex>guard(mutex_);
            if(!frozen_local_)first_upload=map_latch_.begin();}
          if(!frozen_local_&&!first_upload)throw std::runtime_error(map_load_error_.empty()?"map load is not authorized; no upload":map_load_error_);
          nav3::MapReceipt result;
          try {
            // Once committed, both bitmap AND geometry are frozen. This call is
            // then the client's exact-cache check; it cannot emit another BEGIN.
            result=client3->upload(local.blocked,{mm(local.origin_x),mm(local.origin_y),1});
            if(result.committed&&!frozen_local_){frozen_local_=local;std::lock_guard<std::mutex>guard(mutex_);map_latch_.commit();}
          } catch(const std::exception& e) {
            map_load_error_=e.what();{std::lock_guard<std::mutex>guard(mutex_);map_latch_.fail();}throw;
          }
          generation=result.revision;uploaded=result.committed;state_verified=result.state_verified;
          if(enable_plan_&&job.goal&&state_verified) {
            if(attempted_goal_!=job.goal_version || attempted_map_!=generation) {
              attempted_goal_=job.goal_version;attempted_map_=generation;last_plan_={};last_plan_error_.clear();
              try {
                const auto fresh_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(60);
                while(current_pose().source_stamp_ns==client3->last_pose_stamp()&&std::chrono::steady_clock::now()<fresh_deadline)
                  std::this_thread::sleep_for(std::chrono::milliseconds(1));
                const auto transform=tf_->lookupTransform(source.header.frame_id,base_frame_,tf2::TimePointZero);
                const auto stamp=rclcpp::Time(transform.header.stamp,get_clock()->get_clock_type());
                const auto age=(now()-stamp).nanoseconds();
                if(age<0 || age>250000000LL)throw std::runtime_error("planning TF expired/future; no POSE/GOAL sent");
                const auto &t=transform.transform;nav3::PoseSample pose;
                const auto to_mm=[](double m)->int32_t {const auto v=std::round(m*1000.0);if(!std::isfinite(v)||v<INT32_MIN||v>INT32_MAX)throw std::runtime_error("position outside int32 mm");return int32_t(v);};
                pose.x_mm=to_mm(t.translation.x);pose.y_mm=to_mm(t.translation.y);
                pose.yaw_urad=navigation_heading_.microradians(planar_yaw(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w));
                pose.source_stamp_ns=stamp.nanoseconds();pose.source_age_ns=uint64_t(age);pose.sampled_at=std::chrono::steady_clock::now();
                std::array<uint8_t,16> goal_id{};
                for(unsigned i=0;i<8;++i){goal_id[7-i]=uint8_t(client3->session()>>(i*8));goal_id[15-i]=uint8_t(job.goal_version>>(i*8));}
                last_plan_=client3->plan(goal_id,pose,to_mm(job.goal->x),to_mm(job.goal->y));
                plan_goal_stamp_=job.goal_source_stamp_ns;plan_goal_version_=job.goal_version;
                plan_goal_x_=job.goal->x;plan_goal_y_=job.goal->y;publish_complete_plan(local);
                if(last_plan_.status!=nav3::Status::Ok)last_plan_error_="FPGA PLAN_RESULT status="+std::to_string(unsigned(last_plan_.status));
              }catch(const std::exception& e){last_plan_error_=e.what();}
            }
            path=last_plan_.cells;plan_error=last_plan_error_;
          }
        }else if(!client3){
        if(!channel){channel=std::make_unique<wire::RawEthernet>(interface_=="auto"?wire::find_interface():interface_,host_proc_root_);client=std::make_unique<wire::Client>(*channel);}
        generation=client->upload(local.blocked);uploaded=true;
        if(enable_plan_&&job.goal){
          try{
            path=client->plan(local.blocked,local.cell(job.pose.x,job.pose.y),local.cell(job.goal->x,job.goal->y));
            if((now()-job.pose_stamp).seconds()>pose_timeout_){path.clear();throw std::runtime_error("planned path pose expired; not published");}
          }
          catch(const std::exception&e){plan_error=e.what();}
        }
        }
      }
    }catch(const std::exception&e){upload_error=e.what();client.reset();channel.reset();}
    if(!path.empty()){
      nav_msgs::msg::Path result;result.header=message.header;
      for(uint16_t id:path){const auto p=local.center(id);geometry_msgs::msg::PoseStamped point;point.header=result.header;point.pose.position.x=p.x;point.pose.position.y=p.y;point.pose.orientation.w=1;result.poses.push_back(point);}
      path_pub_->publish(result);
    }else{
      // A stale latched route must not survive a new failed/disabled PLAN.
      nav_msgs::msg::Path empty;empty.header=message.header;path_pub_->publish(empty);
    }
    const uint32_t checksum=wire::crc32(local.blocked.data(),local.blocked.size());
    const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    const auto stats=client?client->statistics():wire::Statistics{};
    uint64_t superseded,ignored_updates;std::string map_load_state;
    {std::lock_guard<std::mutex>guard(mutex_);superseded=superseded_;ignored_updates=source_updates_after_request_;map_load_state=map_latch_.name();}
    std::ostringstream json;json<<std::fixed<<std::setprecision(9)
      <<"{\"protocol\":\""<<(protocol_=="nav3"?"NAV3-v1":"NAV2-v1")<<"\",\"snapshot_id\":"<<job.snapshot
      <<",\"map_load_generation\":"<<job.map_load_generation
      <<",\"dry_run\":"<<(dry_run_?"true":"false")<<",\"uploaded\":"<<(uploaded?"true":"false")
      <<",\"generation\":"<<generation<<",\"bitmap_crc32\":"<<checksum
      <<",\"local_odometry\":"<<((client3&&(client3->last_info().extra.capabilities&64u))?"true":"false")
      <<",\"session_id\":"<<(client3?client3->session():0)<<",\"map_epoch\":"<<(client3?1:0)<<",\"map_revision\":"<<generation<<",\"state_crc_verified\":"<<(state_verified?"true":"false")
      <<",\"frame_id\":\""<<json_escape(source.header.frame_id)<<"\",\"width\":64,\"height\":64"
      <<",\"source_map_stamp_ns\":"<<rclcpp::Time(source.header.stamp,get_clock()->get_clock_type()).nanoseconds()
      <<",\"pose_stamp_ns\":"<<job.pose_stamp.nanoseconds()<<",\"published_stamp_ns\":"<<now().nanoseconds()
      <<",\"static_map_allowed\":"<<((map_timeout_==0||frozen_local_)?"true":"false")
      <<",\"nav3_map_mode\":\""<<(client3?"manual_frozen":"not_applicable")<<"\",\"map_load_state\":\""<<map_load_state<<"\""
      <<",\"source_updates_after_request\":"<<ignored_updates
      <<",\"map_replace_requires_cancel_and_restart\":"<<(client3&&!execution_enabled_?"true":"false")
      <<",\"resolution_m\":"<<local.resolution<<",\"origin_m\":["<<local.origin_x<<','<<local.origin_y<<"]"
      <<",\"origin_yaw_rad\":0,\"pose_m_rad\":["<<job.pose.x<<','<<job.pose.y<<','<<job.pose.yaw<<']'
      <<",\"navigation_heading_offset_rad\":"<<std::setprecision(17)<<(protocol_=="nav3"?navigation_heading_.offset():0.)<<std::setprecision(9)
      <<",\"navigation_pose_m_rad\":["<<job.pose.x<<','<<job.pose.y<<','<<footprint_pose.yaw<<']'
      <<",\"nav3_plan_id\":"<<(client3?last_plan_.identity.plan_id:0)<<",\"nav3_plan_status\":"<<(client3?unsigned(last_plan_.status):0)
      <<",\"free_cells\":"<<local.free_cells<<",\"path_cells\":"<<path.size()
      <<",\"transport_frames_sent\":"<<(client3?client3->statistics().sent:stats.sent)<<",\"transport_retries\":"<<stats.retries
      <<",\"superseded_snapshots\":"<<superseded<<",\"elapsed_ms\":"<<elapsed
      <<",\"upload_error\":\""<<json_escape(upload_error)<<"\",\"plan_error\":\""<<json_escape(plan_error)
      <<"\",\"geometry_resides_on_rk\":"<<(protocol_=="nav3"?"false":"true")<<",\"sends_motor_commands\":false}";
    std_msgs::msg::String status;status.data=json.str();metadata_pub_->publish(status);write_atomic(metadata_path_,status.data);
    RCLCPP_INFO(get_logger(),"MAP_BRIDGE snapshot=%llu uploaded=%d gen=%u crc32=%08x origin=(%.3f,%.3f) res=%.3f free=%u path=%zu elapsed_ms=%.2f%s%s",
      static_cast<unsigned long long>(job.snapshot),int(uploaded),generation,checksum,local.origin_x,local.origin_y,local.resolution,local.free_cells,path.size(),elapsed,upload_error.empty()?"":" error=",upload_error.c_str());
  }
  void run(){
    if(protocol_=="nav3") {runNav3();return;}
    std::unique_ptr<wire::RawEthernet> channel;std::unique_ptr<wire::Client> client;
    for(;;){Job job;{std::unique_lock<std::mutex>guard(mutex_);cv_.wait(guard,[this]{return stop_||pending_.has_value();});if(stop_)return;job=std::move(*pending_);pending_.reset();}
      try{process(job,channel,client);}catch(const std::exception&e){RCLCPP_ERROR(get_logger(),"MAP_BRIDGE rejected snapshot: %s",e.what());}
    }
  }
  void runNav3(){
    // Stage3: PLAN only after a STATE-confirmed pose. There is no START/PERMIT sender.
    try {
      if(dry_run_) {
        std::unique_ptr<wire::RawEthernet> channel;std::unique_ptr<wire::Client> client;
        for(;;){Job job;{std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return stop_||pending_.has_value();});if(stop_)return;job=std::move(*pending_);pending_.reset();}
          try{process(job,channel,client);}catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"NAV3 preview rejected: %s",e.what());}}
      }
      if(!dry_run_) {
        nav3::RawEthernet channel(interface_=="auto"?wire::find_interface():interface_,host_proc_root_,execution_enabled_);
        nav3::Client client(channel,250,[this](const std::string& text){RCLCPP_INFO(get_logger(),"%s",text.c_str());},execution_enabled_);
        client.handshake(nav3::random_session(),1,nav3_frame_);
        std::ostringstream json;json<<"{\"protocol\":\"NAV3-v1\",\"stage\":1,\"handshake\":true,\"uploaded\":false,\"session_id\":"
          <<client.session()<<",\"anchor_id\":"<<client.anchor()<<",\"nav_epoch\":"<<client.last_ack().nav_epoch
          <<",\"device_ms\":"<<client.last_ack().device_ms<<",\"sends_motor_commands\":false}";
        std_msgs::msg::String m;m.data=json.str();metadata_pub_->publish(m);write_atomic(metadata_path_,m.data);
        RCLCPP_INFO(get_logger(),"NAV3 handshake complete; map preview only until explicit /fpga/load_map after mapping finished and vehicle stopped");
        std::unique_ptr<wire::RawEthernet> unused_channel;std::unique_ptr<wire::Client> unused_client;
        auto next_pose=std::chrono::steady_clock::now(),next_state=next_pose,service_state_at=next_pose;
        const auto service_pose=[&]{
          const auto service_started=std::chrono::steady_clock::now();
          if(nav3_stream_fault_||!client.active_map().state_verified||service_started<next_pose)return;
          const auto state=client.state();
          if((client.last_info().extra.capabilities&64u)&&state&&(state->state==2||state->state==3||(state->state==1&&state->owner==1))){
            sensor_msgs::msg::LaserScan::ConstSharedPtr scan;{std::lock_guard<std::mutex> lock(mutex_);scan=rear_scan_;}
            next_pose=service_started+std::chrono::milliseconds(20);if(!scan)return;
            try{const auto stamp=rclcpp::Time(scan->header.stamp,get_clock()->get_clock_type());const auto age=(now()-stamp).nanoseconds();if(age<0||age>250000000)return;
              const auto t=tf_->lookupTransform(base_frame_,scan->header.frame_id,tf2::TimePointZero);
              if(std::hypot(t.transform.translation.x,t.transform.translation.y)>.02)return;
              const auto&q=t.transform.rotation;const double a=planar_yaw(q.x,q.y,q.z,q.w)-navigation_heading_.offset();
              const auto clear=rear_clearance(scan->ranges,scan->angle_min,scan->angle_increment,scan->range_min,scan->range_max,a);
              client.stream_rear(clear.travel_mm,clear.observed,stamp.nanoseconds(),uint64_t(age));
            }catch(const std::exception& e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Rear clearance unavailable: %s",e.what());}
            return;
          }
          try{
            const auto p=current_pose();
            if(p.source_stamp_ns<client.last_pose_stamp()){nav3_stream_fault_=true;throw std::runtime_error("TF source time reversed; explicit new mapping session required");}
            if(client.stream_pose(p)){++pose_stream_sent_;pose_stream_error_.clear();next_pose=nav3::next_pose_slot(next_pose,service_started);
            }else{++pose_stream_duplicates_;} // retry a duplicate next service tick, never re-age it
          }catch(const std::exception& e){++pose_stream_rejected_;pose_stream_error_=e.what();next_pose=std::chrono::steady_clock::now()+std::chrono::milliseconds(20);}
        };
        client.set_service([&]{service_pose();publish_new_state(client);if(std::chrono::steady_clock::now()>=service_state_at){publish_state(client);service_state_at=std::chrono::steady_clock::now()+std::chrono::milliseconds(100);}});
        client.set_interrupt([this]{if(cancel_pending_)throw std::runtime_error("CANCEL_PRIORITY_INTERRUPT");});
        for(;;){
          std::optional<Job> work;std::optional<Control> command;
          {std::unique_lock<std::mutex> lock(mutex_);cv_.wait_for(lock,std::chrono::milliseconds(2),[this]{return stop_||pending_.has_value()||control_.has_value();});if(stop_)return;
            if(control_){command=std::move(control_);control_.reset();cancel_pending_=false;}
            if(pending_){work=std::move(pending_);pending_.reset();}}
          if(command)process_control(client,*command);
          if(cancel_pending_)continue;
          client.pump();publish_new_state(client);stopped_snapshot_=client.stopped_for_map();
          if(client.active_map().state_verified&&!nav3_stream_fault_){
            try{client.refresh_anchor();}catch(const std::exception& e){if(!cancel_pending_)nav3_stream_fault_=true;pose_stream_error_=e.what();}
          }
          if(!nav3_stream_fault_&&work){
            try{process(*work,unused_channel,unused_client,&client);}catch(const std::exception& e){
              if(work->nav3_load_authorized){std::lock_guard<std::mutex>guard(mutex_);map_latch_.fail();map_load_error_=e.what();}
              RCLCPP_ERROR(get_logger(),"NAV3 map rejected: %s",e.what());}
          }
          if(cancel_pending_)continue;
          service_pose();client.pump();publish_new_state(client);stopped_snapshot_=client.stopped_for_map();
          if(frozen_local_&&!nav3_stream_fault_){
            try{if(auto replan=client.poll_replan()){last_plan_=*replan;last_plan_error_.clear();publish_complete_plan(*frozen_local_);}}
            catch(const std::exception& e){last_plan_error_=e.what();RCLCPP_WARN(get_logger(),"NAV3 replan display: %s",e.what());}
          }
          const auto state=client.state();
          if(!last_plan_.cells.empty() && (nav3_stream_fault_ || (state && state->state==5 &&
              state->identity.run_generation==last_plan_.identity.run_generation && state->identity.goal_id==last_plan_.identity.goal_id))){
            last_plan_.cells.clear();last_plan_error_="FPGA cancelled or bridge reference fault; path invalidated";
            nav_msgs::msg::Path empty;empty.header.frame_id=nav3_frame_;empty.header.stamp=now();path_pub_->publish(empty);
          }
          if(std::chrono::steady_clock::now()>=next_state){publish_state(client);
            const auto &i=client.anchor_info();std_msgs::msg::String a;std::ostringstream j;
            j<<"{\"session_id\":"<<client.session()<<",\"anchor_id\":"<<client.anchor()<<",\"device_ms\":"<<i.ack.device_ms<<",\"host_send_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(i.sent_at.time_since_epoch()).count()<<",\"host_receive_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(i.received_at.time_since_epoch()).count()
              <<",\"info_request_id\":"<<i.header.request_id<<",\"anchor_request_id\":"<<client.anchor_request_id()<<",\"anchor_ack_device_ms\":"<<client.anchor_ack().device_ms<<",\"anchor_ack_verified\":true}";a.data=j.str();anchor_pub_->publish(a);next_state=std::chrono::steady_clock::now()+std::chrono::milliseconds(100);}
        }
      }
      RCLCPP_INFO(get_logger(),"NAV3 dry_run: no socket opened and no handshake transmitted");
    }catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"NAV3 stopped: %s",e.what());}
    std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return stop_;});
  }
public:
  MapBridge():Node("acg138_fpga_map_bridge"){
    const auto map_topic=declare_parameter<std::string>("map_topic","/map");
    base_frame_=declare_parameter<std::string>("base_frame","base_link");interface_=declare_parameter<std::string>("interface","auto");
    protocol_=declare_parameter<std::string>("protocol","nav2");
    navigation_heading_=NavigationHeading(declare_parameter<double>("navigation_heading_offset_rad",0.));
    nav3_frame_=declare_parameter<std::string>("nav3_frame","map");
    if(protocol_!="nav2"&&protocol_!="nav3")throw std::runtime_error("protocol must be nav2 or nav3");
    const char*host_proc=std::getenv("ACG138_HOST_PROC_ROOT");
    host_proc_root_=declare_parameter<std::string>("host_proc_root",host_proc?host_proc:"/proc");
    rear_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan){std::lock_guard<std::mutex> lock(mutex_);rear_scan_=std::move(scan);});
    metadata_path_=declare_parameter<std::string>("metadata_path","/tmp/acg138_ros2_fpga_map.json");
    execution_enabled_=declare_parameter<bool>("enable_execution",false);
    dry_run_=declare_parameter<bool>("dry_run",true);enable_plan_=declare_parameter<bool>("enable_plan",false);
    const bool owner=declare_parameter<bool>("exclusive_nav2_owner",false);
    if(!dry_run_&&!owner)throw std::runtime_error("live upload requires exclusive_nav2_owner:=true after stopping legacy NAV2 planner");
    if(!dry_run_&&wire::legacy_navigation_running(host_proc_root_))throw std::runtime_error("legacy robot-navigation is running; no live ROS NAV2 upload allowed");
    crop_.resolution=declare_parameter<double>("local_resolution",crop_.resolution);crop_.footprint_radius=declare_parameter<double>("footprint_radius",crop_.footprint_radius);
    crop_.safety_margin=declare_parameter<double>("safety_margin",crop_.safety_margin);crop_.free_threshold=declare_parameter<int>("free_threshold",25);
    crop_.body_width=declare_parameter<double>("body_width",crop_.body_width);
    crop_.body_front=declare_parameter<double>("body_front",crop_.body_front);
    crop_.body_rear=declare_parameter<double>("body_rear",crop_.body_rear);
    crop_.self_footprint_exemption=declare_parameter<bool>("self_footprint_exemption",crop_.self_footprint_exemption);
    if(protocol_=="nav3"&&std::abs(crop_.resolution-.1)>1e-9)throw std::runtime_error("NAV3 requires 0.1m cells");
    map_timeout_=declare_parameter<double>("map_timeout_s",10);pose_timeout_=declare_parameter<double>("pose_timeout_s",.25);
    const double period=declare_parameter<double>("upload_period_s",1);
    if(!std::isfinite(period)||period<.5||period>10||!std::isfinite(map_timeout_)||map_timeout_<0||!std::isfinite(pose_timeout_)||pose_timeout_<=0||pose_timeout_>.25)
      throw std::runtime_error("invalid timing parameter; upload_period_s must be 0.5..10 seconds");
    local_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/fpga/local_costmap",rclcpp::QoS(1).reliable().transient_local());
    path_pub_=create_publisher<nav_msgs::msg::Path>("/fpga/planned_path",rclcpp::QoS(1).reliable().transient_local());
    plan_result_pub_=create_publisher<std_msgs::msg::String>("/fpga/plan_result",rclcpp::QoS(1).reliable().transient_local());
    command_result_pub_=create_publisher<std_msgs::msg::String>("/fpga/command_result",rclcpp::QoS(8).reliable());
    anchor_pub_=create_publisher<std_msgs::msg::String>("/fpga/clock_anchor",rclcpp::QoS(1).reliable());
    if(protocol_=="nav3"&&execution_enabled_){
      start_service_=create_service<Execution>("/fpga/start",[this](const std::shared_ptr<Execution::Request> q,std::shared_ptr<Execution::Response> a){enqueue(q,a,false);});
      cancel_service_=create_service<Execution>("/fpga/cancel",[this](const std::shared_ptr<Execution::Request> q,std::shared_ptr<Execution::Response> a){enqueue(q,a,true);});
    }
    if(protocol_=="nav3")state_pub_=create_publisher<std_msgs::msg::String>("/fpga/state",rclcpp::QoS(1).reliable().transient_local());
    metadata_pub_=create_publisher<std_msgs::msg::String>("/fpga/map_metadata",rclcpp::QoS(1).reliable().transient_local());
    tf_=std::make_unique<tf2_ros::Buffer>(get_clock());listener_=std::make_unique<tf2_ros::TransformListener>(*tf_);
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic,rclcpp::QoS(1).reliable().transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr value){std::lock_guard<std::mutex>guard(mutex_);if(selected_map_)++source_updates_after_request_;latest_map_=std::move(value);});
    if(protocol_=="nav3")load_map_service_=create_service<std_srvs::srv::Trigger>("/fpga/load_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        std::lock_guard<std::mutex>guard(mutex_);
        if(dry_run_){response->message="dry_run preview: no upload authorized";return;}
        if(!latest_map_){response->message="no ROS map available";return;}
        const double age=(now()-rclcpp::Time(latest_map_->header.stamp,get_clock()->get_clock_type())).seconds();
        if(latest_map_->header.frame_id!=nav3_frame_||age<-.05||(map_timeout_>0&&age>map_timeout_)){
          response->message="map frame/time invalid; no snapshot selected";return;}
        if(execution_enabled_&&map_latch_.state()!=Nav3MapLatch::State::Preview){
          if(!stopped_snapshot_||control_||cancel_pending_||!map_latch_.reset_stopped(true)){response->message="cancel + fresh stopped state required before reload";return;}
          reset_frozen_=true;
        }
        if(!map_latch_.request()){response->message="map already requested/frozen/failed; cancel task and stop, then restart bridge before loading another map";return;}
        ++map_load_generation_;selected_map_=latest_map_;pending_.reset();goal_.reset();++goal_version_;
        response->success=true;response->message="map snapshot selected; upload queued, not yet committed. Check map_metadata uploaded/state_crc_verified/FROZEN. FPGA retains its stopped-only gate.";
        RCLCPP_INFO(get_logger(),"NAV3 explicit map load queued; subsequent SLAM maps will not replace this snapshot");
      });
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/fpga/goal",rclcpp::QoS(1),[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr value){std::lock_guard<std::mutex>guard(mutex_);if(execution_enabled_&&(!stopped_snapshot_||control_||cancel_pending_)){RCLCPP_WARN(get_logger(),"GOAL rejected: stop/cancel current task first");return;}goal_=std::move(value);++goal_version_;});
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),[this]{tick();});
    worker_=std::thread([this]{run();});
    RCLCPP_INFO(get_logger(),"%s bridge dry_run=%d plan=%d fixed=64x64 res=%.3f; never sends motor commands",protocol_.c_str(),int(dry_run_),int(enable_plan_),crop_.resolution);
    RCLCPP_INFO(get_logger(),"NAV3 body heading = TF yaw + %.12f rad (normalized); XY, targets, TF and source age unchanged; restart required to change calibration",navigation_heading_.offset());
  }
  ~MapBridge()override{{std::lock_guard<std::mutex>guard(mutex_);stop_=true;pending_.reset();}cv_.notify_all();if(worker_.joinable())worker_.join();}
};
}  // namespace acg138
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<acg138::MapBridge>());}catch(const std::exception&e){std::fprintf(stderr,"acg138_fpga_map_bridge: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
