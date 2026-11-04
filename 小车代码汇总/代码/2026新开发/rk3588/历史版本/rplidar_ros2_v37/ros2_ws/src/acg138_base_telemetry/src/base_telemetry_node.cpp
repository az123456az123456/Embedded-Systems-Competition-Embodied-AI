#include "acg138_base_telemetry/odometry.hpp"
#include "acg138_base_telemetry/rx_stamp.hpp"
#include <cstdio>
#include <vector>
#include <stdexcept>
extern "C" {
#include "imu_data.h"
#include "sensor_data.h"
}
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
using namespace std::chrono_literals;
class BaseTelemetry final:public rclcpp::Node {
 public:
 BaseTelemetry():Node("base_telemetry"){
  acg138::OdometryConfig c;
  const auto interface=declare_parameter<std::string>("interface","eth0");
  c.circumference_m=declare_parameter("wheel_circumference_m",c.circumference_m);
  c.cpr=declare_parameter("encoder_ticks_per_turn",c.cpr);c.track_m=declare_parameter("track_m",c.track_m);
  const auto mask=declare_parameter<int>("left_wheel_mask",10);if(mask<1||mask>14)throw std::runtime_error("left_wheel_mask out of range");c.left_mask=mask;
  const auto signs=declare_parameter<std::vector<std::int64_t>>("encoder_signs",{-1,1,-1,1});
  if(signs.size()!=4)throw std::runtime_error("four encoder_signs required");
  for(unsigned i=0;i<4;i++){if(signs[i]!=-1&&signs[i]!=1)throw std::runtime_error("encoder signs +/-1 required");c.signs[i]=static_cast<int>(signs[i]);}
  c.max_wheel_mps=declare_parameter("max_wheel_mps",1.5);c.max_gap_s=declare_parameter("max_encoder_gap_s",.75);
  odom_=std::make_unique<acg138::WheelOdometry>(c);
  yaw_sign_=declare_parameter("imu_yaw_sign",1.);if(yaw_sign_!=1.&&yaw_sign_!=-1.)throw std::runtime_error("imu_yaw_sign must +/-1");
  odom_frame_=declare_parameter<std::string>("odom_frame","odom");base_frame_=declare_parameter<std::string>("base_frame","base_link");imu_frame_=declare_parameter<std::string>("imu_frame","imu_link");
  direct_=declare_parameter("publish_direct_odom",false);
  wheel_pub_=create_publisher<nav_msgs::msg::Odometry>("wheel/odometry",rclcpp::QoS(10));
  if(direct_){odom_pub_=create_publisher<nav_msgs::msg::Odometry>("odom",rclcpp::QoS(10));tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);}
  imu_pub_=create_publisher<sensor_msgs::msg::Imu>("imu/data",rclcpp::SensorDataQoS());
  diag_pub_=create_publisher<std_msgs::msg::String>("base/diagnostics",rclcpp::QoS(1).transient_local());
  sensor_init(&sensor_);imu_init(&imu_);
  sensor_fd_=sensor_open_socket(interface.c_str());if(sensor_fd_<0)throw std::runtime_error("SNS1 raw socket: "+std::string(std::strerror(errno))+" (needs host eth0 and CAP_NET_RAW)");
  char actual[16]={};imu_fd_=imu_open_socket(interface.c_str(),actual);if(imu_fd_<0){close(sensor_fd_);sensor_fd_=-1;throw std::runtime_error("IMU9 raw socket: "+std::string(std::strerror(errno)));}
  int timestamp_on=1;
  if(setsockopt(sensor_fd_,SOL_SOCKET,SO_TIMESTAMPNS,&timestamp_on,sizeof(timestamp_on))||setsockopt(imu_fd_,SOL_SOCKET,SO_TIMESTAMPNS,&timestamp_on,sizeof(timestamp_on)))throw std::runtime_error("kernel SO_TIMESTAMPNS is required for non-rejuvenated odometry timestamps");
  if(has_parameter("use_sim_time")&&get_parameter("use_sim_time").as_bool())throw std::runtime_error("base telemetry requires live system clock, not use_sim_time");
  timer_=create_wall_timer(5ms,[this]{poll();});diag_timer_=create_wall_timer(1s,[this]{diagnostics();});
  RCLCPP_INFO(get_logger(),"Read-only SNS1/IMU9 on %s; CPR=%.1f, sign=(-,+,-,+) configurable; no motor transmission",interface.c_str(),c.cpr);
 }
 ~BaseTelemetry()override{if(sensor_fd_>=0)close(sensor_fd_);if(imu_fd_>=0)close(imu_fd_);}
 private:
 struct Packet {ssize_t size=-1;std::optional<acg138::RxStamp> stamp;};
 Packet receive(int fd,unsigned char* data,std::size_t capacity){
  alignas(cmsghdr) unsigned char ancillary[CMSG_SPACE(sizeof(timespec))]={};
  iovec io{data,capacity};msghdr message{};message.msg_iov=&io;message.msg_iovlen=1;message.msg_control=ancillary;message.msg_controllen=sizeof(ancillary);
  Packet p;p.size=recvmsg(fd,&message,MSG_DONTWAIT);if(p.size<0)return p;
  const auto realtime=rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
  p.stamp=acg138::receive_stamp(message,realtime,imu_clock_ms());if(!p.stamp)++timestamp_bad_;
  return p;
 }
 void poll(){
  unsigned char data[2048];
  for(unsigned i=0;i<64;i++){
   const auto packet=receive(sensor_fd_,data,sizeof(data));const auto n=packet.size;if(n<0)break;if(!packet.stamp)continue;
   sensor_sample sample{};if(!sensor_decode(data,static_cast<std::size_t>(n),&sample)){++sensor_bad_;continue;}
   if(!sensor_update(&sensor_,&sample,packet.stamp->mono_ms))continue;
   if(sensor_.restarts!=restarts_){restarts_=sensor_.restarts;odom_->freeze("FPGA_restarted_restart_SLAM_and_odometry");}
   std::array<std::int32_t,4> counts{};for(unsigned j=0;j<4;j++)counts[j]=sample.position[j];
   if(odom_->update(counts,sample.uptime_ms,sample.sequence,sample.invert_mask,sample.illegal_mask))publishOdometry(packet.stamp->realtime_ns);
  }
  for(unsigned i=0;i<64;i++){
   const auto packet=receive(imu_fd_,data,sizeof(data));const auto n=packet.size;if(n<0)break;if(!packet.stamp)continue;
   imu_sample sample{};if(!imu_decode(data,static_cast<std::size_t>(n),&sample)){++imu_bad_;continue;}
   if(!imu_update(&imu_,&sample,packet.stamp->mono_ms)||!imu_.bias_valid||imu_.calibrating||!imu_fresh(&imu_,imu_clock_ms()))continue;
   sensor_msgs::msg::Imu out;out.header.stamp=rclcpp::Time(packet.stamp->realtime_ns-std::int64_t(sample.ag_age)*1000000LL);out.header.frame_id=imu_frame_;
   out.orientation.w=1.;out.orientation_covariance[0]=-1.; // no absolute heading or magnetic calibration asserted
   out.angular_velocity.x=(sample.gyro[0]-imu_.bias[0])*IMU_PI/180.;
   out.angular_velocity.y=(sample.gyro[1]-imu_.bias[1])*IMU_PI/180.;
   out.angular_velocity.z=(sample.gyro[2]-imu_.bias[2])*IMU_PI/180.*yaw_sign_;
   out.linear_acceleration.x=sample.accel[0]*IMU_G;out.linear_acceleration.y=sample.accel[1]*IMU_G;out.linear_acceleration.z=sample.accel[2]*IMU_G;
   for(int j:{0,4,8}){out.angular_velocity_covariance[j]=.0004;out.linear_acceleration_covariance[j]=.04;}
   imu_pub_->publish(out);
  }
 }
 void publishOdometry(std::int64_t sample_ns){
  const auto &s=odom_->state();const auto stamp=rclcpp::Time(sample_ns);
  if(have_stamp_&&stamp.nanoseconds()<=last_stamp_){odom_->freeze("ROS_clock_changed_restart_SLAM");return;}
  have_stamp_=true;last_stamp_=stamp.nanoseconds();
  nav_msgs::msg::Odometry out;out.header.stamp=stamp;out.header.frame_id=odom_frame_;out.child_frame_id=base_frame_;
  out.pose.pose.position.x=s.x;out.pose.pose.position.y=s.y;out.pose.pose.orientation.z=std::sin(s.yaw*.5);out.pose.pose.orientation.w=std::cos(s.yaw*.5);
  out.twist.twist.linear.x=s.v;out.twist.twist.angular.z=s.w;
  // Conservative skid-steer wheel uncertainty; laser SLAM supplies map correction.
  for(int i:{0,7,35})out.pose.covariance[i]=.1;
  for(int i:{14,21,28})out.pose.covariance[i]=1000.;
  out.twist.covariance[0]=.01;out.twist.covariance[7]=.02;out.twist.covariance[35]=.04;
  for(int i:{14,21,28})out.twist.covariance[i]=1000.;
  wheel_pub_->publish(out);
  if(direct_){odom_pub_->publish(out);geometry_msgs::msg::TransformStamped transform;transform.header=out.header;transform.child_frame_id=base_frame_;transform.transform.translation.x=s.x;transform.transform.translation.y=s.y;transform.transform.rotation=out.pose.pose.orientation;tf_->sendTransform(transform);}
 }
 void diagnostics(){
  std_msgs::msg::String out;std::ostringstream s;
  s<<"{\"encoder_fresh\":"<<(sensor_link_fresh(&sensor_,imu_clock_ms())?"true":"false")<<",\"odom_frozen\":"<<(odom_->state().frozen?"true":"false")<<",\"odom_reason\":\""<<odom_->state().reason<<"\",\"imu_bias_valid\":"<<(imu_.bias_valid?"true":"false")<<",\"sensor_rejected\":"<<sensor_bad_<<",\"imu_rejected\":"<<imu_bad_<<",\"timestamp_rejected\":"<<timestamp_bad_<<",\"read_only\":true}";
  out.data=s.str();diag_pub_->publish(out);
 }
 int sensor_fd_=-1,imu_fd_=-1;unsigned restarts_=0;std::uint64_t sensor_bad_=0,imu_bad_=0,timestamp_bad_=0;std::int64_t last_stamp_=0;bool direct_=false,have_stamp_=false;double yaw_sign_=1;
 sensor_state sensor_{};imu_state imu_{};std::string odom_frame_,base_frame_,imu_frame_;
 std::unique_ptr<acg138::WheelOdometry> odom_;std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
 rclcpp::TimerBase::SharedPtr timer_,diag_timer_;
 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub_,odom_pub_;
 rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<BaseTelemetry>());rclcpp::shutdown();return 0;}catch(const std::exception &e){std::fprintf(stderr,"base_telemetry: %s\n",e.what());rclcpp::shutdown();return 1;}}
