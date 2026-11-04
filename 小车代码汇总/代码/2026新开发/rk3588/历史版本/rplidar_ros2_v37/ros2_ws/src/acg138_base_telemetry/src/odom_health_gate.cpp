// Optional EKF output gate: stale input must not become a fresh-looking TF.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <stdexcept>
#include <cstdint>
#include <string>
#include <memory>
class OdomGate final:public rclcpp::Node {
 public:
 OdomGate():Node("odom_health_gate"){
  max_age_=declare_parameter("max_encoder_age_s",.25);
  if(!std::isfinite(max_age_)||max_age_<=0||max_age_>1.)throw std::runtime_error("invalid odometry max age");
  odom_frame_=declare_parameter<std::string>("odom_frame","odom");base_frame_=declare_parameter<std::string>("base_frame","base_link");
  tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);pub_=create_publisher<nav_msgs::msg::Odometry>("odom",10);
  wheel_=create_subscription<nav_msgs::msg::Odometry>("wheel/odometry",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
   if(m->header.frame_id==odom_frame_&&m->child_frame_id==base_frame_){wheel_ns_=rclcpp::Time(m->header.stamp).nanoseconds();have_wheel_=true;}
  });
  filtered_=create_subscription<nav_msgs::msg::Odometry>("odometry/filtered",10,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
   const auto now=get_clock()->now().nanoseconds(),stamp=rclcpp::Time(m->header.stamp).nanoseconds();
   if(stamp<=last_sent_ns_||!have_wheel_||now<wheel_ns_||double(now-wheel_ns_)*1e-9>max_age_||now<stamp||now-stamp>100000000LL||m->header.frame_id!=odom_frame_||m->child_frame_id!=base_frame_)return;
   const auto&p=m->pose.pose;const auto&q=p.orientation;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
   if(!std::isfinite(p.position.x)||!std::isfinite(p.position.y)||!std::isfinite(norm)||std::abs(norm-1.)>.01)return;
   last_sent_ns_=stamp;pub_->publish(*m);geometry_msgs::msg::TransformStamped t;t.header=m->header;t.child_frame_id=base_frame_;t.transform.translation.x=p.position.x;t.transform.translation.y=p.position.y;t.transform.rotation=q;tf_->sendTransform(t);
  });
 }
 private:double max_age_=.25;bool have_wheel_=false;std::int64_t wheel_ns_=0,last_sent_ns_=0;std::string odom_frame_,base_frame_;
 std::unique_ptr<tf2_ros::TransformBroadcaster>tf_;rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_,filtered_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<OdomGate>());rclcpp::shutdown();}
