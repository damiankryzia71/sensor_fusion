#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomTfNode : public rclcpp::Node {
public:
  OdomTfNode() : Node("odom_tf"){
    odom_topic_ = declare_parameter<std::string>("odom_topic");
    parent_ = declare_parameter<std::string>("parent_frame", "x500_vision_0/odom");
    child_  = declare_parameter<std::string>("child_frame", "x500_vision_0/base_footprint");

    if (!get_parameter("odom_topic", odom_topic_)) {
      RCLCPP_ERROR(get_logger(), "Required parameter odom_topic is missing. Aborting...");
      throw std::runtime_error("Missing required parameter");
    }
    if (!get_parameter("parent_frame", parent_)) {
      RCLCPP_ERROR(get_logger(), "Required parameter parent_frame is missing. Aborting...");
      throw std::runtime_error("Missing required parameter");
    }
    if (!get_parameter("child_frame", child_)) {
      RCLCPP_ERROR(get_logger(), "Required parameter child_frame is missing. Aborting...");
      throw std::runtime_error("Missing required parameter");
    }

    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(50),
      std::bind(&OdomTfNode::cb, this, std::placeholders::_1));
  }
private:
  void cb(const nav_msgs::msg::Odometry::ConstSharedPtr m){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = m->header.stamp;
    t.header.frame_id = parent_;
    t.child_frame_id = child_;
    t.transform.translation.x = m->pose.pose.position.x;
    t.transform.translation.y = m->pose.pose.position.y;
    t.transform.translation.z = m->pose.pose.position.z;
    t.transform.rotation = m->pose.pose.orientation;
    br_->sendTransform(t);
  }

  std::string odom_topic_, parent_, child_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfNode>());
  rclcpp::shutdown();
  return 0;
}