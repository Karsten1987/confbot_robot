#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

struct RobotPosition
{
  float x = 0.0f;
  float y = -2.0f;
  float heading = 0.0f;

  void to_transform(geometry_msgs::msg::Transform & transform)
  {
    float cy = cos(heading * 0.5);
    float sy = sin(heading * 0.5);
    float cr = cos(0 * 0.5);
    float sr = sin(0 * 0.5);
    float cp = cos(0 * 0.5);
    float sp = sin(0 * 0.5);

    transform.translation.x = x;
    transform.translation.y = y;
    transform.rotation.w = cy * cr * cp + sy * sr * sp;
    transform.rotation.x = cy * sr * cp - sy * cr * sp;
    transform.rotation.y = cy * cr * sp + sy * sr * cp;
    transform.rotation.z = sy * cr * cp - cy * sr * sp;
  }
};

class ConfbotDriver : public rclcpp::Node
{
public:
  explicit ConfbotDriver()
  : Node("confbot_driver")
  {}

  void init()
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
    timer_ = this->create_wall_timer(100ms, std::bind(&ConfbotDriver::update_odometry, this));

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", std::bind(&ConfbotDriver::update_position, this, std::placeholders::_1));
  }

  void update_odometry() {
    geometry_msgs::msg::TransformStamped msg;
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    msg.header.stamp = clock->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    robot_position_.to_transform(msg.transform);

    tf_broadcaster_->sendTransform(msg);
  }

  void update_position(std::shared_ptr<geometry_msgs::msg::Twist> twist_msg)
  {
    robot_position_.heading += twist_msg->angular.z;
    robot_position_.x += 2.0 * cos(robot_position_.heading) * twist_msg->linear.x;
    robot_position_.y += 2.0 * sin(robot_position_.heading) * twist_msg->angular.z;
  }

private:

  RobotPosition robot_position_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ConfbotDriver>();
  node->init();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
