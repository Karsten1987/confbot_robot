#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TwistPublisher : public rclcpp::Node
{
public:
  explicit TwistPublisher()
  : Node("twist_publisher")
  {
    msg_ = std::make_shared<geometry_msgs::msg::Twist>();

    auto publish_message =
      [this]() -> void
      {
        msg_->linear.x = 0.1;
        msg_->angular.z = 0.1;
        pub_->publish(msg_);
      };

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", custom_qos_profile);

    timer_ = this->create_wall_timer(100ms, publish_message);
  }

private:
  std::shared_ptr<geometry_msgs::msg::Twist> msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TwistPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
