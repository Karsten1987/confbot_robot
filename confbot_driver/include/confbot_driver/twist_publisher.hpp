#ifndef CONFBOT_DRIVER__TWIST_PUBLISHER_HPP_
#define CONFBOT_DRIVER__TWIST_PUBLISHER_HPP_

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace confbot_driver
{

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
        msg_->linear.x = speed_;
        msg_->angular.z = speed_;
        pub_->publish(msg_);
      };

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", custom_qos_profile);

    timer_ = this->create_wall_timer(100ms, publish_message);
  }

  void init() {
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(shared_from_this());
    while (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.")
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...")
    }

    // Setup callback for changes to parameters.
    param_sub_ = parameters_client_->on_parameter_event(
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
      {
        for (auto & new_parameter : event->new_parameters) {
          RCLCPP_INFO(get_logger(),
            "set new parameter \"%s\" to \"%f\"",
            new_parameter.name.c_str(),
            new_parameter.value.double_value);
          if (new_parameter.name == "speed") {
            speed_ = new_parameter.value.double_value;
          }
        }
        for (auto & changed_parameter : event->changed_parameters) {
          RCLCPP_INFO(get_logger(),
            "changed parameter \"%s\" to \"%f\"",
            changed_parameter.name.c_str(),
            changed_parameter.value.double_value);
          if (changed_parameter.name == "speed") {
            speed_ = changed_parameter.value.double_value;
          }
        }
      });
  }

private:
  geometry_msgs::msg::Twist::SharedPtr msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::SyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;

  float speed_ = 0.1f;
};

}  // namespace confbot_driver

#endif  // CONFBOT_DRIVER__TWIST_PUBLISHER_HPP_
