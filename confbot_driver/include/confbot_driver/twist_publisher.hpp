// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONFBOT_DRIVER__TWIST_PUBLISHER_HPP_
#define CONFBOT_DRIVER__TWIST_PUBLISHER_HPP_

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "geometry_msgs/msg/twist.hpp"

namespace confbot_driver
{

class TwistPublisher : public rclcpp::Node
{
public:
  TwistPublisher()
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

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), publish_message);
  }

  void init()
  {
    // Setup callback for changes to parameters.
    auto parameter_change_cb =
      [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto parameter : parameters) {
          RCLCPP_INFO(get_logger(),
            "set parameter \"%s\" to \"%f\"",
            parameter.get_name().c_str(),
            parameter.as_double());
          if (parameter.get_name() == "speed") {
            speed_ = parameter.as_double();
          }
        }
        return result;
      };
    this->register_param_change_callback(parameter_change_cb);
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
