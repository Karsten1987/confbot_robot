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

#ifndef CONFBOT_DRIVER__NODES__TWIST_PUBLISHER_HPP_
#define CONFBOT_DRIVER__NODES__TWIST_PUBLISHER_HPP_

#include <chrono>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "geometry_msgs/msg/twist.hpp"

namespace confbot_driver
{
namespace nodes
{

class TwistPublisher : public rclcpp::Node
{
public:
  explicit TwistPublisher(rclcpp::NodeOptions options = rclcpp::NodeOptions());

  virtual ~TwistPublisher() = default;

private:
  geometry_msgs::msg::Twist::SharedPtr msg_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::SyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;

  float speed_ = 0.1f;
};

}  // namespace nodes
}  // namespace confbot_driver

#endif  // CONFBOT_DRIVER__NODES__TWIST_PUBLISHER_HPP_
