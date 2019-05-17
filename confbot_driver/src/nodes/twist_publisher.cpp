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

#include <memory>
#include <string>
#include <vector>

#include "confbot_driver/nodes/twist_publisher.hpp"

using namespace std::chrono_literals;

namespace confbot_driver
{
namespace nodes
{

TwistPublisher::TwistPublisher(rclcpp::NodeOptions options)
: Node("twist_publisher", options)
{
  auto publish_message =
    [this]() -> void
    {
      msg_.linear.x = speed_;
      msg_.angular.z = speed_;
      pub_->publish(msg_);
    };

  rclcpp::QoS qos(rclcpp::KeepLast(7));
  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  timer_ = this->create_wall_timer(100ms, publish_message);

  this->declare_parameter<double>("speed", speed_);
  // Setup callback for changes to parameters.
  auto parameter_change_cb =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name() == "speed") {
          if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter.get_type()) {
            RCLCPP_INFO(this->get_logger(),
              "cannot delete '%s' as it is a required parameter",
              parameter.get_name().c_str()
            );
            result.successful = false;
          } else {
            RCLCPP_INFO(this->get_logger(),
              "set parameter '%s' to '%f'",
              parameter.get_name().c_str(),
              parameter.as_double());
            speed_ = parameter.as_double();
          }
        }
      }
      return result;
    };
  this->set_on_parameters_set_callback(parameter_change_cb);
}

}  // namespace nodes
}  // namespace confbot_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(confbot_driver::nodes::TwistPublisher)
