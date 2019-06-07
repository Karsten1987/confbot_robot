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
#include "confbot_sensors/nodes/confbot_laser.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "sensor_msgs/msg/laser_scan.hpp"

#define DEG2RAD M_PI / 180.0

using namespace std::chrono_literals;

namespace confbot_sensors
{
namespace nodes
{

ConfbotLaser::ConfbotLaser(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("confbot_laser", options),
  clock_(RCL_ROS_TIME)
{}

void ConfbotLaser::publish()
{
  static auto distance = 1.0f;
  for (size_t i = 0; i < msg_.ranges.size(); ++i) {
    msg_.ranges[i] = distance / cos((msg_.angle_min + i * msg_.angle_increment));
  }
  msg_.header.stamp = clock_.now();

  pub_->publish(msg_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConfbotLaser::on_configure(const rclcpp_lifecycle::State &)
{
  msg_ = sensor_msgs::msg::LaserScan();
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  timer_ = this->create_wall_timer(
    50ms, std::bind(&ConfbotLaser::publish, this));

  msg_.header.frame_id = "laser_frame";

  msg_.angle_increment = 5 * DEG2RAD;
  msg_.angle_min = -25 * DEG2RAD;
  msg_.angle_max = 25 * DEG2RAD;
  msg_.ranges.resize((msg_.angle_max - msg_.angle_min) / msg_.angle_increment);
  msg_.range_min = 0.0f;
  msg_.range_max = 10.0f;

  RCLCPP_DEBUG(get_logger(), "angle inc:\t%f", msg_.angle_increment);
  RCLCPP_DEBUG(get_logger(), "scan size:\t%zu", msg_.ranges.size());
  RCLCPP_DEBUG(get_logger(), "scan time increment: \t%f", msg_.time_increment);
  RCLCPP_INFO(get_logger(), "laser is configured");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConfbotLaser::on_activate(const rclcpp_lifecycle::State &)
{
  pub_->on_activate();

  RCUTILS_LOG_INFO_NAMED(get_name(), "laser is activated");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConfbotLaser::on_deactivate(const rclcpp_lifecycle::State &)
{
  pub_->on_deactivate();

  RCUTILS_LOG_INFO_NAMED(get_name(), "laser is deactivated");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConfbotLaser::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();

  RCUTILS_LOG_INFO_NAMED(get_name(), "laser node is cleaning up");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
}  // namespace nodes
}  // namespace confbot_sensors

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(confbot_sensors::nodes::ConfbotLaser)
