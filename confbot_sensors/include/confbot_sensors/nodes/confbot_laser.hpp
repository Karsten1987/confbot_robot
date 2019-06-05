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

#ifndef CONFBOT_SENSORS__NODES__CONFBOT_LASER_HPP_
#define CONFBOT_SENSORS__NODES__CONFBOT_LASER_HPP_

#include <memory>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace confbot_sensors
{
namespace nodes
{

class ConfbotLaser : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ConfbotLaser(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~ConfbotLaser() = default;

  void publish();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

private:
  rclcpp::Clock clock_;
  sensor_msgs::msg::LaserScan msg_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nodes
}  // namespace confbot_sensors

#endif  // CONFBOT_SENSORS__NODES__CONFBOT_LASER_HPP_
