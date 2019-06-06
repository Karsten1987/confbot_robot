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

#ifndef CONFBOT_DRIVER__NODES__CONFBOT_DRIVER_HPP_
#define CONFBOT_DRIVER__NODES__CONFBOT_DRIVER_HPP_

#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <cmath>

#include "confbot_msgs/action/move_command.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

namespace confbot_driver
{
namespace nodes
{

struct RobotPosition
{
  float x = 0.0f;
  float y = -2.0f;
  float heading = 0.0f;
  float distance_traveled_ = 0.0f;

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
    transform.translation.z = 0.2;
    transform.rotation.w = cy * cr * cp + sy * sr * sp;
    transform.rotation.x = cy * sr * cp - sy * cr * sp;
    transform.rotation.y = cy * cr * sp + sy * sr * cp;
    transform.rotation.z = sy * cr * cp - cy * sr * sp;
  }

  void reset_odometry()
  {
    distance_traveled_ = 0.0f;
  }
};

class ConfbotDriver : public rclcpp::Node
{
  using MoveCommand = confbot_msgs::action::MoveCommand;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<MoveCommand>;

public:
  explicit ConfbotDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~ConfbotDriver() = default;

protected:
  void update_robot_position(float vel_lin, float vel_ang);

  void update_odometry();

  void update_velocity(std::shared_ptr<geometry_msgs::msg::Twist> twist_msg);

  rclcpp_action::GoalResponse handle_goal(
    const std::array<uint8_t, 16> & uuid, std::shared_ptr<const MoveCommand::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ServerGoalHandle> goal_handle);

  void update_feedback(
    const std::shared_ptr<ServerGoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle);

private:
  RobotPosition robot_position_;

  rclcpp::Clock clock_;

  geometry_msgs::msg::TransformStamped msg_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr odometry_timer_;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_subscriber_;

  std::shared_ptr<rclcpp_action::Server<MoveCommand>> action_server_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Time action_start_time_;

  std::mutex cmd_vel_mutex_;
  std::unique_lock<std::mutex> cmd_vel_lock_;
};

}  // namespace nodes
}  // namespace confbot_driver

#endif  // CONFBOT_DRIVER__NODES__CONFBOT_DRIVER_HPP_
