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

#ifndef CONFBOT_DRIVER__CONFBOT_DRIVER_HPP_
#define CONFBOT_DRIVER__CONFBOT_DRIVER_HPP_

#include <chrono>
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
    transform.translation.z = 0.2;
    transform.rotation.w = cy * cr * cp + sy * sr * sp;
    transform.rotation.x = cy * sr * cp - sy * cr * sp;
    transform.rotation.y = cy * cr * sp + sy * sr * cp;
    transform.rotation.z = sy * cr * cp - cy * sr * sp;
  }
};

class ConfbotDriver : public rclcpp::Node
{
  using MoveCommand = confbot_msgs::action::MoveCommand;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<MoveCommand>;

public:
  ConfbotDriver()
  : Node("confbot_driver"), cmd_vel_lock_(cmd_vel_mutex_, std::defer_lock)
  {}

  void init()
  {
    clock_ = rclcpp::Clock(RCL_ROS_TIME);
    msg_.header.stamp = clock_.now();
    msg_.header.frame_id = "odom";
    msg_.child_frame_id = "base_link";

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
    odometry_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ConfbotDriver::update_odometry, this));

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", std::bind(&ConfbotDriver::update_velocity, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<MoveCommand>(
      shared_from_this(),
      "move_command",
      std::bind(
        &ConfbotDriver::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &ConfbotDriver::handle_cancel, this, std::placeholders::_1),
      std::bind(
        &ConfbotDriver::handle_accepted, this, std::placeholders::_1));
  }

protected:
  void reset_speed()
  {
    vel_lin_ = 0.0f;
    vel_ang_ = 0.0f;
    distance_traveled_ = 0.0f;
  }

  void update_odometry()
  {
    robot_position_.heading += vel_ang_;
    robot_position_.x += 2.0 * cos(robot_position_.heading) * vel_lin_;
    robot_position_.y += 2.0 * sin(robot_position_.heading) * vel_lin_;
    robot_position_.to_transform(msg_.transform);

    tf_broadcaster_->sendTransform(msg_);
  }

  void update_velocity(std::shared_ptr<geometry_msgs::msg::Twist> twist_msg)
  {
    auto sub_lock = std::unique_lock<std::mutex>(cmd_vel_mutex_, std::try_to_lock);
    if (sub_lock.owns_lock()) {
      vel_lin_ = twist_msg->linear.x;
      vel_ang_ = twist_msg->angular.z;
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const std::array<uint8_t, 16> & uuid, std::shared_ptr<const MoveCommand::Goal> goal)
  {
    RCLCPP_INFO(
      get_logger(),
      "Got goal request with duration %d, linear velocity %f and angular velocity %f",
      goal->duration, goal->linear_velocity, goal->angular_velocity);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Got request to cancel goal");
    (void)goal_handle;
    reset_speed();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void update_feedback(
    const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    distance_traveled_ += vel_lin_;

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveCommand::Result>();

    auto now = clock_.now();
    auto time_elapsed = now - action_start_time_;
    if (time_elapsed >= goal->duration) {
      result->distance_traveled = distance_traveled_;
      goal_handle->succeed(result);
      feedback_timer_->cancel();
      reset_speed();
      cmd_vel_lock_.unlock();
      return;
    }

    auto feedback = std::make_shared<MoveCommand::Feedback>();
    feedback->distance_traveled = distance_traveled_;
    feedback->time_elapsed = time_elapsed;

    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "Publish Feedback");
  }

  void handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    std::function<void()> fnc =
      std::bind(&ConfbotDriver::update_feedback, this, goal_handle);

    cmd_vel_lock_.lock();

    feedback_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), fnc);

    vel_lin_ = goal_handle->get_goal()->linear_velocity;
    vel_ang_ = goal_handle->get_goal()->angular_velocity;

    action_start_time_ = clock_.now();
  }

private:
  float vel_lin_ = 0.0f;
  float vel_ang_ = 0.0f;
  float distance_traveled_ = 0.0f;

  geometry_msgs::msg::TransformStamped msg_;
  RobotPosition robot_position_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr odometry_timer_;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_subscriber_;

  std::shared_ptr<rclcpp_action::Server<MoveCommand>> action_server_;
  rclcpp::Clock clock_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Time action_start_time_;

  std::mutex cmd_vel_mutex_;
  std::unique_lock<std::mutex> cmd_vel_lock_;
};

}  // namespace confbot_driver

#endif  // CONFBOT_DRIVER__CONFBOT_DRIVER_HPP_
