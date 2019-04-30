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

#include "confbot_driver/nodes/confbot_driver.hpp"


using namespace std::chrono_literals;

namespace confbot_driver
{
namespace nodes
{

ConfbotDriver::ConfbotDriver(rclcpp::NodeOptions options)
: Node("confbot_driver", options),
  clock_(RCL_ROS_TIME),
  cmd_vel_lock_(cmd_vel_mutex_, std::defer_lock)
{
  msg_.header.stamp = clock_.now();
  msg_.header.frame_id = "odom";
  msg_.child_frame_id = "base_link";

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(
    this->get_node_topics_interface());
  odometry_timer_ = this->create_wall_timer(
    100ms, std::bind(&ConfbotDriver::update_odometry, this));

  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", std::bind(&ConfbotDriver::update_velocity, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<MoveCommand>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "move_command",
    std::bind(
      &ConfbotDriver::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ConfbotDriver::handle_cancel, this, std::placeholders::_1),
    std::bind(
      &ConfbotDriver::handle_accepted, this, std::placeholders::_1));
}

void
ConfbotDriver::reset_speed()
{
  vel_lin_ = 0.0f;
  vel_ang_ = 0.0f;
  distance_traveled_ = 0.0f;
}

void
ConfbotDriver::update_odometry()
{
  robot_position_.heading += vel_ang_;
  robot_position_.x += 2.0 * cos(robot_position_.heading) * vel_lin_;
  robot_position_.y += 2.0 * sin(robot_position_.heading) * vel_lin_;
  robot_position_.to_transform(msg_.transform);

  tf_broadcaster_->sendTransform(msg_);
}

void
ConfbotDriver::update_velocity(std::shared_ptr<geometry_msgs::msg::Twist> twist_msg)
{
  auto sub_lock = std::unique_lock<std::mutex>(cmd_vel_mutex_, std::try_to_lock);
  if (sub_lock.owns_lock()) {
    vel_lin_ = twist_msg->linear.x;
    vel_ang_ = twist_msg->angular.z;
  }
}

rclcpp_action::GoalResponse
ConfbotDriver::handle_goal(
  const std::array<uint8_t, 16> & uuid, std::shared_ptr<const MoveCommand::Goal> goal)
{
  RCLCPP_INFO(
    get_logger(),
    "Got goal request with duration %d, linear velocity %f and angular velocity %f",
    goal->duration, goal->linear_velocity, goal->angular_velocity);
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ConfbotDriver::handle_cancel(
  const std::shared_ptr<ServerGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Got request to cancel goal");
  (void)goal_handle;
  reset_speed();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ConfbotDriver::update_feedback(
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

void
ConfbotDriver::handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle)
{
  std::function<void()> fnc =
    std::bind(&ConfbotDriver::update_feedback, this, goal_handle);

  cmd_vel_lock_.lock();

  feedback_timer_ = this->create_wall_timer(1s, fnc);

  vel_lin_ = goal_handle->get_goal()->linear_velocity;
  vel_ang_ = goal_handle->get_goal()->angular_velocity;

  action_start_time_ = clock_.now();
}

}  // namespace nodes
}  // namespace confbot_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(confbot_driver::nodes::ConfbotDriver)
