// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <inttypes.h>
#include <memory>

#include "confbot_msgs/action/move_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using MoveCommand = confbot_msgs::action::MoveCommand;

using namespace std::chrono_literals;

void feedback_callback(
  rclcpp_action::ClientGoalHandle<MoveCommand>::SharedPtr,
  const std::shared_ptr<const MoveCommand::Feedback> feedback,
  rclcpp::Logger logger)
{
  RCLCPP_INFO(
    logger, "Confbot traveled %f so far in %d seconds",
    feedback->distance_traveled, feedback->time_elapsed.sec);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("confbot_actionclient");
  auto action_client = rclcpp_action::create_client<MoveCommand>(node, "move_command");

  if (!action_client->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate a goal
  auto goal_msg = MoveCommand::Goal();
  goal_msg.duration.sec = 10;
  goal_msg.duration.nanosec = 0u;
  goal_msg.linear_velocity = 0.1;
  goal_msg.angular_velocity = 0.1;

  RCLCPP_INFO(node->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<MoveCommand>::SendGoalOptions();
  // workaround to pass more objects, i. e. the logger, to the function callback
  send_goal_options.feedback_callback = std::bind(
    feedback_callback, std::placeholders::_1, std::placeholders::_2, node->get_logger());
  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<MoveCommand>::SharedPtr goal_handle = goal_handle_future.get();
  if (goal_handle == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<MoveCommand>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return 1;
  }

  RCLCPP_INFO(node->get_logger(), "result received");
  RCLCPP_INFO(
    node->get_logger(), "robot traveled %f meters", wrapped_result.result->distance_traveled);

  action_client.reset();
  rclcpp::shutdown();
  return 0;
}
