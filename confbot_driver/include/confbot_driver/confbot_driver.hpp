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
#include <string>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

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
public:
  ConfbotDriver()
  : Node("confbot_driver")
  {}

  void init()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    msg_.header.stamp = clock->now();
    msg_.header.frame_id = "odom";
    msg_.child_frame_id = "base_link";

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ConfbotDriver::update_odometry, this));

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", std::bind(&ConfbotDriver::update_position, this, std::placeholders::_1));
  }

  void update_odometry()
  {
    robot_position_.heading += vel_ang_;
    robot_position_.x += 2.0 * cos(robot_position_.heading) * vel_lin_;
    robot_position_.y += 2.0 * sin(robot_position_.heading) * vel_lin_;
    robot_position_.to_transform(msg_.transform);

    tf_broadcaster_->sendTransform(msg_);
  }

  void update_position(std::shared_ptr<geometry_msgs::msg::Twist> twist_msg)
  {
    vel_lin_ = twist_msg->linear.x;
    vel_ang_ = twist_msg->angular.z;
  }

private:
  float vel_lin_ = 0.0f;
  float vel_ang_ = 0.0f;

  geometry_msgs::msg::TransformStamped msg_;
  RobotPosition robot_position_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_subscriber_;
};

}  // namespace confbot_driver

#endif  // CONFBOT_DRIVER__CONFBOT_DRIVER_HPP_
