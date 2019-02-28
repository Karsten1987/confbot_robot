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

#include "confbot_driver/confbot_driver.hpp"
#include "confbot_driver/twist_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto confbot_driver = std::make_shared<confbot_driver::ConfbotDriver>();
  confbot_driver->init();
  auto twist_publisher = std::make_shared<confbot_driver::TwistPublisher>();
  twist_publisher->init();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(confbot_driver);
  exe.add_node(twist_publisher);
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
