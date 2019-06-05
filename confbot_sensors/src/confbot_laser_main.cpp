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

#include "confbot_sensors/nodes/confbot_laser.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto laser_node = std::make_shared<confbot_sensors::nodes::ConfbotLaser>();
  if (argc > 1) {
    if (strcmp(argv[1], "--activate") == 0) {
      fprintf(stderr, "activating laser node\n");
      laser_node->configure();
      laser_node->activate();
    }
  }

  exe.add_node(laser_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
