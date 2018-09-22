#include "confbot_sensors/confbot_laser.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto laser_node = std::make_shared<confbot_sensors::ConfbotLaser>("confbot_laser");

  exe.add_node(laser_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
