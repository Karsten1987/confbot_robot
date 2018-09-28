#include "confbot_driver/twist_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<confbot_driver::TwistPublisher>();
  node->init();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
