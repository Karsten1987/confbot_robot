#include "confbot_driver/confbot_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<confbot_driver::ConfbotDriver>();
  node->init();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
