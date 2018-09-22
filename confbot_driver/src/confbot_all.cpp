#include "confbot_driver/confbot_driver.hpp"
#include "confbot_driver/twist_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto confbot_driver = std::make_shared<confbot_driver::ConfbotDriver>();
  auto twist_publisher = std::make_shared<confbot_driver::TwistPublisher>();
  confbot_driver->init();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(confbot_driver);
  exe.add_node(twist_publisher);
  exe.spin();

  rclcpp::shutdown();
  return 0;
}

