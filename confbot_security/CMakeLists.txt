cmake_minimum_required(VERSION 3.5)
project(confbot_security)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sros2_cmake REQUIRED)

ros2_secure_node(
  NODES
  /confbot_driver
  /confbot_laser
  /launch_ros
  /robot_state_publisher
  /rviz
  /safe_zone_publisher
  /twist_publisher
)

ament_package()
