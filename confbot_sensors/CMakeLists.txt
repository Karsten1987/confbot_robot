cmake_minimum_required(VERSION 3.5)
project(confbot_sensors)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  confbot_laser_node
  SHARED
  src/nodes/confbot_laser.cpp
)
target_include_directories(confbot_laser_node PUBLIC include)
ament_target_dependencies(
  confbot_laser_node
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "rcutils"
  "sensor_msgs")
rclcpp_components_register_nodes(confbot_laser_node "confbot_sensors::nodes::ConfbotLaser")

add_executable(
  confbot_laser
  src/confbot_laser_main.cpp
)
target_include_directories(confbot_laser PUBLIC include)
target_link_libraries(
  confbot_laser
  confbot_laser_node
)
install(
  TARGETS confbot_laser_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS confbot_laser
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
