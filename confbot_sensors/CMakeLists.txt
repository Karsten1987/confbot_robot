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
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
install(
  DIRECTORY include/
  DESTINATION include)

add_executable(
  confbot_laser
  src/confbot_laser.cpp)
ament_target_dependencies(
  confbot_laser
  "rclcpp"
  "rclcpp_lifecycle"
  "rcutils"
  "sensor_msgs")
install(
  TARGETS confbot_laser
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
