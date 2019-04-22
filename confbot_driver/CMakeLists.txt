cmake_minimum_required(VERSION 3.5)
project(confbot_driver)

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
find_package(confbot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(
  confbot_driver_node
  SHARED
  src/nodes/confbot_driver.cpp
)
target_include_directories(confbot_driver_node PUBLIC include)
ament_target_dependencies(
  confbot_driver_node
  "confbot_msgs"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "tf2_ros"
)
rclcpp_components_register_nodes(confbot_driver_node "confbot_driver::nodes::ConfbotDriver")

add_library(
  twist_publisher_node
  SHARED
  src/nodes/twist_publisher.cpp
)
target_include_directories(twist_publisher_node PUBLIC include)
ament_target_dependencies(
  twist_publisher_node
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "tf2_ros"
  "geometry_msgs"
)
rclcpp_components_register_nodes(twist_publisher_node "confbot_driver::nodes::TwistPublisher")

add_executable(
  confbot_driver
  src/confbot_driver_main.cpp
)
target_include_directories(confbot_driver PUBLIC include)
target_link_libraries(
  confbot_driver
  confbot_driver_node
)

add_executable(
  twist_publisher
  src/twist_publisher_main.cpp
)
target_include_directories(twist_publisher PUBLIC include)
target_link_libraries(
  twist_publisher
  twist_publisher_node
)

add_executable(
  confbot_all
  src/confbot_all.cpp
)
target_include_directories(confbot_all PUBLIC include)
target_link_libraries(
  confbot_all
  confbot_driver_node
  twist_publisher_node
)

add_executable(
  confbot_actionclient
  src/confbot_actionclient.cpp
)
ament_target_dependencies(
  confbot_actionclient
  "confbot_msgs"
  "rclcpp"
  "rclcpp_action"
)

install(
  TARGETS confbot_driver_node twist_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS confbot_driver twist_publisher confbot_all confbot_actionclient
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
