cmake_minimum_required(VERSION 3.5)
project(confbot_simulation)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch world
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS ${PROJECT_NAME}/urdf_spawner.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
