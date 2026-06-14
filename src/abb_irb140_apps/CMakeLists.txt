cmake_minimum_required(VERSION 3.8)
project(abb_irb140_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

add_executable(move_home
  src/move_home.cpp
)

ament_target_dependencies(move_home
  rclcpp
  moveit_ros_planning_interface
)

add_executable(robot_controller_test
  src/robot_controller_test.cpp
  src/robot_controller.cpp
)

ament_target_dependencies(robot_controller_test
  rclcpp
  moveit_ros_planning_interface
)

add_executable(pick_and_place_demo
  src/pick_and_place_demo.cpp
  src/robot_controller.cpp
)

ament_target_dependencies(pick_and_place_demo
  rclcpp
  moveit_ros_planning_interface
)

install(
  TARGETS
    move_home
    robot_controller_test
    pick_and_place_demo
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
