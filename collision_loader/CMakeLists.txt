cmake_minimum_required(VERSION 3.8)
project(collision_loader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit REQUIRED)

add_executable(collision_loader_node src/add_collisions.cpp)
add_executable(human_movement_node src/human_movement.cpp)
add_executable(hybrid_node src/hybrid.cpp)

ament_target_dependencies(collision_loader_node rclcpp moveit_ros_planning_interface moveit_ros_move_group moveit_msgs moveit_servo)
ament_target_dependencies(human_movement_node rclcpp moveit_ros_planning_interface moveit_ros_move_group moveit_msgs moveit_servo)
ament_target_dependencies(hybrid_node rclcpp moveit_ros_planning_interface moveit_ros_move_group moveit_msgs moveit_servo moveit)

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
install(TARGETS
  collision_loader_node 
  human_movement_node
  hybrid_node
  DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
