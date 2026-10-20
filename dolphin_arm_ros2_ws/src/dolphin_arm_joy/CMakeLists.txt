cmake_minimum_required(VERSION 3.8)
project(dolphin_arm_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(JOY_NODES
  joy_inspector
  joy_to_arm
  joy_to_gripper
  joy_to_velocity
)

foreach(node ${JOY_NODES})
  add_executable(${node} src/${node}.cpp)
endforeach()

ament_target_dependencies(joy_inspector
  rclcpp
  sensor_msgs
)

ament_target_dependencies(joy_to_arm
  rclcpp
  sensor_msgs
  trajectory_msgs
)

# joy_to_gripper 用的是 std_msgs::msg::Float64MultiArray
ament_target_dependencies(joy_to_gripper
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(joy_to_velocity
  rclcpp
  trajectory_msgs
  geometry_msgs
)

install(TARGETS ${JOY_NODES}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
