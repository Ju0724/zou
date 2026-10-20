cmake_minimum_required(VERSION 3.8)
project(dolphin_arm_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---- LibSerial ----
# 原来硬编码 /usr/local/include + /usr/local/lib，放到 /usr 下的系统安装会找不到。
find_library(LIBSERIAL_LIBRARY NAMES serial libserial)
find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h)
if(NOT LIBSERIAL_LIBRARY OR NOT LIBSERIAL_INCLUDE_DIR)
  message(FATAL_ERROR
    "找不到 LibSerial。请先安装（例如 `sudo apt install libserial-dev`），"
    "或从 https://github.com/crayzeewulf/libserial 编译安装。")
endif()

set(BRIDGE_NODES
  serial_bridge
  joystick_gripper
  joystick_arm
)

foreach(node ${BRIDGE_NODES})
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE ${LIBSERIAL_INCLUDE_DIR})
  target_link_libraries(${node} ${LIBSERIAL_LIBRARY})
endforeach()

ament_target_dependencies(serial_bridge
  rclcpp
)

ament_target_dependencies(joystick_gripper
  rclcpp
  sensor_msgs
)

ament_target_dependencies(joystick_arm
  rclcpp
  sensor_msgs
)

install(TARGETS ${BRIDGE_NODES}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
