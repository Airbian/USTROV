cmake_minimum_required(VERSION 3.8)
project(ustrov_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ustrov_control_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 节点入口和桥接器实现编译为一个可执行程序。
add_executable(px4_actuator_bridge
  src/main.cpp
  src/px4_actuator_bridge.cpp
)
target_include_directories(px4_actuator_bridge
  PRIVATE
    include
)
ament_target_dependencies(px4_actuator_bridge
  rclcpp
  ustrov_control_msgs
  px4_msgs
  std_srvs
)

# PX4 原始传感器 -> 标准 ROS 2 消息，只读，不涉及执行器控制。
add_executable(px4_sensor_bridge
  src/px4_sensor_bridge_main.cpp
  src/px4_sensor_bridge.cpp
)
target_include_directories(px4_sensor_bridge
  PRIVATE
    include
)
ament_target_dependencies(px4_sensor_bridge
  rclcpp
  px4_msgs
  sensor_msgs
)

install(TARGETS
  px4_actuator_bridge
  px4_sensor_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# README 随包安装，容器内也能直接查看接口和测试步骤。
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
