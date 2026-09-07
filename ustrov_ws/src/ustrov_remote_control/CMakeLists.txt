# Copyright (C) 2023 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

cmake_minimum_required(VERSION 3.8)
project(ustrov_remote_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ustrov_common REQUIRED)
find_package(ustrov_control_msgs REQUIRED)

set(component_name joystick_component)
add_library(${component_name}
  SHARED
  src/joystick/joystick_component.cpp
  src/joystick/joystick_component_params.cpp
)
target_include_directories(${component_name}
  PRIVATE src/joystick
)
ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  rcl_interfaces
  sensor_msgs
  ustrov_common
  ustrov_control_msgs
)
rclcpp_components_register_node(${component_name}
  PLUGIN "ustrov_remote_control::joystick::JoyStick"
  EXECUTABLE joystick_node
)

add_executable(keyboard_input_node
  src/keyboard/keyboard_input_node.cpp
)
ament_target_dependencies(keyboard_input_node
  rclcpp
  sensor_msgs
)

install(TARGETS ${component_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS keyboard_input_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
