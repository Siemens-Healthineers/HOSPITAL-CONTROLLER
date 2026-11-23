# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.8)
project(hospital_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hospital_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(robot_server_component SHARED
  src/robot_server_node.cpp
)
target_compile_features(robot_server_component PUBLIC cxx_std_20)
ament_target_dependencies(robot_server_component
  hospital_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  tf2_ros
  visualization_msgs
)
rclcpp_components_register_nodes(robot_server_component
  "hospital_robot::RobotServerNode"
)

# Robot-agnostic room-dwell + task activity logger.
add_library(activity_logger_component SHARED
  src/activity_logger_node.cpp
)
target_compile_features(activity_logger_component PUBLIC cxx_std_20)
ament_target_dependencies(activity_logger_component
  hospital_interfaces
  rclcpp
  rclcpp_components
  geometry_msgs
  std_srvs
)
rclcpp_components_register_nodes(activity_logger_component
  "hospital_robot::ActivityLoggerNode"
)

install(TARGETS robot_server_component activity_logger_component
  EXPORT robot_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_server test/test_robot_server.cpp TIMEOUT 120)
  target_compile_features(test_robot_server PUBLIC cxx_std_20)
  ament_target_dependencies(test_robot_server
    hospital_interfaces rclcpp rclcpp_action rclcpp_components
    geometry_msgs tf2_ros visualization_msgs
  )
  find_package(ament_lint_auto REQUIRED)
  # Licensing is checked repo-wide with `reuse lint` (see CONTRIBUTING.md).
  # C++ follows Google style, so clang-format replaces uncrustify (ROS style).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  set(ament_cmake_clang_format_CONFIG_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format")
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
