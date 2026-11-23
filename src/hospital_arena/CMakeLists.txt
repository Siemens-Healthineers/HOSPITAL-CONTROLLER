# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.8)
project(hospital_arena)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hospital_interfaces REQUIRED)

add_executable(joint_bridge_node src/joint_bridge_node.cpp)
target_compile_features(joint_bridge_node PUBLIC cxx_std_20)
ament_target_dependencies(joint_bridge_node
  rclcpp
  sensor_msgs
  hospital_interfaces
)

install(TARGETS joint_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
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
