# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.8)
project(hospital_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hospital_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(door_controller_component SHARED
  src/door_controller_node.cpp
)
target_compile_features(door_controller_component PUBLIC cxx_std_20)
target_include_directories(door_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(door_controller_component
  hospital_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(door_controller_component
  "hospital_controller::DoorControllerNode"
)

install(TARGETS door_controller_component
  EXPORT door_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(ct_scanner_controller_component SHARED
  src/ct_scanner_controller_node.cpp
)
target_compile_features(ct_scanner_controller_component PUBLIC cxx_std_20)
target_include_directories(ct_scanner_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ct_scanner_controller_component
  hospital_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(ct_scanner_controller_component
  "hospital_controller::CtScannerControllerNode"
)

install(TARGETS ct_scanner_controller_component
  EXPORT ct_scanner_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(lab_controller_component SHARED
  src/lab_controller_node.cpp
)
target_compile_features(lab_controller_component PUBLIC cxx_std_20)
target_include_directories(lab_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lab_controller_component
  hospital_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(lab_controller_component
  "hospital_controller::LabControllerNode"
)

install(TARGETS lab_controller_component
  EXPORT lab_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(operating_room_controller_component SHARED
  src/operating_room_controller_node.cpp
)
target_compile_features(operating_room_controller_component PUBLIC cxx_std_20)
target_include_directories(operating_room_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(operating_room_controller_component
  hospital_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(operating_room_controller_component
  "hospital_controller::OperatingRoomControllerNode"
)

install(TARGETS operating_room_controller_component
  EXPORT operating_room_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lab_result test/test_lab_result.cpp)
  target_compile_features(test_lab_result PUBLIC cxx_std_20)
  target_link_libraries(test_lab_result lab_controller_component)
  ament_add_gtest(test_door_stopped test/test_door_stopped.cpp)
  target_compile_features(test_door_stopped PUBLIC cxx_std_20)
  target_link_libraries(test_door_stopped door_controller_component)
  ament_add_gtest(test_ct_retraction test/test_ct_retraction.cpp)
  target_compile_features(test_ct_retraction PUBLIC cxx_std_20)
  target_link_libraries(test_ct_retraction ct_scanner_controller_component)
  ament_add_gtest(test_operating_room_state test/test_operating_room_state.cpp)
  target_compile_features(test_operating_room_state PUBLIC cxx_std_20)
  target_link_libraries(test_operating_room_state operating_room_controller_component)
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

ament_export_targets(
  door_controller_component
  ct_scanner_controller_component
  lab_controller_component
  operating_room_controller_component
  HAS_LIBRARY_TARGET
)
ament_export_dependencies(
  hospital_interfaces rclcpp rclcpp_action rclcpp_components std_msgs
)

ament_package()
