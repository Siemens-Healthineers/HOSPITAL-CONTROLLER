# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.8)
project(hospital_rviz_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hospital_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# RViz panels are Qt widgets that declare Q_OBJECT, so they need MOC. Listing
# the headers in the target sources lets AUTOMOC pick them up.
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/hospital_panel.cpp
  src/door_control_widget.cpp
  src/ct_control_widget.cpp
  src/lab_control_widget.cpp
  src/collapsible_section.cpp
  src/ward_management_widget.cpp
  src/operating_room_control_widget.cpp
  src/recording_control_widget.cpp
  src/toggle_switch.cpp
  include/hospital_rviz_panel/hospital_panel.hpp
  include/hospital_rviz_panel/door_control_widget.hpp
  include/hospital_rviz_panel/ct_control_widget.hpp
  include/hospital_rviz_panel/lab_control_widget.hpp
  include/hospital_rviz_panel/collapsible_section.hpp
  include/hospital_rviz_panel/ward_management_widget.hpp
  include/hospital_rviz_panel/operating_room_control_widget.hpp
  include/hospital_rviz_panel/recording_control_widget.hpp
  include/hospital_rviz_panel/toggle_switch.hpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  rclcpp_action
  rviz_common
  pluginlib
  hospital_interfaces
  std_msgs
  std_srvs
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# Tell RViz where to find the panel plugin.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch rviz resource
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  target_compile_features(test_plugin_loading PUBLIC cxx_std_20)
  target_link_libraries(test_plugin_loading Qt5::Widgets)
  ament_target_dependencies(test_plugin_loading pluginlib rviz_common)
  add_dependencies(test_plugin_loading ${PROJECT_NAME})
  ament_add_gtest(test_ct_lift_stop test/test_ct_lift_stop.cpp)
  target_compile_features(test_ct_lift_stop PUBLIC cxx_std_20)
  target_link_libraries(test_ct_lift_stop ${PROJECT_NAME} Qt5::Widgets)
  ament_target_dependencies(test_ct_lift_stop
    hospital_interfaces rclcpp rclcpp_action rviz_common
  )
  ament_add_gtest(test_lab_panel_result test/test_lab_panel_result.cpp)
  target_compile_features(test_lab_panel_result PUBLIC cxx_std_20)
  target_link_libraries(test_lab_panel_result ${PROJECT_NAME} Qt5::Widgets)
  ament_target_dependencies(test_lab_panel_result
    hospital_interfaces rclcpp rclcpp_action rviz_common
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
