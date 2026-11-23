# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.8)
project(hospital_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CtTableState.msg"
  "msg/CtScannerState.msg"
  "msg/CtScanProcedureState.msg"
  "msg/DoorState.msg"
  "msg/DoorStateArray.msg"
  "msg/LabState.msg"
  "msg/OperatingRoomState.msg"
  "msg/Stopwatch.msg"
  "msg/TaskEvent.msg"
  "srv/SetCleanliness.srv"
  "srv/GetOperatingRoomState.srv"
  "srv/GetDoorStates.srv"
  "action/AnalyzeBlood.action"
  "action/ExecuteTask.action"
  "action/MoveTable.action"
  "action/OperateDoor.action"
  "action/RunScan.action"
  DEPENDENCIES action_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Licensing is checked repo-wide with `reuse lint` (see CONTRIBUTING.md).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
