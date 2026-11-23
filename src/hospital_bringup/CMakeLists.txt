# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.8)
project(hospital_bringup)

find_package(ament_cmake REQUIRED)

# Bringup is launch/config only: install the launch files and parameter YAMLs.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Licensing is checked repo-wide with `reuse lint` (see CONTRIBUTING.md).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
