// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_CONTROLLER__PARAMETER_VALIDATION_HPP_
#define HOSPITAL_CONTROLLER__PARAMETER_VALIDATION_HPP_

#include <string>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace hospital_controller {

// Bounds shared by every controller. rclcpp enforces a descriptor range both
// for the startup value (YAML / overrides: an out-of-range value makes the
// node's constructor throw) and for every runtime set request, where it
// rejects the whole request before anything is applied.
inline constexpr double kMinDurationSeconds = 0.001;
inline constexpr double kMaxDurationSeconds = 3600.0;
inline constexpr double kMinPublishRateHz = 0.1;
inline constexpr double kMaxPublishRateHz = 1000.0;

// Descriptor for a double parameter restricted to the inclusive range
// [from, to].
inline rcl_interfaces::msg::ParameterDescriptor ranged_descriptor(
    const std::string& description, double from, double to) {
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = 0.0;  // Continuous.
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

inline rcl_interfaces::msg::ParameterDescriptor duration_descriptor(
    const std::string& description) {
  return ranged_descriptor(description + " [s]", kMinDurationSeconds,
                           kMaxDurationSeconds);
}

inline rcl_interfaces::msg::ParameterDescriptor publish_rate_descriptor(
    const std::string& description) {
  return ranged_descriptor(description + " [Hz]", kMinPublishRateHz,
                           kMaxPublishRateHz);
}

}  // namespace hospital_controller

#endif  // HOSPITAL_CONTROLLER__PARAMETER_VALIDATION_HPP_
