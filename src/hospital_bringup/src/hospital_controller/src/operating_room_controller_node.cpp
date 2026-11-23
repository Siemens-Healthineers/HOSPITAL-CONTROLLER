// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_controller/operating_room_controller_node.hpp"

#include <string_view>

#include "rclcpp_components/register_node_macro.hpp"

namespace hospital_controller {
// Simulates a single operating room. The room is either CLEAN or DIRTY and is
// set through the `set_cleanliness` service. Whether
// the X-ray is currently radiating is controlled by the `xray_active`
// parameter, so it can be set from a launch/params file or with
// `ros2 param set` at runtime. The state topic is latched and published on
// startup and on every change (cleanliness or X-ray), so a late subscriber
// always sees the current state; `/hospital/operating_room/get_state` only
// returns it in the service response.
OperatingRoomControllerNode::OperatingRoomControllerNode(
    const rclcpp::NodeOptions& options)
    : Node("operating_room_controller_node", options) {
  this->declare_parameter<bool>("xray_active", false);

  // The room starts clean with the X-ray reflecting the parameter value.
  state_.cleanliness = OperatingRoomState::CLEAN;
  state_.xray_active = this->get_parameter("xray_active").as_bool();

  state_pub_ = this->create_publisher<OperatingRoomState>(
      "/hospital/operating_room/state", rclcpp::QoS(1).transient_local());
  state_pub_->publish(state_);

  set_cleanliness_service_ = this->create_service<SetCleanliness>(
      "/hospital/operating_room/set_cleanliness",
      [this](const std::shared_ptr<SetCleanliness::Request> request,
             std::shared_ptr<SetCleanliness::Response> response) {
        handle_set_cleanliness(request, response);
      });

  get_state_service_ = this->create_service<GetOperatingRoomState>(
      "/hospital/operating_room/get_state",
      [this](const std::shared_ptr<GetOperatingRoomState::Request> request,
             std::shared_ptr<GetOperatingRoomState::Response> response) {
        handle_get_state(request, response);
      });

  // Applied after rclcpp committed the change, so the published state never
  // reports a value that was rejected.
  parameter_callback_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        apply_parameters(parameters);
      });
}

void OperatingRoomControllerNode::handle_set_cleanliness(
    const std::shared_ptr<SetCleanliness::Request> request,
    std::shared_ptr<SetCleanliness::Response> response) {
  if (request->target_state != OperatingRoomState::CLEAN &&
      request->target_state != OperatingRoomState::DIRTY) {
    response->accepted = false;
    RCLCPP_WARN(this->get_logger(),
                "Rejecting set_cleanliness request with invalid target_state "
                "%u (expected DIRTY=%u or CLEAN=%u).",
                request->target_state, OperatingRoomState::DIRTY,
                OperatingRoomState::CLEAN);
    return;
  }

  OperatingRoomState snapshot;
  {
    std::scoped_lock lock(mutex_);
    state_.cleanliness = request->target_state;
    snapshot = state_;
  }
  const std::string_view label =
      (snapshot.cleanliness == OperatingRoomState::CLEAN) ? "CLEAN" : "DIRTY";
  response->accepted = true;
  RCLCPP_INFO(this->get_logger(), "Operating room set to %s.", label.data());
  state_pub_->publish(snapshot);
}

void OperatingRoomControllerNode::handle_get_state(
    const std::shared_ptr<GetOperatingRoomState::Request>,
    std::shared_ptr<GetOperatingRoomState::Response> response) {
  OperatingRoomState snapshot;
  {
    std::scoped_lock lock(mutex_);
    snapshot = state_;
  }
  response->state = snapshot;
}

void OperatingRoomControllerNode::apply_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& parameter : parameters) {
    if (parameter.get_name() != "xray_active") {
      continue;
    }
    OperatingRoomState snapshot;
    {
      std::scoped_lock lock(mutex_);
      if (state_.xray_active == parameter.as_bool()) {
        continue;
      }
      state_.xray_active = parameter.as_bool();
      snapshot = state_;
    }
    RCLCPP_INFO(this->get_logger(), "X-ray set to %s.",
                snapshot.xray_active ? "ACTIVE" : "INACTIVE");
    state_pub_->publish(snapshot);
  }
}
}  // namespace hospital_controller
RCLCPP_COMPONENTS_REGISTER_NODE(
    hospital_controller::OperatingRoomControllerNode)
