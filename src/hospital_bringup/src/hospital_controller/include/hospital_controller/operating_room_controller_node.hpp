// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_CONTROLLER__OPERATING_ROOM_CONTROLLER_NODE_HPP_
#define HOSPITAL_CONTROLLER__OPERATING_ROOM_CONTROLLER_NODE_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include "hospital_interfaces/msg/operating_room_state.hpp"
#include "hospital_interfaces/srv/get_operating_room_state.hpp"
#include "hospital_interfaces/srv/set_cleanliness.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hospital_controller {

class OperatingRoomControllerNode : public rclcpp::Node {
 public:
  using OperatingRoomState = hospital_interfaces::msg::OperatingRoomState;
  using SetCleanliness = hospital_interfaces::srv::SetCleanliness;
  using GetOperatingRoomState = hospital_interfaces::srv::GetOperatingRoomState;

  explicit OperatingRoomControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void handle_set_cleanliness(
      std::shared_ptr<SetCleanliness::Request> request,
      std::shared_ptr<SetCleanliness::Response> response);
  void handle_get_state(
      std::shared_ptr<GetOperatingRoomState::Request> request,
      std::shared_ptr<GetOperatingRoomState::Response> response);
  void apply_parameters(const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::Publisher<OperatingRoomState>::SharedPtr state_pub_;
  rclcpp::Service<SetCleanliness>::SharedPtr set_cleanliness_service_;
  rclcpp::Service<GetOperatingRoomState>::SharedPtr get_state_service_;
  PostSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Services and parameter updates may run on different executor threads.
  OperatingRoomState state_;
  std::mutex mutex_;
};

}  // namespace hospital_controller

#endif  // HOSPITAL_CONTROLLER__OPERATING_ROOM_CONTROLLER_NODE_HPP_
