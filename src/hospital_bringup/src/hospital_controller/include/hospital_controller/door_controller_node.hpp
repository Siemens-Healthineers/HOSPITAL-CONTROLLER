// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_CONTROLLER__DOOR_CONTROLLER_NODE_HPP_
#define HOSPITAL_CONTROLLER__DOOR_CONTROLLER_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hospital_interfaces/action/operate_door.hpp"
#include "hospital_interfaces/msg/door_state.hpp"
#include "hospital_interfaces/msg/door_state_array.hpp"
#include "hospital_interfaces/msg/stopwatch.hpp"
#include "hospital_interfaces/srv/get_door_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace hospital_controller {

class DoorControllerNode : public rclcpp::Node {
 public:
  using OperateDoor = hospital_interfaces::action::OperateDoor;
  using GoalHandleOperateDoor = rclcpp_action::ServerGoalHandle<OperateDoor>;
  using DoorState = hospital_interfaces::msg::DoorState;
  using DoorStateArray = hospital_interfaces::msg::DoorStateArray;
  using GetDoorStates = hospital_interfaces::srv::GetDoorStates;

  explicit DoorControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DoorControllerNode() override;

 private:
  struct DoorRuntime {
    DoorState telemetry;
    bool is_running{false};
    std::jthread execution_thread;
    std::mutex mutex;
  };

  std::shared_ptr<DoorRuntime> get_door_runtime(
      const std::string& device_id) const;
  static float clamp_percentage(float percentage_open);
  static uint8_t state_for_progress(float percentage_open, bool opening);
  static uint8_t resting_state_for_progress(float percentage_open);
  DoorState update_door_state(const std::shared_ptr<DoorRuntime>& door_runtime,
                              uint8_t current_state, float percentage_open);
  void publish_door_state(const DoorState& door_state);
  void publish_door_states_array(const DoorState& door_state);
  void handle_get_states(std::shared_ptr<GetDoorStates::Request> request,
                         std::shared_ptr<GetDoorStates::Response> response);
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const OperateDoor::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      std::shared_ptr<GoalHandleOperateDoor> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleOperateDoor> goal_handle);
  void execute(std::stop_token stop_token,
               std::shared_ptr<GoalHandleOperateDoor> goal_handle,
               std::shared_ptr<DoorRuntime> door_runtime);
  void apply_parameters(const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::Publisher<DoorState>::SharedPtr door_state_pub_;
  rclcpp::Publisher<DoorStateArray>::SharedPtr door_states_pub_;
  rclcpp::Publisher<hospital_interfaces::msg::Stopwatch>::SharedPtr
      stopwatch_pub_;
  rclcpp::Service<GetDoorStates>::SharedPtr door_states_service_;
  rclcpp_action::Server<OperateDoor>::SharedPtr door_action_server_;
  PostSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::vector<std::string> door_ids_;
  std::atomic<double> open_duration_;
  std::atomic<double> close_duration_;
  std::atomic<double> publish_rate_;
  std::unordered_map<std::string, std::shared_ptr<DoorRuntime>> door_runtime_;

  // The index is immutable after construction. The mutex serializes aggregate
  // snapshots from concurrently moving doors and service callbacks.
  DoorStateArray door_states_;
  std::mutex door_states_mutex_;
  std::unordered_map<std::string, std::size_t> door_index_;
};

}  // namespace hospital_controller

#endif  // HOSPITAL_CONTROLLER__DOOR_CONTROLLER_NODE_HPP_
