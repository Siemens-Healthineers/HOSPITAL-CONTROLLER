// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_CONTROLLER__LAB_CONTROLLER_NODE_HPP_
#define HOSPITAL_CONTROLLER__LAB_CONTROLLER_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <string_view>
#include <thread>
#include <vector>

#include "hospital_interfaces/action/analyze_blood.hpp"
#include "hospital_interfaces/msg/lab_state.hpp"
#include "hospital_interfaces/msg/stopwatch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace hospital_controller {

// Simulates the hospital's single blood analyser and its cardiac markers.
class LabControllerNode : public rclcpp::Node {
 public:
  using AnalyzeBlood = hospital_interfaces::action::AnalyzeBlood;
  using GoalHandleAnalyzeBlood = rclcpp_action::ServerGoalHandle<AnalyzeBlood>;

  static constexpr std::string_view kCommandStartBloodtest = "START_BLOODTEST";

  explicit LabControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LabControllerNode() override;

 private:
  struct MarkerLevels {
    float ctnt;
    float ctni;
    float ck_mb;
  };

  hospital_interfaces::msg::LabState update_lab_state(uint8_t state,
                                                      float progress);
  hospital_interfaces::msg::LabState set_lab_result(
      const MarkerLevels& markers);
  void publish_lab_state(const hospital_interfaces::msg::LabState& lab_state);
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const AnalyzeBlood::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      std::shared_ptr<GoalHandleAnalyzeBlood> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleAnalyzeBlood> goal_handle);
  void execute(std::stop_token stop_token,
               std::shared_ptr<GoalHandleAnalyzeBlood> goal_handle);
  MarkerLevels simulate_markers();
  void apply_parameters(const std::vector<rclcpp::Parameter>& parameters);

  static constexpr float kCtntUpperLimit = 14.0f;  // cTnT in ng/L
  static constexpr float kCtniUpperLimit = 20.0f;  // cTnI in ng/L
  static constexpr float kCkMbUpperLimit = 5.0f;   // CK-MB in %

  rclcpp::Publisher<hospital_interfaces::msg::LabState>::SharedPtr
      lab_state_pub_;
  rclcpp::Publisher<hospital_interfaces::msg::Stopwatch>::SharedPtr
      stopwatch_pub_;
  rclcpp_action::Server<AnalyzeBlood>::SharedPtr analyze_blood_action_server_;
  PostSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::atomic<double> analysis_duration_;
  std::atomic<double> publish_rate_;
  // Read-only after construction; worker threads may read without locking.
  bool cardiac_abnormal_{false};

  hospital_interfaces::msg::LabState telemetry_;
  bool is_running_{false};
  std::jthread execution_thread_;
  std::mutex mutex_;
  std::mt19937 rng_{std::random_device{}()};
};

}  // namespace hospital_controller

#endif  // HOSPITAL_CONTROLLER__LAB_CONTROLLER_NODE_HPP_
