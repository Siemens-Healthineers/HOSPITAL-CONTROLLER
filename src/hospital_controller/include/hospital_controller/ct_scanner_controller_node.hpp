// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_CONTROLLER__CT_SCANNER_CONTROLLER_NODE_HPP_
#define HOSPITAL_CONTROLLER__CT_SCANNER_CONTROLLER_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>
#include <thread>
#include <vector>

#include "hospital_interfaces/action/move_table.hpp"
#include "hospital_interfaces/action/run_scan.hpp"
#include "hospital_interfaces/msg/ct_scan_procedure_state.hpp"
#include "hospital_interfaces/msg/ct_scanner_state.hpp"
#include "hospital_interfaces/msg/ct_table_state.hpp"
#include "hospital_interfaces/msg/stopwatch.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace hospital_controller {

class RunReservation;

class CtScannerControllerNode : public rclcpp::Node {
 public:
  using RunScan = hospital_interfaces::action::RunScan;
  using GoalHandleRunScan = rclcpp_action::ServerGoalHandle<RunScan>;
  using MoveTable = hospital_interfaces::action::MoveTable;
  using GoalHandleMoveTable = rclcpp_action::ServerGoalHandle<MoveTable>;
  using ProcedureState = hospital_interfaces::msg::CtScanProcedureState;

  explicit CtScannerControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CtScannerControllerNode() override;

 private:
  struct CtScannerRuntime {
    hospital_interfaces::msg::CtTableState table_telemetry;
    hospital_interfaces::msg::CtScannerState scan_telemetry;
    // Physical axis positions: BOTTOM/OUT = 0, TOP/IN = 1.
    double lift_position{0.0};
    double top_position{0.0};
    bool is_moving{false};
    bool is_scanning{false};
    std::jthread table_thread;
    std::jthread scan_thread;
    std::mutex mutex;
  };

  static constexpr std::string_view kCtScannerId = "ct_scanner";
  static constexpr double kLiftBottom = 0.0;
  static constexpr double kLiftTop = 1.0;
  static constexpr double kTopOut = 0.0;
  static constexpr double kTopIn = 1.0;
  static constexpr double kPosEps = 1e-6;

  struct TableMove {
    uint8_t moving_state{0};
    uint8_t final_state{0};
    double target_position{0.0};
  };

  hospital_interfaces::msg::CtTableState set_table_state(
      const std::shared_ptr<CtScannerRuntime>& ct_scanner_runtime,
      uint8_t state, double position);
  hospital_interfaces::msg::CtScannerState set_scan_state(
      const std::shared_ptr<CtScannerRuntime>& ct_scanner_runtime,
      uint8_t state, float percentage_done, bool radiating);
  void publish_table_state(
      const hospital_interfaces::msg::CtTableState& table_state);
  void publish_scan_state(
      const hospital_interfaces::msg::CtScannerState& scan_state);
  void publish_scan_feedback(
      const std::shared_ptr<GoalHandleRunScan>& goal_handle,
      const std::shared_ptr<CtScannerRuntime>& ct_scanner_runtime,
      uint8_t phase);
  static float sweep_pct(bool moving_in, double top_position);
  static uint8_t resting_state_for_lift(double position);
  static std::optional<TableMove> resolve_move(uint8_t command,
                                               double position);

  rclcpp_action::GoalResponse handle_move_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveTable::Goal> goal);
  rclcpp_action::CancelResponse handle_move_cancel(
      std::shared_ptr<GoalHandleMoveTable> goal_handle);
  void handle_move_accepted(std::shared_ptr<GoalHandleMoveTable> goal_handle);
  void execute_move(std::stop_token stop_token,
                    std::shared_ptr<GoalHandleMoveTable> goal_handle,
                    std::shared_ptr<CtScannerRuntime> ct_scanner_runtime,
                    TableMove move);

  rclcpp_action::GoalResponse handle_scan_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const RunScan::Goal> goal);
  rclcpp_action::CancelResponse handle_scan_cancel(
      std::shared_ptr<GoalHandleRunScan> goal_handle);
  void handle_scan_accepted(std::shared_ptr<GoalHandleRunScan> goal_handle);
  bool animate_lift(std::stop_token stop_token,
                    std::shared_ptr<GoalHandleRunScan> goal_handle,
                    std::shared_ptr<CtScannerRuntime> ct_scanner_runtime,
                    double target_position);
  bool animate_sweep(std::stop_token stop_token,
                     std::shared_ptr<GoalHandleRunScan> goal_handle,
                     std::shared_ptr<CtScannerRuntime> ct_scanner_runtime,
                     bool moving_in);
  bool retract_table_top(std::stop_token stop_token,
                         std::shared_ptr<GoalHandleRunScan> goal_handle,
                         std::shared_ptr<CtScannerRuntime> ct_scanner_runtime);
  void on_scan_interrupted(std::shared_ptr<GoalHandleRunScan> goal_handle,
                           std::shared_ptr<CtScannerRuntime> ct_scanner_runtime,
                           RunReservation& running_guard);
  void execute_scan(std::stop_token stop_token,
                    std::shared_ptr<GoalHandleRunScan> goal_handle,
                    std::shared_ptr<CtScannerRuntime> ct_scanner_runtime);
  void execute_retract(std::stop_token stop_token,
                       std::shared_ptr<GoalHandleRunScan> goal_handle,
                       std::shared_ptr<CtScannerRuntime> ct_scanner_runtime);
  void apply_parameters(const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::Publisher<hospital_interfaces::msg::CtTableState>::SharedPtr
      table_state_pub_;
  rclcpp::Publisher<hospital_interfaces::msg::CtScannerState>::SharedPtr
      scan_state_pub_;
  rclcpp::Publisher<ProcedureState>::SharedPtr procedure_state_pub_;
  rclcpp::Publisher<hospital_interfaces::msg::Stopwatch>::SharedPtr
      stopwatch_pub_;
  rclcpp_action::Server<MoveTable>::SharedPtr move_table_action_server_;
  rclcpp_action::Server<RunScan>::SharedPtr run_scan_action_server_;
  PostSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::atomic<double> table_move_duration_;
  std::atomic<double> scan_duration_;
  std::atomic<double> publish_rate_;
  std::shared_ptr<CtScannerRuntime> ct_scanner_runtime_;
};

}  // namespace hospital_controller

#endif  // HOSPITAL_CONTROLLER__CT_SCANNER_CONTROLLER_NODE_HPP_
