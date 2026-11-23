// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_controller/ct_scanner_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "hospital_controller/controller_utils.hpp"
#include "hospital_controller/parameter_validation.hpp"
#include "hospital_controller/worker_thread.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hospital_controller {
CtScannerControllerNode::CtScannerControllerNode(
    const rclcpp::NodeOptions& options)
    : Node("ct_scanner_controller_node", options) {
  // Declare parameters and get their values.
  // Range descriptors validate both the startup values and every runtime
  // change (see parameter_validation.hpp).
  this->declare_parameter<double>(
      "table_move_duration", 5.0,
      duration_descriptor("Time for a full table-lift stroke"));
  this->declare_parameter<double>(
      "scan_duration", 10.0,
      duration_descriptor("Time for a full scan sweep (in and out)"));
  this->declare_parameter<double>(
      "publish_rate", 3.0,
      publish_rate_descriptor("CT state/feedback publish rate"));
  table_move_duration_.store(
      this->get_parameter("table_move_duration").as_double());
  scan_duration_.store(this->get_parameter("scan_duration").as_double());
  publish_rate_.store(this->get_parameter("publish_rate").as_double());

  // Table and scan state. Latched (transient_local) so a freshly-started
  // subscriber (e.g. the RViz panel) opened while the scanner is idle
  // immediately sees the current state instead of waiting for the next scan.
  rclcpp::QoS table_state_qos(10);
  table_state_qos.transient_local();
  table_state_pub_ =
      this->create_publisher<hospital_interfaces::msg::CtTableState>(
          "/hospital/ct/table/state", table_state_qos);
  rclcpp::QoS scan_state_qos(10);
  scan_state_qos.transient_local();
  scan_state_pub_ =
      this->create_publisher<hospital_interfaces::msg::CtScannerState>(
          "/hospital/ct/scan/state", scan_state_qos);
  procedure_state_pub_ = this->create_publisher<ProcedureState>(
      "/hospital/ct/scan/procedure_state", rclcpp::QoS(1).transient_local());

  // Stopwatch readings for the scan duration. Latched so a late subscriber
  // immediately sees the most recent reading.
  rclcpp::QoS stopwatch_qos(20);
  stopwatch_qos.transient_local();
  stopwatch_pub_ = this->create_publisher<hospital_interfaces::msg::Stopwatch>(
      "/hospital/stopwatch", stopwatch_qos);

  // Publish an initial idle reading so a late (latched) subscriber sees a
  // valid stopwatch before the first scan runs.
  hospital_interfaces::msg::Stopwatch initial_stopwatch;
  initial_stopwatch.device_id = kCtScannerId;
  initial_stopwatch.kind = hospital_interfaces::msg::Stopwatch::KIND_CT_SCAN;
  initial_stopwatch.start_stamp = this->now();
  initial_stopwatch.elapsed_seconds = 0.0f;
  initial_stopwatch.final_seconds = 0.0f;
  initial_stopwatch.running = false;
  initial_stopwatch.interrupted = false;
  stopwatch_pub_->publish(initial_stopwatch);

  // Initialize the CT scanner runtime state and publish the initial state.
  // The lift starts at BOTTOM (boarding position) and the scan unit IDLE.
  ct_scanner_runtime_ = std::make_shared<CtScannerRuntime>();
  ct_scanner_runtime_->table_telemetry.state =
      hospital_interfaces::msg::CtTableState::BOTTOM;
  ct_scanner_runtime_->table_telemetry.percentage_done = 0.0f;
  ct_scanner_runtime_->scan_telemetry.state =
      hospital_interfaces::msg::CtScannerState::IDLE;
  ct_scanner_runtime_->scan_telemetry.percentage_done = 0.0f;
  ct_scanner_runtime_->scan_telemetry.radiating = false;
  table_state_pub_->publish(ct_scanner_runtime_->table_telemetry);
  scan_state_pub_->publish(ct_scanner_runtime_->scan_telemetry);
  publish_scan_feedback(nullptr, ct_scanner_runtime_,
                        ProcedureState::PHASE_INACTIVE);

  // Action server for table movement.
  this->move_table_action_server_ = rclcpp_action::create_server<MoveTable>(
      this, "/hospital/ct/table/move",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const MoveTable::Goal> goal) {
        return handle_move_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleMoveTable> goal_handle) {
        return handle_move_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleMoveTable> goal_handle) {
        handle_move_accepted(goal_handle);
      });

  // Action server for running scans.
  this->run_scan_action_server_ = rclcpp_action::create_server<RunScan>(
      this, "/hospital/ct/scan/run",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const RunScan::Goal> goal) {
        return handle_scan_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleRunScan> goal_handle) {
        return handle_scan_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleRunScan> goal_handle) {
        handle_scan_accepted(goal_handle);
      });

  // Values were already range-checked by rclcpp, which rejects the whole
  // request before anything is applied; only committed values reach here.
  parameter_callback_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        apply_parameters(parameters);
      });
}

// Stop and join both workers before any member is destroyed. The workers
// co-own the runtime, so without this they would outlive the node (and could
// end up destroying, i.e. self-joining, their own std::jthread).
CtScannerControllerNode::~CtScannerControllerNode() {
  std::jthread table_thread;
  std::jthread scan_thread;
  {
    std::scoped_lock lock(ct_scanner_runtime_->mutex);
    ct_scanner_runtime_->table_thread.request_stop();
    ct_scanner_runtime_->scan_thread.request_stop();
    table_thread = std::move(ct_scanner_runtime_->table_thread);
    scan_thread = std::move(ct_scanner_runtime_->scan_thread);
  }
  // Joined here, outside the runtime lock the workers need on exit.
}

// Set both the lift state and its physical position. `percentage_done` is
// derived from the position so it always describes the absolute location
// (0 = BOTTOM, 100 = TOP).
hospital_interfaces::msg::CtTableState CtScannerControllerNode::set_table_state(
    const std::shared_ptr<CtScannerRuntime>& ct_scanner_runtime, uint8_t state,
    double position) {
  std::scoped_lock lock(ct_scanner_runtime->mutex);
  ct_scanner_runtime->lift_position = position;
  ct_scanner_runtime->table_telemetry.state = state;
  ct_scanner_runtime->table_telemetry.percentage_done =
      static_cast<float>(std::clamp(position * 100.0, 0.0, 100.0));
  return ct_scanner_runtime->table_telemetry;
}

hospital_interfaces::msg::CtScannerState
CtScannerControllerNode::set_scan_state(
    const std::shared_ptr<CtScannerRuntime>& ct_scanner_runtime, uint8_t state,
    float percentage_done, bool radiating) {
  std::scoped_lock lock(ct_scanner_runtime->mutex);
  ct_scanner_runtime->scan_telemetry.state = state;
  ct_scanner_runtime->scan_telemetry.percentage_done =
      std::clamp(percentage_done, 0.0f, 100.0f);
  ct_scanner_runtime->scan_telemetry.radiating = radiating;
  return ct_scanner_runtime->scan_telemetry;
}

void CtScannerControllerNode::publish_table_state(
    const hospital_interfaces::msg::CtTableState& table_state) {
  table_state_pub_->publish(table_state);
}

void CtScannerControllerNode::publish_scan_state(
    const hospital_interfaces::msg::CtScannerState& scan_state) {
  scan_state_pub_->publish(scan_state);
}

void CtScannerControllerNode::publish_scan_feedback(
    const std::shared_ptr<GoalHandleRunScan>& goal_handle,
    const std::shared_ptr<CtScannerRuntime>& ct_scanner_runtime,
    uint8_t phase) {
  ProcedureState snapshot;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    snapshot.phase = phase;
    snapshot.table_telemetry = ct_scanner_runtime->table_telemetry;
    snapshot.telemetry = ct_scanner_runtime->scan_telemetry;
  }
  procedure_state_pub_->publish(snapshot);
  if (goal_handle) {
    auto feedback = std::make_shared<RunScan::Feedback>();
    feedback->phase = snapshot.phase;
    feedback->table_telemetry = snapshot.table_telemetry;
    feedback->telemetry = snapshot.telemetry;
    goal_handle->publish_feedback(feedback);
  }
}

// Overall scan-cycle completion for a given table-top insertion depth. The
// sweep in (OUT -> IN) maps to 0..50, the sweep out (IN -> OUT) to 50..100.
float CtScannerControllerNode::sweep_pct(bool moving_in, double top_position) {
  const double clamped = std::clamp(top_position, 0.0, 1.0);
  const double pct = moving_in ? clamped * 50.0 : 50.0 + (1.0 - clamped) * 50.0;
  return static_cast<float>(pct);
}

// Map a lift position to the state to publish when at rest there: an anchor
// state if essentially on an anchor, otherwise PAUSED (stopped mid-travel).
uint8_t CtScannerControllerNode::resting_state_for_lift(double position) {
  using CtTableState = hospital_interfaces::msg::CtTableState;
  if (std::abs(position - kLiftBottom) < kPosEps) {
    return CtTableState::BOTTOM;
  }
  if (std::abs(position - kLiftTop) < kPosEps) {
    return CtTableState::TOP;
  }
  return CtTableState::PAUSED;
}

// Resolve a movement command against the current (possibly partial) lift
// position. MoveTable only drives the vertical lift: COMMAND_UP raises all
// the way toward TOP (z-position 100%) and COMMAND_DOWN lowers all the way to
// BOTTOM. The movement runs until it is stopped or the anchor is reached.
std::optional<CtScannerControllerNode::TableMove>
CtScannerControllerNode::resolve_move(uint8_t command, double position) {
  using CtTableState = hospital_interfaces::msg::CtTableState;
  switch (command) {
    case MoveTable::Goal::COMMAND_UP:
      // Raise toward TOP; valid anywhere below TOP.
      if (position < kLiftTop - kPosEps) {
        return TableMove{CtTableState::MOVING_UP, CtTableState::TOP, kLiftTop};
      }
      break;
    case MoveTable::Goal::COMMAND_DOWN:
      // Lower toward BOTTOM; valid anywhere above BOTTOM (including from TOP,
      // e.g. lowering the patient after a completed scan).
      if (position > kLiftBottom + kPosEps) {
        return TableMove{CtTableState::MOVING_DOWN, CtTableState::BOTTOM,
                         kLiftBottom};
      }
      break;
    default:
      break;
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// Table movement action
// ---------------------------------------------------------------------------
// Handle a new table movement goal. Reject if the command is invalid, a scan
// is in progress, or another movement is already running. Otherwise accept
// the goal and start executing it.
rclcpp_action::GoalResponse CtScannerControllerNode::handle_move_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const MoveTable::Goal> goal) {
  const auto ct_scanner_runtime = ct_scanner_runtime_;

  if (goal->command != MoveTable::Goal::COMMAND_UP &&
      goal->command != MoveTable::Goal::COMMAND_DOWN) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting move goal with unknown command %u.", goal->command);
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock(ct_scanner_runtime->mutex);
  // A table movement must never run while the scan unit is radiating.
  if (ct_scanner_runtime->is_scanning) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting move goal: a scan is in progress.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Only one movement may run at a time.
  if (ct_scanner_runtime->is_moving) {
    RCLCPP_WARN(this->get_logger(),
                "Table is already moving. Rejecting new goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (ct_scanner_runtime->top_position != kTopOut) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting move goal: table-top is not fully retracted. "
                "Run RETRACT before moving the lift.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!resolve_move(goal->command, ct_scanner_runtime->lift_position)) {
    RCLCPP_INFO(
        this->get_logger(),
        "Rejecting move goal: command %u already at target position %.2f.",
        goal->command, ct_scanner_runtime->lift_position);
    return rclcpp_action::GoalResponse::REJECT;
  }

  ct_scanner_runtime->is_moving = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CtScannerControllerNode::handle_move_cancel(
    const std::shared_ptr<GoalHandleMoveTable>) {
  const auto ct_scanner_runtime = ct_scanner_runtime_;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel table movement.");
  std::scoped_lock lock(ct_scanner_runtime->mutex);
  ct_scanner_runtime->table_thread.request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CtScannerControllerNode::handle_move_accepted(
    const std::shared_ptr<GoalHandleMoveTable> goal_handle) {
  const auto goal = goal_handle->get_goal();
  const auto ct_scanner_runtime = ct_scanner_runtime_;

  // Resolve the requested movement against the current physical position.
  double current_position = 0.0;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    current_position = ct_scanner_runtime->lift_position;
  }
  const auto move = resolve_move(goal->command, current_position);
  if (!move) {
    {
      std::scoped_lock lock(ct_scanner_runtime->mutex);
      ct_scanner_runtime->is_moving = false;
    }
    auto result = std::make_shared<MoveTable::Result>();
    result->success = false;
    result->message = "Table position changed after goal acceptance";
    result->final_position = resting_state_for_lift(current_position);
    goal_handle->abort(result);
    RCLCPP_WARN(this->get_logger(),
                "Aborting move: command %u not valid from position %.2f.",
                goal->command, current_position);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Table movement accepted.");

  // Swap in the new worker under the lock, but join the previous (finished)
  // thread outside the lock. A jthread move-assignment would otherwise
  // request_stop() and join() while the mutex is held, and the outgoing
  // worker's exit path also needs that mutex.
  std::jthread previous_thread;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    previous_thread = std::move(ct_scanner_runtime->table_thread);
    ct_scanner_runtime->table_thread = make_worker_thread(
        this->get_logger(), [this, goal_handle, ct_scanner_runtime,
                             move = *move](std::stop_token stop_token) {
          execute_move(stop_token, goal_handle, ct_scanner_runtime, move);
        });
  }
}

void CtScannerControllerNode::execute_move(
    std::stop_token stop_token,
    const std::shared_ptr<GoalHandleMoveTable> goal_handle,
    const std::shared_ptr<CtScannerRuntime> ct_scanner_runtime,
    TableMove move) {
  RunReservation moving_guard{ct_scanner_runtime->mutex,
                              ct_scanner_runtime->is_moving};
  auto feedback = std::make_shared<MoveTable::Feedback>();
  auto result = std::make_shared<MoveTable::Result>();

  double start_position = 0.0;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    start_position = ct_scanner_runtime->lift_position;
  }

  const double hz = std::max(publish_rate_.load(), 0.1);
  // table_move_duration_ is the time for the full BOTTOM<->TOP travel (axis
  // distance 1.0). Scale by the distance actually travelled so the table
  // moves at a constant speed, including when resuming a movement from a
  // paused partial position.
  const double full_duration = std::max(table_move_duration_.load(), 0.0);
  const double distance = std::abs(move.target_position - start_position);
  const double duration = full_duration * distance;
  auto current_state =
      set_table_state(ct_scanner_runtime, move.moving_state, start_position);
  publish_table_state(current_state);

  RCLCPP_INFO(this->get_logger(), "Starting table movement.");

  double current_position = start_position;
  if (!animate_steps(stop_token, duration, hz, [&](double fraction) {
        current_position =
            start_position + (move.target_position - start_position) * fraction;
        current_state = set_table_state(ct_scanner_runtime, move.moving_state,
                                        current_position);
        publish_table_state(current_state);
        feedback->telemetry.state = current_state.state;
        feedback->telemetry.percentage_done = current_state.percentage_done;
        goal_handle->publish_feedback(feedback);
      })) {
    // Freeze in place: the table holds the physical position it has reached
    // so a later movement can resume from here. The published state becomes
    // PAUSED (or the anchor state if we happened to stop exactly on one).
    const uint8_t frozen_state = resting_state_for_lift(current_position);
    current_state =
        set_table_state(ct_scanner_runtime, frozen_state, current_position);
    publish_table_state(current_state);
    result->success = false;
    result->final_position = frozen_state;
    // A stop arrives through an action cancel request. Only the cancel path
    // may move the goal to CANCELED; otherwise the goal is aborted.
    if (goal_handle->is_canceling()) {
      result->message = "Movement canceled";
      moving_guard.complete([&] { goal_handle->canceled(result); });
      RCLCPP_INFO(this->get_logger(), "Table movement canceled; table paused.");
    } else {
      result->message = "Movement halted";
      moving_guard.complete([&] { goal_handle->abort(result); });
      RCLCPP_INFO(this->get_logger(), "Table movement halted; table paused.");
    }
    return;
  }

  current_state = set_table_state(ct_scanner_runtime, move.final_state,
                                  move.target_position);
  publish_table_state(current_state);
  result->success = true;
  result->message = "Movement completed successfully";
  result->final_position = move.final_state;
  moving_guard.complete([&] { goal_handle->succeed(result); });
  RCLCPP_INFO(this->get_logger(), "Table movement completed successfully.");
}

// ---------------------------------------------------------------------------
// Scan action
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CtScannerControllerNode::handle_scan_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const RunScan::Goal> goal) {
  const auto ct_scanner_runtime = ct_scanner_runtime_;

  if (goal->command != RunScan::Goal::COMMAND_START &&
      goal->command != RunScan::Goal::COMMAND_STOP &&
      goal->command != RunScan::Goal::COMMAND_RETRACT) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting scan goal with unknown command %u.", goal->command);
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock(ct_scanner_runtime->mutex);
  if (goal->command == RunScan::Goal::COMMAND_STOP) {
    // A STOP goal does not start work; it asks the scan or retraction to halt
    // and hold the table-top wherever it currently is (radiation off, the
    // lift stays put). It does NOT pull the table-top out of the gantry (use
    // COMMAND_RETRACT for that). It must bypass the is_scanning guard (the
    // START or RETRACT goal owns is_scanning) and is only rejected when
    // nothing is running.
    if (!ct_scanner_runtime->is_scanning) {
      RCLCPP_INFO(this->get_logger(), "Scanner is already idle.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  if (goal->command == RunScan::Goal::COMMAND_RETRACT) {
    // Drive the table-top out of the gantry (the lift stays put). Rejected
    // only while another scan or movement is running (it reserves the scan
    // unit while it runs).
    if (ct_scanner_runtime->is_scanning || ct_scanner_runtime->is_moving) {
      RCLCPP_WARN(this->get_logger(), "Rejecting retract: scanner busy.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    ct_scanner_runtime->is_scanning = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // command == COMMAND_START: only one scan may run at a time, and never
  // while the table is moving.
  if (ct_scanner_runtime->is_scanning) {
    RCLCPP_WARN(this->get_logger(),
                "Scanner is already running. Rejecting new goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (ct_scanner_runtime->is_moving) {
    RCLCPP_WARN(this->get_logger(), "Rejecting scan goal: table is moving.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // A scan may start from any stationary lift position. The scan itself first
  // raises the lift to TOP (no radiation) before sweeping the table-top.
  ct_scanner_runtime->is_scanning = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CtScannerControllerNode::handle_scan_cancel(
    const std::shared_ptr<GoalHandleRunScan>) {
  const auto ct_scanner_runtime = ct_scanner_runtime_;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel scan operation.");
  std::scoped_lock lock(ct_scanner_runtime->mutex);
  ct_scanner_runtime->scan_thread.request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CtScannerControllerNode::handle_scan_accepted(
    const std::shared_ptr<GoalHandleRunScan> goal_handle) {
  const auto goal = goal_handle->get_goal();
  const auto ct_scanner_runtime = ct_scanner_runtime_;

  if (goal->command == RunScan::Goal::COMMAND_STOP) {
    // Route the STOP command to the scan/retraction thread and complete the
    // STOP goal immediately. The worker observes the stop request and
    // halts the table-top in place with radiation off (see
    // on_scan_interrupted()). It is left inside the gantry;
    // COMMAND_RETRACT pulls it out.
    RCLCPP_INFO(this->get_logger(), "Stop requested for scanner.");
    {
      std::scoped_lock lock(ct_scanner_runtime->mutex);
      ct_scanner_runtime->scan_thread.request_stop();
    }
    auto result = std::make_shared<RunScan::Result>();
    result->success = true;
    result->message = "Stop requested";
    goal_handle->succeed(result);
    return;
  }

  if (goal->command == RunScan::Goal::COMMAND_RETRACT) {
    RCLCPP_INFO(this->get_logger(), "Recovery (retract) accepted.");
    std::jthread previous_thread;
    {
      std::scoped_lock lock(ct_scanner_runtime->mutex);
      previous_thread = std::move(ct_scanner_runtime->scan_thread);
      ct_scanner_runtime->scan_thread = make_worker_thread(
          this->get_logger(),
          [this, goal_handle, ct_scanner_runtime](std::stop_token stop_token) {
            execute_retract(stop_token, goal_handle, ct_scanner_runtime);
          });
    }
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Scan operation accepted.");

  std::jthread previous_thread;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    previous_thread = std::move(ct_scanner_runtime->scan_thread);
    ct_scanner_runtime->scan_thread = make_worker_thread(
        this->get_logger(),
        [this, goal_handle, ct_scanner_runtime](std::stop_token stop_token) {
          execute_scan(stop_token, goal_handle, ct_scanner_runtime);
        });
  }
}

// Raise/lower the lift to `target_position` while a scan goal owns the
// scanner, publishing lift telemetry on the table topic. Returns false if a
// stop was requested mid-motion (the caller decides freeze vs retract).
bool CtScannerControllerNode::animate_lift(
    std::stop_token stop_token,
    const std::shared_ptr<GoalHandleRunScan> goal_handle,
    const std::shared_ptr<CtScannerRuntime> ct_scanner_runtime,
    double target_position) {
  double start_position = 0.0;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    start_position = ct_scanner_runtime->lift_position;
  }
  if (std::abs(target_position - start_position) < kPosEps) {
    auto resting =
        set_table_state(ct_scanner_runtime,
                        resting_state_for_lift(start_position), start_position);
    publish_table_state(resting);
    publish_scan_feedback(goal_handle, ct_scanner_runtime,
                          ProcedureState::PHASE_PREPARING_LIFT);
    return !stop_token.stop_requested();
  }
  const double hz = std::max(publish_rate_.load(), 0.1);
  const double full_duration = std::max(table_move_duration_.load(), 0.0);
  const double distance = std::abs(target_position - start_position);
  const double duration = full_duration * distance;
  const uint8_t moving_state =
      (target_position > start_position)
          ? hospital_interfaces::msg::CtTableState::MOVING_UP
          : hospital_interfaces::msg::CtTableState::MOVING_DOWN;

  auto current =
      set_table_state(ct_scanner_runtime, moving_state, start_position);
  publish_table_state(current);
  publish_scan_feedback(goal_handle, ct_scanner_runtime,
                        ProcedureState::PHASE_PREPARING_LIFT);

  double current_position = start_position;
  if (!animate_steps(stop_token, duration, hz, [&](double fraction) {
        current_position =
            start_position + (target_position - start_position) * fraction;
        current =
            set_table_state(ct_scanner_runtime, moving_state, current_position);
        publish_table_state(current);
        publish_scan_feedback(goal_handle, ct_scanner_runtime,
                              ProcedureState::PHASE_PREPARING_LIFT);
      })) {
    current = set_table_state(ct_scanner_runtime,
                              resting_state_for_lift(current_position),
                              current_position);
    publish_table_state(current);
    return false;
  }
  const uint8_t final_state = resting_state_for_lift(target_position);
  current = set_table_state(ct_scanner_runtime, final_state, target_position);
  publish_table_state(current);
  publish_scan_feedback(goal_handle, ct_scanner_runtime,
                        ProcedureState::PHASE_PREPARING_LIFT);
  return true;
}

// Sweep the table-top through the gantry (radiation on). `moving_in` selects
// OUT -> IN, otherwise IN -> OUT. Returns false if a stop was requested
// mid-sweep.
bool CtScannerControllerNode::animate_sweep(
    std::stop_token stop_token,
    const std::shared_ptr<GoalHandleRunScan> goal_handle,
    const std::shared_ptr<CtScannerRuntime> ct_scanner_runtime,
    bool moving_in) {
  double start_top = 0.0;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    start_top = ct_scanner_runtime->top_position;
  }
  const double target_top = moving_in ? kTopIn : kTopOut;
  const uint8_t moving_state =
      moving_in ? hospital_interfaces::msg::CtScannerState::MOVING_IN
                : hospital_interfaces::msg::CtScannerState::MOVING_OUT;
  const double hz = std::max(publish_rate_.load(), 0.1);
  const double half_duration = std::max(scan_duration_.load(), 0.0) / 2.0;
  const double distance = std::abs(target_top - start_top);
  const double duration = half_duration * distance;
  const uint8_t phase = moving_in ? ProcedureState::PHASE_SWEEP_IN
                                  : ProcedureState::PHASE_SWEEP_OUT;

  auto current = set_scan_state(ct_scanner_runtime, moving_state,
                                sweep_pct(moving_in, start_top), true);
  publish_scan_state(current);
  publish_scan_feedback(goal_handle, ct_scanner_runtime, phase);

  if (!animate_steps(stop_token, duration, hz, [&](double fraction) {
        const double top = start_top + (target_top - start_top) * fraction;
        {
          std::scoped_lock lock(ct_scanner_runtime->mutex);
          ct_scanner_runtime->top_position = top;
        }
        current = set_scan_state(ct_scanner_runtime, moving_state,
                                 sweep_pct(moving_in, top), true);
        publish_scan_state(current);
        publish_scan_feedback(goal_handle, ct_scanner_runtime, phase);
      })) {
    return false;
  }
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    ct_scanner_runtime->top_position = target_top;
  }
  current = set_scan_state(ct_scanner_runtime, moving_state,
                           sweep_pct(moving_in, target_top), true);
  publish_scan_state(current);
  publish_scan_feedback(goal_handle, ct_scanner_runtime, phase);
  return true;
}

// Pull the table-top horizontally out of the gantry (radiation off). The
// lift is left at its current vertical position.
bool CtScannerControllerNode::retract_table_top(
    std::stop_token stop_token,
    const std::shared_ptr<GoalHandleRunScan> goal_handle,
    const std::shared_ptr<CtScannerRuntime> ct_scanner_runtime) {
  const double hz = std::max(publish_rate_.load(), 0.1);
  if (stop_token.stop_requested()) {
    return false;
  }

  // Pull the table-top OUT of the gantry (no radiation).
  double start_top = 0.0;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    start_top = ct_scanner_runtime->top_position;
  }
  if (start_top > kTopOut) {
    const double half_duration = std::max(scan_duration_.load(), 0.0) / 2.0;
    const double duration = half_duration * start_top;
    using CtScannerState = hospital_interfaces::msg::CtScannerState;
    auto current =
        set_scan_state(ct_scanner_runtime, CtScannerState::MOVING_OUT,
                       sweep_pct(false, start_top), false);
    publish_scan_state(current);
    publish_scan_feedback(goal_handle, ct_scanner_runtime,
                          ProcedureState::PHASE_RETRACTING);
    if (!animate_steps(stop_token, duration, hz, [&](double fraction) {
          const double top = start_top + (kTopOut - start_top) * fraction;
          {
            std::scoped_lock lock(ct_scanner_runtime->mutex);
            ct_scanner_runtime->top_position = top;
          }
          current =
              set_scan_state(ct_scanner_runtime, CtScannerState::MOVING_OUT,
                             sweep_pct(false, top), false);
          publish_scan_state(current);
          publish_scan_feedback(goal_handle, ct_scanner_runtime,
                                ProcedureState::PHASE_RETRACTING);
        })) {
      return false;
    }
    {
      std::scoped_lock lock(ct_scanner_runtime->mutex);
      ct_scanner_runtime->top_position = kTopOut;
    }
    auto idle =
        set_scan_state(ct_scanner_runtime, CtScannerState::IDLE, 0.0f, false);
    publish_scan_state(idle);
  }
  return true;
}

// Complete a scan goal that was interrupted mid-flight (cancel or
// COMMAND_STOP) by halting the table-top where it is with radiation off,
// leaving both the lift and the table-top in place. The table-top is left
// wherever it stopped inside the gantry; COMMAND_RETRACT pulls it out.
void CtScannerControllerNode::on_scan_interrupted(
    const std::shared_ptr<GoalHandleRunScan> goal_handle,
    const std::shared_ptr<CtScannerRuntime> ct_scanner_runtime,
    RunReservation& running_guard) {
  // Freeze the table-top at its current position with radiation off. While it
  // is still inside the gantry the scan unit reports PAUSED; if it happened
  // to halt fully out it reports IDLE. The physical top_position is preserved
  // by the horizontal movement loop, so a later RETRACT resumes from here.
  double current_top = 0.0;
  double current_lift = 0.0;
  float current_pct = 0.0f;
  {
    std::scoped_lock lock(ct_scanner_runtime->mutex);
    current_top = ct_scanner_runtime->top_position;
    current_lift = ct_scanner_runtime->lift_position;
    current_pct = ct_scanner_runtime->scan_telemetry.percentage_done;
  }
  auto resting = set_table_state(
      ct_scanner_runtime, resting_state_for_lift(current_lift), current_lift);
  publish_table_state(resting);
  const uint8_t halted_state =
      (current_top > kTopOut + kPosEps)
          ? hospital_interfaces::msg::CtScannerState::PAUSED
          : hospital_interfaces::msg::CtScannerState::IDLE;
  auto halted =
      set_scan_state(ct_scanner_runtime, halted_state, current_pct, false);
  publish_scan_state(halted);
  publish_scan_feedback(goal_handle, ct_scanner_runtime,
                        ProcedureState::PHASE_INTERRUPTED);
  auto result = std::make_shared<RunScan::Result>();
  const bool retracting =
      goal_handle->get_goal()->command == RunScan::Goal::COMMAND_RETRACT;
  result->success = !retracting;
  result->message = retracting ? "Retraction halted; table-top holding position"
                               : "Scan halted; table-top holding position";
  RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
  if (goal_handle->is_canceling()) {
    running_guard.complete([&] { goal_handle->canceled(result); });
  } else {
    running_guard.complete([&] { goal_handle->abort(result); });
  }
}

void CtScannerControllerNode::execute_scan(
    std::stop_token stop_token,
    const std::shared_ptr<GoalHandleRunScan> goal_handle,
    const std::shared_ptr<CtScannerRuntime> ct_scanner_runtime) {
  RunReservation running_guard{ct_scanner_runtime->mutex,
                               ct_scanner_runtime->is_scanning};

  RCLCPP_INFO(this->get_logger(), "Starting scan: %s", kCtScannerId.data());

  // Phase 1: raise the lift to TOP (no radiation). Not timed.
  if (!animate_lift(stop_token, goal_handle, ct_scanner_runtime, kLiftTop)) {
    on_scan_interrupted(goal_handle, ct_scanner_runtime, running_guard);
    return;
  }

  // Start the horizontal-movement stopwatch now that the lift is at TOP.
  Stopwatch stopwatch{stopwatch_pub_, this->get_logger(), kCtScannerId,
                      hospital_interfaces::msg::Stopwatch::KIND_CT_SCAN,
                      this->now()};
  ScopeGuard terminal_guard{
      [&]() noexcept { stopwatch.interrupt_if_running(); }};
  stopwatch.tick();

  // Phase 2: sweep the table-top into the gantry (radiation on).
  if (!animate_sweep(stop_token, goal_handle, ct_scanner_runtime,
                     /*moving_in=*/true)) {
    stopwatch.finish(/*interrupted=*/true);
    on_scan_interrupted(goal_handle, ct_scanner_runtime, running_guard);
    return;
  }
  // Phase 3: sweep the table-top back out of the gantry (radiation on).
  if (!animate_sweep(stop_token, goal_handle, ct_scanner_runtime,
                     /*moving_in=*/false)) {
    stopwatch.finish(/*interrupted=*/true);
    on_scan_interrupted(goal_handle, ct_scanner_runtime, running_guard);
    return;
  }

  // Done: table-top out, radiation off, lift at TOP.
  auto idle = set_scan_state(ct_scanner_runtime,
                             hospital_interfaces::msg::CtScannerState::IDLE,
                             0.0f, false);
  publish_scan_state(idle);
  stopwatch.finish(/*interrupted=*/false);
  publish_scan_feedback(goal_handle, ct_scanner_runtime,
                        ProcedureState::PHASE_COMPLETED);
  auto result = std::make_shared<RunScan::Result>();
  result->success = true;
  result->message = "Scan completed successfully";
  running_guard.complete([&] { goal_handle->succeed(result); });
  RCLCPP_INFO(this->get_logger(), "Scan operation completed successfully.");
}

// Recovery action (COMMAND_RETRACT): pull the table-top out of the gantry,
// leaving the lift in place.
void CtScannerControllerNode::execute_retract(
    std::stop_token stop_token,
    const std::shared_ptr<GoalHandleRunScan> goal_handle,
    const std::shared_ptr<CtScannerRuntime> ct_scanner_runtime) {
  RunReservation running_guard{ct_scanner_runtime->mutex,
                               ct_scanner_runtime->is_scanning};

  RCLCPP_INFO(this->get_logger(), "Recovering scanner; table-top out.");
  publish_scan_feedback(goal_handle, ct_scanner_runtime,
                        ProcedureState::PHASE_RETRACTING);

  if (!retract_table_top(stop_token, goal_handle, ct_scanner_runtime)) {
    on_scan_interrupted(goal_handle, ct_scanner_runtime, running_guard);
    return;
  }
  publish_scan_feedback(goal_handle, ct_scanner_runtime,
                        ProcedureState::PHASE_COMPLETED);
  auto result = std::make_shared<RunScan::Result>();
  result->success = true;
  result->message = "Recovered: table-top out of the gantry";
  running_guard.complete([&] { goal_handle->succeed(result); });
  RCLCPP_INFO(this->get_logger(), "Recovery complete; table-top out.");
}

void CtScannerControllerNode::apply_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "table_move_duration") {
      table_move_duration_.store(parameter.as_double());
    } else if (parameter.get_name() == "scan_duration") {
      scan_duration_.store(parameter.as_double());
    } else if (parameter.get_name() == "publish_rate") {
      publish_rate_.store(parameter.as_double());
    }
  }
}
}  // namespace hospital_controller
RCLCPP_COMPONENTS_REGISTER_NODE(hospital_controller::CtScannerControllerNode)
