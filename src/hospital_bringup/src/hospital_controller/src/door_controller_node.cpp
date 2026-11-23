// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_controller/door_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "hospital_controller/controller_utils.hpp"
#include "hospital_controller/parameter_validation.hpp"
#include "hospital_controller/worker_thread.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hospital_controller {
DoorControllerNode::DoorControllerNode(const rclcpp::NodeOptions& options)
    : Node("door_controller_node", options) {
  // Declare parameters and get their values. door_ids is read-only so it
  // can only be set from a startup/launch file before the node runs.
  rcl_interfaces::msg::ParameterDescriptor door_ids_descriptor;
  door_ids_descriptor.read_only = true;
  this->declare_parameter<std::vector<std::string>>(
      "door_ids",
      {"clinical_corridor_3", "clinical_corridor_2", "clinical_corridor_1",
       "phlebotomy_room", "changing_room", "intervention_or_2",
       "scanning_room_2", "laboratory", "entry_l", "entry_r",
       "intervention_or_1", "scanning_room_1"},
      door_ids_descriptor);
  // Range descriptors validate both the startup values and every runtime
  // change (see parameter_validation.hpp).
  this->declare_parameter<double>(
      "open_duration", 5.0, duration_descriptor("Time to fully open a door"));
  this->declare_parameter<double>(
      "close_duration", 5.0, duration_descriptor("Time to fully close a door"));
  this->declare_parameter<double>(
      "publish_rate", 5.0,
      publish_rate_descriptor("Door state/feedback publish rate"));
  door_ids_ = this->get_parameter("door_ids").as_string_array();
  for (std::size_t index = 0; index < door_ids_.size(); ++index) {
    const auto& door_id = door_ids_[index];
    if (!door_index_.emplace(door_id, index).second) {
      throw rclcpp::exceptions::InvalidParameterValueException(
          "door_ids contains duplicate ID '" + door_id + "'");
    }
  }
  open_duration_.store(this->get_parameter("open_duration").as_double());
  close_duration_.store(this->get_parameter("close_duration").as_double());
  publish_rate_.store(this->get_parameter("publish_rate").as_double());
  // Per-door state stream. Latched (transient_local) with a depth large
  // enough to retain every door's initial state so the opening publishes
  // issued below in the constructor are delivered to late subscribers
  // instead of being dropped by a volatile queue.
  rclcpp::QoS door_state_qos(std::max<std::size_t>(10, door_ids_.size()));
  door_state_qos.transient_local();
  door_state_pub_ = this->create_publisher<hospital_interfaces::msg::DoorState>(
      "/hospital/doors/state", door_state_qos);
  // Aggregated snapshot of every door. Latched (transient_local) so a late
  // subscriber immediately receives the most recent full array.
  rclcpp::QoS door_states_qos(10);
  door_states_qos.transient_local();
  door_states_pub_ = this->create_publisher<DoorStateArray>(
      "/hospital/doors/states", door_states_qos);

  // Stopwatch readings for the opening duration of each door. Latched so a
  // late subscriber (e.g. the RViz panel) immediately sees the most recent
  // readings. The depth is scaled to comfortably exceed the door count so
  // each door's terminal (running=false) reading survives in the latched
  // history even while many doors publish running ticks concurrently; a late
  // subscriber then never replays only stale running=true messages.
  rclcpp::QoS stopwatch_qos(std::max<std::size_t>(20, door_ids_.size() * 2));
  stopwatch_qos.transient_local();
  stopwatch_pub_ = this->create_publisher<hospital_interfaces::msg::Stopwatch>(
      "/hospital/stopwatch", stopwatch_qos);

  for (const auto& door_id : door_ids_) {
    auto runtime = std::make_shared<DoorRuntime>();
    runtime->telemetry.device_id = door_id;
    runtime->telemetry.state = hospital_interfaces::msg::DoorState::CLOSED;
    runtime->telemetry.percentage_open = 0.0f;
    door_runtime_.emplace(door_id, runtime);
    // The validated index map and aggregate use the same door_ids_ order.
    door_states_.doors.push_back(runtime->telemetry);
    door_state_pub_->publish(runtime->telemetry);
  }
  door_states_pub_->publish(door_states_);

  // Instantiate Action Server using lambda functions for callbacks
  this->door_action_server_ = rclcpp_action::create_server<OperateDoor>(
      this, "/hospital/doors/operate",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const OperateDoor::Goal> goal) {
        return handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleOperateDoor> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleOperateDoor> goal_handle) {
        handle_accepted(goal_handle);
      });

  // Values were already range-checked by rclcpp, which rejects the whole
  // request before anything is applied; only committed values reach here.
  parameter_callback_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        apply_parameters(parameters);
      });

  door_states_service_ = this->create_service<GetDoorStates>(
      "/hospital/doors/get_states",
      [this](const std::shared_ptr<GetDoorStates::Request> request,
             std::shared_ptr<GetDoorStates::Response> response) {
        handle_get_states(request, response);
      });
}

// Stop and join every worker before any member is destroyed. Without this
// the workers would outlive the mutexes/publishers they use, and because a
// worker co-owns its DoorRuntime it could end up destroying (= self-joining)
// its own std::jthread.
DoorControllerNode::~DoorControllerNode() {
  std::vector<std::jthread> workers;
  for (auto& [door_id, runtime] : door_runtime_) {
    std::scoped_lock lock(runtime->mutex);
    runtime->execution_thread.request_stop();
    workers.push_back(std::move(runtime->execution_thread));
  }
  // Joined here (outside the per-door locks the workers need on exit).
  workers.clear();
}

std::shared_ptr<DoorControllerNode::DoorRuntime>
DoorControllerNode::get_door_runtime(const std::string& device_id) const {
  const auto it = door_runtime_.find(device_id);
  if (it == door_runtime_.end()) {
    return nullptr;
  }
  return it->second;
}

float DoorControllerNode::clamp_percentage(float percentage_open) {
  return std::clamp(percentage_open, 0.0f, 100.0f);
}

uint8_t DoorControllerNode::state_for_progress(float percentage_open,
                                               bool opening) {
  if (percentage_open <= 0.0f) {
    return hospital_interfaces::msg::DoorState::CLOSED;
  }
  if (percentage_open >= 100.0f) {
    return hospital_interfaces::msg::DoorState::OPEN;
  }
  return opening ? hospital_interfaces::msg::DoorState::OPENING
                 : hospital_interfaces::msg::DoorState::CLOSING;
}

uint8_t DoorControllerNode::resting_state_for_progress(float percentage_open) {
  if (percentage_open <= 0.0f) {
    return DoorState::CLOSED;
  }
  if (percentage_open >= 100.0f) {
    return DoorState::OPEN;
  }
  return DoorState::STOPPED;
}

hospital_interfaces::msg::DoorState DoorControllerNode::update_door_state(
    const std::shared_ptr<DoorRuntime>& door_runtime, uint8_t current_state,
    float percentage_open) {
  std::scoped_lock lock(door_runtime->mutex);
  door_runtime->telemetry.state = current_state;
  door_runtime->telemetry.percentage_open = clamp_percentage(percentage_open);
  return door_runtime->telemetry;
}

void DoorControllerNode::publish_door_state(
    const hospital_interfaces::msg::DoorState& door_state) {
  door_state_pub_->publish(door_state);
  publish_door_states_array(door_state);
}

// Refresh the affected entry in the aggregate and publish the whole array.
// The per-door mutex is never held while this runs (callers update the door
// telemetry first, release it, then publish), so taking `door_states_mutex_`
// here cannot deadlock against a door mutex. The update and the publish are
// performed together under the lock so that, when several doors move at once,
// the order in which snapshots reach the wire matches the order in which they
// were produced; publishing outside the lock could let an older snapshot
// overtake a newer one and momentarily revert a door's state for subscribers.
void DoorControllerNode::publish_door_states_array(
    const hospital_interfaces::msg::DoorState& door_state) {
  std::scoped_lock lock(door_states_mutex_);
  const auto it = door_index_.find(door_state.device_id);
  if (it != door_index_.end()) {
    door_states_.doors[it->second] = door_state;
  }
  door_states_pub_->publish(door_states_);
}

void DoorControllerNode::handle_get_states(
    const std::shared_ptr<GetDoorStates::Request>,
    std::shared_ptr<GetDoorStates::Response> response) {
  // Publish under the same lock as the worker threads so this reply is
  // serialised into the aggregate topic's stream and cannot reorder against a
  // concurrent per-door update.
  std::scoped_lock lock(door_states_mutex_);
  response->door_states = door_states_;
  door_states_pub_->publish(door_states_);
}

rclcpp_action::GoalResponse DoorControllerNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const OperateDoor::Goal> goal) {
  const auto door_runtime = get_door_runtime(goal->device_id);
  if (door_runtime == nullptr) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting door goal for unknown device_id '%s'.",
                goal->device_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock(door_runtime->mutex);
  if (door_runtime->is_running) {
    RCLCPP_WARN(this->get_logger(),
                "Door '%s' is already moving. Rejecting new goal.",
                goal->device_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->command != OperateDoor::Goal::COMMAND_OPEN &&
      goal->command != OperateDoor::Goal::COMMAND_CLOSE) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting door goal for '%s' with unknown command %u.",
                goal->device_id.c_str(), goal->command);
    return rclcpp_action::GoalResponse::REJECT;
  }

  const float current_percentage = door_runtime->telemetry.percentage_open;
  if (goal->command == OperateDoor::Goal::COMMAND_OPEN &&
      current_percentage >= 100.0f) {
    RCLCPP_INFO(this->get_logger(), "Door '%s' is already open.",
                goal->device_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->command == OperateDoor::Goal::COMMAND_CLOSE &&
      current_percentage <= 0.0f) {
    RCLCPP_INFO(this->get_logger(), "Door '%s' is already closed.",
                goal->device_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  door_runtime->is_running = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DoorControllerNode::handle_cancel(
    const std::shared_ptr<GoalHandleOperateDoor> goal_handle) {
  const auto goal = goal_handle->get_goal();
  const auto door_runtime = get_door_runtime(goal->device_id);
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel door operation for '%s'.",
              goal->device_id.c_str());
  if (door_runtime != nullptr) {
    std::scoped_lock lock(door_runtime->mutex);
    door_runtime->execution_thread.request_stop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DoorControllerNode::handle_accepted(
    const std::shared_ptr<GoalHandleOperateDoor> goal_handle) {
  const auto goal = goal_handle->get_goal();
  const auto door_runtime = get_door_runtime(goal->device_id);
  if (door_runtime == nullptr) {
    auto result = std::make_shared<OperateDoor::Result>();
    result->success = false;
    result->message = "Door runtime missing";
    result->final_percentage = 0.0f;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Door operation accepted for '%s'.",
              goal->device_id.c_str());

  // Move any previous (already finished) worker out under the lock, then let
  // it join when `finished_thread` is destroyed *after* the lock is released.
  // Destroying a joinable jthread runs request_stop() + join(); doing that
  // while holding `door_runtime->mutex` could deadlock against the outgoing
  // worker's RunReservation, which also takes that mutex on its way out.
  std::jthread finished_thread;
  {
    std::scoped_lock lock(door_runtime->mutex);
    finished_thread = std::move(door_runtime->execution_thread);
    door_runtime->execution_thread = make_worker_thread(
        this->get_logger(),
        [this, goal_handle, door_runtime](std::stop_token stop_token) {
          execute(stop_token, goal_handle, door_runtime);
        });
  }
}

void DoorControllerNode::execute(
    std::stop_token stop_token,
    const std::shared_ptr<GoalHandleOperateDoor> goal_handle,
    const std::shared_ptr<DoorRuntime> door_runtime) {
  RunReservation running_guard{door_runtime->mutex, door_runtime->is_running};
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<OperateDoor::Feedback>();
  auto result = std::make_shared<OperateDoor::Result>();

  const bool opening = (goal->command == OperateDoor::Goal::COMMAND_OPEN);
  const float target_percentage = opening ? 100.0f : 0.0f;
  const double duration =
      opening ? open_duration_.load() : close_duration_.load();
  // publish_rate is the feedback/update frequency in Hz. Clamp to a small
  // positive floor so an unexpected non-positive value can never make the
  // interval zero or negative.
  const double hz = std::max(publish_rate_.load(), 0.1);
  const float start_percentage = [&door_runtime]() {
    std::scoped_lock lock(door_runtime->mutex);
    return door_runtime->telemetry.percentage_open;
  }();

  const float percentage_delta = target_percentage - start_percentage;
  const double remaining_ratio =
      std::abs(static_cast<double>(percentage_delta)) / 100.0;
  // Only the remaining travel needs to be animated, so the effective move
  // time scales with how far the door still has to go.
  const double move_duration = duration * remaining_ratio;
  auto current_state = update_door_state(
      door_runtime, state_for_progress(start_percentage, opening),
      start_percentage);
  publish_door_state(current_state);

  Stopwatch stopwatch{
      stopwatch_pub_,  this->get_logger(),
      goal->device_id, hospital_interfaces::msg::Stopwatch::KIND_DOOR_OPEN,
      this->now(),     move_duration};
  ScopeGuard terminal_guard{
      [&]() noexcept { stopwatch.interrupt_if_running(); }};
  stopwatch.tick();

  RCLCPP_INFO(this->get_logger(), "Operating door: %s",
              goal->device_id.c_str());

  if (!animate_steps(
          stop_token, move_duration, hz,
          [&](double fraction) {
            const float percentage_open = clamp_percentage(
                start_percentage +
                percentage_delta * static_cast<float>(fraction));
            current_state = update_door_state(
                door_runtime, state_for_progress(percentage_open, opening),
                percentage_open);
            publish_door_state(current_state);
            feedback->telemetry = current_state;
            goal_handle->publish_feedback(feedback);
            stopwatch.tick();
          },
          stopwatch.start_time())) {
    const float canceled_percentage = current_state.percentage_open;
    current_state = update_door_state(
        door_runtime, resting_state_for_progress(canceled_percentage),
        canceled_percentage);
    publish_door_state(current_state);
    result->success = false;
    result->final_percentage = current_state.percentage_open;
    stopwatch.finish(/*interrupted=*/true);
    // A stop arrives either through an action cancel request or when the
    // node shuts down and the worker's stop_token is triggered. Only the
    // cancel path may move the goal to CANCELED; otherwise the goal is
    // aborted.
    if (goal_handle->is_canceling()) {
      result->message = "Operation canceled";
      running_guard.complete([&] { goal_handle->canceled(result); });
      RCLCPP_INFO(this->get_logger(), "Door operation canceled.");
    } else {
      result->message = "Operation stopped";
      running_guard.complete([&] { goal_handle->abort(result); });
      RCLCPP_INFO(this->get_logger(), "Door operation stopped.");
    }
    return;
  }

  // publish "result"
  current_state = update_door_state(
      door_runtime, state_for_progress(target_percentage, opening),
      target_percentage);
  publish_door_state(current_state);
  result->success = true;
  result->message = "Operation completed successfully";
  result->final_percentage = current_state.percentage_open;
  stopwatch.finish(/*interrupted=*/false);
  running_guard.complete([&] { goal_handle->succeed(result); });
  RCLCPP_INFO(this->get_logger(), "Door operation completed successfully.");
}

void DoorControllerNode::apply_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "open_duration") {
      open_duration_.store(parameter.as_double());
    } else if (parameter.get_name() == "close_duration") {
      close_duration_.store(parameter.as_double());
    } else if (parameter.get_name() == "publish_rate") {
      publish_rate_.store(parameter.as_double());
    }
  }
}
}  // namespace hospital_controller
RCLCPP_COMPONENTS_REGISTER_NODE(hospital_controller::DoorControllerNode)
