// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_controller/lab_controller_node.hpp"

#include <algorithm>
#include <utility>

#include "hospital_controller/controller_utils.hpp"
#include "hospital_controller/parameter_validation.hpp"
#include "hospital_controller/worker_thread.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hospital_controller {
LabControllerNode::LabControllerNode(const rclcpp::NodeOptions& options)
    : Node("lab_controller_node", options) {
  this->declare_parameter<double>(
      "analysis_duration", 10.0,
      duration_descriptor("Time for one blood analysis"));
  this->declare_parameter<double>(
      "publish_rate", 1.0,
      publish_rate_descriptor("Lab state/feedback publish rate"));
  // cardiac_abnormal is read-only: it can only be set from a launch/params
  // file before the node starts, never via `ros2 param set` at runtime. When
  // true the analyser reports out-of-norm cardiac markers (cTnT > 14 ng/L,
  // cTnI > 20 ng/L, CK-MB > 5%); otherwise all markers stay within norm.
  rcl_interfaces::msg::ParameterDescriptor cardiac_abnormal_descriptor;
  cardiac_abnormal_descriptor.read_only = true;
  cardiac_abnormal_descriptor.description =
      "If true, reported cardiac markers are out of norm (cTnT > 14 ng/L, "
      "cTnI > 20 ng/L, CK-MB > 5%). Read-only: set only before launch.";
  this->declare_parameter<bool>("cardiac_abnormal", false,
                                cardiac_abnormal_descriptor);
  analysis_duration_.store(
      this->get_parameter("analysis_duration").as_double());
  publish_rate_.store(this->get_parameter("publish_rate").as_double());
  cardiac_abnormal_ = this->get_parameter("cardiac_abnormal").as_bool();

  // Retain only the current snapshot, including a result only until the next
  // accepted analysis clears it.
  rclcpp::QoS lab_state_qos(1);
  lab_state_qos.transient_local();
  lab_state_pub_ = this->create_publisher<hospital_interfaces::msg::LabState>(
      "/hospital/lab/state", lab_state_qos);

  // Stopwatch readings for the analysis duration. Latched so a late
  // subscriber immediately sees the most recent reading.
  rclcpp::QoS stopwatch_qos(20);
  stopwatch_qos.transient_local();
  stopwatch_pub_ = this->create_publisher<hospital_interfaces::msg::Stopwatch>(
      "/hospital/stopwatch", stopwatch_qos);

  // The analyser starts idle and ready to accept a blood test.
  telemetry_.state = hospital_interfaces::msg::LabState::READY;
  telemetry_.progress_percentage = 0.0f;
  lab_state_pub_->publish(telemetry_);

  analyze_blood_action_server_ = rclcpp_action::create_server<AnalyzeBlood>(
      this, "/hospital/lab/analyze",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const AnalyzeBlood::Goal> goal) {
        return handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleAnalyzeBlood> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleAnalyzeBlood> goal_handle) {
        handle_accepted(goal_handle);
      });

  // Values were already range-checked by rclcpp, which rejects the whole
  // request before anything is applied; only committed values reach here.
  parameter_callback_handle_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        apply_parameters(parameters);
      });
}

// Stop and join the worker before any member it uses (mutex_, rng_, ...) is
// destroyed.
LabControllerNode::~LabControllerNode() {
  std::jthread worker;
  {
    std::scoped_lock lock(mutex_);
    execution_thread_.request_stop();
    worker = std::move(execution_thread_);
  }
  // Joined here, outside mutex_ which the worker needs on exit.
}

hospital_interfaces::msg::LabState LabControllerNode::update_lab_state(
    uint8_t state, float progress) {
  std::scoped_lock lock(mutex_);
  telemetry_.state = state;
  telemetry_.progress_percentage = std::clamp(progress, 0.0f, 100.0f);
  return telemetry_;
}

// Keep the completed measurements until another analysis is accepted.
hospital_interfaces::msg::LabState LabControllerNode::set_lab_result(
    const MarkerLevels& markers) {
  std::scoped_lock lock(mutex_);
  telemetry_.state = hospital_interfaces::msg::LabState::DONE;
  telemetry_.progress_percentage = 100.0f;
  telemetry_.result_valid = true;
  telemetry_.ctnt = markers.ctnt;
  telemetry_.ctni = markers.ctni;
  telemetry_.ck_mb = markers.ck_mb;
  return telemetry_;
}

void LabControllerNode::publish_lab_state(
    const hospital_interfaces::msg::LabState& lab_state) {
  lab_state_pub_->publish(lab_state);
}

rclcpp_action::GoalResponse LabControllerNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const AnalyzeBlood::Goal> goal) {
  if (goal->command != kCommandStartBloodtest) {
    RCLCPP_WARN(this->get_logger(),
                "Rejecting blood test goal with unknown command '%s'.",
                goal->command.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::scoped_lock lock(mutex_);
  if (is_running_) {
    RCLCPP_WARN(this->get_logger(),
                "Analyser is busy. Rejecting new blood test goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  is_running_ = true;
  telemetry_ = hospital_interfaces::msg::LabState{};
  telemetry_.state = hospital_interfaces::msg::LabState::ANALYSING;
  publish_lab_state(telemetry_);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LabControllerNode::handle_cancel(
    const std::shared_ptr<GoalHandleAnalyzeBlood>) {
  RCLCPP_INFO(this->get_logger(),
              "Received request to cancel blood test analysis.");
  std::scoped_lock lock(mutex_);
  execution_thread_.request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LabControllerNode::handle_accepted(
    const std::shared_ptr<GoalHandleAnalyzeBlood> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Blood test analysis accepted.");
  std::jthread previous_thread;
  {
    std::scoped_lock lock(mutex_);
    previous_thread = std::move(execution_thread_);
    execution_thread_ = make_worker_thread(
        this->get_logger(), [this, goal_handle](std::stop_token stop_token) {
          execute(stop_token, goal_handle);
        });
  }
}

void LabControllerNode::execute(
    std::stop_token stop_token,
    const std::shared_ptr<GoalHandleAnalyzeBlood> goal_handle) {
  RunReservation running_guard{mutex_, is_running_};
  auto feedback = std::make_shared<AnalyzeBlood::Feedback>();
  auto result = std::make_shared<AnalyzeBlood::Result>();

  const double duration = std::max(analysis_duration_.load(), 0.0);
  // publish_rate is the feedback/update frequency in Hz. Clamp to a small
  // positive floor so a non-positive value can never make the interval zero.
  const double hz = std::max(publish_rate_.load(), 0.1);
  hospital_interfaces::msg::LabState current_state;
  RCLCPP_INFO(this->get_logger(), "Starting blood test analysis.");

  Stopwatch stopwatch{
      stopwatch_pub_, this->get_logger(),
      "lab",          hospital_interfaces::msg::Stopwatch::KIND_LAB_ANALYSIS,
      this->now(),    duration};
  ScopeGuard terminal_guard{
      [&]() noexcept { stopwatch.interrupt_if_running(); }};
  stopwatch.tick();

  float progress = 0.0f;
  if (!animate_steps(
          stop_token, duration, hz,
          [&](double fraction) {
            progress = 100.0f * static_cast<float>(fraction);
            current_state = update_lab_state(
                hospital_interfaces::msg::LabState::ANALYSING, progress);
            publish_lab_state(current_state);
            // Feedback carries live progress, never completed measurements.
            feedback->telemetry = current_state;
            feedback->telemetry.result_valid = false;
            feedback->telemetry.ctnt = 0.0f;
            feedback->telemetry.ctni = 0.0f;
            feedback->telemetry.ck_mb = 0.0f;
            goal_handle->publish_feedback(feedback);
            stopwatch.tick();
          },
          stopwatch.start_time())) {
    current_state =
        update_lab_state(hospital_interfaces::msg::LabState::READY, 0.0f);
    publish_lab_state(current_state);
    result->success = false;
    result->ctnt = 0.0f;
    result->ctni = 0.0f;
    result->ck_mb = 0.0f;
    stopwatch.finish(/*interrupted=*/true);
    if (goal_handle->is_canceling()) {
      result->message = "Analysis canceled";
      running_guard.complete([&] { goal_handle->canceled(result); });
      RCLCPP_INFO(this->get_logger(), "Blood test analysis canceled at %.0f%%.",
                  progress);
    } else {
      result->message = "Analysis stopped";
      running_guard.complete([&] { goal_handle->abort(result); });
      RCLCPP_INFO(this->get_logger(), "Blood test analysis stopped at %.0f%%.",
                  progress);
    }
    return;
  }

  // Analysis complete: publish the DONE state and simulated marker levels.
  const auto markers = simulate_markers();
  current_state = set_lab_result(markers);
  publish_lab_state(current_state);
  stopwatch.finish(/*interrupted=*/false);
  result->success = true;
  result->message = "Analysis completed successfully";
  result->ctnt = markers.ctnt;
  result->ctni = markers.ctni;
  result->ck_mb = markers.ck_mb;

  // Return to READY so the next blood test can be started.
  current_state =
      update_lab_state(hospital_interfaces::msg::LabState::READY, 0.0f);
  publish_lab_state(current_state);
  running_guard.complete([&] { goal_handle->succeed(result); });
  RCLCPP_INFO(this->get_logger(),
              "Blood test complete: cTnT=%.1f ng/L, cTnI=%.1f ng/L, "
              "CK-MB=%.1f %%.",
              markers.ctnt, markers.ctni, markers.ck_mb);
}

// Produce plausible cardiac-marker concentrations for the simulated sample.
// When `cardiac_abnormal_` is set the values are drawn strictly above the
// reference limits, otherwise they stay within (at or below) them.
LabControllerNode::MarkerLevels LabControllerNode::simulate_markers() {
  if (cardiac_abnormal_) {
    std::uniform_real_distribution<float> ctnt_dist(kCtntUpperLimit, 100.0f);
    std::uniform_real_distribution<float> ctni_dist(kCtniUpperLimit, 120.0f);
    std::uniform_real_distribution<float> ck_mb_dist(kCkMbUpperLimit, 25.0f);
    // Nudge off the inclusive lower bound so the value is strictly out of
    // norm even in the (negligible) case the distribution returns the limit.
    return MarkerLevels{ctnt_dist(rng_) + 0.1f, ctni_dist(rng_) + 0.1f,
                        ck_mb_dist(rng_) + 0.1f};
  }
  std::uniform_real_distribution<float> ctnt_dist(0.0f, kCtntUpperLimit);
  std::uniform_real_distribution<float> ctni_dist(0.0f, kCtniUpperLimit);
  std::uniform_real_distribution<float> ck_mb_dist(0.0f, kCkMbUpperLimit);
  return MarkerLevels{ctnt_dist(rng_), ctni_dist(rng_), ck_mb_dist(rng_)};
}

void LabControllerNode::apply_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "analysis_duration") {
      analysis_duration_.store(parameter.as_double());
    } else if (parameter.get_name() == "publish_rate") {
      publish_rate_.store(parameter.as_double());
    }
  }
}

}  // namespace hospital_controller
RCLCPP_COMPONENTS_REGISTER_NODE(hospital_controller::LabControllerNode)
