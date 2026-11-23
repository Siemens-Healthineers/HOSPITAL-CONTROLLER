// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <chrono>
#include <functional>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "hospital_controller/lab_controller_node.hpp"

using namespace std::chrono_literals;

namespace hospital_controller {
class LabResultTest : public testing::Test {
 protected:
  using LabState = hospital_interfaces::msg::LabState;
  using AnalyzeBlood = hospital_interfaces::action::AnalyzeBlood;
  using GoalHandle = rclcpp_action::ClientGoalHandle<AnalyzeBlood>;

  static void SetUpTestSuite() {
    rclcpp::InitOptions options;
    options.set_domain_id(46);
    rclcpp::init(0, nullptr, options);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("analysis_duration", 0.5),
                                 rclcpp::Parameter("publish_rate", 20.0),
                                 rclcpp::Parameter("cardiac_abnormal", true)});
    controller_ = std::make_shared<LabControllerNode>(options);
    client_node_ = std::make_shared<rclcpp::Node>("lab_result_test");
    client_ = rclcpp_action::create_client<AnalyzeBlood>(
        client_node_, "/hospital/lab/analyze");
    subscription_ = client_node_->create_subscription<LabState>(
        "/hospital/lab/state", rclcpp::QoS(10).transient_local(),
        [this](LabState::ConstSharedPtr state) { states_.push_back(*state); });
    executor_.add_node(controller_);
    executor_.add_node(client_node_);
    ASSERT_TRUE(client_->wait_for_action_server(10s));
  }

  void TearDown() override {
    if (controller_) {
      executor_.remove_node(controller_);
    }
    executor_.remove_node(client_node_);
    controller_.reset();
    client_node_.reset();
  }

  bool wait_for(const std::function<bool()>& predicate,
                std::chrono::milliseconds timeout = 10000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  template <typename Future>
  bool ready(Future& future) {
    return wait_for(
        [&future] { return future.wait_for(0s) == std::future_status::ready; });
  }

  GoalHandle::SharedPtr send_goal(
      const std::string& command = "START_BLOODTEST") {
    AnalyzeBlood::Goal goal;
    goal.command = command;
    rclcpp_action::Client<AnalyzeBlood>::SendGoalOptions options;
    options.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const AnalyzeBlood::Feedback> feedback) {
          feedback_.push_back(feedback->telemetry);
        };
    auto response = client_->async_send_goal(goal, options);
    if (!ready(response)) {
      ADD_FAILURE() << "Timed out waiting for lab goal response";
      return nullptr;
    }
    return response.get();
  }

  static void expect_empty(const LabState& state) {
    EXPECT_FALSE(state.result_valid);
    EXPECT_FLOAT_EQ(state.ctnt, 0.0f);
    EXPECT_FLOAT_EQ(state.ctni, 0.0f);
    EXPECT_FLOAT_EQ(state.ck_mb, 0.0f);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<LabControllerNode> controller_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<AnalyzeBlood>::SharedPtr client_;
  rclcpp::Subscription<LabState>::SharedPtr subscription_;
  std::vector<LabState> states_;
  std::vector<LabState> feedback_;
};

TEST_F(LabResultTest, ResultCallbackCanImmediatelyStartNextAnalysis) {
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("analysis_duration", 0.2))
          .successful);
  ASSERT_TRUE(wait_for([this] { return !states_.empty(); }));
  for (const bool cancel_first : {false, true}) {
    for (int cycle = 0; cycle < 3; ++cycle) {
      std::shared_future<GoalHandle::SharedPtr> restart;
      bool completed = false;
      rclcpp_action::Client<AnalyzeBlood>::SendGoalOptions options;
      options.result_callback = [this, cancel_first, &restart, &completed](
                                    const GoalHandle::WrappedResult& result) {
        EXPECT_EQ(result.code, cancel_first
                                   ? rclcpp_action::ResultCode::CANCELED
                                   : rclcpp_action::ResultCode::SUCCEEDED);
        AnalyzeBlood::Goal next;
        next.command = "START_BLOODTEST";
        restart = client_->async_send_goal(next);
        completed = true;
      };
      AnalyzeBlood::Goal goal;
      goal.command = "START_BLOODTEST";
      auto response = client_->async_send_goal(goal, options);
      ASSERT_TRUE(ready(response));
      ASSERT_NE(response.get(), nullptr);
      if (cancel_first) {
        ASSERT_TRUE(wait_for([this] {
          return states_.back().state == LabState::ANALYSING &&
                 states_.back().progress_percentage > 0.0f;
        }));
        auto cancel = client_->async_cancel_goal(response.get());
        ASSERT_TRUE(ready(cancel));
        EXPECT_EQ(cancel.get()->goals_canceling.size(), 1u);
      }
      ASSERT_TRUE(wait_for([&completed] { return completed; }));
      ASSERT_TRUE(ready(restart));
      ASSERT_NE(restart.get(), nullptr) << "Cycle " << cycle;
      ASSERT_TRUE(wait_for([this] {
        return states_.back().state == LabState::ANALYSING &&
               states_.back().progress_percentage > 0.0f;
      }));
      expect_empty(states_.back());
      auto result = client_->async_get_result(restart.get());
      ASSERT_TRUE(ready(result));
      EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
      ASSERT_TRUE(wait_for([this] {
        return states_.back().state == LabState::READY &&
               states_.back().result_valid;
      }));
      EXPECT_FLOAT_EQ(states_.back().ctnt, result.get().result->ctnt);
    }
  }
}

TEST_F(LabResultTest, StopwatchFinalizesOnceOnSuccessAndCancellation) {
  using StopwatchMessage = hospital_interfaces::msg::Stopwatch;
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("analysis_duration", 0.4))
          .successful);
  std::vector<StopwatchMessage> readings;
  auto subscription = client_node_->create_subscription<StopwatchMessage>(
      "/hospital/stopwatch", rclcpp::QoS(20).transient_local(),
      [&readings](StopwatchMessage::ConstSharedPtr message) {
        if (message->device_id == "lab") {
          readings.push_back(*message);
        }
      });

  for (const bool cancel : {false, true}) {
    const auto first_reading = readings.size();
    auto goal = send_goal();
    ASSERT_NE(goal, nullptr);
    auto result = client_->async_get_result(goal);
    if (cancel) {
      ASSERT_TRUE(wait_for([&readings, first_reading] {
        return readings.size() > first_reading && readings.back().running &&
               readings.back().elapsed_seconds >= 0.05f;
      }));
      auto cancellation = client_->async_cancel_goal(goal);
      ASSERT_TRUE(ready(cancellation));
      EXPECT_EQ(cancellation.get()->goals_canceling.size(), 1u);
    }
    ASSERT_TRUE(ready(result));
    EXPECT_EQ(result.get().code, cancel ? rclcpp_action::ResultCode::CANCELED
                                        : rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_TRUE(wait_for([&readings, first_reading] {
      return readings.size() > first_reading && !readings.back().running;
    }));
    wait_for([] { return false; }, 50ms);
    EXPECT_TRUE(readings[first_reading].running);
    EXPECT_FLOAT_EQ(readings[first_reading].nominal_seconds, 0.4f);
    EXPECT_EQ(std::count_if(readings.begin() + first_reading, readings.end(),
                            [](const StopwatchMessage& reading) {
                              return !reading.running;
                            }),
              1);
    EXPECT_EQ(readings.back().interrupted, cancel);
    EXPECT_FLOAT_EQ(readings.back().final_seconds,
                    readings.back().elapsed_seconds);
    EXPECT_GT(readings.back().final_seconds, 0.0f);
  }
}

TEST_F(LabResultTest, ClearOnAcceptedStartAndKeepEmptyAfterCancellation) {
  ASSERT_TRUE(wait_for([this] { return !states_.empty(); }));
  EXPECT_EQ(states_.back().state, LabState::READY);
  EXPECT_FLOAT_EQ(states_.back().progress_percentage, 0.0f);
  expect_empty(states_.back());

  auto first = send_goal();
  ASSERT_NE(first, nullptr);
  auto first_result = client_->async_get_result(first);
  ASSERT_TRUE(ready(first_result));
  EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(first_result.get().result->success);
  ASSERT_TRUE(wait_for([this] {
    return states_.back().state == LabState::READY &&
           states_.back().result_valid;
  }));
  const auto completed = states_.back();
  EXPECT_FLOAT_EQ(completed.ctnt, first_result.get().result->ctnt);
  EXPECT_FLOAT_EQ(completed.ctni, first_result.get().result->ctni);
  EXPECT_FLOAT_EQ(completed.ck_mb, first_result.get().result->ck_mb);
  EXPECT_EQ(send_goal("INVALID"), nullptr);
  EXPECT_TRUE(states_.back().result_valid);
  EXPECT_FLOAT_EQ(states_.back().ctnt, completed.ctnt);

  const auto second_start = states_.size();
  auto second = send_goal();
  ASSERT_NE(second, nullptr);
  auto second_result = client_->async_get_result(second);
  ASSERT_TRUE(wait_for([this, second_start] {
    return states_.size() > second_start &&
           states_.back().state == LabState::ANALYSING;
  }));
  EXPECT_FLOAT_EQ(states_[second_start].progress_percentage, 0.0f);
  expect_empty(states_.back());
  EXPECT_EQ(send_goal(), nullptr);

  std::vector<LabState> late_states;
  auto late_subscription = client_node_->create_subscription<LabState>(
      "/hospital/lab/state", rclcpp::QoS(10).transient_local(),
      [&late_states](LabState::ConstSharedPtr state) {
        late_states.push_back(*state);
      });
  ASSERT_TRUE(wait_for([&late_states] { return !late_states.empty(); }));
  for (const auto& state : late_states) {
    EXPECT_EQ(state.state, LabState::ANALYSING);
    expect_empty(state);
  }
  auto cancellation = client_->async_cancel_goal(second);
  ASSERT_TRUE(ready(cancellation));
  EXPECT_EQ(cancellation.get()->goals_canceling.size(), 1u);
  ASSERT_TRUE(ready(second_result));
  EXPECT_EQ(second_result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(second_result.get().result->success);
  EXPECT_FLOAT_EQ(second_result.get().result->ctnt, 0.0f);
  EXPECT_FLOAT_EQ(second_result.get().result->ctni, 0.0f);
  EXPECT_FLOAT_EQ(second_result.get().result->ck_mb, 0.0f);
  ASSERT_TRUE(
      wait_for([this] { return states_.back().state == LabState::READY; }));
  EXPECT_FLOAT_EQ(states_.back().progress_percentage, 0.0f);
  for (auto index = second_start; index < states_.size(); ++index) {
    expect_empty(states_[index]);
  }

  auto third = send_goal();
  ASSERT_NE(third, nullptr);
  auto third_result = client_->async_get_result(third);
  ASSERT_TRUE(ready(third_result));
  EXPECT_EQ(third_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(third_result.get().result->success);
  ASSERT_TRUE(wait_for([this] {
    return states_.back().state == LabState::READY &&
           states_.back().result_valid;
  }));
  EXPECT_FLOAT_EQ(states_.back().ctnt, third_result.get().result->ctnt);
  ASSERT_FALSE(feedback_.empty());
  for (const auto& state : feedback_) {
    expect_empty(state);
  }
}
// Destroying the node mid-analysis must stop and join the worker before the
// members it uses (mutex_, rng_, publishers) are destroyed.
TEST_F(LabResultTest, DestroyingControllerMidAnalysisJoinsWorker) {
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("analysis_duration", 5.0))
          .successful);
  auto goal = send_goal();
  ASSERT_NE(goal, nullptr);
  ASSERT_TRUE(wait_for([this] { return !feedback_.empty(); }));
  executor_.remove_node(controller_);
  controller_.reset();
}

// Out-of-range startup values (YAML / overrides) must be rejected instead of
// silently driving the simulation with e.g. a negative duration.
TEST_F(LabResultTest, RejectsOutOfRangeStartupValues) {
  for (const auto& parameter : {rclcpp::Parameter("analysis_duration", -1.0),
                                rclcpp::Parameter("analysis_duration", 0.0),
                                rclcpp::Parameter("publish_rate", 0.0)}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW(std::make_shared<LabControllerNode>(options),
                 rclcpp::exceptions::InvalidParameterValueException)
        << parameter.get_name() << "=" << parameter.value_to_string();
  }
}

// A batch with one invalid value must not apply any of its values, neither to
// the parameter store nor to the value the worker actually uses.
TEST_F(LabResultTest, RejectedParameterBatchAppliesNothing) {
  const auto result = controller_->set_parameters_atomically(
      {rclcpp::Parameter("analysis_duration", 5.0),
       rclcpp::Parameter("publish_rate", -1.0)});
  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(controller_->get_parameter("analysis_duration").as_double(),
                   0.5);
  EXPECT_DOUBLE_EQ(controller_->get_parameter("publish_rate").as_double(),
                   20.0);

  // The analysis still takes the original 0.5 s, not the rejected 5 s.
  auto goal = send_goal();
  ASSERT_NE(goal, nullptr);
  auto analysis = client_->async_get_result(goal);
  ASSERT_TRUE(wait_for(
      [&analysis] {
        return analysis.wait_for(0s) == std::future_status::ready;
      },
      2000ms));
  EXPECT_EQ(analysis.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(LabResultTest, ValidParameterBatchIsApplied) {
  ASSERT_TRUE(controller_
                  ->set_parameters_atomically(
                      {rclcpp::Parameter("analysis_duration", 0.1),
                       rclcpp::Parameter("publish_rate", 50.0)})
                  .successful);
  EXPECT_DOUBLE_EQ(controller_->get_parameter("analysis_duration").as_double(),
                   0.1);
  EXPECT_DOUBLE_EQ(controller_->get_parameter("publish_rate").as_double(),
                   50.0);
}

}  // namespace hospital_controller
