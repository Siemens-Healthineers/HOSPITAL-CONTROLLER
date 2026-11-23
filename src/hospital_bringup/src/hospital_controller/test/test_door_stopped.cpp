// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include <chrono>
#include <functional>
#include <thread>
#include <tuple>

#include "gtest/gtest.h"
#include "hospital_controller/door_controller_node.hpp"

using namespace std::chrono_literals;

namespace hospital_controller {
class DoorStoppedTest : public testing::TestWithParam<std::tuple<bool, bool>> {
 protected:
  using OperateDoor = hospital_interfaces::action::OperateDoor;
  using DoorState = hospital_interfaces::msg::DoorState;
  using DoorStateArray = hospital_interfaces::msg::DoorStateArray;
  using GetDoorStates = hospital_interfaces::srv::GetDoorStates;
  using DoorGoal = rclcpp_action::ClientGoalHandle<OperateDoor>;

  static void SetUpTestSuite() {
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(41);
    rclcpp::init(0, nullptr, init_options);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("door_ids", std::vector<std::string>{"test_door"}),
         rclcpp::Parameter("open_duration", 1.0),
         rclcpp::Parameter("close_duration", 1.0),
         rclcpp::Parameter("publish_rate", 50.0)});
    controller_ = std::make_shared<DoorControllerNode>(options);
    client_node_ = std::make_shared<rclcpp::Node>("door_stopped_test");
    action_client_ = rclcpp_action::create_client<OperateDoor>(
        client_node_, "/hospital/doors/operate");
    states_client_ = client_node_->create_client<GetDoorStates>(
        "/hospital/doors/get_states");
    state_sub_ = client_node_->create_subscription<DoorState>(
        "/hospital/doors/state", rclcpp::QoS(10).transient_local(),
        [this](DoorState::ConstSharedPtr state) { state_ = *state; });
    states_sub_ = client_node_->create_subscription<DoorStateArray>(
        "/hospital/doors/states", rclcpp::QoS(10).transient_local(),
        [this](DoorStateArray::ConstSharedPtr states) { states_ = *states; });
    executor_.add_node(controller_);
    executor_.add_node(client_node_);
    ASSERT_TRUE(action_client_->wait_for_action_server(10s));
    ASSERT_TRUE(states_client_->wait_for_service(10s));
    ASSERT_TRUE(wait_for([this] { return state_.device_id == "test_door"; }));
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

  DoorGoal::SharedPtr send(bool opening) {
    OperateDoor::Goal goal;
    goal.device_id = "test_door";
    goal.command = opening ? OperateDoor::Goal::COMMAND_OPEN
                           : OperateDoor::Goal::COMMAND_CLOSE;
    auto response = action_client_->async_send_goal(goal);
    if (!ready(response)) {
      ADD_FAILURE() << "Timed out waiting for door goal response";
      return nullptr;
    }
    return response.get();
  }

  void finish(bool opening) {
    auto goal = send(opening);
    ASSERT_NE(goal, nullptr);
    auto result = action_client_->async_get_result(goal);
    ASSERT_TRUE(ready(result));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(result.get().result->success);
    EXPECT_FLOAT_EQ(result.get().result->final_percentage,
                    opening ? 100.0f : 0.0f);
    ASSERT_TRUE(wait_for([this, opening] {
      return state_.state == (opening ? DoorState::OPEN : DoorState::CLOSED);
    }));
  }

  void check_snapshot(uint8_t expected_state, float expected_percentage) {
    ASSERT_TRUE(wait_for([this, expected_state, expected_percentage] {
      return state_.state == expected_state &&
             state_.percentage_open == expected_percentage &&
             states_.doors.size() == 1 &&
             states_.doors.front().state == expected_state &&
             states_.doors.front().percentage_open == expected_percentage;
    }));
    auto response = states_client_->async_send_request(
        std::make_shared<GetDoorStates::Request>());
    ASSERT_TRUE(ready(response));
    const auto snapshot = response.get()->door_states;
    ASSERT_EQ(snapshot.doors.size(), 1u);
    EXPECT_EQ(snapshot.doors.front().state, expected_state);
    EXPECT_FLOAT_EQ(snapshot.doors.front().percentage_open,
                    expected_percentage);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<DoorControllerNode> controller_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<OperateDoor>::SharedPtr action_client_;
  rclcpp::Client<GetDoorStates>::SharedPtr states_client_;
  rclcpp::Subscription<DoorState>::SharedPtr state_sub_;
  rclcpp::Subscription<DoorStateArray>::SharedPtr states_sub_;
  DoorState state_;
  DoorStateArray states_;
};

TEST_P(DoorStoppedTest, ResultCallbackCanImmediatelyReverseDoor) {
  const auto [opening, cancel_first] = GetParam();
  ASSERT_TRUE(controller_
                  ->set_parameter(rclcpp::Parameter("open_duration",
                                                    cancel_first ? 0.2 : 0.03))
                  .successful);
  ASSERT_TRUE(controller_
                  ->set_parameter(rclcpp::Parameter("close_duration",
                                                    cancel_first ? 0.2 : 0.03))
                  .successful);
  if (!opening) {
    ASSERT_NO_FATAL_FAILURE(finish(true));
  }
  for (int cycle = 0; cycle < 3; ++cycle) {
    std::shared_future<DoorGoal::SharedPtr> reversal;
    bool completed = false;
    rclcpp_action::Client<OperateDoor>::SendGoalOptions options;
    options.result_callback =
        [this, opening, cancel_first, &reversal,
         &completed](const DoorGoal::WrappedResult& result) {
          EXPECT_EQ(result.code, cancel_first
                                     ? rclcpp_action::ResultCode::CANCELED
                                     : rclcpp_action::ResultCode::SUCCEEDED);
          OperateDoor::Goal reverse;
          reverse.device_id = "test_door";
          reverse.command = opening ? OperateDoor::Goal::COMMAND_CLOSE
                                    : OperateDoor::Goal::COMMAND_OPEN;
          reversal = action_client_->async_send_goal(reverse);
          completed = true;
        };
    OperateDoor::Goal goal;
    goal.device_id = "test_door";
    goal.command = opening ? OperateDoor::Goal::COMMAND_OPEN
                           : OperateDoor::Goal::COMMAND_CLOSE;
    auto response = action_client_->async_send_goal(goal, options);
    ASSERT_TRUE(ready(response));
    ASSERT_NE(response.get(), nullptr);
    if (cancel_first) {
      ASSERT_TRUE(wait_for([this] {
        return state_.percentage_open >= 25.0f &&
               state_.percentage_open <= 75.0f;
      }));
      auto cancel = action_client_->async_cancel_goal(response.get());
      ASSERT_TRUE(ready(cancel));
      EXPECT_EQ(cancel.get()->goals_canceling.size(), 1u);
    }
    ASSERT_TRUE(wait_for([&completed] { return completed; }));
    ASSERT_TRUE(ready(reversal));
    ASSERT_NE(reversal.get(), nullptr) << "Cycle " << cycle;
    auto result = action_client_->async_get_result(reversal.get());
    ASSERT_TRUE(ready(result));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_FLOAT_EQ(result.get().result->final_percentage,
                    opening ? 0.0f : 100.0f);
  }
}

TEST_P(DoorStoppedTest, CancelHoldsPositionAndNewGoalReachesEndpoint) {
  const auto [opening, restart_opening] = GetParam();
  if (!opening) {
    ASSERT_NO_FATAL_FAILURE(finish(true));
  }
  auto goal = send(opening);
  ASSERT_NE(goal, nullptr);
  auto result = action_client_->async_get_result(goal);
  ASSERT_TRUE(wait_for([this, opening] {
    return state_.state ==
               (opening ? DoorState::OPENING : DoorState::CLOSING) &&
           state_.percentage_open >= 30.0f && state_.percentage_open <= 70.0f;
  }));
  auto cancel = action_client_->async_cancel_goal(goal);
  ASSERT_TRUE(ready(cancel));
  EXPECT_EQ(cancel.get()->goals_canceling.size(), 1u);
  ASSERT_TRUE(ready(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_EQ(result.get().result->message, "Operation canceled");
  const float held_percentage = result.get().result->final_percentage;
  EXPECT_GT(held_percentage, 0.0f);
  EXPECT_LT(held_percentage, 100.0f);
  ASSERT_NO_FATAL_FAILURE(check_snapshot(DoorState::STOPPED, held_percentage));
  EXPECT_FALSE(wait_for(
      [this, held_percentage] {
        return state_.state != DoorState::STOPPED ||
               state_.percentage_open != held_percentage;
      },
      100ms));
  ASSERT_NO_FATAL_FAILURE(finish(restart_opening));
}

TEST_P(DoorStoppedTest, CancelBeforeFirstStepPreservesEndpoint) {
  const bool opening = std::get<0>(GetParam());
  if (!opening) {
    ASSERT_NO_FATAL_FAILURE(finish(true));
  }
  ASSERT_TRUE(controller_->set_parameter(rclcpp::Parameter("publish_rate", 0.1))
                  .successful);
  auto goal = send(opening);
  ASSERT_NE(goal, nullptr);
  auto result = action_client_->async_get_result(goal);
  auto cancel = action_client_->async_cancel_goal(goal);
  ASSERT_TRUE(ready(cancel));
  ASSERT_TRUE(ready(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(result.get().result->success);
  EXPECT_FLOAT_EQ(result.get().result->final_percentage,
                  opening ? 0.0f : 100.0f);
  ASSERT_NO_FATAL_FAILURE(check_snapshot(
      opening ? DoorState::CLOSED : DoorState::OPEN, opening ? 0.0f : 100.0f));
}

// Destroying the node while a door moves must stop and join the worker before
// the node's members go away (previously use-after-destroy / self-join).
TEST_P(DoorStoppedTest, DestroyingControllerMidMotionJoinsWorker) {
  auto goal = send(/*opening=*/true);
  ASSERT_NE(goal, nullptr);
  ASSERT_TRUE(wait_for([this] { return state_.state == DoorState::OPENING; }));
  executor_.remove_node(controller_);
  controller_.reset();
}

TEST_P(DoorStoppedTest, RejectsOutOfRangeParameters) {
  for (const auto& parameter : {rclcpp::Parameter("open_duration", -1.0),
                                rclcpp::Parameter("close_duration", 0.0),
                                rclcpp::Parameter("publish_rate", 0.0)}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW(std::make_shared<DoorControllerNode>(options),
                 rclcpp::exceptions::InvalidParameterValueException)
        << parameter.get_name() << "=" << parameter.value_to_string();
  }
  EXPECT_FALSE(controller_
                   ->set_parameters_atomically(
                       {rclcpp::Parameter("open_duration", 2.0),
                        rclcpp::Parameter("close_duration", -2.0)})
                   .successful);
  EXPECT_DOUBLE_EQ(controller_->get_parameter("open_duration").as_double(),
                   1.0);
}

TEST_P(DoorStoppedTest, RejectsDuplicateDoorIdsAtStartup) {
  for (const auto& ids :
       {std::vector<std::string>{"first", "first"},
        std::vector<std::string>{"first", "second", "first"}}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("door_ids", ids)});
    EXPECT_THROW(std::make_shared<DoorControllerNode>(options),
                 rclcpp::exceptions::InvalidParameterValueException);
  }
}

INSTANTIATE_TEST_SUITE_P(OpenAndClose, DoorStoppedTest,
                         testing::Combine(testing::Bool(), testing::Bool()));
}  // namespace hospital_controller
