// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <thread>
#include <vector>

// The node is defined only in its translation unit; include it to test
// internals.
#include "../src/robot_server_node.cpp"  // NOLINT(build/include)
#include "gtest/gtest.h"

using namespace std::chrono_literals;

namespace hospital_robot {
class RobotServerTest : public testing::Test {
 protected:
  using ExecuteTask = hospital_interfaces::action::ExecuteTask;
  using TaskGoal = rclcpp_action::ClientGoalHandle<ExecuteTask>;

  static void SetUpTestSuite() {
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(43);
    rclcpp::init(0, nullptr, init_options);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    // No controllers run in this test, so the protocol's first step must be a
    // plain (door-less) navigation; a long travel time keeps the task running
    // while the test interrupts it.
    options.parameter_overrides(
        {rclcpp::Parameter("robot_id", "test_robot"),
         rclcpp::Parameter("travel_duration", 5.0),
         rclcpp::Parameter("interaction_duration", 5.0)});
    robot_ = std::make_shared<RobotServerNode>(options);
    client_node_ = std::make_shared<rclcpp::Node>("robot_server_test");
    client_ = rclcpp_action::create_client<ExecuteTask>(
        client_node_, "/hospital/test_robot/ward/execute_task");
    executor_.add_node(robot_);
    executor_.add_node(client_node_);
    ASSERT_TRUE(client_->wait_for_action_server(10s));
  }

  void TearDown() override {
    if (robot_) {
      executor_.remove_node(robot_);
    }
    executor_.remove_node(client_node_);
    robot_.reset();
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

  TaskGoal::SharedPtr send(uint8_t task_type, const std::string& task_id) {
    ExecuteTask::Goal goal;
    goal.value = task_type;
    goal.task_id = task_id;
    rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
    options.feedback_callback =
        [this](TaskGoal::SharedPtr,
               const std::shared_ptr<const ExecuteTask::Feedback> feedback) {
          if (feedback->current_phase != ExecuteTask::Goal::PHASE_QUEUED &&
              (started_.empty() || started_.back() != feedback->task_id)) {
            started_.push_back(feedback->task_id);
          }
        };
    auto response = client_->async_send_goal(goal, options);
    if (!ready(response)) {
      ADD_FAILURE() << "Timed out waiting for task goal response";
      return nullptr;
    }
    return response.get();
  }

  // Give the worker time to start the protocol. (Its first feedback is sent
  // before the client has processed the goal response, so rclcpp drops it and
  // it cannot be used as a start signal.)
  void let_task_start() {
    wait_for([] { return false; }, 300ms);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<RobotServerNode> robot_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr client_;
  // Task ids in the order their first non-QUEUED feedback arrived.
  std::vector<std::string> started_;
};

TEST_F(RobotServerTest, RejectsUnknownTaskType) {
  EXPECT_EQ(send(/*task_type=*/200, "unknown"), nullptr);
}

TEST_F(RobotServerTest, CancelRunningTaskReportsCanceled) {
  auto goal = send(ExecuteTask::Goal::CLEAN_OR, "cancel_running");
  ASSERT_NE(goal, nullptr);
  let_task_start();
  auto result = client_->async_get_result(goal);
  client_->async_cancel_goal(goal);
  ASSERT_TRUE(ready(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(result.get().result->success);
}

// Destroying the node mid-task interrupts the worker without a cancel request.
// The worker must ABORT (calling canceled() on a goal that is not canceling
// throws inside the std::jthread -> std::terminate) and must be joined before
// the node's members are destroyed.
TEST_F(RobotServerTest, DestroyingNodeMidTaskAbortsInsteadOfCrashing) {
  auto running = send(ExecuteTask::Goal::CLEAN_OR, "running");
  ASSERT_NE(running, nullptr);
  let_task_start();
  // A second task is queued behind the first one and is interrupted while it
  // still waits for the robot.
  auto queued = send(ExecuteTask::Goal::CLEAN_OR, "queued");
  ASSERT_NE(queued, nullptr);
  executor_.remove_node(robot_);
  robot_.reset();
}

// A task waiting in the queue must be cancellable right away, without waiting
// for the running task to finish (each of its steps takes 5 s here).
TEST_F(RobotServerTest, CancelQueuedTaskIsCanceledPromptly) {
  auto running = send(ExecuteTask::Goal::CLEAN_OR, "running");
  ASSERT_NE(running, nullptr);
  let_task_start();
  auto queued = send(ExecuteTask::Goal::CLEAN_OR, "queued");
  ASSERT_NE(queued, nullptr);
  auto queued_result = client_->async_get_result(queued);
  auto running_result = client_->async_get_result(running);

  const auto start = std::chrono::steady_clock::now();
  client_->async_cancel_goal(queued);
  ASSERT_TRUE(ready(queued_result));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
  EXPECT_EQ(queued_result.get().code, rclcpp_action::ResultCode::CANCELED);
  // The running task is unaffected.
  EXPECT_NE(running_result.wait_for(0s), std::future_status::ready);

  client_->async_cancel_goal(running);
  ASSERT_TRUE(ready(running_result));
  EXPECT_EQ(running_result.get().code, rclcpp_action::ResultCode::CANCELED);
}

// Queued tasks run in the order they were accepted.
TEST_F(RobotServerTest, QueuedTasksStartInFifoOrder) {
  auto first = send(ExecuteTask::Goal::CLEAN_OR, "first");
  ASSERT_NE(first, nullptr);
  let_task_start();
  const std::vector<std::string> ids{"second", "third", "fourth"};
  std::vector<TaskGoal::SharedPtr> queued;
  for (const auto& id : ids) {
    queued.push_back(send(ExecuteTask::Goal::CLEAN_OR, id));
    ASSERT_NE(queued.back(), nullptr);
  }
  let_task_start();

  // Cancel the running task; the oldest queued task must be the one to start.
  auto current = first;
  for (std::size_t i = 0; i < queued.size(); ++i) {
    auto result = client_->async_get_result(current);
    client_->async_cancel_goal(current);
    ASSERT_TRUE(ready(result));
    ASSERT_TRUE(wait_for([this, &ids, i] {
      return !started_.empty() && started_.back() == ids[i];
    })) << "expected '"
        << ids[i] << "' to start next";
    current = queued[i];
  }
  auto result = client_->async_get_result(current);
  client_->async_cancel_goal(current);
  ASSERT_TRUE(ready(result));

  // "first" may have published its non-QUEUED feedback before the client
  // processed its goal response (then it is dropped), so compare the tail.
  ASSERT_GE(started_.size(), ids.size());
  EXPECT_TRUE(std::equal(ids.begin(), ids.end(), started_.end() - ids.size()));
}

}  // namespace hospital_robot
