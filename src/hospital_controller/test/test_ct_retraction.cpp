// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "hospital_controller/controller_utils.hpp"
#include "hospital_controller/ct_scanner_controller_node.hpp"

using namespace std::chrono_literals;

namespace hospital_controller {

TEST(ControllerUtilsTest, AnimationDeadlinesDoNotAccumulateUpdateWork) {
  using Clock = std::chrono::steady_clock;
  Clock::duration update_time{};
  std::vector<double> fractions;
  const auto started = Clock::now();

  // Expired deadlines must not reintroduce sleeps after slow updates.
  EXPECT_TRUE(animate_steps(
      {}, 0.4, 25.0,
      [&](double fraction) {
        const auto update_started = Clock::now();
        std::this_thread::sleep_for(20ms);
        fractions.push_back(fraction);
        update_time += Clock::now() - update_started;
      },
      started - 1s));
  EXPECT_LT(Clock::now() - started - update_time, 200ms);
  ASSERT_EQ(fractions.size(), 10u);
  EXPECT_TRUE(std::is_sorted(fractions.begin(), fractions.end()));
  EXPECT_DOUBLE_EQ(fractions.back(), 1.0);
}

TEST(ControllerUtilsTest, StopInterruptsLongAnimationWaitBeforeAnyUpdate) {
  std::promise<void> started;
  auto ready = started.get_future();
  bool completed = true;
  int updates = 0;
  std::jthread worker([&](std::stop_token stop_token) {
    started.set_value();
    completed = animate_steps(stop_token, 5.0, 0.1, [&](double) { ++updates; });
  });
  ASSERT_EQ(ready.wait_for(1s), std::future_status::ready);
  std::this_thread::sleep_for(25ms);
  const auto stopped = std::chrono::steady_clock::now();
  worker.request_stop();
  worker.join();
  EXPECT_LT(std::chrono::steady_clock::now() - stopped, 500ms);
  EXPECT_FALSE(completed);
  EXPECT_EQ(updates, 0);
}

TEST(ControllerUtilsTest, CleanupReleasesReservationWhenCompletionThrows) {
  std::mutex mutex;
  bool running = true;
  bool cleaned_up = false;
  bool reserved_during_cleanup = false;
  const auto complete = [&] {
    RunReservation reservation{mutex, running};
    ScopeGuard cleanup{[&]() noexcept {
      reserved_during_cleanup = running;
      cleaned_up = true;
    }};
    reservation.complete([&] {
      EXPECT_TRUE(running);
      throw std::runtime_error("completion failed");
    });
  };
  EXPECT_THROW(complete(), std::runtime_error);
  EXPECT_TRUE(cleaned_up);
  EXPECT_TRUE(reserved_during_cleanup);
  EXPECT_FALSE(running);
}

class CtRetractionTest : public testing::TestWithParam<bool> {
 protected:
  using RunScan = hospital_interfaces::action::RunScan;
  using MoveTable = hospital_interfaces::action::MoveTable;
  using ScanState = hospital_interfaces::msg::CtScannerState;
  using TableState = hospital_interfaces::msg::CtTableState;
  using ProcedureState = hospital_interfaces::msg::CtScanProcedureState;
  using ScanGoal = rclcpp_action::ClientGoalHandle<RunScan>;

  static void SetUpTestSuite() {
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(45);
    rclcpp::init(0, nullptr, init_options);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("table_move_duration", 0.1),
                                 rclcpp::Parameter("scan_duration", 8.0),
                                 rclcpp::Parameter("publish_rate", 20.0)});
    controller_ = std::make_shared<CtScannerControllerNode>(options);
    client_node_ = std::make_shared<rclcpp::Node>("ct_retraction_test");
    scan_client_ = rclcpp_action::create_client<RunScan>(
        client_node_, "/hospital/ct/scan/run");
    move_client_ = rclcpp_action::create_client<MoveTable>(
        client_node_, "/hospital/ct/table/move");
    scan_sub_ = client_node_->create_subscription<ScanState>(
        "/hospital/ct/scan/state", rclcpp::QoS(10).transient_local(),
        [this](ScanState::ConstSharedPtr state) { scan_state_ = *state; });
    table_sub_ = client_node_->create_subscription<TableState>(
        "/hospital/ct/table/state", rclcpp::QoS(10).transient_local(),
        [this](TableState::ConstSharedPtr state) { table_state_ = *state; });
    procedure_sub_ = client_node_->create_subscription<ProcedureState>(
        "/hospital/ct/scan/procedure_state", rclcpp::QoS(1).transient_local(),
        [this](ProcedureState::ConstSharedPtr state) {
          procedure_state_ = *state;
          procedure_snapshots_.push_back(*state);
        });
    stopwatch_sub_ =
        client_node_->create_subscription<hospital_interfaces::msg::Stopwatch>(
            "/hospital/stopwatch", rclcpp::QoS(20).transient_local(),
            [this](hospital_interfaces::msg::Stopwatch::ConstSharedPtr state) {
              stopwatches_.push_back(*state);
            });
    executor_.add_node(controller_);
    executor_.add_node(client_node_);
    ASSERT_TRUE(scan_client_->wait_for_action_server(10s));
    ASSERT_TRUE(move_client_->wait_for_action_server(10s));
    ASSERT_TRUE(wait_for([this] { return !procedure_snapshots_.empty(); }));
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

  ScanGoal::SharedPtr send_scan(uint8_t command) {
    RunScan::Goal goal;
    goal.command = command;
    rclcpp_action::Client<RunScan>::SendGoalOptions options;
    options.feedback_callback =
        [this](ScanGoal::SharedPtr,
               const std::shared_ptr<const RunScan::Feedback> feedback) {
          feedback_.push_back(*feedback);
        };
    auto future = scan_client_->async_send_goal(goal, options);
    if (!ready(future)) {
      ADD_FAILURE() << "Timed out waiting for scan goal response";
      return nullptr;
    }
    return future.get();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<CtScannerControllerNode> controller_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<RunScan>::SharedPtr scan_client_;
  rclcpp_action::Client<MoveTable>::SharedPtr move_client_;
  rclcpp::Subscription<ScanState>::SharedPtr scan_sub_;
  rclcpp::Subscription<TableState>::SharedPtr table_sub_;
  rclcpp::Subscription<ProcedureState>::SharedPtr procedure_sub_;
  rclcpp::Subscription<hospital_interfaces::msg::Stopwatch>::SharedPtr
      stopwatch_sub_;
  ScanState scan_state_;
  TableState table_state_;
  ProcedureState procedure_state_;
  std::vector<ProcedureState> procedure_snapshots_;
  std::vector<RunScan::Feedback> feedback_;
  std::vector<hospital_interfaces::msg::Stopwatch> stopwatches_;
};

TEST_P(CtRetractionTest, IdlePreparationKeepsReservationUntilResult) {
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("table_move_duration", 2.0))
          .successful);
  auto start = send_scan(RunScan::Goal::COMMAND_START);
  ASSERT_NE(start, nullptr);
  auto result = scan_client_->async_get_result(start);
  ASSERT_TRUE(wait_for([this] {
    return procedure_state_.phase == ProcedureState::PHASE_PREPARING_LIFT &&
           procedure_state_.table_telemetry.percentage_done >= 10.0f;
  }));
  EXPECT_EQ(procedure_state_.telemetry.state, ScanState::IDLE);
  EXPECT_EQ(send_scan(RunScan::Goal::COMMAND_START), nullptr);
  EXPECT_EQ(send_scan(RunScan::Goal::COMMAND_RETRACT), nullptr);
  MoveTable::Goal down;
  down.command = MoveTable::Goal::COMMAND_DOWN;
  auto movement = move_client_->async_send_goal(down);
  ASSERT_TRUE(ready(movement));
  EXPECT_EQ(movement.get(), nullptr);

  if (GetParam()) {
    auto cancel = scan_client_->async_cancel_goal(start);
    ASSERT_TRUE(ready(cancel));
    EXPECT_EQ(cancel.get()->goals_canceling.size(), 1u);
  } else {
    ASSERT_NE(send_scan(RunScan::Goal::COMMAND_STOP), nullptr);
  }
  ASSERT_TRUE(ready(result));
  EXPECT_EQ(result.get().code, GetParam() ? rclcpp_action::ResultCode::CANCELED
                                          : rclcpp_action::ResultCode::ABORTED);
  auto retract = send_scan(RunScan::Goal::COMMAND_RETRACT);
  ASSERT_NE(retract, nullptr);
  auto retraction = scan_client_->async_get_result(retract);
  ASSERT_TRUE(ready(retraction));
  EXPECT_EQ(retraction.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  movement = move_client_->async_send_goal(down);
  ASSERT_TRUE(ready(movement));
  ASSERT_NE(movement.get(), nullptr);
  auto lowered = move_client_->async_get_result(movement.get());
  ASSERT_TRUE(ready(lowered));
  EXPECT_EQ(lowered.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_P(CtRetractionTest, ScanResultCallbackCanImmediatelyLowerTable) {
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("table_move_duration", 0.02))
          .successful);
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("scan_duration", 0.02))
          .successful);
  for (int cycle = 0; cycle < 10; ++cycle) {
    using MoveGoal = rclcpp_action::ClientGoalHandle<MoveTable>;
    std::shared_future<MoveGoal::SharedPtr> movement;
    bool completed = false;
    rclcpp_action::Client<RunScan>::SendGoalOptions options;
    options.result_callback =
        [this, &movement, &completed](const ScanGoal::WrappedResult& result) {
          EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
          MoveTable::Goal down;
          down.command = MoveTable::Goal::COMMAND_DOWN;
          movement = move_client_->async_send_goal(down);
          completed = true;
        };
    RunScan::Goal scan;
    scan.command = RunScan::Goal::COMMAND_START;
    auto response = scan_client_->async_send_goal(scan, options);
    ASSERT_TRUE(ready(response));
    ASSERT_NE(response.get(), nullptr);
    ASSERT_TRUE(wait_for([&completed] { return completed; }));
    ASSERT_TRUE(ready(movement));
    ASSERT_NE(movement.get(), nullptr) << "Cycle " << cycle;
    auto lowered = move_client_->async_get_result(movement.get());
    ASSERT_TRUE(ready(lowered));
    EXPECT_EQ(lowered.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  }
}

TEST_P(CtRetractionTest, LiftInterruptionReportsRestingStateAndPhase) {
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("table_move_duration", 2.0))
          .successful);
  auto start = send_scan(RunScan::Goal::COMMAND_START);
  ASSERT_NE(start, nullptr);
  auto result = scan_client_->async_get_result(start);
  ASSERT_TRUE(wait_for([this] {
    return !feedback_.empty() &&
           feedback_.back().phase == ProcedureState::PHASE_PREPARING_LIFT &&
           feedback_.back().table_telemetry.percentage_done >= 10.0f;
  }));
  for (const auto& feedback : feedback_) {
    EXPECT_EQ(feedback.phase, ProcedureState::PHASE_PREPARING_LIFT);
    EXPECT_EQ(feedback.telemetry.state, ScanState::IDLE);
    EXPECT_FLOAT_EQ(feedback.telemetry.percentage_done, 0.0f);
    EXPECT_FALSE(feedback.telemetry.radiating);
  }
  for (const auto& stopwatch : stopwatches_) {
    EXPECT_FALSE(stopwatch.running);
    EXPECT_FLOAT_EQ(stopwatch.elapsed_seconds, 0.0f);
  }
  if (GetParam()) {
    auto cancel = scan_client_->async_cancel_goal(start);
    ASSERT_TRUE(ready(cancel));
    EXPECT_EQ(cancel.get()->goals_canceling.size(), 1u);
  } else {
    ASSERT_NE(send_scan(RunScan::Goal::COMMAND_STOP), nullptr);
  }
  ASSERT_TRUE(ready(result));
  EXPECT_EQ(result.get().code, GetParam() ? rclcpp_action::ResultCode::CANCELED
                                          : rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(wait_for([this] {
    return procedure_state_.phase == ProcedureState::PHASE_INTERRUPTED &&
           table_state_.state == TableState::PAUSED && !feedback_.empty() &&
           feedback_.back().phase == ProcedureState::PHASE_INTERRUPTED;
  }));
  EXPECT_EQ(procedure_state_.table_telemetry.state, TableState::PAUSED);
  EXPECT_EQ(feedback_.back().table_telemetry.state, TableState::PAUSED);
  EXPECT_GT(table_state_.percentage_done, 0.0f);
  EXPECT_LT(table_state_.percentage_done, 100.0f);
  EXPECT_FALSE(procedure_state_.telemetry.radiating);
  const float height = table_state_.percentage_done;
  EXPECT_FALSE(wait_for(
      [this, height] { return table_state_.percentage_done != height; },
      100ms));
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("table_move_duration", 0.2))
          .successful);
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("scan_duration", 0.4))
          .successful);
  feedback_.clear();
  procedure_snapshots_.clear();
  start = send_scan(RunScan::Goal::COMMAND_START);
  ASSERT_NE(start, nullptr);
  result = scan_client_->async_get_result(start);
  ASSERT_TRUE(ready(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_FALSE(feedback_.empty());
  EXPECT_EQ(feedback_.front().phase, ProcedureState::PHASE_PREPARING_LIFT);
  EXPECT_TRUE(std::any_of(
      procedure_snapshots_.begin(), procedure_snapshots_.end(),
      [height](const ProcedureState& state) {
        return state.phase == ProcedureState::PHASE_PREPARING_LIFT &&
               state.table_telemetry.percentage_done == height;
      }));
}

TEST_P(CtRetractionTest, LiftStopBeforeFirstStepReportsBottom) {
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("table_move_duration", 2.0))
          .successful);
  ASSERT_TRUE(controller_->set_parameter(rclcpp::Parameter("publish_rate", 0.1))
                  .successful);
  auto start = send_scan(RunScan::Goal::COMMAND_START);
  ASSERT_NE(start, nullptr);
  auto result = scan_client_->async_get_result(start);
  ASSERT_TRUE(wait_for([this] {
    return procedure_state_.phase == ProcedureState::PHASE_PREPARING_LIFT;
  }));
  if (GetParam()) {
    auto cancel = scan_client_->async_cancel_goal(start);
    ASSERT_TRUE(ready(cancel));
    EXPECT_EQ(cancel.get()->goals_canceling.size(), 1u);
  } else {
    ASSERT_NE(send_scan(RunScan::Goal::COMMAND_STOP), nullptr);
  }
  ASSERT_TRUE(ready(result));
  ASSERT_TRUE(wait_for([this] {
    return procedure_state_.phase == ProcedureState::PHASE_INTERRUPTED &&
           table_state_.state == TableState::BOTTOM;
  }));
  EXPECT_EQ(procedure_state_.table_telemetry.state, TableState::BOTTOM);
  EXPECT_FLOAT_EQ(procedure_state_.table_telemetry.percentage_done, 0.0f);
  EXPECT_FALSE(procedure_state_.telemetry.radiating);
}

TEST_P(CtRetractionTest, AlreadyTopStartKeepsLiftAtRest) {
  MoveTable::Goal up;
  up.command = MoveTable::Goal::COMMAND_UP;
  auto response = move_client_->async_send_goal(up);
  ASSERT_TRUE(ready(response));
  ASSERT_NE(response.get(), nullptr);
  auto movement = move_client_->async_get_result(response.get());
  ASSERT_TRUE(ready(movement));
  ASSERT_TRUE(
      wait_for([this] { return table_state_.state == TableState::TOP; }));
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("scan_duration", 0.4))
          .successful);
  auto start = send_scan(RunScan::Goal::COMMAND_START);
  ASSERT_NE(start, nullptr);
  auto result = scan_client_->async_get_result(start);
  ASSERT_TRUE(ready(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_for([this] {
    return !feedback_.empty() &&
           feedback_.back().phase == ProcedureState::PHASE_COMPLETED;
  }));
  EXPECT_TRUE(feedback_.front().phase == ProcedureState::PHASE_PREPARING_LIFT ||
              feedback_.front().phase == ProcedureState::PHASE_SWEEP_IN);
  for (const auto& feedback : feedback_) {
    EXPECT_EQ(feedback.table_telemetry.state, TableState::TOP);
    EXPECT_FLOAT_EQ(feedback.table_telemetry.percentage_done, 100.0f);
  }
}

TEST_P(CtRetractionTest, FeedbackSeparatesLiftFromSweepAndLatchesCompletion) {
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("scan_duration", 0.4))
          .successful);
  auto start = send_scan(RunScan::Goal::COMMAND_START);
  ASSERT_NE(start, nullptr);
  auto result = scan_client_->async_get_result(start);
  ASSERT_TRUE(ready(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wait_for([this] {
    return !feedback_.empty() &&
           feedback_.back().phase == ProcedureState::PHASE_COMPLETED;
  }));
  std::vector<uint8_t> phases;
  for (const auto& feedback : feedback_) {
    if (phases.empty() || phases.back() != feedback.phase) {
      phases.push_back(feedback.phase);
    }
    if (feedback.phase == ProcedureState::PHASE_SWEEP_IN ||
        feedback.phase == ProcedureState::PHASE_SWEEP_OUT) {
      EXPECT_EQ(feedback.table_telemetry.state, TableState::TOP);
      EXPECT_FLOAT_EQ(feedback.table_telemetry.percentage_done, 100.0f);
      EXPECT_TRUE(feedback.telemetry.radiating);
      if (feedback.phase == ProcedureState::PHASE_SWEEP_IN) {
        EXPECT_LE(feedback.telemetry.percentage_done, 50.0f);
      } else {
        EXPECT_GE(feedback.telemetry.percentage_done, 50.0f);
      }
    }
  }
  EXPECT_EQ(phases, (std::vector<uint8_t>{ProcedureState::PHASE_PREPARING_LIFT,
                                          ProcedureState::PHASE_SWEEP_IN,
                                          ProcedureState::PHASE_SWEEP_OUT,
                                          ProcedureState::PHASE_COMPLETED}));
  ProcedureState late_state;
  bool received = false;
  auto late_sub = client_node_->create_subscription<ProcedureState>(
      "/hospital/ct/scan/procedure_state", rclcpp::QoS(1).transient_local(),
      [&late_state, &received](ProcedureState::ConstSharedPtr state) {
        late_state = *state;
        received = true;
      });
  ASSERT_TRUE(wait_for([&received] { return received; }));
  EXPECT_EQ(late_state.phase, ProcedureState::PHASE_COMPLETED);
  EXPECT_EQ(late_state.table_telemetry.state, TableState::TOP);
  EXPECT_FALSE(late_state.telemetry.radiating);
  EXPECT_FLOAT_EQ(late_state.telemetry.percentage_done, 0.0f);
  ASSERT_TRUE(wait_for([this] {
    return !stopwatches_.empty() && !stopwatches_.back().running &&
           stopwatches_.back().final_seconds > 0.0f;
  }));
  EXPECT_FALSE(stopwatches_.back().interrupted);
  EXPECT_GE(stopwatches_.back().final_seconds, 0.4f);
}

TEST_P(CtRetractionTest, HaltRecoveryAndFinishWithAnotherRetract) {
  auto start = send_scan(RunScan::Goal::COMMAND_START);
  ASSERT_NE(start, nullptr);
  auto start_result = scan_client_->async_get_result(start);
  ASSERT_TRUE(wait_for([this] {
    return scan_state_.state == ScanState::MOVING_IN &&
           scan_state_.percentage_done >= 20.0f;
  }));
  auto stop = send_scan(RunScan::Goal::COMMAND_STOP);
  ASSERT_NE(stop, nullptr);
  auto stop_result = scan_client_->async_get_result(stop);
  ASSERT_TRUE(ready(stop_result));
  EXPECT_TRUE(stop_result.get().result->success);
  ASSERT_TRUE(ready(start_result));
  EXPECT_EQ(start_result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_TRUE(start_result.get().result->success);
  ASSERT_TRUE(
      wait_for([this] { return scan_state_.state == ScanState::PAUSED; }));
  const float lift_position = table_state_.percentage_done;

  auto retract = send_scan(RunScan::Goal::COMMAND_RETRACT);
  ASSERT_NE(retract, nullptr);
  auto retract_result = scan_client_->async_get_result(retract);
  ASSERT_TRUE(wait_for([this] {
    return scan_state_.state == ScanState::MOVING_OUT &&
           scan_state_.percentage_done > 85.0f;
  }));
  EXPECT_FALSE(scan_state_.radiating);
  const auto stop_time = std::chrono::steady_clock::now();
  if (GetParam()) {
    auto cancel = scan_client_->async_cancel_goal(retract);
    ASSERT_TRUE(ready(cancel));
    EXPECT_EQ(cancel.get()->goals_canceling.size(), 1u);
  } else {
    stop = send_scan(RunScan::Goal::COMMAND_STOP);
    ASSERT_NE(stop, nullptr);
    stop_result = scan_client_->async_get_result(stop);
    ASSERT_TRUE(ready(stop_result));
    EXPECT_TRUE(stop_result.get().result->success);
  }
  ASSERT_TRUE(ready(retract_result));
  EXPECT_LT(std::chrono::steady_clock::now() - stop_time, 500ms);
  EXPECT_EQ(retract_result.get().code,
            GetParam() ? rclcpp_action::ResultCode::CANCELED
                       : rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(retract_result.get().result->success);
  EXPECT_EQ(retract_result.get().result->message,
            "Retraction halted; table-top holding position");
  ASSERT_TRUE(
      wait_for([this] { return scan_state_.state == ScanState::PAUSED; }));
  EXPECT_FALSE(scan_state_.radiating);
  EXPECT_FLOAT_EQ(table_state_.percentage_done, lift_position);
  const float held_position = scan_state_.percentage_done;
  EXPECT_FALSE(wait_for(
      [this, held_position] {
        return scan_state_.percentage_done != held_position;
      },
      200ms));

  for (const auto command :
       {MoveTable::Goal::COMMAND_UP, MoveTable::Goal::COMMAND_DOWN}) {
    MoveTable::Goal move;
    move.command = command;
    auto response = move_client_->async_send_goal(move);
    ASSERT_TRUE(ready(response));
    EXPECT_EQ(response.get(), nullptr);
  }

  retract = send_scan(RunScan::Goal::COMMAND_RETRACT);
  ASSERT_NE(retract, nullptr);
  retract_result = scan_client_->async_get_result(retract);
  ASSERT_TRUE(ready(retract_result));
  EXPECT_EQ(retract_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(retract_result.get().result->success);
  ASSERT_TRUE(
      wait_for([this] { return scan_state_.state == ScanState::IDLE; }));
  EXPECT_FALSE(scan_state_.radiating);
  EXPECT_FLOAT_EQ(table_state_.percentage_done, lift_position);

  retract = send_scan(RunScan::Goal::COMMAND_RETRACT);
  ASSERT_NE(retract, nullptr);
  retract_result = scan_client_->async_get_result(retract);
  ASSERT_TRUE(ready(retract_result));
  EXPECT_TRUE(retract_result.get().result->success);
  EXPECT_EQ(send_scan(RunScan::Goal::COMMAND_STOP), nullptr);
  ASSERT_TRUE(wait_for([this] {
    return !feedback_.empty() &&
           feedback_.back().phase == ProcedureState::PHASE_COMPLETED;
  }));
  bool saw_retract = false;
  for (const auto& feedback : feedback_) {
    if (feedback.phase == ProcedureState::PHASE_RETRACTING) {
      saw_retract = true;
      EXPECT_FALSE(feedback.telemetry.radiating);
      EXPECT_EQ(feedback.table_telemetry.state, TableState::TOP);
      EXPECT_FLOAT_EQ(feedback.table_telemetry.percentage_done, lift_position);
    }
  }
  EXPECT_TRUE(saw_retract);

  MoveTable::Goal down;
  down.command = MoveTable::Goal::COMMAND_DOWN;
  auto response = move_client_->async_send_goal(down);
  ASSERT_TRUE(ready(response));
  ASSERT_NE(response.get(), nullptr);
  auto down_result = move_client_->async_get_result(response.get());
  EXPECT_EQ(send_scan(RunScan::Goal::COMMAND_STOP), nullptr);
  ASSERT_TRUE(ready(down_result));
  EXPECT_EQ(down_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_P(CtRetractionTest, RejectEndpointGoalsWithoutReservingLift) {
  ASSERT_TRUE(
      wait_for([this] { return table_state_.state == TableState::BOTTOM; }));
  for (const auto command :
       {MoveTable::Goal::COMMAND_DOWN, MoveTable::Goal::COMMAND_UP,
        MoveTable::Goal::COMMAND_UP, MoveTable::Goal::COMMAND_DOWN,
        MoveTable::Goal::COMMAND_DOWN, MoveTable::Goal::COMMAND_UP}) {
    MoveTable::Goal goal;
    goal.command = command;
    const bool at_target = command == MoveTable::Goal::COMMAND_UP
                               ? table_state_.state == TableState::TOP
                               : table_state_.state == TableState::BOTTOM;
    auto response = move_client_->async_send_goal(goal);
    ASSERT_TRUE(ready(response));
    auto handle = response.get();
    if (at_target) {
      ASSERT_EQ(handle, nullptr);
      continue;
    }
    ASSERT_NE(handle, nullptr);
    auto result = move_client_->async_get_result(handle);
    ASSERT_TRUE(ready(result));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(result.get().result->success);
    const auto target = command == MoveTable::Goal::COMMAND_UP
                            ? TableState::TOP
                            : TableState::BOTTOM;
    EXPECT_EQ(result.get().result->final_position, target);
    ASSERT_TRUE(
        wait_for([this, target] { return table_state_.state == target; }));
  }
}

// Destroying the node mid-scan must stop and join the worker before the node's
// members go away (the worker co-owns the runtime and could self-join).
TEST_P(CtRetractionTest, DestroyingControllerMidScanJoinsWorker) {
  auto start = send_scan(RunScan::Goal::COMMAND_START);
  ASSERT_NE(start, nullptr);
  ASSERT_TRUE(wait_for([this] {
    return procedure_state_.phase == ProcedureState::PHASE_SWEEP_IN;
  }));
  executor_.remove_node(controller_);
  controller_.reset();
}

TEST_P(CtRetractionTest, RejectsOutOfRangeParameters) {
  for (const auto& parameter : {rclcpp::Parameter("table_move_duration", -1.0),
                                rclcpp::Parameter("scan_duration", 0.0),
                                rclcpp::Parameter("publish_rate", 0.0)}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW(std::make_shared<CtScannerControllerNode>(options),
                 rclcpp::exceptions::InvalidParameterValueException)
        << parameter.get_name() << "=" << parameter.value_to_string();
  }
  EXPECT_FALSE(
      controller_
          ->set_parameters_atomically({rclcpp::Parameter("scan_duration", 2.0),
                                       rclcpp::Parameter("publish_rate", -2.0)})
          .successful);
  EXPECT_DOUBLE_EQ(controller_->get_parameter("scan_duration").as_double(),
                   8.0);
}

INSTANTIATE_TEST_SUITE_P(StopAndCancel, CtRetractionTest, testing::Bool());
}  // namespace hospital_controller
