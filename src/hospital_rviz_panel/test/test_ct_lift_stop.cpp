// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include <QApplication>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "hospital_rviz_panel/hospital_panel.hpp"

using namespace std::chrono_literals;

namespace hospital_rviz_panel {
class CtLiftStopTest : public testing::Test {
 protected:
  using MoveTable = hospital_interfaces::action::MoveTable;
  using RunScan = hospital_interfaces::action::RunScan;
  using Procedure = hospital_interfaces::msg::CtScanProcedureState;
  using Table = hospital_interfaces::msg::CtTableState;

  static void SetUpTestSuite() {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    application_ = std::make_unique<QApplication>(argc_, argv_);
    rclcpp::InitOptions options;
    options.set_domain_id(42);
    rclcpp::init(0, nullptr, options);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
    application_.reset();
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("ct_lift_stop_test");
    panel_ = std::make_unique<HospitalControlPanel>();
    panel_->node_ = node_;
    panel_->move_action_ =
        rclcpp_action::create_client<MoveTable>(node_, "/test/ct/table/move");
    panel_->scan_action_ =
        rclcpp_action::create_client<RunScan>(node_, "/test/ct/scan/run");
    move_server_ = rclcpp_action::create_server<MoveTable>(
        node_, "/test/ct/table/move",
        [](const rclcpp_action::GoalUUID&,
           std::shared_ptr<const MoveTable::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTable>>) {
          ++move_cancels_;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](
            std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTable>> goal) {
          move_goals_.push_back(goal);
        });
    scan_server_ = rclcpp_action::create_server<RunScan>(
        node_, "/test/ct/scan/run",
        [this](const rclcpp_action::GoalUUID&,
               std::shared_ptr<const RunScan::Goal> goal) {
          if (goal->command == RunScan::Goal::COMMAND_STOP) {
            ++scan_stops_;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<rclcpp_action::ServerGoalHandle<RunScan>>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [](std::shared_ptr<rclcpp_action::ServerGoalHandle<RunScan>> goal) {
          auto result = std::make_shared<RunScan::Result>();
          result->success = true;
          goal->succeed(result);
        });
    executor_.add_node(node_);
    ASSERT_TRUE(panel_->move_action_->wait_for_action_server(10s));
    ASSERT_TRUE(panel_->scan_action_->wait_for_action_server(10s));
  }

  void TearDown() override {
    panel_.reset();
    executor_.remove_node(node_);
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

  void set_state(uint8_t phase, uint8_t table_state) {
    panel_->ct_procedure_phase_ = phase;
    panel_->ct_table_cache_.state = table_state;
  }

  void stop_lift() { panel_->cancelMoveGoal(); }

  void start_move(uint8_t command, bool owned) {
    MoveTable::Goal goal;
    goal.command = command;
    auto response = panel_->move_action_->async_send_goal(goal);
    ASSERT_TRUE(wait_for([&response] {
      return response.wait_for(0s) == std::future_status::ready;
    }));
    ASSERT_NE(response.get(), nullptr);
    if (owned) {
      panel_->move_goal_handle_ = response.get();
    }
  }

  void expect_pending_cancel() {
    panel_->move_in_flight_ = true;
    stop_lift();
    EXPECT_TRUE(panel_->move_cancel_pending_);
  }

  template <typename Callback>
  auto make_guarded_callback(Callback callback) {
    return panel_->guardCallback(std::move(callback));
  }

  inline static int argc_{1};
  inline static char application_name_[] = "ct_lift_stop_test";
  inline static char* argv_[] = {application_name_, nullptr};
  inline static std::unique_ptr<QApplication> application_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<HospitalControlPanel> panel_;
  rclcpp_action::Server<MoveTable>::SharedPtr move_server_;
  rclcpp_action::Server<RunScan>::SharedPtr scan_server_;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTable>>>
      move_goals_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  int move_cancels_{0};
  int scan_stops_{0};
};

TEST_F(CtLiftStopTest, IgnoresCallbackAfterPanelDestruction) {
  int callback_count = 0;
  auto callback =
      make_guarded_callback([&callback_count] { ++callback_count; });

  panel_.reset();
  callback();

  EXPECT_EQ(callback_count, 0);
}

TEST_F(CtLiftStopTest, StopsScanPreparationFromBottomOrPartialHeight) {
  for (const auto state : {Table::BOTTOM, Table::PAUSED, Table::MOVING_UP}) {
    set_state(Procedure::PHASE_PREPARING_LIFT, state);
    const int previous = scan_stops_;
    stop_lift();
    ASSERT_TRUE(wait_for([this, previous] { return scan_stops_ > previous; }));
  }
  EXPECT_EQ(move_cancels_, 0);
}

TEST_F(CtLiftStopTest, CancelsPanelOwnedUpAndDown) {
  for (const auto command :
       {MoveTable::Goal::COMMAND_UP, MoveTable::Goal::COMMAND_DOWN}) {
    start_move(command, true);
    const int previous = move_cancels_;
    stop_lift();
    ASSERT_TRUE(
        wait_for([this, previous] { return move_cancels_ > previous; }));
  }
  EXPECT_EQ(scan_stops_, 0);
}

TEST_F(CtLiftStopTest, CancelsUntrackedStandaloneUpAndDown) {
  for (const auto command :
       {MoveTable::Goal::COMMAND_UP, MoveTable::Goal::COMMAND_DOWN}) {
    set_state(Procedure::PHASE_COMPLETED, command == MoveTable::Goal::COMMAND_UP
                                              ? Table::MOVING_UP
                                              : Table::MOVING_DOWN);
    start_move(command, false);
    const int previous = move_cancels_;
    stop_lift();
    ASSERT_TRUE(
        wait_for([this, previous] { return move_cancels_ > previous; }));
  }
  EXPECT_EQ(scan_stops_, 0);
}

TEST_F(CtLiftStopTest, DoesNotStopGantrySweepOrRetraction) {
  for (const auto phase :
       {Procedure::PHASE_SWEEP_IN, Procedure::PHASE_SWEEP_OUT,
        Procedure::PHASE_RETRACTING}) {
    set_state(phase, Table::TOP);
    stop_lift();
  }
  EXPECT_FALSE(
      wait_for([this] { return scan_stops_ || move_cancels_; }, 100ms));
}

TEST_F(CtLiftStopTest, PreservesPendingStandaloneCancellation) {
  expect_pending_cancel();
  EXPECT_EQ(scan_stops_, 0);
}
}  // namespace hospital_rviz_panel
