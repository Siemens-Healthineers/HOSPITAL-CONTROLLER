// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include <QApplication>
#include <QLabel>
#include <QPushButton>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "hospital_rviz_panel/hospital_panel.hpp"
#include "hospital_rviz_panel/lab_control_widget.hpp"

using namespace std::chrono_literals;

namespace hospital_rviz_panel {
class LabPanelResultTest : public testing::Test {
 protected:
  using AnalyzeBlood = hospital_interfaces::action::AnalyzeBlood;
  using LabState = hospital_interfaces::msg::LabState;
  using ServerGoal = rclcpp_action::ServerGoalHandle<AnalyzeBlood>;

  static void SetUpTestSuite() {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    application_ = std::make_unique<QApplication>(argc_, argv_);
    rclcpp::InitOptions options;
    options.set_domain_id(44);
    rclcpp::init(0, nullptr, options);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
    application_.reset();
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("lab_panel_result_test");
    panel_ = std::make_unique<HospitalControlPanel>();
    panel_->node_ = node_;
    panel_->lab_widget_ = new LabControlWidget(panel_.get());
    panel_->status_bar_ = new QLabel(panel_.get());
    panel_->lab_action_ =
        rclcpp_action::create_client<AnalyzeBlood>(node_, "/test/lab/analyze");
    panel_->lab_state_sub_ = node_->create_subscription<LabState>(
        "/test/lab/state", rclcpp::QoS(1).transient_local(),
        [this](LabState::ConstSharedPtr state) { panel_->onLabState(state); });
    state_pub_ = node_->create_publisher<LabState>(
        "/test/lab/state", rclcpp::QoS(1).transient_local());
    server_ = rclcpp_action::create_server<AnalyzeBlood>(
        node_, "/test/lab/analyze",
        [](const rclcpp_action::GoalUUID&,
           std::shared_ptr<const AnalyzeBlood::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<ServerGoal>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<ServerGoal> goal) { goal_ = goal; });
    executor_.add_node(node_);
    ASSERT_TRUE(panel_->lab_action_->wait_for_action_server(10s));
    ASSERT_TRUE(
        wait_for([this] { return state_pub_->get_subscription_count(); }));
  }

  void TearDown() override {
    executor_.remove_node(node_);
    panel_.reset();
  }

  bool wait_for(const std::function<bool()>& predicate,
                std::chrono::milliseconds timeout = 10000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      QApplication::processEvents();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  void publish_state(bool completed, uint8_t state = LabState::READY) {
    LabState snapshot;
    snapshot.state = state;
    snapshot.result_valid = completed;
    snapshot.ctnt = completed ? 42.0f : 0.0f;
    snapshot.ctni = completed ? 55.0f : 0.0f;
    snapshot.ck_mb = completed ? 8.0f : 0.0f;
    state_pub_->publish(snapshot);
    ASSERT_TRUE(wait_for([this, completed, state] {
      return panel_->lab_state_dirty_ &&
             panel_->lab_state_cache_.state == state &&
             panel_->lab_state_cache_.result_valid == completed;
    }));
    panel_->refreshWidgets();
  }

  void start_manual_test() {
    publish_state(false);
    auto* button = panel_->lab_widget_->findChild<QPushButton*>();
    ASSERT_NE(button, nullptr);
    QObject::connect(button, &QPushButton::clicked, panel_.get(),
                     [this] { panel_->sendAnalyzeBloodGoal(); });
    button->click();
    ASSERT_TRUE(wait_for([this] { return goal_ != nullptr; }));
    EXPECT_EQ(goal_->get_goal()->command, "START_BLOODTEST");
    publish_state(false, LabState::ANALYSING);
    EXPECT_FALSE(button->isEnabled());
  }

  void finish_with_delayed_feedback(bool completed) {
    publish_state(completed);
    auto feedback = std::make_shared<AnalyzeBlood::Feedback>();
    feedback->telemetry.state = LabState::ANALYSING;
    feedback->telemetry.progress_percentage = 100.0f;
    goal_->publish_feedback(feedback);
    auto result = std::make_shared<AnalyzeBlood::Result>();
    result->success = completed;
    result->message = "Analysis completed successfully";
    if (completed) {
      goal_->succeed(result);
    } else {
      goal_->abort(result);
    }
    ASSERT_TRUE(wait_for([this, completed] {
      return panel_->status_bar_->text() ==
             (completed ? "Analysis completed successfully"
                        : "Blood analysis did not succeed");
    }));
    panel_->refreshWidgets();
    EXPECT_TRUE(panel_->lab_widget_->findChild<QPushButton*>()->isEnabled());
    EXPECT_EQ(panel_->lab_state_cache_.state, LabState::READY);
    EXPECT_EQ(panel_->lab_state_cache_.result_valid, completed);
  }

  void expect_result(bool completed) {
    bool found = false;
    for (auto* label : panel_->lab_widget_->findChildren<QLabel*>()) {
      if (label->text().startsWith("Result:")) {
        found = true;
        if (completed) {
          EXPECT_TRUE(label->text().contains("42.0"));
          EXPECT_TRUE(label->text().contains("55.0"));
          EXPECT_TRUE(label->text().contains("8.0"));
        } else {
          EXPECT_EQ(label->text(), "Result: --");
        }
      }
    }
    EXPECT_TRUE(found);
  }

  inline static int argc_{1};
  inline static char application_name_[] = "lab_panel_result_test";
  inline static char* argv_[] = {application_name_, nullptr};
  inline static std::unique_ptr<QApplication> application_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<HospitalControlPanel> panel_;
  rclcpp::Publisher<LabState>::SharedPtr state_pub_;
  rclcpp_action::Server<AnalyzeBlood>::SharedPtr server_;
  std::shared_ptr<ServerGoal> goal_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(LabPanelResultTest, ManualCompletionSurvivesDelayedFeedback) {
  start_manual_test();
  finish_with_delayed_feedback(true);
  expect_result(true);
  publish_state(false, LabState::ANALYSING);
  expect_result(false);
}

TEST_F(LabPanelResultTest, InterruptedAnalysisSurvivesDelayedFeedback) {
  start_manual_test();
  finish_with_delayed_feedback(false);
  expect_result(false);
}

TEST_F(LabPanelResultTest, RobotCompletionUsesTheSameTopicState) {
  publish_state(false, LabState::ANALYSING);
  publish_state(true);
  expect_result(true);
  publish_state(false, LabState::ANALYSING);
  expect_result(false);
}
}  // namespace hospital_rviz_panel
