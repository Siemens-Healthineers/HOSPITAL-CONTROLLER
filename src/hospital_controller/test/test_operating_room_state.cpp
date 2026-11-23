// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include <chrono>
#include <functional>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "hospital_controller/operating_room_controller_node.hpp"

using namespace std::chrono_literals;

namespace hospital_controller {
class OperatingRoomStateTest : public testing::Test {
 protected:
  using OperatingRoomState = hospital_interfaces::msg::OperatingRoomState;
  using SetCleanliness = hospital_interfaces::srv::SetCleanliness;
  using GetOperatingRoomState = hospital_interfaces::srv::GetOperatingRoomState;

  static void SetUpTestSuite() {
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(47);
    rclcpp::init(0, nullptr, init_options);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("xray_active", true)});
    controller_ = std::make_shared<OperatingRoomControllerNode>(options);
    client_node_ = std::make_shared<rclcpp::Node>("operating_room_state_test");
    executor_.add_node(controller_);
    executor_.add_node(client_node_);
  }

  void TearDown() override {
    executor_.remove_node(controller_);
    executor_.remove_node(client_node_);
    controller_.reset();
    client_node_.reset();
  }

  // Subscribes like a late-started panel would: only after the controller has
  // already published its initial state.
  void subscribe() {
    subscription_ = client_node_->create_subscription<OperatingRoomState>(
        "/hospital/operating_room/state", rclcpp::QoS(1).transient_local(),
        [this](OperatingRoomState::ConstSharedPtr state) {
          states_.push_back(*state);
        });
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

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<OperatingRoomControllerNode> controller_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Subscription<OperatingRoomState>::SharedPtr subscription_;
  std::vector<OperatingRoomState> states_;
};

TEST_F(OperatingRoomStateTest, LateSubscriberReceivesInitialState) {
  subscribe();
  ASSERT_TRUE(wait_for([this] { return !states_.empty(); }));
  EXPECT_EQ(states_.back().cleanliness, OperatingRoomState::CLEAN);
  EXPECT_TRUE(states_.back().xray_active);
}

TEST_F(OperatingRoomStateTest, XrayParameterChangePublishesState) {
  subscribe();
  ASSERT_TRUE(wait_for([this] { return !states_.empty(); }));
  ASSERT_TRUE(
      controller_->set_parameter(rclcpp::Parameter("xray_active", false))
          .successful);
  ASSERT_TRUE(wait_for([this] { return !states_.back().xray_active; }));
  EXPECT_EQ(states_.back().cleanliness, OperatingRoomState::CLEAN);
}

TEST_F(OperatingRoomStateTest, CleanlinessChangeIsLatchedAndGetterIsPure) {
  auto set_client = client_node_->create_client<SetCleanliness>(
      "/hospital/operating_room/set_cleanliness");
  auto get_client = client_node_->create_client<GetOperatingRoomState>(
      "/hospital/operating_room/get_state");
  ASSERT_TRUE(set_client->wait_for_service(10s));
  ASSERT_TRUE(get_client->wait_for_service(10s));

  auto request = std::make_shared<SetCleanliness::Request>();
  request->target_state = OperatingRoomState::DIRTY;
  auto set_response = set_client->async_send_request(request);
  ASSERT_TRUE(ready(set_response));
  ASSERT_TRUE(set_response.get()->accepted);

  // A subscriber joining after the change still sees the current state.
  subscribe();
  ASSERT_TRUE(wait_for([this] { return !states_.empty(); }));
  EXPECT_EQ(states_.back().cleanliness, OperatingRoomState::DIRTY);

  const auto published = states_.size();
  auto get_response = get_client->async_send_request(
      std::make_shared<GetOperatingRoomState::Request>());
  ASSERT_TRUE(ready(get_response));
  EXPECT_EQ(get_response.get()->state.cleanliness, OperatingRoomState::DIRTY);
  wait_for([] { return false; }, 200ms);
  EXPECT_EQ(states_.size(), published);
}

}  // namespace hospital_controller
