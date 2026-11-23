// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

// Joint-state bridge: drives the real 3-D URDF model from the controllers'
// abstract device telemetry.
//
// The hospital controllers report state as enum + percentage only (no joint
// positions). This node maps that abstract state onto the articulated joints
// defined in hospital_description/urdf/hospital.urdf.xacro and publishes a
// sensor_msgs/JointState on /joint_states. robot_state_publisher then turns
// those positions into TF so an RViz RobotModel shows the live, controllable
// scene. The hospital_rviz_panel commands the controllers, closing the loop.
//
// Joint conventions (must match the xacro):
//   * Swing doors   -> "<id>_hinge" revolute. The meshes are baked OPEN, so the
//                      hinge limit (+/-90 deg) is CLOSED and 0 is fully open.
//   * Sliding doors -> "<id>_slide" prismatic, position 0 = closed,
//                      travel value (metres) = fully open.
//   * CT lift        -> "ct_lift" prismatic, 0 = TOP (extended), -kCtH = BOTTOM
//                      (collapsed). CtTableState.percentage_done
//                      0=BOTTOM..100=TOP.
//   * CT table slide -> "ct_table_slide" prismatic, 0 = parked,
//                      kCtSlide = fully in the bore.
//                      CtScannerState.percentage_done sweeps 0 (out) -> 50 (in)
//                      -> 100 (out).
// The five telescopic shrouds are <mimic> joints in the URDF, so they are NOT
// published here; robot_state_publisher derives them from ct_lift.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hospital_interfaces/msg/ct_scanner_state.hpp"
#include "hospital_interfaces/msg/ct_table_state.hpp"
#include "hospital_interfaces/msg/door_state_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hospital_arena {

using sensor_msgs::msg::JointState;
using DoorStateArray = hospital_interfaces::msg::DoorStateArray;
using CtTableState = hospital_interfaces::msg::CtTableState;
using CtScannerState = hospital_interfaces::msg::CtScannerState;

namespace {
// CT telescopic column travel (metres); matches ct_H in the xacro.
constexpr double kCtH = 0.479147;
// CT table slide travel into the bore (metres); matches ct_table_slide upper.
constexpr double kCtSlide = 1.8;

// One articulated door: the controller device_id, the URDF joint it drives, and
// the joint position (rad for revolute, m for prismatic) at the closed and the
// fully-open pose. The bridge interpolates between them by percentage_open.
//
// The swing-door meshes are baked in the OPEN pose, so for them q=0 is open and
// the hinge limit (+/-pi/2) is closed -> closed_value is the limit and
// open_value is 0. The sliding-door meshes carry a closed-pose offset in the
// xacro, so for them q=0 is closed and the travel value is open.
struct DoorJoint {
  const char* device_id;
  const char* joint_name;
  double closed_value;
  double open_value;
};

// Swing doors (revolute "_hinge", +/-pi/2) and sliding doors (prismatic
// "_slide"). Signs/values copied straight from hospital.urdf.xacro so the model
// opens in the same direction the joint limits allow.
constexpr double kQuarterTurn = M_PI / 2.0;  // 1.5708 rad
constexpr DoorJoint kDoorJoints[] = {
    {"clinical_corridor_3", "clinical_corridor_3_hinge", -kQuarterTurn, 0.0},
    {"clinical_corridor_2", "clinical_corridor_2_hinge", -kQuarterTurn, 0.0},
    {"clinical_corridor_1", "clinical_corridor_1_hinge", kQuarterTurn, 0.0},
    {"phlebotomy_room", "phlebotomy_room_hinge", kQuarterTurn, 0.0},
    {"changing_room", "changing_room_hinge", -kQuarterTurn, 0.0},
    {"intervention_or_2", "intervention_or_2_hinge", kQuarterTurn, 0.0},
    {"scanning_room_2", "scanning_room_2_hinge", -kQuarterTurn, 0.0},
    {"laboratory", "laboratory_hinge", kQuarterTurn, 0.0},
    {"entry_l", "entry_l_hinge", kQuarterTurn, 0.0},
    {"entry_r", "entry_r_hinge", -kQuarterTurn, 0.0},
    {"intervention_or_1", "intervention_or_1_slide", 0.0, 1.2588},
    {"scanning_room_1", "scanning_room_1_slide", 0.0, 1.71951},
};
}  // namespace

class JointBridgeNode : public rclcpp::Node {
 public:
  JointBridgeNode() : rclcpp::Node("joint_bridge_node") {
    const double publish_rate = declare_parameter<double>("publish_rate", 30.0);

    // Seed every joint at rest (closed / BOTTOM / parked) so the model is in a
    // sane pose before any telemetry arrives.
    for (const auto& d : kDoorJoints) {
      door_percent_[d.device_id] = 0.0;
    }

    // The controllers latch (transient_local) these states, so a matching QoS
    // gives a late-started bridge the current pose of every door and the CT
    // instead of the rest pose until something moves. The aggregated door
    // array carries every door, so none is missed.
    const auto latched = rclcpp::QoS(1).transient_local();
    door_sub_ = create_subscription<DoorStateArray>(
        "/hospital/doors/states", latched,
        std::bind(&JointBridgeNode::onDoorStates, this, std::placeholders::_1));
    table_sub_ = create_subscription<CtTableState>(
        "/hospital/ct/table/state", latched,
        std::bind(&JointBridgeNode::onTableState, this, std::placeholders::_1));
    scan_sub_ = create_subscription<CtScannerState>(
        "/hospital/ct/scan/state", latched,
        std::bind(&JointBridgeNode::onScanState, this, std::placeholders::_1));

    joint_pub_ = create_publisher<JointState>("/joint_states", 10);

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&JointBridgeNode::publishJointStates, this));

    RCLCPP_INFO(get_logger(),
                "joint_bridge_node up: publishing /joint_states at %.1f Hz",
                publish_rate);
  }

 private:
  // -- State callbacks --------------------------------------------------------

  void onDoorStates(const DoorStateArray::ConstSharedPtr msg) {
    for (const auto& door : msg->doors) {
      door_percent_[door.device_id] = door.percentage_open;
    }
  }

  void onTableState(const CtTableState::ConstSharedPtr msg) {
    table_percent_ = msg->percentage_done;
  }

  void onScanState(const CtScannerState::ConstSharedPtr msg) {
    scan_percent_ = msg->percentage_done;
  }

  // -- Publish ----------------------------------------------------------------

  void publishJointStates() {
    JointState js;
    js.header.stamp = now();

    for (const auto& d : kDoorJoints) {
      const auto it = door_percent_.find(d.device_id);
      const double percent = (it != door_percent_.end()) ? it->second : 0.0;
      const double fraction = std::clamp(percent / 100.0, 0.0, 1.0);
      js.name.emplace_back(d.joint_name);
      js.position.push_back(d.closed_value +
                            fraction * (d.open_value - d.closed_value));
    }

    // CT lift: 0 % (BOTTOM) -> collapsed (-kCtH), 100 % (TOP) -> extended (0).
    const double lift_fraction = std::clamp(table_percent_ / 100.0, 0.0, 1.0);
    js.name.emplace_back("ct_lift");
    js.position.push_back(-kCtH * (1.0 - lift_fraction));

    // CT table slide: scan sweep 0 (out) -> 50 (fully in) -> 100 (out).
    const double pct = std::clamp(scan_percent_, 0.0, 100.0);
    const double insertion =
        (pct <= 50.0) ? (pct / 50.0) : ((100.0 - pct) / 50.0);
    js.name.emplace_back("ct_table_slide");
    js.position.push_back(insertion * kCtSlide);

    joint_pub_->publish(js);
  }

  std::map<std::string, double> door_percent_;
  double table_percent_{0.0};
  double scan_percent_{0.0};

  rclcpp::Subscription<DoorStateArray>::SharedPtr door_sub_;
  rclcpp::Subscription<CtTableState>::SharedPtr table_sub_;
  rclcpp::Subscription<CtScannerState>::SharedPtr scan_sub_;
  rclcpp::Publisher<JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hospital_arena

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hospital_arena::JointBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
