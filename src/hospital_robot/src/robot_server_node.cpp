// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

// Protocol-driven robot simulation.
//
// The node hosts the ExecuteTask ward action server and keeps a discrete pose
// (one waypoint per room) published as geometry_msgs/PoseStamped plus a TF
// frame. On each goal it runs a fixed step protocol built from the primitive
// helpers (navigateToRoom / openDoor / interact / getOperatingRoomClean /
// setOperatingRoomClean): the robot never moves freely, it only teleports
// between named room waypoints, drives the door/operating-room controllers,
// and simulates patient/room interaction with timers. Tasks are serialised so
// the robot runs one at a time: accepted tasks wait in a FIFO queue and a
// queued task can be cancelled without waiting for the running one.
//
// Implemented protocols: CLEAN_OR (CleanPrep_OR), COLLECT_LAB, GUIDE_CT,
// HEART_ATTACK and SURGICAL_ASSIST.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>  // NOLINT(build/include_order): C++20 header unknown to cpplint
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hospital_interfaces/action/analyze_blood.hpp"
#include "hospital_interfaces/action/execute_task.hpp"
#include "hospital_interfaces/action/move_table.hpp"
#include "hospital_interfaces/action/operate_door.hpp"
#include "hospital_interfaces/action/run_scan.hpp"
#include "hospital_interfaces/msg/operating_room_state.hpp"
#include "hospital_interfaces/srv/get_operating_room_state.hpp"
#include "hospital_interfaces/srv/set_cleanliness.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace hospital_robot {

namespace {
// Distinct display color per robot, indexed by roster position. Shared *by
// convention* with the RViz panel's colorForIndex (ward_management_widget.cpp):
// a robot's 3D marker (arrow + floating id label) and its Mission-dispatch
// table row use the same color so the operator can tell which arrow is which
// robot. Keep the two palettes in the same order as the rviz2 robot_ids list.
struct Rgb {
  float r;
  float g;
  float b;
};
Rgb colorForIndex(int index) {
  static const std::array<Rgb, 8> kPalette = {{
      {0.0F, 0.78F, 0.0F},   // green   -> robot_1
      {0.10F, 0.49F, 1.0F},  // blue    -> robot_2
      {1.0F, 0.55F, 0.0F},   // orange  -> robot_3
      {0.78F, 0.0F, 0.78F},  // magenta
      {0.0F, 0.78F, 0.78F},  // cyan
      {1.0F, 0.0F, 0.0F},    // red
      {0.59F, 0.39F, 0.0F},  // brown
      {0.47F, 0.0F, 1.0F},   // purple
  }};
  if (index < 0) {
    index = 0;
  }
  return kPalette[static_cast<std::size_t>(index) % kPalette.size()];
}
}  // namespace

using namespace std::chrono_literals;

class RobotServerNode : public rclcpp::Node {
 public:
  using ExecuteTask = hospital_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using OperateDoor = hospital_interfaces::action::OperateDoor;
  using AnalyzeBlood = hospital_interfaces::action::AnalyzeBlood;
  using RunScan = hospital_interfaces::action::RunScan;
  using MoveTable = hospital_interfaces::action::MoveTable;
  using SetCleanliness = hospital_interfaces::srv::SetCleanliness;
  using GetOperatingRoomState = hospital_interfaces::srv::GetOperatingRoomState;
  using OperatingRoomState = hospital_interfaces::msg::OperatingRoomState;

  explicit RobotServerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("robot_server_node", options) {
    declareParameters();
    loadWaypoints();

    // Discrete pose stream. Latched so a late subscriber (RViz) immediately
    // sees where the robot currently is.
    rclcpp::QoS pose_qos(1);
    pose_qos.transient_local();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        pose_topic_, pose_qos);
    // Colored arrow + floating id label so the robot is identifiable directly
    // in the 3D scene. Latched like the pose so RViz shows it immediately.
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, pose_qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Start parked.
    current_pose_.header.frame_id = pose_frame_id_;
    setPoseFromWaypoint(parking_room_);

    // Clients used to drive the rest of the simulation. Created up front so the
    // task protocols can use them without re-discovery cost.
    door_client_ =
        rclcpp_action::create_client<OperateDoor>(this, door_action_name_);
    lab_client_ =
        rclcpp_action::create_client<AnalyzeBlood>(this, lab_action_name_);
    scan_client_ =
        rclcpp_action::create_client<RunScan>(this, scan_action_name_);
    move_table_client_ =
        rclcpp_action::create_client<MoveTable>(this, move_table_action_name_);
    cleanliness_client_ =
        this->create_client<SetCleanliness>(cleanliness_service_name_);
    operating_room_state_client_ = this->create_client<GetOperatingRoomState>(
        operating_room_state_service_name_);
    // Parameter clients for the controllers whose durations bound how long the
    // robot waits for their action results.
    door_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, door_controller_node_);
    lab_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, lab_controller_node_);
    ct_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, ct_scanner_controller_node_);

    action_server_ = rclcpp_action::create_server<ExecuteTask>(
        this, execute_task_action_name_,
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const ExecuteTask::Goal> goal) {
          return handleGoal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
          return handleCancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
          handleAccepted(goal_handle);
        });

    // Re-publish the pose (and TF) at a fixed low rate so TF stays fresh for
    // RViz even while the robot sits still between tasks.
    const auto period =
        std::chrono::duration<double>(1.0 / std::max(0.1, pose_publish_rate_));
    pose_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { publishPose(); });

    // rclcpp::shutdown() (e.g. Ctrl-C) must interrupt running and queued tasks
    // right away instead of letting them sleep out their current step.
    shutdown_callback_handle_ =
        this->get_node_base_interface()
            ->get_context()
            ->add_on_shutdown_callback([this]() { stopAllWorkers(); });

    RCLCPP_INFO(this->get_logger(),
                "Robot server ready on %s (parked at '%s').",
                execute_task_action_name_.c_str(), parking_room_.c_str());
  }

  // Stop and join every task worker before any member it uses is destroyed.
  ~RobotServerNode() override {
    this->get_node_base_interface()->get_context()->remove_on_shutdown_callback(
        shutdown_callback_handle_);
    std::vector<Worker> workers;
    {
      std::scoped_lock lock(workers_mutex_);
      workers = std::move(workers_);
    }
    for (auto& worker : workers) {
      worker.thread.request_stop();
    }
    // Joined here; a worker interrupted this way aborts its goal.
  }

 private:
  // Per-task flags shared between the action callbacks and the task's worker.
  struct TaskState {
    std::atomic<bool> done{false};
    std::atomic<bool> cancel_requested{false};
  };

  struct Worker {
    std::shared_ptr<GoalHandleExecuteTask> goal;
    std::shared_ptr<TaskState> state;
    std::jthread thread;
  };

  struct Waypoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  // Runs an arbitrary action when it goes out of scope. Guarantees the worker
  // bookkeeping is cleaned up even if the protocol returns early or throws.
  struct ScopeGuard {
    std::function<void()> on_exit;
    ~ScopeGuard() {
      if (on_exit) {
        on_exit();
      }
    }
  };

  // ---- Parameters -----------------------------------------------------------

  // Descriptor restricting a double parameter to [from, to]. rclcpp enforces
  // it on the startup value too, so a bad YAML value fails construction.
  static rcl_interfaces::msg::ParameterDescriptor rangedDescriptor(
      const std::string& description, double from, double to) {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = from;
    range.to_value = to;
    range.step = 0.0;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.floating_point_range.push_back(range);
    return descriptor;
  }

  void declareParameters() {
    this->declare_parameter<std::vector<std::string>>(
        "room_names", std::vector<std::string>{});
    this->declare_parameter<std::vector<double>>("room_x",
                                                 std::vector<double>{});
    this->declare_parameter<std::vector<double>>("room_y",
                                                 std::vector<double>{});
    this->declare_parameter<std::vector<double>>("room_yaw",
                                                 std::vector<double>{});
    this->declare_parameter<std::string>("parking_room", "parking");

    this->declare_parameter<double>(
        "travel_duration", 3.0,
        rangedDescriptor("Simulated travel time between rooms [s]", 0.0,
                         3600.0));
    this->declare_parameter<double>(
        "interaction_duration", 4.0,
        rangedDescriptor("Simulated patient/room interaction time [s]", 0.0,
                         3600.0));
    this->declare_parameter<double>(
        "pose_publish_rate", 5.0,
        rangedDescriptor("Pose/TF re-publish rate [Hz]", 0.1, 1000.0));

    // Controllers whose duration parameters bound how long the robot waits for
    // their action results: timeout = factor * duration + margin. If a
    // controller's parameters cannot be read, the fixed *_fallback_timeout
    // values below are used instead.
    this->declare_parameter<std::string>("door_controller_node",
                                         "/door_controller_node");
    this->declare_parameter<std::string>("lab_controller_node",
                                         "/lab_controller_node");
    this->declare_parameter<std::string>("ct_scanner_controller_node",
                                         "/ct_scanner_controller_node");
    this->declare_parameter<double>(
        "controller_timeout_factor", 1.5,
        rangedDescriptor("Result timeout = factor * controller duration + "
                         "margin",
                         1.0, 100.0));
    this->declare_parameter<double>(
        "controller_timeout_margin", 10.0,
        rangedDescriptor("Constant grace added to every result timeout [s]",
                         0.0, 3600.0));
    this->declare_parameter<double>(
        "door_fallback_timeout", 30.0,
        rangedDescriptor("Door result timeout if its duration is unknown [s]",
                         1.0, 36000.0));
    this->declare_parameter<double>(
        "lab_fallback_timeout", 60.0,
        rangedDescriptor("Lab result timeout if its duration is unknown [s]",
                         1.0, 36000.0));
    this->declare_parameter<double>(
        "scan_fallback_timeout", 60.0,
        rangedDescriptor("CT scan result timeout if its duration is unknown "
                         "[s]",
                         1.0, 36000.0));
    this->declare_parameter<double>(
        "table_fallback_timeout", 30.0,
        rangedDescriptor("CT table result timeout if its duration is unknown "
                         "[s]",
                         1.0, 36000.0));

    this->declare_parameter<std::string>("pose_topic", "/hospital/robot/pose");
    this->declare_parameter<std::string>("pose_frame_id", "world");
    this->declare_parameter<std::string>("base_frame_id", "robot_base_link");
    // Shared latched topic for the per-robot arrow + id label markers.
    this->declare_parameter<std::string>("marker_topic",
                                         "/hospital/robot_markers");

    this->declare_parameter<std::string>("execute_task_action",
                                         "/hospital/ward/execute_task");
    this->declare_parameter<std::string>("door_action",
                                         "/hospital/doors/operate");
    this->declare_parameter<std::string>("lab_action", "/hospital/lab/analyze");
    this->declare_parameter<std::string>("scan_action",
                                         "/hospital/ct/scan/run");
    this->declare_parameter<std::string>("move_table_action",
                                         "/hospital/ct/table/move");
    this->declare_parameter<std::string>(
        "cleanliness_service", "/hospital/operating_room/set_cleanliness");
    this->declare_parameter<std::string>("operating_room_state_service",
                                         "/hospital/operating_room/get_state");

    // Multi-robot identity. An empty robot_id uses the configured single-robot
    // names; a non-empty id derives the robot's own action/pose/frame names via
    // the convention shared with the RViz panel.
    this->declare_parameter<std::string>("robot_id", "");
    this->declare_parameter<double>("pose_offset_x", 0.0);
    this->declare_parameter<double>("pose_offset_y", 0.0);
    // Roster position; selects this robot's color from the shared palette so it
    // matches the RViz panel's table row (see colorForIndex above).
    this->declare_parameter<int>("robot_index", 0);

    parking_room_ = this->get_parameter("parking_room").as_string();
    travel_duration_ = this->get_parameter("travel_duration").as_double();
    interaction_duration_ =
        this->get_parameter("interaction_duration").as_double();
    pose_publish_rate_ = this->get_parameter("pose_publish_rate").as_double();
    pose_topic_ = this->get_parameter("pose_topic").as_string();
    pose_frame_id_ = this->get_parameter("pose_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    execute_task_action_name_ =
        this->get_parameter("execute_task_action").as_string();
    door_action_name_ = this->get_parameter("door_action").as_string();
    lab_action_name_ = this->get_parameter("lab_action").as_string();
    scan_action_name_ = this->get_parameter("scan_action").as_string();
    move_table_action_name_ =
        this->get_parameter("move_table_action").as_string();
    cleanliness_service_name_ =
        this->get_parameter("cleanliness_service").as_string();
    operating_room_state_service_name_ =
        this->get_parameter("operating_room_state_service").as_string();
    door_controller_node_ =
        this->get_parameter("door_controller_node").as_string();
    lab_controller_node_ =
        this->get_parameter("lab_controller_node").as_string();
    ct_scanner_controller_node_ =
        this->get_parameter("ct_scanner_controller_node").as_string();
    controller_timeout_factor_ =
        this->get_parameter("controller_timeout_factor").as_double();
    controller_timeout_margin_ =
        this->get_parameter("controller_timeout_margin").as_double();
    door_fallback_timeout_ =
        this->get_parameter("door_fallback_timeout").as_double();
    lab_fallback_timeout_ =
        this->get_parameter("lab_fallback_timeout").as_double();
    scan_fallback_timeout_ =
        this->get_parameter("scan_fallback_timeout").as_double();
    table_fallback_timeout_ =
        this->get_parameter("table_fallback_timeout").as_double();

    robot_id_ = this->get_parameter("robot_id").as_string();
    pose_offset_x_ = this->get_parameter("pose_offset_x").as_double();
    pose_offset_y_ = this->get_parameter("pose_offset_y").as_double();
    robot_index_ =
        static_cast<int>(this->get_parameter("robot_index").as_int());

    // Derive the robot's own interface names from its id. The shared
    // controllers the robot drives (doors/lab/CT/operating room) keep their
    // absolute names so every robot still talks to the same simulation. This
    // convention MUST match the one the RViz panel uses to build its per-robot
    // ExecuteTask client (see hospital_panel.cpp).
    if (!robot_id_.empty()) {
      execute_task_action_name_ =
          "/hospital/" + robot_id_ + "/ward/execute_task";
      pose_topic_ = "/hospital/" + robot_id_ + "/pose";
      base_frame_id_ = robot_id_ + "_base_link";
    }
  }

  // Build the room -> waypoint table from the parallel coordinate arrays. A
  // mismatch in lengths is tolerated (the shortest array wins) so a partially
  // configured params file still brings the node up.
  void loadWaypoints() {
    const auto names = this->get_parameter("room_names").as_string_array();
    const auto xs = this->get_parameter("room_x").as_double_array();
    const auto ys = this->get_parameter("room_y").as_double_array();
    const auto yaws = this->get_parameter("room_yaw").as_double_array();

    const std::size_t count =
        std::min({names.size(), xs.size(), ys.size(), yaws.size()});
    if (names.size() != xs.size() || xs.size() != ys.size() ||
        ys.size() != yaws.size()) {
      RCLCPP_WARN(this->get_logger(),
                  "room_names/x/y/yaw lengths differ (%zu/%zu/%zu/%zu); using "
                  "the first %zu entries.",
                  names.size(), xs.size(), ys.size(), yaws.size(), count);
    }
    for (std::size_t i = 0; i < count; ++i) {
      waypoints_[names[i]] = Waypoint{xs[i], ys[i], yaws[i]};
    }

    // Guarantee a parking waypoint exists so the robot always has a home.
    if (waypoints_.find(parking_room_) == waypoints_.end()) {
      RCLCPP_WARN(this->get_logger(),
                  "No waypoint for parking_room '%s'; defaulting to origin.",
                  parking_room_.c_str());
      waypoints_[parking_room_] = Waypoint{};
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu room waypoint(s).",
                waypoints_.size());
  }

  // ---- Pose -----------------------------------------------------------------

  // Snap the robot's pose to a named room waypoint. No-op (with a warning) for
  // an unknown room so a mis-typed target never crashes a task.
  bool setPoseFromWaypoint(const std::string& room) {
    const auto it = waypoints_.find(room);
    if (it == waypoints_.end()) {
      RCLCPP_WARN(this->get_logger(), "Unknown room waypoint '%s'.",
                  room.c_str());
      return false;
    }
    const Waypoint& wp = it->second;
    std::scoped_lock lock(pose_mutex_);
    // Shift by the per-robot offset so several robots sharing a room waypoint
    // stand side by side instead of overlapping.
    current_pose_.pose.position.x = wp.x + pose_offset_x_;
    current_pose_.pose.position.y = wp.y + pose_offset_y_;
    current_pose_.pose.position.z = 0.0;
    // Yaw-only orientation (planar robot); build the quaternion directly to
    // avoid pulling in a tf2 math dependency.
    current_pose_.pose.orientation.x = 0.0;
    current_pose_.pose.orientation.y = 0.0;
    current_pose_.pose.orientation.z = std::sin(wp.yaw * 0.5);
    current_pose_.pose.orientation.w = std::cos(wp.yaw * 0.5);
    current_room_ = room;
    return true;
  }

  void publishPose() {
    geometry_msgs::msg::PoseStamped pose;
    {
      std::scoped_lock lock(pose_mutex_);
      pose = current_pose_;
    }
    pose.header.stamp = this->now();
    pose.header.frame_id = pose_frame_id_;

    // The published pose and the TF share the robot's true heading (yaw about
    // Z), so consumers such as the activity logger get a correct pose. The
    // vertical arrow seen in RViz comes from publishMarkers(), whose arrow is
    // built from explicit base/tip points and ignores this orientation.
    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose.header;
    tf.child_frame_id = base_frame_id_;
    tf.transform.translation.x = pose.pose.position.x;
    tf.transform.translation.y = pose.pose.position.y;
    tf.transform.translation.z = pose.pose.position.z;
    tf.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    pose_pub_->publish(pose);

    publishMarkers(pose.pose.position.x, pose.pose.position.y,
                   pose.header.stamp);
  }

  // Publish this robot's colored vertical arrow and a floating id label at its
  // current planar position so it is identifiable directly in the 3D view.
  void publishMarkers(double x, double y,
                      const builtin_interfaces::msg::Time& stamp) {
    const Rgb c = colorForIndex(robot_index_);
    const std::string ns = robot_id_.empty() ? std::string("robot") : robot_id_;

    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = pose_frame_id_;
    arrow.header.stamp = stamp;
    arrow.ns = ns;
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point base;
    base.x = x;
    base.y = y;
    base.z = 0.0;
    geometry_msgs::msg::Point tip = base;
    tip.z = 1.0;
    arrow.points = {base, tip};
    arrow.scale.x = 0.12;  // shaft diameter
    arrow.scale.y = 0.28;  // head diameter
    arrow.scale.z = 0.30;  // head length
    arrow.color.r = c.r;
    arrow.color.g = c.g;
    arrow.color.b = c.b;
    arrow.color.a = 1.0F;
    markers.markers.push_back(arrow);

    visualization_msgs::msg::Marker label;
    label.header = arrow.header;
    label.ns = ns;
    label.id = 1;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = x;
    label.pose.position.y = y;
    label.pose.position.z = 1.35;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.35;  // text height
    label.color.r = c.r;
    label.color.g = c.g;
    label.color.b = c.b;
    label.color.a = 1.0F;
    label.text = ns;
    markers.markers.push_back(label);

    marker_pub_->publish(markers);
  }

  // ---- Feedback / timing helpers -------------------------------------------

  void publishFeedback(
      const std::shared_ptr<GoalHandleExecuteTask>& goal_handle, uint8_t phase,
      float progress) {
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->current_phase = phase;
    feedback->progress = progress;
    feedback->task_id = goal_handle->get_goal()->task_id;
    goal_handle->publish_feedback(feedback);
  }

  // Sleep for `seconds`, waking early (returning false) if the goal is being
  // cancelled or the node is stopping. Returns true if the full time elapsed.
  // Cancellation, node destruction and rclcpp shutdown all request a stop on
  // the worker's token, which wakes the wait immediately; the deadline is
  // absolute so the wait does not drift.
  bool waitCancellable(
      const std::stop_token& stop_token,
      const std::shared_ptr<GoalHandleExecuteTask>& goal_handle,
      double seconds) {
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(seconds));
    std::mutex mutex;
    std::condition_variable_any wake;
    std::unique_lock lock(mutex);
    wake.wait_until(lock, stop_token, deadline, [] { return false; });
    return !stop_token.stop_requested() && !goal_handle->is_canceling() &&
           rclcpp::ok();
  }

  // ---- Action server callbacks ---------------------------------------------

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& /*uuid*/,
      std::shared_ptr<const ExecuteTask::Goal> goal) {
    switch (goal->value) {
      case ExecuteTask::Goal::CLEAN_OR:
      case ExecuteTask::Goal::COLLECT_LAB:
      case ExecuteTask::Goal::GUIDE_CT:
      case ExecuteTask::Goal::HEART_ATTACK:
      case ExecuteTask::Goal::SURGICAL_ASSIST:
        RCLCPP_INFO(this->get_logger(), "Accepted task %s (type %u).",
                    goal->task_id.c_str(), goal->value);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      default:
        RCLCPP_WARN(this->get_logger(),
                    "Rejecting task %s: unknown task type %u.",
                    goal->task_id.c_str(), goal->value);
        return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleExecuteTask>& goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel requested for task %s.",
                goal_handle->get_goal()->task_id.c_str());
    // Wake the task's worker right away, whether it is running a step or still
    // waiting in the queue.
    std::scoped_lock lock(workers_mutex_);
    for (auto& worker : workers_) {
      if (worker.goal == goal_handle) {
        worker.state->cancel_requested.store(true);
        worker.thread.request_stop();
      }
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
      const std::shared_ptr<GoalHandleExecuteTask>& goal_handle) {
    std::scoped_lock lock(workers_mutex_);
    // Reap workers that have already finished (their jthread destructor joins).
    std::erase_if(workers_,
                  [](const Worker& w) { return w.state->done.load(); });

    // Take the queue position here, on the executor thread, in acceptance
    // order: the worker threads start in no particular order.
    std::uint64_t ticket = 0;
    {
      std::scoped_lock queue_lock(queue_mutex_);
      ticket = next_ticket_++;
      queue_.push_back(ticket);
    }

    auto state = std::make_shared<TaskState>();
    workers_.push_back(Worker{goal_handle, state,
                              std::jthread([this, goal_handle, state, ticket](
                                               std::stop_token stop_token) {
                                execute(stop_token, goal_handle, state, ticket);
                              })});
  }

  // Request a stop on every worker (rclcpp shutdown).
  void stopAllWorkers() {
    std::scoped_lock lock(workers_mutex_);
    for (auto& worker : workers_) {
      worker.thread.request_stop();
    }
  }

  // Worker entry point. Any exception is caught here: escaping a std::jthread
  // would call std::terminate and take down every node in the container.
  void execute(std::stop_token stop_token,
               const std::shared_ptr<GoalHandleExecuteTask> goal_handle,
               std::shared_ptr<TaskState> state, std::uint64_t ticket) {
    const ScopeGuard mark_done{[state]() { state->done.store(true); }};
    try {
      executeQueued(stop_token, goal_handle, ticket);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Task %s failed with exception: %s",
                   goal_handle->get_goal()->task_id.c_str(), e.what());
      abortIfActive(goal_handle, std::string("Internal error: ") + e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Task %s failed with unknown exception.",
                   goal_handle->get_goal()->task_id.c_str());
      abortIfActive(goal_handle, "Internal error");
    }
  }

  // Best-effort abort used on the exception path, where the goal may already
  // be terminal (a second transition would throw again).
  void abortIfActive(const std::shared_ptr<GoalHandleExecuteTask>& gh,
                     const std::string& message) noexcept {
    try {
      if (gh->is_active()) {
        auto result = std::make_shared<ExecuteTask::Result>();
        result->success = false;
        result->message = message;
        gh->abort(result);
      }
    } catch (...) {
      // Nothing left to report the failure to.
    }
  }

  void executeQueued(const std::stop_token& stop_token,
                     const std::shared_ptr<GoalHandleExecuteTask>& goal_handle,
                     std::uint64_t ticket) {
    // Until this worker owns the robot the task stays QUEUED in the panel.
    publishFeedback(goal_handle, ExecuteTask::Goal::PHASE_QUEUED, 0.0f);

    // The robot is a single physical entity: only one task runs at a time.
    // Queued workers wait here in FIFO order; a cancel / stop wakes a queued
    // worker immediately and removes it from the queue.
    if (!acquireRobot(stop_token, ticket)) {
      finishInterrupted(goal_handle, /*return_home=*/false);
      return;
    }
    const ScopeGuard release{[this]() { releaseRobot(); }};
    if (stop_token.stop_requested() || goal_handle->is_canceling() ||
        !rclcpp::ok()) {
      finishInterrupted(goal_handle, /*return_home=*/false);
      return;
    }

    executeProtocol(stop_token, goal_handle);
  }

  // Block until `ticket` is at the head of the queue and the robot is free.
  // Returns false (after leaving the queue) if a stop is requested first.
  bool acquireRobot(const std::stop_token& stop_token, std::uint64_t ticket) {
    std::unique_lock lock(queue_mutex_);
    const bool acquired = queue_cv_.wait(lock, stop_token, [this, ticket] {
      return !robot_busy_ && !queue_.empty() && queue_.front() == ticket;
    });
    if (!acquired) {
      std::erase(queue_, ticket);
      // The head of the queue may have changed.
      queue_cv_.notify_all();
      return false;
    }
    queue_.pop_front();
    robot_busy_ = true;
    return true;
  }

  void releaseRobot() {
    {
      std::scoped_lock lock(queue_mutex_);
      robot_busy_ = false;
    }
    queue_cv_.notify_all();
  }

  // ---- Task protocol --------------------------------------------------------
  //
  // Each supported ExecuteTask type maps to a fixed, ordered sequence of the
  // primitive steps below. There is no free movement: the robot only ever
  // teleports between the named room waypoints via setPoseFromWaypoint().
  // Patient/room interactions are simulated with timers (interact()).
  void executeProtocol(
      const std::stop_token& stop_token,
      const std::shared_ptr<GoalHandleExecuteTask>& goal_handle) {
    switch (goal_handle->get_goal()->value) {
      case ExecuteTask::Goal::CLEAN_OR:
        runCleanPrepOr(stop_token, goal_handle);
        return;
      case ExecuteTask::Goal::COLLECT_LAB:
        runCollectLab(stop_token, goal_handle);
        return;
      case ExecuteTask::Goal::GUIDE_CT:
        runGuideCt(stop_token, goal_handle);
        return;
      case ExecuteTask::Goal::HEART_ATTACK:
        runHeartAttack(stop_token, goal_handle);
        return;
      case ExecuteTask::Goal::SURGICAL_ASSIST:
        runSurgicalAssist(stop_token, goal_handle);
        return;
      default: {
        // No protocol for this task type.
        auto result = std::make_shared<ExecuteTask::Result>();
        RCLCPP_WARN(this->get_logger(), "Task %s: type %u not implemented yet.",
                    goal_handle->get_goal()->task_id.c_str(),
                    goal_handle->get_goal()->value);
        result->success = false;
        result->message = "Task type not implemented yet";
        goal_handle->abort(result);
        return;
      }
    }
  }

  // CleanPrep_OR: drive from the parking spot to the intervention OR, opening
  // and closing every door behind the robot; if the room is not clean, fetch
  // supplies from the storage and mark it clean, then retrace the complete
  // pathway back to the parking spot (again opening and closing each door).
  // Any primitive returning false means the goal is being cancelled -> the task
  // is finalised as canceled.
  void runCleanPrepOr(const std::stop_token& st,
                      const std::shared_ptr<GoalHandleExecuteTask>& gh) {
    RCLCPP_INFO(this->get_logger(), "CleanPrep_OR: starting.");

    // 1. Parking -> Entrance and waiting area (no door on this leg).
    if (!navigateToRoom(st, gh, "entrance_and_waiting_area", 0.05f))
      return finishInterrupted(gh);
    // 2. Through clinical_corridor_1 into the clinical corridor (open, pass,
    // close behind).
    if (!passThroughDoor(st, gh, "clinical_corridor_1", "clinical_corridor",
                         0.15f)) {
      return finishInterrupted(gh);
    }
    // 3. Through intervention_or_1 into the intervention OR.
    if (!passThroughDoor(st, gh, "intervention_or_1", "intervention_or",
                         0.30f)) {
      return finishInterrupted(gh);
    }

    // 4. Inspect and, if dirty, prepare + mark the intervention OR clean. The
    // robot is already inside the intervention OR at this point.
    bool clean = true;
    if (!cleanOperatingRoom(st, gh, clean, 0.40f, 0.30f)) {
      return finishInterrupted(gh);
    }

    // 5. Retrace the complete pathway back, opening and closing each door.
    if (!passThroughDoor(st, gh, "intervention_or_1", "clinical_corridor",
                         0.80f)) {
      return finishInterrupted(gh);
    }
    if (!passThroughDoor(st, gh, "clinical_corridor_1",
                         "entrance_and_waiting_area", 0.90f)) {
      return finishInterrupted(gh);
    }
    if (!navigateToRoom(st, gh, parking_room_, 0.97f)) {
      return finishInterrupted(gh);
    }

    publishFeedback(gh, ExecuteTask::Goal::PHASE_DONE, 1.0f);
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = true;
    result->message = clean ? "Intervention OR already clean"
                            : "Intervention OR prepared and cleaned";
    gh->succeed(result);
    RCLCPP_INFO(this->get_logger(), "CleanPrep_OR: done (%s).",
                result->message.c_str());
  }

  // Surgical Assistance and Interaction: drive from the parking spot to the
  // intervention OR (opening and closing every door behind the robot), then
  // ferry each piece of non-sterile waste from the intervention OR to the
  // storage room for disposal, one object at a time. The robot accepts an
  // object in the intervention OR, carries it through the intervention_or_2
  // door into the storage room, disposes of it, and returns for the next
  // object. When every object is handled it retraces the complete pathway back
  // to the parking spot. Any primitive returning false means the goal is being
  // cancelled -> the task is finalised as canceled.
  void runSurgicalAssist(const std::stop_token& st,
                         const std::shared_ptr<GoalHandleExecuteTask>& gh) {
    RCLCPP_INFO(this->get_logger(), "Surgical_Assist: starting.");

    // 1. Parking -> Entrance and waiting area (no door on this leg).
    if (!navigateToRoom(st, gh, "entrance_and_waiting_area", 0.05f))
      return finishInterrupted(gh);
    // 2. Through clinical_corridor_1 into the clinical corridor (open, pass,
    // close behind).
    if (!passThroughDoor(st, gh, "clinical_corridor_1", "clinical_corridor",
                         0.10f)) {
      return finishInterrupted(gh);
    }
    // 3. Through intervention_or_1 into the intervention OR.
    if (!passThroughDoor(st, gh, "intervention_or_1", "intervention_or",
                         0.20f)) {
      return finishInterrupted(gh);
    }

    // 4. Ferry each object from the intervention OR to the storage. Progress
    // spans [0.25, 0.75] split evenly across the objects.
    const std::array<const char*, 2> objects = {"Object A", "Object B"};
    const float ferry_start = 0.25f;
    const float ferry_span = 0.50f;
    const float per_object = ferry_span / static_cast<float>(objects.size());
    for (std::size_t i = 0; i < objects.size(); ++i) {
      const float base = ferry_start + per_object * static_cast<float>(i);
      if (!disposeObject(st, gh, objects[i], base, per_object)) {
        return finishInterrupted(gh);
      }
    }

    // 5. Retrace the complete pathway back, opening and closing each door.
    if (!passThroughDoor(st, gh, "intervention_or_1", "clinical_corridor",
                         0.85f)) {
      return finishInterrupted(gh);
    }
    if (!passThroughDoor(st, gh, "clinical_corridor_1",
                         "entrance_and_waiting_area", 0.92f)) {
      return finishInterrupted(gh);
    }
    if (!navigateToRoom(st, gh, parking_room_, 0.97f)) {
      return finishInterrupted(gh);
    }

    publishFeedback(gh, ExecuteTask::Goal::PHASE_DONE, 1.0f);
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = true;
    result->message = "Non-sterile waste disposed in the storage";
    gh->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Surgical_Assist: done.");
  }

  // Accept a single object in the intervention OR, carry it through the
  // intervention_or_2 door into the storage, dispose of it, and return to the
  // intervention OR. The robot must already be inside the intervention OR; it
  // is left in the intervention OR on return. Sub-step progress spans
  // [progress_start, progress_start + progress_span]. Returns false only if the
  // goal is being cancelled.
  bool disposeObject(const std::stop_token& st,
                     const std::shared_ptr<GoalHandleExecuteTask>& gh,
                     const std::string& object, float progress_start,
                     float progress_span) {
    if (!interact(st, gh, "Robot accepts " + object, interaction_duration_,
                  progress_start + progress_span * 0.20f)) {
      return false;
    }
    if (!passThroughDoor(st, gh, "intervention_or_2", "storage",
                         progress_start + progress_span * 0.45f)) {
      return false;
    }
    if (!interact(st, gh, "Robot disposes " + object, interaction_duration_,
                  progress_start + progress_span * 0.70f)) {
      return false;
    }
    return passThroughDoor(st, gh, "intervention_or_2", "intervention_or",
                           progress_start + progress_span);
  }

  // Inspect the intervention OR's cleanliness and, if it is dirty, fetch
  // supplies from the storage (via intervention_or_2, closing behind) and
  // mark it clean. The robot must already be inside the intervention OR; it is
  // left in the intervention OR on return. out_clean reports whether the room
  // was already clean. Sub-step progress spans [progress_start, progress_start
  // + progress_span]. Returns false only if the goal is being cancelled.
  bool cleanOperatingRoom(const std::stop_token& st,
                          const std::shared_ptr<GoalHandleExecuteTask>& gh,
                          bool& out_clean, float progress_start,
                          float progress_span) {
    if (!getOperatingRoomClean(st, gh, out_clean, progress_start)) {
      return false;
    }
    if (out_clean) {
      return true;
    }
    if (!passThroughDoor(st, gh, "intervention_or_2", "storage",
                         progress_start + progress_span * 0.30f)) {
      return false;
    }
    if (!interact(st, gh, "Preparing (fetching supplies)",
                  interaction_duration_,
                  progress_start + progress_span * 0.50f)) {
      return false;
    }
    if (!passThroughDoor(st, gh, "intervention_or_2", "intervention_or",
                         progress_start + progress_span * 0.75f)) {
      return false;
    }
    return setOperatingRoomClean(st, gh, progress_start + progress_span);
  }

  // Collect_lab: pick up a blood sample from the phlebotomy room, carry it to
  // the laboratory for analysis, and return to the parking spot. Every door is
  // opened and closed behind the robot on the way in and out.
  void runCollectLab(const std::stop_token& st,
                     const std::shared_ptr<GoalHandleExecuteTask>& gh) {
    RCLCPP_INFO(this->get_logger(), "Collect_lab: starting.");

    // 1. Parking -> Entrance and waiting area (no door on this leg).
    if (!navigateToRoom(st, gh, "entrance_and_waiting_area", 0.10f))
      return finishInterrupted(gh);

    // 2. Phlebotomy room: accept the blood sample, then return to the entrance
    // and waiting area.
    if (!passThroughDoor(st, gh, "phlebotomy_room", "phlebotomy_room", 0.25f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh, "Accepting blood sample", interaction_duration_,
                  0.35f)) {
      return finishInterrupted(gh);
    }
    if (!passThroughDoor(st, gh, "phlebotomy_room", "entrance_and_waiting_area",
                         0.45f)) {
      return finishInterrupted(gh);
    }

    // 3. Laboratory: run the blood test, then return to the entrance and
    // waiting area.
    if (!passThroughDoor(st, gh, "laboratory", "laboratory", 0.60f)) {
      return finishInterrupted(gh);
    }
    if (!runLabAnalysis(st, gh, 0.72f)) return finishInterrupted(gh);
    if (!passThroughDoor(st, gh, "laboratory", "entrance_and_waiting_area",
                         0.85f)) {
      return finishInterrupted(gh);
    }

    // 4. Return to the parking spot.
    if (!navigateToRoom(st, gh, parking_room_, 0.95f)) {
      return finishInterrupted(gh);
    }

    publishFeedback(gh, ExecuteTask::Goal::PHASE_DONE, 1.0f);
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = true;
    result->message = "Blood sample collected and analysed";
    gh->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Collect_lab: done.");
  }

  // Guide_Ct: greet a patient at the entrance and waiting area, send them to
  // change, meet them in the clinical corridor, guide them into the scanning
  // room for a scan, then guide them back out and bid farewell before returning
  // to the parking spot. Patient interactions are timers; the CT scan and table
  // are driven on the real CT scanner controller.
  void runGuideCt(const std::stop_token& st,
                  const std::shared_ptr<GoalHandleExecuteTask>& gh) {
    RCLCPP_INFO(this->get_logger(), "Guide_Ct: starting.");

    // 1. Parking -> Entrance and waiting area and receive the patient.
    if (!navigateToRoom(st, gh, "entrance_and_waiting_area", 0.05f))
      return finishInterrupted(gh);
    if (!interact(st, gh, "Greeting patient", interaction_duration_, 0.10f)) {
      return finishInterrupted(gh);
    }
    // Open the changing room for the patient (the robot does not enter), brief
    // them, then close it again.
    if (!operateDoor(st, gh, "changing_room", OperateDoor::Goal::COMMAND_OPEN,
                     0.15f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh,
                  "Explaining the patient to change and enter the "
                  "clinical corridor when changed",
                  interaction_duration_, 0.18f)) {
      return finishInterrupted(gh);
    }
    if (!operateDoor(st, gh, "changing_room", OperateDoor::Goal::COMMAND_CLOSE,
                     0.22f)) {
      return finishInterrupted(gh);
    }

    // 2. Meet the patient in the clinical corridor and wait for them to change.
    if (!passThroughDoor(st, gh, "clinical_corridor_1", "clinical_corridor",
                         0.30f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh, "Waiting for the patient to change",
                  interaction_duration_, 0.38f)) {
      return finishInterrupted(gh);
    }

    // 3. Guide the patient into the scanning room, scan, then lower the table.
    if (!passThroughDoor(st, gh, "scanning_room_1", "scanning_room", 0.50f)) {
      return finishInterrupted(gh);
    }
    if (!runCtScan(st, gh, 0.62f)) return finishInterrupted(gh);
    if (!moveCtTableDown(st, gh, 0.70f)) return finishInterrupted(gh);

    // 4. Guide the patient back out to the clinical corridor and on to the
    // changing room.
    if (!passThroughDoor(st, gh, "scanning_room_1", "clinical_corridor",
                         0.78f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh, "Pointing the patient to the changing room",
                  interaction_duration_, 0.84f)) {
      return finishInterrupted(gh);
    }

    // 5. Return to the entrance and waiting area, see the patient off, then
    // park.
    if (!passThroughDoor(st, gh, "clinical_corridor_1",
                         "entrance_and_waiting_area", 0.90f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh, "Waiting for the patient", interaction_duration_,
                  0.93f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh, "Farewell patient", interaction_duration_, 0.96f)) {
      return finishInterrupted(gh);
    }
    if (!navigateToRoom(st, gh, parking_room_, 0.98f)) {
      return finishInterrupted(gh);
    }

    publishFeedback(gh, ExecuteTask::Goal::PHASE_DONE, 1.0f);
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = true;
    result->message = "Patient guided through CT scan";
    gh->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Guide_Ct: done.");
  }

  // Heart_Attack: an emergency mission combining guidance, lab and cleaning.
  // The robot guides the patient to the phlebotomy room for a blood draw, then
  // guides them (via the changing room) to the scanning room where a called-in
  // healthcare professional performs the scan. While they scan, the robot
  // carries the sample to the laboratory
  // for analysis. If the cardiac markers come back abnormal it prepares and
  // cleans the intervention OR; otherwise it heads straight home. Every door is
  // opened and closed behind the robot.
  void runHeartAttack(const std::stop_token& st,
                      const std::shared_ptr<GoalHandleExecuteTask>& gh) {
    RCLCPP_INFO(this->get_logger(), "Heart_Attack: starting.");

    // A. Receive the patient and bring them to the phlebotomy room.
    if (!navigateToRoom(st, gh, "entrance_and_waiting_area", 0.03f))
      return finishInterrupted(gh);
    if (!interact(st, gh, "Greeting patient", interaction_duration_, 0.06f)) {
      return finishInterrupted(gh);
    }
    if (!passThroughDoor(st, gh, "phlebotomy_room", "phlebotomy_room", 0.10f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh, "Waiting for a professional to take the blood test",
                  interaction_duration_, 0.16f)) {
      return finishInterrupted(gh);
    }

    // B. Guide the patient toward the scanning room via the changing room
    // brief.
    if (!passThroughDoor(st, gh, "phlebotomy_room", "entrance_and_waiting_area",
                         0.22f)) {
      return finishInterrupted(gh);
    }
    if (!operateDoor(st, gh, "changing_room", OperateDoor::Goal::COMMAND_OPEN,
                     0.25f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh,
                  "Explaining the patient to change and enter the "
                  "clinical corridor when changed",
                  interaction_duration_, 0.27f)) {
      return finishInterrupted(gh);
    }
    if (!operateDoor(st, gh, "changing_room", OperateDoor::Goal::COMMAND_CLOSE,
                     0.29f)) {
      return finishInterrupted(gh);
    }
    if (!passThroughDoor(st, gh, "clinical_corridor_1", "clinical_corridor",
                         0.33f)) {
      return finishInterrupted(gh);
    }
    if (!interact(st, gh, "Waiting for the patient to change",
                  interaction_duration_, 0.37f)) {
      return finishInterrupted(gh);
    }
    if (!passThroughDoor(st, gh, "scanning_room_1", "scanning_room", 0.41f)) {
      return finishInterrupted(gh);
    }
    // The robot does not scan the patient itself: it calls in a healthcare
    // professional who arrives after a short delay, runs the CT scan and lowers
    // the table. This runs concurrently so the robot can leave for the lab
    // while the professional handles the scan.
    // The helper gets its own stop source (linked to the task's) so it can also
    // be stopped when the task ends for a reason other than cancellation.
    // `fail` joins it before the goal reaches a terminal state, so it never
    // publishes feedback on a finished goal.
    std::stop_source hp_stop;
    const std::stop_callback forward_stop(
        st, [&hp_stop] { hp_stop.request_stop(); });
    auto hp_scan = callHealthcareProfessional(hp_stop.get_token(), gh, 0.45f);
    const auto fail = [&] {
      hp_stop.request_stop();
      hp_scan.wait();
      finishInterrupted(gh);
    };

    // C. Leave for the laboratory while the professional scans, and analyse.
    if (!passThroughDoor(st, gh, "scanning_room_1", "clinical_corridor",
                         0.50f)) {
      return fail();
    }
    if (!passThroughDoor(st, gh, "clinical_corridor_1",
                         "entrance_and_waiting_area", 0.55f)) {
      return fail();
    }
    if (!passThroughDoor(st, gh, "laboratory", "laboratory", 0.60f)) {
      return fail();
    }
    bool abnormal = false;
    if (!runLabAnalysis(st, gh, 0.66f, &abnormal)) return fail();
    if (!passThroughDoor(st, gh, "laboratory", "entrance_and_waiting_area",
                         0.70f)) {
      return fail();
    }

    std::string message;
    if (abnormal) {
      // D. Abnormal markers -> go to the intervention OR and clean it.
      if (!passThroughDoor(st, gh, "clinical_corridor_1", "clinical_corridor",
                           0.74f)) {
        return fail();
      }
      if (!passThroughDoor(st, gh, "intervention_or_1", "intervention_or",
                           0.79f)) {
        return fail();
      }
      bool clean = true;
      if (!cleanOperatingRoom(st, gh, clean, 0.83f, 0.08f)) {
        return fail();
      }
      if (!passThroughDoor(st, gh, "intervention_or_1", "clinical_corridor",
                           0.93f)) {
        return fail();
      }
      if (!passThroughDoor(st, gh, "clinical_corridor_1",
                           "entrance_and_waiting_area", 0.96f)) {
        return fail();
      }
      message = "Heart attack: abnormal markers -> intervention OR cleaned";
    } else {
      message = "Heart attack: markers normal -> no cleaning required";
    }

    // Make sure the called-in healthcare professional has finished the scan
    // (and lowered the table) before the mission is reported complete.
    hp_scan.wait();

    // E. Return to the parking spot.
    if (!navigateToRoom(st, gh, parking_room_, 0.98f)) {
      return fail();
    }

    publishFeedback(gh, ExecuteTask::Goal::PHASE_DONE, 1.0f);
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = true;
    result->message = message;
    gh->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Heart_Attack: done (%s).",
                message.c_str());
  }

  // ---- Primitive steps ------------------------------------------------------

  // Teleport to a room waypoint after a simulated travel delay.
  bool navigateToRoom(const std::stop_token& st,
                      const std::shared_ptr<GoalHandleExecuteTask>& gh,
                      const std::string& room, float progress) {
    RCLCPP_INFO(this->get_logger(), "Navigating to '%s'.", room.c_str());
    publishFeedback(gh, ExecuteTask::Goal::PHASE_NAVIGATING, progress);
    if (!waitCancellable(st, gh, travel_duration_)) {
      return false;
    }
    setPoseFromWaypoint(room);
    publishPose();
    return true;
  }

  // Open a door, move into the room it leads to, then close it behind the
  // robot. Encapsulates the "no open door left behind" rule that every
  // traversal follows.
  bool passThroughDoor(const std::stop_token& st,
                       const std::shared_ptr<GoalHandleExecuteTask>& gh,
                       const std::string& door_id, const std::string& room,
                       float progress) {
    if (!operateDoor(st, gh, door_id, OperateDoor::Goal::COMMAND_OPEN,
                     progress)) {
      return false;
    }
    if (!navigateToRoom(st, gh, room, progress)) {
      return false;
    }
    return operateDoor(st, gh, door_id, OperateDoor::Goal::COMMAND_CLOSE,
                       progress);
  }

  // Command a door open or closed and wait for it to finish. A rejected goal
  // (typically the door is already in the requested state) or a non-success
  // result is logged and treated as passable so the mission is never blocked by
  // an already-open/closed door.
  bool operateDoor(const std::stop_token& st,
                   const std::shared_ptr<GoalHandleExecuteTask>& gh,
                   const std::string& door_id, uint8_t command,
                   float progress) {
    const char* verb =
        command == OperateDoor::Goal::COMMAND_OPEN ? "Opening" : "Closing";
    publishFeedback(gh, ExecuteTask::Goal::PHASE_NAVIGATING, progress);
    RCLCPP_INFO(this->get_logger(), "%s door '%s'.", verb, door_id.c_str());
    if (!door_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Door action server unavailable; skipping '%s'.",
                  door_id.c_str());
      return true;
    }
    OperateDoor::Goal goal;
    goal.device_id = door_id;
    goal.command = command;
    auto goal_future = door_client_->async_send_goal(goal);
    if (!awaitFuture(st, gh, goal_future, 10.0)) {
      return false;
    }
    auto door_goal_handle = goal_future.get();
    if (!door_goal_handle) {
      RCLCPP_INFO(this->get_logger(),
                  "Door '%s' rejected the goal (already in that state?); "
                  "continuing.",
                  door_id.c_str());
      return true;
    }
    auto result_future = door_client_->async_get_result(door_goal_handle);
    const auto timeout = resultTimeout(
        st, gh, door_params_client_,
        {command == OperateDoor::Goal::COMMAND_OPEN ? "open_duration"
                                                    : "close_duration"},
        door_fallback_timeout_);
    if (!timeout || !awaitFuture(st, gh, result_future, *timeout)) {
      return false;
    }
    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(),
                  "Door '%s' did not complete cleanly; continuing.",
                  door_id.c_str());
    }
    return true;
  }

  // Simulate a patient/room interaction as a plain cancellable timer.
  bool interact(const std::stop_token& st,
                const std::shared_ptr<GoalHandleExecuteTask>& gh,
                const std::string& label, double seconds, float progress) {
    RCLCPP_INFO(this->get_logger(), "%s...", label.c_str());
    publishFeedback(gh, ExecuteTask::Goal::PHASE_EXECUTING, progress);
    return waitCancellable(st, gh, seconds);
  }

  // Query the operating room controller for the current cleanliness. If the
  // service is unavailable the room is assumed dirty so preparation still runs.
  bool getOperatingRoomClean(const std::stop_token& st,
                             const std::shared_ptr<GoalHandleExecuteTask>& gh,
                             bool& out_clean, float progress) {
    publishFeedback(gh, ExecuteTask::Goal::PHASE_EXECUTING, progress);
    if (!operating_room_state_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Operating-room state service unavailable; assuming DIRTY.");
      out_clean = false;
      return true;
    }
    auto request = std::make_shared<GetOperatingRoomState::Request>();
    auto future = operating_room_state_client_->async_send_request(request);
    if (!awaitFuture(st, gh, future, 5.0)) {
      return false;
    }
    out_clean = future.get()->state.cleanliness == OperatingRoomState::CLEAN;
    RCLCPP_INFO(this->get_logger(), "Operating room is %s.",
                out_clean ? "CLEAN" : "DIRTY");
    return true;
  }

  // Mark the operating room clean via the SetCleanliness service.
  bool setOperatingRoomClean(const std::stop_token& st,
                             const std::shared_ptr<GoalHandleExecuteTask>& gh,
                             float progress) {
    publishFeedback(gh, ExecuteTask::Goal::PHASE_EXECUTING, progress);
    if (!cleanliness_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Set-cleanliness service unavailable; skipping.");
      return true;
    }
    auto request = std::make_shared<SetCleanliness::Request>();
    request->target_state = OperatingRoomState::CLEAN;
    auto future = cleanliness_client_->async_send_request(request);
    if (!awaitFuture(st, gh, future, 5.0)) {
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Set operating room CLEAN (accepted=%d).",
                future.get()->accepted);
    return true;
  }

  // Run a blood-test analysis on the lab controller and wait for the result. A
  // rejected goal or a non-success result is logged and treated as passable.
  // When out_abnormal is given it is set true if any cardiac marker exceeds its
  // reference limit (cTnT > 14 ng/L, cTnI > 20 ng/L, CK-MB > 5%).
  bool runLabAnalysis(const std::stop_token& st,
                      const std::shared_ptr<GoalHandleExecuteTask>& gh,
                      float progress, bool* out_abnormal = nullptr) {
    if (out_abnormal != nullptr) {
      *out_abnormal = false;
    }
    publishFeedback(gh, ExecuteTask::Goal::PHASE_EXECUTING, progress);
    RCLCPP_INFO(this->get_logger(), "Running blood-test analysis.");
    if (!lab_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Lab action server unavailable; skipping analysis.");
      return true;
    }
    AnalyzeBlood::Goal goal;
    goal.command = "START_BLOODTEST";
    auto goal_future = lab_client_->async_send_goal(goal);
    if (!awaitFuture(st, gh, goal_future, 10.0)) {
      return false;
    }
    auto lab_goal_handle = goal_future.get();
    if (!lab_goal_handle) {
      RCLCPP_WARN(this->get_logger(),
                  "Lab rejected the analysis goal; continuing.");
      return true;
    }
    auto result_future = lab_client_->async_get_result(lab_goal_handle);
    const auto timeout =
        resultTimeout(st, gh, lab_params_client_, {"analysis_duration"},
                      lab_fallback_timeout_);
    if (!timeout || !awaitFuture(st, gh, result_future, *timeout)) {
      return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      const bool abnormal = wrapped.result->ctnt > 14.0f ||
                            wrapped.result->ctni > 20.0f ||
                            wrapped.result->ck_mb > 5.0f;
      if (out_abnormal != nullptr) {
        *out_abnormal = abnormal;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Blood test done: cTnT=%.1f cTnI=%.1f CK-MB=%.1f -> %s (%s).",
                  wrapped.result->ctnt, wrapped.result->ctni,
                  wrapped.result->ck_mb, abnormal ? "ABNORMAL" : "normal",
                  wrapped.result->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Blood test did not complete cleanly; continuing.");
    }
    return true;
  }

  // Run a CT scan on the scanner controller and wait for it to finish. A
  // rejected goal or a non-success result is logged and treated as passable.
  bool runCtScan(const std::stop_token& st,
                 const std::shared_ptr<GoalHandleExecuteTask>& gh,
                 float progress) {
    publishFeedback(gh, ExecuteTask::Goal::PHASE_EXECUTING, progress);
    RCLCPP_INFO(this->get_logger(), "Starting CT scan.");
    if (!scan_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(),
                  "CT scan action server unavailable; skipping scan.");
      return true;
    }
    RunScan::Goal goal;
    goal.command = RunScan::Goal::COMMAND_START;
    auto goal_future = scan_client_->async_send_goal(goal);
    if (!awaitFuture(st, gh, goal_future, 10.0)) {
      return false;
    }
    auto scan_goal_handle = goal_future.get();
    if (!scan_goal_handle) {
      RCLCPP_WARN(this->get_logger(),
                  "CT scanner rejected the scan goal; continuing.");
      return true;
    }
    auto result_future = scan_client_->async_get_result(scan_goal_handle);
    // A scan first lifts the table to the top, then sweeps.
    const auto timeout = resultTimeout(st, gh, ct_params_client_,
                                       {"table_move_duration", "scan_duration"},
                                       scan_fallback_timeout_);
    if (!timeout || !awaitFuture(st, gh, result_future, *timeout)) {
      return false;
    }
    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(),
                  "CT scan did not complete cleanly; continuing.");
    }
    return true;
  }

  // Lower the CT table (MoveTable DOWN) and wait for it to finish. A rejected
  // goal or a non-success result is logged and treated as passable.
  bool moveCtTableDown(const std::stop_token& st,
                       const std::shared_ptr<GoalHandleExecuteTask>& gh,
                       float progress) {
    publishFeedback(gh, ExecuteTask::Goal::PHASE_EXECUTING, progress);
    RCLCPP_INFO(this->get_logger(), "Lowering CT table.");
    if (!move_table_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(),
                  "CT table action server unavailable; skipping.");
      return true;
    }
    MoveTable::Goal goal;
    goal.command = MoveTable::Goal::COMMAND_DOWN;
    auto goal_future = move_table_client_->async_send_goal(goal);
    if (!awaitFuture(st, gh, goal_future, 10.0)) {
      return false;
    }
    auto table_goal_handle = goal_future.get();
    if (!table_goal_handle) {
      RCLCPP_WARN(this->get_logger(),
                  "CT table rejected the move goal; continuing.");
      return true;
    }
    auto result_future =
        move_table_client_->async_get_result(table_goal_handle);
    const auto timeout =
        resultTimeout(st, gh, ct_params_client_, {"table_move_duration"},
                      table_fallback_timeout_);
    if (!timeout || !awaitFuture(st, gh, result_future, *timeout)) {
      return false;
    }
    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(),
                  "CT table did not complete cleanly; continuing.");
    }
    return true;
  }

  // Call a healthcare professional to take over the CT scan and run it
  // concurrently with the rest of the mission. Returns immediately with a
  // future the caller can wait on: on its own thread the professional arrives
  // after a simulated delay, runs the CT scan and lowers the table. Every step
  // honours the goal's stop token / cancellation, so the future completes
  // promptly if the task is being cancelled.
  std::future<void> callHealthcareProfessional(
      std::stop_token st, std::shared_ptr<GoalHandleExecuteTask> gh,
      float progress) {
    return std::async(std::launch::async, [this, st, gh, progress]() {
      RCLCPP_INFO(this->get_logger(),
                  "Calling a healthcare professional to take the CT scan.");
      // Simulate the professional walking over to the scanning room.
      if (!waitCancellable(st, gh, 10.0)) {
        return;
      }
      // They run the scan and lower the table for the patient.
      if (!runCtScan(st, gh, progress)) {
        return;
      }
      moveCtTableDown(st, gh, progress);
    });
  }

  // How long to wait for a controller's action result: factor * (sum of the
  // controller's `duration_params`) + margin, read from the controller when the
  // goal is sent so a reconfigured duration never trips the timeout. Falls back
  // to `fallback_s` if the parameters cannot be read. Returns nullopt if the
  // task is interrupted while querying.
  std::optional<double> resultTimeout(
      const std::stop_token& st,
      const std::shared_ptr<GoalHandleExecuteTask>& gh,
      const rclcpp::AsyncParametersClient::SharedPtr& client,
      const std::vector<std::string>& duration_params, double fallback_s) {
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot read controller durations; using a %.1f s result "
                  "timeout.",
                  fallback_s);
      return fallback_s;
    }
    auto future = client->get_parameters(duration_params);
    if (!awaitFuture(st, gh, future, 2.0)) {
      if (st.stop_requested() || gh->is_canceling() || !rclcpp::ok()) {
        return std::nullopt;
      }
      return fallback_s;
    }
    double total = 0.0;
    try {
      for (const auto& parameter : future.get()) {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          throw std::runtime_error("parameter '" + parameter.get_name() +
                                   "' is not a double");
        }
        total += parameter.as_double();
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot read controller durations (%s); using a %.1f s "
                  "result timeout.",
                  e.what(), fallback_s);
      return fallback_s;
    }
    return controller_timeout_factor_ * total + controller_timeout_margin_;
  }

  // Wait for a future to become ready, waking early (returning false) on
  // cancellation, node shutdown, or timeout.
  template <typename FutureT>
  bool awaitFuture(const std::stop_token& st,
                   const std::shared_ptr<GoalHandleExecuteTask>& gh,
                   FutureT& future, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_s);
    while (rclcpp::ok()) {
      if (future.wait_for(100ms) == std::future_status::ready) {
        return true;
      }
      if (st.stop_requested() || gh->is_canceling()) {
        return false;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        RCLCPP_WARN(this->get_logger(), "Timed out waiting for a response.");
        return false;
      }
    }
    return false;
  }

  // Finalise a task whose protocol stopped early. Only a goal the client asked
  // to cancel may transition to CANCELED; every other reason (a timeout, node
  // shutdown) must ABORT -- calling canceled() on a goal that is not canceling
  // throws in rclcpp.
  void finishInterrupted(const std::shared_ptr<GoalHandleExecuteTask>& gh,
                         bool return_home = true) {
    if (return_home) {
      // An interrupted mission may leave the robot stranded mid-route, so
      // teleport it straight back to its parking/home waypoint.
      setPoseFromWaypoint(parking_room_);
    }
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = false;
    if (!gh->is_canceling() && cancelRequested(gh)) {
      // The worker was woken from handleCancel, which runs just before rclcpp
      // moves the goal to CANCELING on the executor thread. Wait (briefly) for
      // that transition so the goal can be finalised as CANCELED.
      const auto deadline = std::chrono::steady_clock::now() + 1s;
      while (!gh->is_canceling() &&
             std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
      }
    }
    if (gh->is_canceling()) {
      result->message = "Task cancelled";
      gh->canceled(result);
    } else {
      result->message = "Task aborted (timeout or shutdown)";
      gh->abort(result);
    }
    RCLCPP_INFO(this->get_logger(), "Task %s: %s.",
                gh->get_goal()->task_id.c_str(), result->message.c_str());
  }

  // ---- Members --------------------------------------------------------------

  // Whether handleCancel accepted a cancel request for this goal.
  bool cancelRequested(const std::shared_ptr<GoalHandleExecuteTask>& gh) {
    std::scoped_lock lock(workers_mutex_);
    return std::any_of(
        workers_.begin(), workers_.end(), [&gh](const Worker& w) {
          return w.goal == gh && w.state->cancel_requested.load();
        });
  }

  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp_action::Client<OperateDoor>::SharedPtr door_client_;
  rclcpp_action::Client<AnalyzeBlood>::SharedPtr lab_client_;
  rclcpp_action::Client<RunScan>::SharedPtr scan_client_;
  rclcpp_action::Client<MoveTable>::SharedPtr move_table_client_;
  rclcpp::Client<SetCleanliness>::SharedPtr cleanliness_client_;
  rclcpp::Client<GetOperatingRoomState>::SharedPtr operating_room_state_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  std::unordered_map<std::string, Waypoint> waypoints_;

  geometry_msgs::msg::PoseStamped current_pose_;
  std::string current_room_;
  std::mutex pose_mutex_;

  // FIFO task queue: the robot runs one task at a time. Tickets are handed out
  // in acceptance order; the worker holding queue_.front() runs next once the
  // robot is free.
  std::mutex queue_mutex_;
  std::condition_variable_any queue_cv_;
  std::deque<std::uint64_t> queue_;
  std::uint64_t next_ticket_{0};
  bool robot_busy_{false};
  std::vector<Worker> workers_;
  std::mutex workers_mutex_;
  rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;

  rclcpp::AsyncParametersClient::SharedPtr door_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr lab_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr ct_params_client_;

  // Cached parameter values.
  std::string parking_room_;
  double travel_duration_{3.0};
  double interaction_duration_{4.0};
  double pose_publish_rate_{5.0};
  std::string pose_topic_;
  std::string pose_frame_id_;
  std::string base_frame_id_;
  std::string execute_task_action_name_;
  std::string door_action_name_;
  std::string lab_action_name_;
  std::string scan_action_name_;
  std::string move_table_action_name_;
  std::string cleanliness_service_name_;
  std::string operating_room_state_service_name_;
  std::string door_controller_node_;
  std::string lab_controller_node_;
  std::string ct_scanner_controller_node_;
  double controller_timeout_factor_{1.5};
  double controller_timeout_margin_{10.0};
  double door_fallback_timeout_{30.0};
  double lab_fallback_timeout_{60.0};
  double scan_fallback_timeout_{60.0};
  double table_fallback_timeout_{30.0};
  // Per-robot identity. When robot_id_ is non-empty the node runs as one of
  // several robots: its own action/pose/frame names are derived from the id and
  // every waypoint is shifted by (pose_offset_x_, pose_offset_y_) so multiple
  // robots parked or standing in the same room do not render on top of each
  // other.
  std::string robot_id_;
  double pose_offset_x_{0.0};
  double pose_offset_y_{0.0};
  int robot_index_{0};
  std::string marker_topic_;
};

}  // namespace hospital_robot

RCLCPP_COMPONENTS_REGISTER_NODE(hospital_robot::RobotServerNode)
