// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/hospital_panel.hpp"

#include <QFile>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QScrollArea>
#include <QSize>
#include <QString>
#include <QStringList>
#include <QStyle>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

#include "hospital_rviz_panel/collapsible_section.hpp"
#include "hospital_rviz_panel/ct_control_widget.hpp"
#include "hospital_rviz_panel/door_control_widget.hpp"
#include "hospital_rviz_panel/lab_control_widget.hpp"
#include "hospital_rviz_panel/operating_room_control_widget.hpp"
#include "hospital_rviz_panel/recording_control_widget.hpp"
#include "hospital_rviz_panel/ward_management_widget.hpp"

namespace hospital_rviz_panel {

namespace {
constexpr char kDoorStatesTopic[] = "/hospital/doors/states";
constexpr char kDoorActionName[] = "/hospital/doors/operate";
constexpr char kCtTableStateTopic[] = "/hospital/ct/table/state";
constexpr char kCtScanStateTopic[] = "/hospital/ct/scan/state";
constexpr char kLabStateTopic[] = "/hospital/lab/state";
constexpr char kCtTableActionName[] = "/hospital/ct/table/move";
constexpr char kCtScanActionName[] = "/hospital/ct/scan/run";
constexpr char kStopwatchTopic[] = "/hospital/stopwatch";
constexpr char kAnalyzeBloodActionName[] = "/hospital/lab/analyze";
constexpr char kOperatingRoomStateTopic[] = "/hospital/operating_room/state";
constexpr char kSetCleanlinessService[] =
    "/hospital/operating_room/set_cleanliness";
constexpr char kOperatingRoomNodeName[] = "operating_room_controller_node";
constexpr char kTaskEventTopic[] = "/hospital/task_events";
constexpr char kSetRecordingService[] = "/hospital/logging/set_recording";

// Load the panel theme from the installed package share directory. Returns an
// empty string if the file cannot be found, in which case the panel falls back
// to RViz defaults.
QString loadStyleSheet() {
  try {
    const std::string path =
        ament_index_cpp::get_package_share_directory("hospital_rviz_panel") +
        "/resource/style.qss";
    QFile file(QString::fromStdString(path));
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QTextStream stream(&file);
      return stream.readAll();
    }
  } catch (const std::exception&) {
    // Package not found in the index; fall through to the empty default.
  }
  return QString();
}
}  // namespace

HospitalControlPanel::HospitalControlPanel(QWidget* parent)
    : rviz_common::Panel(parent) {}

HospitalControlPanel::~HospitalControlPanel() {
  if (refresh_timer_ != nullptr) {
    refresh_timer_->stop();
  }

  {
    std::lock_guard lock(callback_lifetime_->mutex);
    callback_lifetime_->active = false;
  }

  door_states_sub_.reset();
  ct_table_sub_.reset();
  ct_scan_sub_.reset();
  ct_procedure_sub_.reset();
  lab_state_sub_.reset();
  stopwatch_sub_.reset();
  operating_room_sub_.reset();

  door_action_.reset();
  move_action_.reset();
  scan_action_.reset();
  ward_actions_.clear();
  lab_action_.reset();
  set_cleanliness_client_.reset();
  set_recording_client_.reset();
  task_event_pub_.reset();
  xray_param_client_.reset();
  node_.reset();
}

QSize HospitalControlPanel::sizeHint() const {
  // Start ~40% wider than the natural content width so every control is
  // comfortably visible when RViz first docks the panel. The user can still
  // resize it afterwards.
  QSize hint = rviz_common::Panel::sizeHint();
  hint.setWidth(static_cast<int>(hint.width() * 1.4));
  return hint;
}

void HospitalControlPanel::onInitialize() {
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // The door and CT layout is fixed in buildUi(); there is a single CT scanner.

  // Robot roster for the Mission dispatch dropdown. Each id gets its own
  // ExecuteTask action client at /hospital/<id>/ward/execute_task (built
  // below). The bringup params file overrides this compiled default.
  if (!node_->has_parameter("robot_ids")) {
    node_->declare_parameter<std::vector<std::string>>(
        "robot_ids", std::vector<std::string>{"robot_1", "robot_2"});
  }
  const auto robot_ids = node_->get_parameter("robot_ids").as_string_array();

  buildUi();

  // Shared state subscriptions (messages carry device_id; widgets filter).
  // The door controller latches (transient_local) the aggregated snapshot of
  // all doors, so a freshly-opened panel immediately shows every door.
  door_states_sub_ =
      node_->create_subscription<hospital_interfaces::msg::DoorStateArray>(
          kDoorStatesTopic, rclcpp::QoS(1).transient_local(),
          std::function<void(
              hospital_interfaces::msg::DoorStateArray::ConstSharedPtr)>(
              guardCallback(
                  [this](
                      hospital_interfaces::msg::DoorStateArray::ConstSharedPtr
                          msg) { onDoorStates(msg); })));
  // The CT table and scan states are latched by the controller
  // (transient_local) so a matching QoS lets a freshly-opened panel immediately
  // show the current CT state even when the scanner is idle.
  rclcpp::QoS ct_table_qos(10);
  ct_table_qos.transient_local();
  ct_table_sub_ =
      node_->create_subscription<hospital_interfaces::msg::CtTableState>(
          kCtTableStateTopic, ct_table_qos,
          std::function<void(
              hospital_interfaces::msg::CtTableState::ConstSharedPtr)>(
              guardCallback(
                  [this](hospital_interfaces::msg::CtTableState::ConstSharedPtr
                             msg) { onCtTableState(msg); })));
  rclcpp::QoS ct_scan_qos(10);
  ct_scan_qos.transient_local();
  ct_scan_sub_ =
      node_->create_subscription<hospital_interfaces::msg::CtScannerState>(
          kCtScanStateTopic, ct_scan_qos,
          std::function<void(
              hospital_interfaces::msg::CtScannerState::ConstSharedPtr)>(
              guardCallback(
                  [this](
                      hospital_interfaces::msg::CtScannerState::ConstSharedPtr
                          msg) { onCtScanState(msg); })));
  ct_procedure_sub_ = node_->create_subscription<
      hospital_interfaces::msg::CtScanProcedureState>(
      "/hospital/ct/scan/procedure_state", rclcpp::QoS(1).transient_local(),
      std::function<void(
          hospital_interfaces::msg::CtScanProcedureState::ConstSharedPtr)>(
          guardCallback(
              [this](
                  hospital_interfaces::msg::CtScanProcedureState::ConstSharedPtr
                      state) {
                std::scoped_lock lock(state_mutex_);
                ct_procedure_phase_ = state->phase;
                ct_procedure_dirty_ = true;
              })));

  // The lab state is latched by the controller (transient_local) so a matching
  // QoS lets a freshly-opened panel immediately show the current snapshot.
  rclcpp::QoS lab_state_qos(1);
  lab_state_qos.transient_local();
  lab_state_sub_ = node_->create_subscription<
      hospital_interfaces::msg::LabState>(
      kLabStateTopic, lab_state_qos,
      std::function<void(hospital_interfaces::msg::LabState::ConstSharedPtr)>(
          guardCallback(
              [this](hospital_interfaces::msg::LabState::ConstSharedPtr msg) {
                onLabState(msg);
              })));

  // Stopwatch readings are latched by the controllers (transient_local), so a
  // matching QoS lets a freshly-opened panel immediately show the last result.
  rclcpp::QoS stopwatch_qos(20);
  stopwatch_qos.transient_local();
  stopwatch_sub_ = node_->create_subscription<
      hospital_interfaces::msg::Stopwatch>(
      kStopwatchTopic, stopwatch_qos,
      std::function<void(hospital_interfaces::msg::Stopwatch::ConstSharedPtr)>(
          guardCallback(
              [this](hospital_interfaces::msg::Stopwatch::ConstSharedPtr msg) {
                onStopwatch(msg);
              })));

  // The operating room controller latches its state (transient_local) and
  // republishes it on every change, so a matching QoS lets a freshly-opened
  // panel immediately show the current room state.
  operating_room_sub_ =
      node_->create_subscription<hospital_interfaces::msg::OperatingRoomState>(
          kOperatingRoomStateTopic, rclcpp::QoS(1).transient_local(),
          std::function<void(
              hospital_interfaces::msg::OperatingRoomState::ConstSharedPtr)>(
              guardCallback([this](hospital_interfaces::msg::
                                       OperatingRoomState::ConstSharedPtr msg) {
                onOperatingRoomState(msg);
              })));

  // Shared action clients (one server each; device_id is in the goal).
  door_action_ =
      rclcpp_action::create_client<OperateDoor>(node_, kDoorActionName);
  move_action_ =
      rclcpp_action::create_client<MoveTable>(node_, kCtTableActionName);
  scan_action_ =
      rclcpp_action::create_client<RunScan>(node_, kCtScanActionName);
  // One ExecuteTask client per robot. The action name convention MUST match
  // RobotServerNode's derived name (see robot_server_node.cpp).
  QStringList robot_labels;
  for (const auto& robot_id : robot_ids) {
    ward_actions_[robot_id] = rclcpp_action::create_client<ExecuteTask>(
        node_, "/hospital/" + robot_id + "/ward/execute_task");
    robot_labels << QString::fromStdString(robot_id);
  }
  if (ward_widget_ != nullptr) {
    ward_widget_->setRobots(robot_labels);
  }
  lab_action_ = rclcpp_action::create_client<AnalyzeBlood>(
      node_, kAnalyzeBloodActionName);

  // Operating room service clients and the X-ray parameter client.
  set_cleanliness_client_ =
      node_->create_client<SetCleanliness>(kSetCleanlinessService);
  xray_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, kOperatingRoomNodeName);

  // Activity logger: SetBool recording toggle + TaskEvent forwarding. The task
  // events carry the exact durations measured by the ward widget so the log
  // matches the on-screen table.
  set_recording_client_ =
      node_->create_client<std_srvs::srv::SetBool>(kSetRecordingService);
  task_event_pub_ =
      node_->create_publisher<hospital_interfaces::msg::TaskEvent>(
          kTaskEventTopic, 10);

  // Apply cached telemetry to the widgets at a fixed UI rate, independent of
  // the controller publish rate. A flood of high-rate feedback therefore only
  // ever updates the cache (cheap) and never floods the Qt event queue.
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this,
          &HospitalControlPanel::refreshWidgets);
  refresh_timer_->start(33);  // ~30 Hz
}

void HospitalControlPanel::buildUi() {
  // Apply the panel theme to the whole panel; it cascades to children.
  const QString style_sheet = loadStyleSheet();
  if (!style_sheet.isEmpty()) {
    setStyleSheet(style_sheet);
  }

  auto* content = new QWidget();
  auto* content_layout = new QVBoxLayout(content);
  content_layout->setContentsMargins(6, 6, 6, 6);
  content_layout->setSpacing(8);

  // Creates a door control, wires its signals to the shared action client and
  // registers it for state routing. A door that connects two rooms is built
  // once per section, so the same device_id collects several mirrored widgets.
  auto make_door = [this](const std::string& device_id,
                          const QString& display_name) -> DoorControlWidget* {
    auto* widget =
        new DoorControlWidget(QString::fromStdString(device_id), display_name);
    connect(widget, &DoorControlWidget::commandRequested, this,
            [this, device_id](int command) {
              sendDoorGoal(device_id, static_cast<uint8_t>(command));
            });
    connect(widget, &DoorControlWidget::stopRequested, this,
            [this, device_id]() { cancelDoorGoal(device_id); });
    connect(widget, &DoorControlWidget::statusPosted, this,
            &HospitalControlPanel::postStatus);
    door_widgets_[device_id].push_back(widget);
    return widget;
  };

  // Each section groups the doors a single hospital location borders. Some
  // doors are shared between rooms and therefore appear in two sections.
  // -- Intervention ORs: two columns, a shared door, then the CT scanner. --
  {
    auto* grid = new QGridLayout();
    grid->setContentsMargins(6, 6, 6, 6);
    grid->addWidget(make_door("intervention_or_2", "Intervention OR 2"), 0, 0);
    grid->addWidget(make_door("scanning_room_2", "Scanning Room 2"), 0, 1);
    grid->addWidget(make_door("intervention_or_1", "Intervention OR 1"), 1, 0);
    grid->addWidget(make_door("scanning_room_1", "Scanning Room 1"), 1, 1);
    grid->addWidget(make_door("clinical_corridor_3", "Clinical Corridor 3"), 2,
                    0);

    auto* widget = new CtControlWidget(content);
    connect(widget, &CtControlWidget::moveRequested, this, [this](int command) {
      sendMoveGoal(static_cast<uint8_t>(command));
    });
    connect(widget, &CtControlWidget::stopMoveRequested, this,
            [this]() { cancelMoveGoal(); });
    connect(widget, &CtControlWidget::scanRequested, this, [this](int command) {
      sendScanGoal(static_cast<uint8_t>(command));
    });
    connect(widget, &CtControlWidget::statusPosted, this,
            &HospitalControlPanel::postStatus);
    ct_widget_ = widget;
    grid->addWidget(widget, 3, 0, 1, 2);

    auto* section = new CollapsibleSection("Intervention ORs", content);
    section->setContentLayout(grid);
    content_layout->addWidget(section);
  }

  // -- Clinical Corridor: two columns plus a shared door spanning beneath them.
  // --
  {
    auto* grid = new QGridLayout();
    grid->setContentsMargins(6, 6, 6, 6);
    grid->addWidget(make_door("intervention_or_1", "Intervention OR 1"), 0, 0);
    grid->addWidget(make_door("clinical_corridor_3", "Clinical Corridor 3"), 0,
                    1);
    grid->addWidget(make_door("scanning_room_1", "Scanning Room 1"), 1, 0);
    grid->addWidget(make_door("clinical_corridor_2", "Clinical Corridor 2"), 1,
                    1);
    grid->addWidget(make_door("clinical_corridor_1", "Clinical Corridor 1"), 2,
                    0);
    auto* section = new CollapsibleSection("Clinical Corridor", content);
    section->setContentLayout(grid);
    content_layout->addWidget(section);
  }

  // -- Entrance and Waiting Area: two columns of three doors. --
  {
    auto* grid = new QGridLayout();
    grid->setContentsMargins(6, 6, 6, 6);
    grid->addWidget(make_door("clinical_corridor_1", "Clinical Corridor 1"), 0,
                    0);
    grid->addWidget(make_door("changing_room", "Changing Room"), 0, 1);
    grid->addWidget(make_door("laboratory", "Laboratory"), 1, 0);
    grid->addWidget(make_door("phlebotomy_room", "Phlebotomy Room"), 1, 1);
    grid->addWidget(make_door("entry_l", "Entry L"), 2, 0);
    grid->addWidget(make_door("entry_r", "Entry R"), 2, 1);
    auto* section =
        new CollapsibleSection("Entrance and Waiting Area", content);
    section->setContentLayout(grid);
    content_layout->addWidget(section);
  }

  // Ward Management: dispatch missions to the robot and time each task. The
  // robot owns the queue, so the panel only fires goals and renders feedback.
  {
    ward_widget_ = new WardManagementWidget(content);
    connect(ward_widget_, &WardManagementWidget::taskSubmitted, this,
            [this](uint8_t value, const QString& robot_id,
                   const QString& target_location, const QString& task_id) {
              sendExecuteTaskGoal(robot_id.toStdString(), value,
                                  target_location.toStdString(),
                                  task_id.toStdString());
            });
    connect(ward_widget_, &WardManagementWidget::statusPosted, this,
            &HospitalControlPanel::postStatus);
    // Abort press: cancel the task's goal so the robot stops (and teleports
    // home) or the queued task is dropped before it starts.
    connect(ward_widget_, &WardManagementWidget::taskAbortRequested, this,
            [this](const QString& robot_id, const QString& task_id,
                   bool /*was_running*/) {
              cancelWardGoal(robot_id.toStdString(), task_id.toStdString());
            });
    // Forward ward-task lifecycle to the activity logger. The widget is the
    // authoritative task timer, so it hands us the exact elapsed value.
    connect(ward_widget_, &WardManagementWidget::taskStarted, this,
            [this](const QString& robot_id, const QString& command_label,
                   const QString& task_id) {
              publishTaskEvent(hospital_interfaces::msg::TaskEvent::START,
                               robot_id.toStdString(),
                               command_label.toStdString(),
                               task_id.toStdString(), false, 0.0F);
            });
    connect(
        ward_widget_, &WardManagementWidget::taskFinished, this,
        [this](const QString& robot_id, const QString& command_label,
               const QString& task_id, bool success, float elapsed_seconds) {
          publishTaskEvent(hospital_interfaces::msg::TaskEvent::END,
                           robot_id.toStdString(), command_label.toStdString(),
                           task_id.toStdString(), success, elapsed_seconds);
        });
    auto* section = new CollapsibleSection("Ward Management", content);
    auto* section_layout = new QVBoxLayout();
    section_layout->setContentsMargins(6, 6, 6, 6);
    section_layout->addWidget(ward_widget_);

    // Activity recording: Start/Stop buttons for the room-dwell + task logger.
    recording_widget_ = new RecordingControlWidget(content);
    connect(recording_widget_, &RecordingControlWidget::startRecordingRequested,
            this, [this]() { setRecording(true); });
    connect(recording_widget_, &RecordingControlWidget::stopRecordingRequested,
            this, [this]() { setRecording(false); });
    section_layout->addWidget(recording_widget_);

    section->setContentLayout(section_layout);
    content_layout->addWidget(section);
  }

  // Laboratory & Operating Room: the read-only lab analysis stopwatch and the
  // operating room controls (cleanliness, state refresh, X-ray toggle) share a
  // single collapsible section.
  {
    // Laboratory: start a blood analysis and render the stopwatch/results.
    lab_widget_ = new LabControlWidget(content);
    connect(lab_widget_, &LabControlWidget::startAnalysisRequested, this,
            &HospitalControlPanel::sendAnalyzeBloodGoal);

    // Operating room: set cleanliness, refresh state, and toggle the X-ray.
    operating_room_widget_ = new OperatingRoomControlWidget(content);
    connect(operating_room_widget_,
            &OperatingRoomControlWidget::setCleanlinessRequested, this,
            [this](int target_state) {
              sendSetCleanliness(static_cast<uint8_t>(target_state));
            });
    connect(operating_room_widget_,
            &OperatingRoomControlWidget::xrayToggleRequested, this,
            [this](bool active) { setXrayActive(active); });
    connect(operating_room_widget_, &OperatingRoomControlWidget::statusPosted,
            this, &HospitalControlPanel::postStatus);

    auto* section_layout = new QVBoxLayout();
    section_layout->setContentsMargins(6, 6, 6, 6);
    section_layout->addWidget(lab_widget_);
    section_layout->addWidget(operating_room_widget_);
    auto* section =
        new CollapsibleSection("Laboratory & Operating Room", content);
    section->setContentLayout(section_layout);
    content_layout->addWidget(section);
  }
  content_layout->addStretch(1);

  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setWidget(content);

  // A single status line, pinned below the scroll area, reports all panel
  // messages and is always visible regardless of scrolling.
  status_bar_ = new QLabel("Ready", this);
  status_bar_->setObjectName("statusBar");
  status_bar_->setWordWrap(true);
  postStatus("Ready", false);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);
  outer->addWidget(scroll, 1);
  outer->addWidget(status_bar_);
}

void HospitalControlPanel::postStatus(const QString& message, bool error) {
  if (status_bar_ == nullptr) {
    return;
  }
  status_bar_->setText(message);
  // Colours live in the shared theme (style.qss); the ``error`` dynamic
  // property drives the QLabel#statusBar[error="true"] rule. Re-polish so the
  // property change is picked up immediately.
  status_bar_->setProperty("error", error);
  status_bar_->style()->unpolish(status_bar_);
  status_bar_->style()->polish(status_bar_);
}

// -- Commanding ---------------------------------------------------------------

void HospitalControlPanel::sendDoorGoal(const std::string& device_id,
                                        uint8_t command) {
  auto widget_it = door_widgets_.find(device_id);
  if (!door_action_->action_server_is_ready()) {
    if (widget_it != door_widgets_.end()) {
      for (auto* widget : widget_it->second) {
        widget->setResult(false, "Door controller not available");
      }
    }
    return;
  }

  OperateDoor::Goal goal;
  goal.device_id = device_id;
  goal.command = command;

  rclcpp_action::Client<OperateDoor>::SendGoalOptions options;
  // Action callbacks run on RViz's ROS executor thread. Qt widgets may only be
  // touched from the GUI thread, so widget updates are marshalled there with
  // QMetaObject::invokeMethod. The goal-handle bookkeeping, however, is kept on
  // the ROS thread under handles_mutex_ (NOT marshalled): deferring it through
  // the Qt event queue let a Stop press outrun the not-yet-stored handle at
  // high publish rates ("No door movement to stop").
  options.feedback_callback = guardCallback(
      [this](rclcpp_action::ClientGoalHandle<OperateDoor>::SharedPtr,
             const std::shared_ptr<const OperateDoor::Feedback> feedback) {
        updateDoorCache(feedback->telemetry);
      });
  options.goal_response_callback = guardCallback(
      [this, device_id](
          rclcpp_action::ClientGoalHandle<OperateDoor>::SharedPtr goal_handle) {
        if (!goal_handle) {
          {
            std::scoped_lock lock(handles_mutex_);
            door_in_flight_.erase(device_id);
            door_cancel_pending_.erase(device_id);
          }
          QMetaObject::invokeMethod(
              this,
              [this, device_id]() {
                auto it = door_widgets_.find(device_id);
                if (it != door_widgets_.end()) {
                  for (auto* widget : it->second) {
                    widget->setResult(false, "Goal rejected by controller");
                  }
                }
              },
              Qt::QueuedConnection);
          return;
        }
        bool cancel_now = false;
        {
          std::scoped_lock lock(handles_mutex_);
          door_goal_handles_[device_id] = goal_handle;
          door_in_flight_.erase(device_id);
          cancel_now = door_cancel_pending_.erase(device_id) > 0;
        }
        if (cancel_now) {
          // Stop was pressed before the goal was accepted; honour it now.
          door_action_->async_cancel_goal(goal_handle);
        }
      });
  options.result_callback = guardCallback(
      [this, device_id](
          const rclcpp_action::ClientGoalHandle<OperateDoor>::WrappedResult&
              result) {
        {
          std::scoped_lock lock(handles_mutex_);
          door_goal_handles_.erase(device_id);
          door_in_flight_.erase(device_id);
          door_cancel_pending_.erase(device_id);
        }
        const bool succeeded =
            (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        const bool success = succeeded && result.result->success;
        const QString message =
            succeeded ? QString::fromStdString(result.result->message)
                      : QString("Goal did not succeed");
        QMetaObject::invokeMethod(
            this,
            [this, device_id, success, message]() {
              auto it = door_widgets_.find(device_id);
              if (it == door_widgets_.end()) {
                return;
              }
              for (auto* widget : it->second) {
                widget->setResult(success, message);
              }
            },
            Qt::QueuedConnection);
      });

  {
    std::scoped_lock lock(handles_mutex_);
    door_in_flight_.insert(device_id);
    door_cancel_pending_.erase(device_id);
  }
  door_action_->async_send_goal(goal, options);
}

void HospitalControlPanel::sendMoveGoal(uint8_t command) {
  if (!move_action_->action_server_is_ready()) {
    if (ct_widget_ != nullptr) {
      ct_widget_->setMoveResult(false, "CT controller not available");
    }
    return;
  }

  MoveTable::Goal goal;
  goal.command = command;

  rclcpp_action::Client<MoveTable>::SendGoalOptions options;
  options.feedback_callback = guardCallback(
      [this](rclcpp_action::ClientGoalHandle<MoveTable>::SharedPtr,
             const std::shared_ptr<const MoveTable::Feedback> feedback) {
        updateCtTableCache(feedback->telemetry);
      });
  options.goal_response_callback = guardCallback(
      [this](
          rclcpp_action::ClientGoalHandle<MoveTable>::SharedPtr goal_handle) {
        if (!goal_handle) {
          {
            std::scoped_lock lock(handles_mutex_);
            move_in_flight_ = false;
            move_cancel_pending_ = false;
          }
          QMetaObject::invokeMethod(
              this,
              [this]() {
                if (ct_widget_ != nullptr) {
                  ct_widget_->setMoveResult(
                      false, "Table-move goal rejected by controller");
                }
              },
              Qt::QueuedConnection);
          return;
        }
        bool cancel_now = false;
        {
          std::scoped_lock lock(handles_mutex_);
          move_goal_handle_ = goal_handle;
          move_in_flight_ = false;
          cancel_now = move_cancel_pending_;
          move_cancel_pending_ = false;
        }
        if (cancel_now) {
          // Stop was pressed before the goal was accepted; honour it now.
          move_action_->async_cancel_goal(goal_handle);
        }
      });
  options.result_callback = guardCallback(
      [this](const rclcpp_action::ClientGoalHandle<MoveTable>::WrappedResult&
                 result) {
        {
          std::scoped_lock lock(handles_mutex_);
          move_goal_handle_.reset();
          move_in_flight_ = false;
          move_cancel_pending_ = false;
        }
        const bool succeeded =
            (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        const bool success = succeeded && result.result->success;
        const QString message =
            succeeded ? QString::fromStdString(result.result->message)
                      : QString("Goal did not succeed");
        QMetaObject::invokeMethod(
            this,
            [this, success, message]() {
              if (ct_widget_ != nullptr) {
                ct_widget_->setMoveResult(success, message);
              }
            },
            Qt::QueuedConnection);
      });

  {
    std::scoped_lock lock(handles_mutex_);
    move_in_flight_ = true;
    move_cancel_pending_ = false;
  }
  move_action_->async_send_goal(goal, options);
}

void HospitalControlPanel::sendScanGoal(uint8_t command) {
  if (!scan_action_->action_server_is_ready()) {
    if (ct_widget_ != nullptr) {
      ct_widget_->setScanResult(false, "CT controller not available");
    }
    return;
  }

  RunScan::Goal goal;
  goal.command = command;

  rclcpp_action::Client<RunScan>::SendGoalOptions options;
  options.feedback_callback = guardCallback(
      [this](rclcpp_action::ClientGoalHandle<RunScan>::SharedPtr,
             const std::shared_ptr<const RunScan::Feedback> feedback) {
        updateCtTableCache(feedback->table_telemetry);
        updateCtScanCache(feedback->telemetry);
      });
  options.goal_response_callback = guardCallback(
      [this](rclcpp_action::ClientGoalHandle<RunScan>::SharedPtr goal_handle) {
        if (goal_handle) {
          return;
        }
        QMetaObject::invokeMethod(
            this,
            [this]() {
              if (ct_widget_ != nullptr) {
                ct_widget_->setScanResult(false,
                                          "Scan goal rejected by controller");
              }
            },
            Qt::QueuedConnection);
      });
  options.result_callback = guardCallback(
      [this](const rclcpp_action::ClientGoalHandle<RunScan>::WrappedResult&
                 result) {
        const bool succeeded =
            (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        const bool success = succeeded && result.result->success;
        const QString message =
            succeeded ? QString::fromStdString(result.result->message)
                      : QString("Goal did not succeed");
        QMetaObject::invokeMethod(
            this,
            [this, success, message]() {
              if (ct_widget_ != nullptr) {
                ct_widget_->setScanResult(success, message);
              }
            },
            Qt::QueuedConnection);
      });

  scan_action_->async_send_goal(goal, options);
}

void HospitalControlPanel::sendExecuteTaskGoal(
    const std::string& robot_id, uint8_t value,
    const std::string& target_location, const std::string& task_id) {
  const auto client_it = ward_actions_.find(robot_id);
  if (client_it == ward_actions_.end() || !client_it->second) {
    if (ward_widget_ != nullptr) {
      ward_widget_->setTaskResult(
          QString::fromStdString(task_id), false,
          "Unknown robot '" + QString::fromStdString(robot_id) + "'");
    }
    return;
  }
  auto& ward_action = client_it->second;

  if (!ward_action->action_server_is_ready()) {
    if (ward_widget_ != nullptr) {
      ward_widget_->setTaskResult(
          QString::fromStdString(task_id), false,
          "Robot '" + QString::fromStdString(robot_id) + "' not available");
    }
    return;
  }

  ExecuteTask::Goal goal;
  goal.value = value;
  goal.target_location = target_location;
  goal.task_id = task_id;

  rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
  options.feedback_callback = guardCallback(
      [this, task_id](
          rclcpp_action::ClientGoalHandle<ExecuteTask>::SharedPtr,
          const std::shared_ptr<const ExecuteTask::Feedback> feedback) {
        const uint8_t phase = feedback->current_phase;
        const float progress = feedback->progress;
        QMetaObject::invokeMethod(
            this,
            [this, task_id, phase, progress]() {
              if (ward_widget_ != nullptr) {
                ward_widget_->updateTaskPhase(QString::fromStdString(task_id),
                                              phase, progress);
              }
            },
            Qt::QueuedConnection);
      });
  options.goal_response_callback = guardCallback(
      [this, task_id, robot_id](
          rclcpp_action::ClientGoalHandle<ExecuteTask>::SharedPtr goal_handle) {
        if (goal_handle) {
          // Store the handle so an Abort can cancel it. If the operator already
          // pressed Abort before acceptance arrived, cancel it right away.
          bool cancel_now = false;
          {
            std::scoped_lock lock(handles_mutex_);
            ward_goal_handles_[task_id] = goal_handle;
            ward_in_flight_.erase(task_id);
            cancel_now = ward_cancel_pending_.erase(task_id) > 0;
          }
          if (cancel_now) {
            const auto it = ward_actions_.find(robot_id);
            if (it != ward_actions_.end() && it->second) {
              it->second->async_cancel_goal(goal_handle);
            }
          }
          return;
        }
        {
          std::scoped_lock lock(handles_mutex_);
          ward_in_flight_.erase(task_id);
          ward_cancel_pending_.erase(task_id);
        }
        QMetaObject::invokeMethod(
            this,
            [this, task_id]() {
              if (ward_widget_ != nullptr) {
                ward_widget_->setTaskResult(QString::fromStdString(task_id),
                                            false,
                                            "Task rejected by controller");
              }
            },
            Qt::QueuedConnection);
      });
  options.result_callback = guardCallback(
      [this, task_id](
          const rclcpp_action::ClientGoalHandle<ExecuteTask>::WrappedResult&
              result) {
        {
          // The goal reached a terminal state; drop its handle.
          std::scoped_lock lock(handles_mutex_);
          ward_goal_handles_.erase(task_id);
          ward_in_flight_.erase(task_id);
          ward_cancel_pending_.erase(task_id);
        }
        const bool succeeded =
            (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        const bool success = succeeded && result.result->success;
        const QString message =
            succeeded ? QString::fromStdString(result.result->message)
                      : QString("Goal did not succeed");
        QMetaObject::invokeMethod(
            this,
            [this, task_id, success, message]() {
              if (ward_widget_ != nullptr) {
                ward_widget_->setTaskResult(QString::fromStdString(task_id),
                                            success, message);
              }
            },
            Qt::QueuedConnection);
      });

  {
    std::scoped_lock lock(handles_mutex_);
    ward_in_flight_.insert(task_id);
    ward_cancel_pending_.erase(task_id);
  }
  ward_action->async_send_goal(goal, options);
}

void HospitalControlPanel::cancelWardGoal(const std::string& robot_id,
                                          const std::string& task_id) {
  const auto client_it = ward_actions_.find(robot_id);
  if (client_it == ward_actions_.end() || !client_it->second) {
    return;
  }
  rclcpp_action::ClientGoalHandle<ExecuteTask>::SharedPtr goal_handle;
  {
    std::scoped_lock lock(handles_mutex_);
    auto handle_it = ward_goal_handles_.find(task_id);
    if (handle_it != ward_goal_handles_.end() && handle_it->second) {
      goal_handle = handle_it->second;
    } else if (ward_in_flight_.count(task_id) > 0) {
      // The goal has been sent but not accepted yet; mark it so the
      // goal_response callback cancels it the moment the handle arrives. A
      // task that was never sent or has already finished is not recorded.
      ward_cancel_pending_.insert(task_id);
    }
  }
  if (goal_handle) {
    client_it->second->async_cancel_goal(goal_handle);
  }
}

void HospitalControlPanel::publishTaskEvent(
    uint8_t kind, const std::string& robot_id, const std::string& task_label,
    const std::string& task_id, bool success, float elapsed_seconds) {
  if (!task_event_pub_) {
    return;
  }
  hospital_interfaces::msg::TaskEvent msg;
  msg.kind = kind;
  msg.robot_id = robot_id;
  msg.task_label = task_label;
  msg.task_id = task_id;
  msg.stamp = node_->now();
  msg.success = success;
  // A negative elapsed marks a task that never started; publish 0 instead.
  msg.elapsed_seconds = elapsed_seconds < 0.0F ? 0.0F : elapsed_seconds;
  task_event_pub_->publish(msg);
}

void HospitalControlPanel::setRecording(bool enable) {
  if (!set_recording_client_) {
    return;
  }
  if (!set_recording_client_->service_is_ready()) {
    postStatus("Activity logger not available", true);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;
  set_recording_client_->async_send_request(
      request,
      std::function<void(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture)>(
          guardCallback(
              [this, enable](
                  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                const auto response = future.get();
                const bool success = response->success;
                const QString detail =
                    QString::fromStdString(response->message);
                // The service reply arrives on the ROS executor thread; touch
                // the widgets only on the GUI thread.
                QMetaObject::invokeMethod(
                    this,
                    [this, enable, success, detail]() {
                      if (recording_widget_ != nullptr) {
                        // Reflect the requested state only when the logger
                        // accepted it.
                        recording_widget_->setRecording(enable && success,
                                                        detail);
                      }
                      postStatus(detail, !success);
                    },
                    Qt::QueuedConnection);
              })));
}

void HospitalControlPanel::cancelDoorGoal(const std::string& device_id) {
  rclcpp_action::ClientGoalHandle<OperateDoor>::SharedPtr handle;
  bool pending = false;
  {
    std::scoped_lock lock(handles_mutex_);
    auto handle_it = door_goal_handles_.find(device_id);
    if (handle_it != door_goal_handles_.end() && handle_it->second) {
      handle = handle_it->second;
    } else if (door_in_flight_.count(device_id) > 0) {
      // Goal sent but not yet accepted; record the intent so goal_response
      // cancels it the instant the handle arrives. Closes the race where Stop
      // is pressed before a (high-rate-delayed) goal acceptance is processed.
      door_cancel_pending_.insert(device_id);
      pending = true;
    }
  }
  if (handle) {
    // Cancelling freezes the door at its current position (controller cancel
    // path holds the physical percentage reached).
    door_action_->async_cancel_goal(handle);
    return;
  }
  if (pending) {
    return;
  }
  auto widget_it = door_widgets_.find(device_id);
  if (widget_it != door_widgets_.end()) {
    for (auto* widget : widget_it->second) {
      widget->setResult(false, "No door movement to stop");
    }
  }
}

void HospitalControlPanel::cancelMoveGoal() {
  rclcpp_action::ClientGoalHandle<MoveTable>::SharedPtr handle;
  bool pending = false;
  {
    std::scoped_lock lock(handles_mutex_);
    if (move_goal_handle_) {
      handle = move_goal_handle_;
    } else if (move_in_flight_) {
      move_cancel_pending_ = true;
      pending = true;
    }
  }
  if (handle) {
    // Cancelling freezes the lift at its current position; the controller
    // reports PAUSED so a later movement can resume from there.
    move_action_->async_cancel_goal(handle);
    return;
  }
  if (pending) {
    return;
  }
  bool preparing_lift = false;
  bool lift_moving = false;
  {
    std::scoped_lock lock(state_mutex_);
    preparing_lift =
        ct_procedure_phase_ ==
        hospital_interfaces::msg::CtScanProcedureState::PHASE_PREPARING_LIFT;
    lift_moving = ct_table_cache_.state ==
                      hospital_interfaces::msg::CtTableState::MOVING_UP ||
                  ct_table_cache_.state ==
                      hospital_interfaces::msg::CtTableState::MOVING_DOWN;
  }
  if (preparing_lift) {
    sendScanGoal(RunScan::Goal::COMMAND_STOP);
    return;
  }
  if (lift_moving) {
    move_action_->async_cancel_all_goals();
    return;
  }
  if (ct_widget_ != nullptr) {
    ct_widget_->setMoveResult(false, "No table movement to stop");
  }
}

// -- Laboratory --------------------------------------------------------------

void HospitalControlPanel::sendAnalyzeBloodGoal() {
  if (!lab_action_->action_server_is_ready()) {
    postStatus("Lab controller not available", true);
    return;
  }

  AnalyzeBlood::Goal goal;
  goal.command = "START_BLOODTEST";

  rclcpp_action::Client<AnalyzeBlood>::SendGoalOptions options;
  options.goal_response_callback = guardCallback(
      [this](rclcpp_action::ClientGoalHandle<AnalyzeBlood>::SharedPtr
                 goal_handle) {
        if (goal_handle) {
          return;
        }
        QMetaObject::invokeMethod(
            this,
            [this]() {
              postStatus("Blood analysis goal rejected by controller", true);
            },
            Qt::QueuedConnection);
      });
  options.result_callback = guardCallback(
      [this](const rclcpp_action::ClientGoalHandle<AnalyzeBlood>::WrappedResult&
                 result) {
        const bool succeeded =
            (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        const bool success = succeeded && result.result->success;
        const QString message =
            succeeded ? QString::fromStdString(result.result->message)
                      : QString("Blood analysis did not succeed");
        QMetaObject::invokeMethod(
            this, [this, success, message]() { postStatus(message, !success); },
            Qt::QueuedConnection);
      });

  lab_action_->async_send_goal(goal, options);
}

// -- Operating room -----------------------------------------------------------

void HospitalControlPanel::sendSetCleanliness(uint8_t target_state) {
  if (!set_cleanliness_client_->service_is_ready()) {
    postStatus("Operating room controller not available", true);
    return;
  }

  auto request = std::make_shared<SetCleanliness::Request>();
  request->target_state = target_state;
  set_cleanliness_client_->async_send_request(
      request,
      std::function<void(rclcpp::Client<SetCleanliness>::SharedFuture)>(
          guardCallback(
              [this](rclcpp::Client<SetCleanliness>::SharedFuture future) {
                const bool accepted = future.get()->accepted;
                QMetaObject::invokeMethod(
                    this,
                    [this, accepted]() {
                      postStatus(accepted ? "Operating room cleanliness updated"
                                          : "Cleanliness request rejected",
                                 !accepted);
                    },
                    Qt::QueuedConnection);
                // The controller publishes the new state on the latched state
                // topic, which refreshes the widget.
              })));
}

void HospitalControlPanel::setXrayActive(bool active) {
  if (!xray_param_client_->service_is_ready()) {
    postStatus("Operating room controller not available", true);
    return;
  }

  xray_param_client_->set_parameters(
      {rclcpp::Parameter("xray_active", active)},
      guardCallback(
          [this](std::shared_future<
                 std::vector<rcl_interfaces::msg::SetParametersResult>>
                     future) {
            const auto results = future.get();
            const bool ok = !results.empty() && results.front().successful;
            QMetaObject::invokeMethod(
                this,
                [this, ok]() {
                  postStatus(ok ? "Operating room X-ray updated"
                                : "X-ray update rejected",
                             !ok);
                },
                Qt::QueuedConnection);
            // An accepted change is published on the latched state topic, which
            // refreshes the toggle.
          }));
}

// -- State subscriptions (ROS executor thread; only cache telemetry) ----------

void HospitalControlPanel::onDoorStates(
    hospital_interfaces::msg::DoorStateArray::ConstSharedPtr msg) {
  std::scoped_lock lock(state_mutex_);
  // The array carries every door on each update; only re-render the ones that
  // actually changed.
  for (const auto& state : msg->doors) {
    auto [it, inserted] = door_state_cache_.try_emplace(state.device_id, state);
    if (inserted || !(it->second == state)) {
      it->second = state;
      door_state_dirty_.insert(state.device_id);
    }
  }
}

void HospitalControlPanel::onCtTableState(
    hospital_interfaces::msg::CtTableState::ConstSharedPtr msg) {
  updateCtTableCache(*msg);
}

void HospitalControlPanel::onCtScanState(
    hospital_interfaces::msg::CtScannerState::ConstSharedPtr msg) {
  updateCtScanCache(*msg);
}

void HospitalControlPanel::onLabState(
    hospital_interfaces::msg::LabState::ConstSharedPtr msg) {
  updateLabCache(*msg);
}

void HospitalControlPanel::onOperatingRoomState(
    hospital_interfaces::msg::OperatingRoomState::ConstSharedPtr msg) {
  updateOperatingRoomCache(*msg);
}

void HospitalControlPanel::onStopwatch(
    hospital_interfaces::msg::Stopwatch::ConstSharedPtr msg) {
  std::scoped_lock lock(state_mutex_);
  stopwatch_cache_[msg->device_id] = *msg;
  stopwatch_received_[msg->device_id] = std::chrono::steady_clock::now();
}

// -- Telemetry cache (ROS thread writes; GUI timer reads) ---------------------

void HospitalControlPanel::updateDoorCache(
    const hospital_interfaces::msg::DoorState& state) {
  std::scoped_lock lock(state_mutex_);
  door_state_cache_[state.device_id] = state;
  door_state_dirty_.insert(state.device_id);
}

void HospitalControlPanel::updateCtTableCache(
    const hospital_interfaces::msg::CtTableState& state) {
  std::scoped_lock lock(state_mutex_);
  ct_table_cache_ = state;
  ct_table_dirty_ = true;
}

void HospitalControlPanel::updateCtScanCache(
    const hospital_interfaces::msg::CtScannerState& state) {
  std::scoped_lock lock(state_mutex_);
  ct_scan_cache_ = state;
  ct_scan_dirty_ = true;
}

void HospitalControlPanel::updateLabCache(
    const hospital_interfaces::msg::LabState& state) {
  std::scoped_lock lock(state_mutex_);
  lab_state_cache_ = state;
  lab_state_dirty_ = true;
}

void HospitalControlPanel::updateOperatingRoomCache(
    const hospital_interfaces::msg::OperatingRoomState& state) {
  std::scoped_lock lock(state_mutex_);
  operating_room_cache_ = state;
  operating_room_dirty_ = true;
}

void HospitalControlPanel::refreshWidgets() {
  std::vector<hospital_interfaces::msg::DoorState> doors;
  hospital_interfaces::msg::CtTableState table_state;
  bool table_dirty = false;
  hospital_interfaces::msg::CtScannerState scan_state;
  bool scan_dirty = false;
  uint8_t procedure_phase = 0;
  bool procedure_dirty = false;
  std::vector<hospital_interfaces::msg::Stopwatch> stopwatches;
  std::vector<std::chrono::steady_clock::time_point> stopwatch_times;
  hospital_interfaces::msg::LabState lab_state;
  bool lab_dirty = false;
  hospital_interfaces::msg::OperatingRoomState operating_room_state;
  bool operating_room_dirty = false;
  {
    std::scoped_lock lock(state_mutex_);
    for (const auto& id : door_state_dirty_) {
      doors.push_back(door_state_cache_[id]);
    }
    door_state_dirty_.clear();
    if (ct_table_dirty_) {
      table_state = ct_table_cache_;
      table_dirty = true;
      ct_table_dirty_ = false;
    }
    if (ct_scan_dirty_) {
      scan_state = ct_scan_cache_;
      scan_dirty = true;
      ct_scan_dirty_ = false;
    }
    if (ct_procedure_dirty_) {
      procedure_phase = ct_procedure_phase_;
      procedure_dirty = true;
      ct_procedure_dirty_ = false;
    }
    if (lab_state_dirty_) {
      lab_state = lab_state_cache_;
      lab_dirty = true;
      lab_state_dirty_ = false;
    }
    if (operating_room_dirty_) {
      operating_room_state = operating_room_cache_;
      operating_room_dirty = true;
      operating_room_dirty_ = false;
    }
    // Stopwatches have no dirty set: re-render every tick so a running
    // stopwatch ticks up smoothly between controller messages.
    stopwatches.reserve(stopwatch_cache_.size());
    stopwatch_times.reserve(stopwatch_cache_.size());
    for (const auto& [id, sw] : stopwatch_cache_) {
      stopwatches.push_back(sw);
      const auto received = stopwatch_received_.find(id);
      stopwatch_times.push_back(received != stopwatch_received_.end()
                                    ? received->second
                                    : std::chrono::steady_clock::now());
    }
  }
  for (const auto& state : doors) {
    auto it = door_widgets_.find(state.device_id);
    if (it != door_widgets_.end()) {
      for (auto* widget : it->second) {
        widget->setState(state.state, state.percentage_open);
      }
    }
  }
  if (table_dirty && ct_widget_ != nullptr) {
    ct_widget_->setTableState(table_state.state, table_state.percentage_done);
  }
  if (scan_dirty && ct_widget_ != nullptr) {
    ct_widget_->setScanState(scan_state.state, scan_state.percentage_done,
                             scan_state.radiating);
  }
  if (procedure_dirty && ct_widget_ != nullptr) {
    ct_widget_->setProcedurePhase(procedure_phase);
  }
  if (lab_dirty && lab_widget_ != nullptr) {
    lab_widget_->setState(lab_state.state);
    lab_widget_->setProgress(lab_state.progress_percentage);
    lab_widget_->setResult(lab_state.result_valid, lab_state.ctnt,
                           lab_state.ctni, lab_state.ck_mb);
  }
  if (operating_room_dirty && operating_room_widget_ != nullptr) {
    operating_room_widget_->setState(operating_room_state.cleanliness,
                                     operating_room_state.xray_active);
  }
  const rclcpp::Time now = node_->now();
  const auto steady_now = std::chrono::steady_clock::now();
  // A running stopwatch whose controller has gone silent this long past its
  // nominal duration is treated as finished: its terminal message was lost or
  // the worker died, so we stop it instead of ticking forever.
  constexpr double kStopwatchStaleTimeout = 3.0;
  for (std::size_t i = 0; i < stopwatches.size(); ++i) {
    const auto& sw = stopwatches[i];
    // While running, tick the live elapsed time locally from the start stamp;
    // once finished, show the controller's authoritative final_seconds.
    bool running = sw.running;
    bool interrupted = sw.interrupted;
    float seconds = sw.final_seconds;
    if (running) {
      const rclcpp::Time start(sw.start_stamp, now.get_clock_type());
      const double elapsed = (now - start).seconds();
      seconds =
          elapsed > 0.0 ? static_cast<float>(elapsed) : sw.elapsed_seconds;
      // Clamp the live estimate to the nominal duration so a running timer can
      // never visibly overshoot (e.g. tick to 6.0 s then snap back to 5.0 s)
      // while it waits for the authoritative terminal message under load.
      if (sw.nominal_seconds > 0.0f && seconds > sw.nominal_seconds) {
        seconds = sw.nominal_seconds;
      }
      // Watchdog: if the controller has stopped publishing well past the
      // nominal duration, the terminal message never arrived. Freeze the timer
      // at the nominal value instead of letting it run forever.
      const double stale =
          std::chrono::duration<double>(steady_now - stopwatch_times[i])
              .count();
      if (sw.nominal_seconds > 0.0f && stale > kStopwatchStaleTimeout) {
        running = false;
        interrupted = true;
        seconds = sw.nominal_seconds;
      }
    }
    switch (sw.kind) {
      case hospital_interfaces::msg::Stopwatch::KIND_DOOR_OPEN: {
        auto it = door_widgets_.find(sw.device_id);
        if (it != door_widgets_.end()) {
          for (auto* widget : it->second) {
            widget->setStopwatch(seconds, running, interrupted);
          }
        }
        break;
      }
      case hospital_interfaces::msg::Stopwatch::KIND_CT_SCAN: {
        if (ct_widget_ != nullptr) {
          ct_widget_->setStopwatch(seconds, running, interrupted);
        }
        break;
      }
      case hospital_interfaces::msg::Stopwatch::KIND_LAB_ANALYSIS: {
        if (lab_widget_ != nullptr) {
          lab_widget_->setStopwatch(seconds, running, interrupted);
        }
        break;
      }
      default:
        break;
    }
  }

  // Tick the client-side per-task stopwatches in the Ward Management table.
  if (ward_widget_ != nullptr) {
    ward_widget_->tickTimers();
  }
}

}  // namespace hospital_rviz_panel

PLUGINLIB_EXPORT_CLASS(hospital_rviz_panel::HospitalControlPanel,
                       rviz_common::Panel)
