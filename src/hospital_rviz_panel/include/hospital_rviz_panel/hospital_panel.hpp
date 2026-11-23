// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__HOSPITAL_PANEL_HPP_
#define HOSPITAL_RVIZ_PANEL__HOSPITAL_PANEL_HPP_

#include <QSize>

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include "hospital_interfaces/action/analyze_blood.hpp"
#include "hospital_interfaces/action/execute_task.hpp"
#include "hospital_interfaces/action/move_table.hpp"
#include "hospital_interfaces/action/operate_door.hpp"
#include "hospital_interfaces/action/run_scan.hpp"
#include "hospital_interfaces/msg/ct_scan_procedure_state.hpp"
#include "hospital_interfaces/msg/ct_scanner_state.hpp"
#include "hospital_interfaces/msg/ct_table_state.hpp"
#include "hospital_interfaces/msg/door_state.hpp"
#include "hospital_interfaces/msg/door_state_array.hpp"
#include "hospital_interfaces/msg/lab_state.hpp"
#include "hospital_interfaces/msg/operating_room_state.hpp"
#include "hospital_interfaces/msg/stopwatch.hpp"
#include "hospital_interfaces/msg/task_event.hpp"
#include "hospital_interfaces/srv/set_cleanliness.hpp"
#include "std_srvs/srv/set_bool.hpp"

class QPushButton;
class QTimer;
class QLabel;

namespace hospital_rviz_panel {

class DoorControlWidget;
class CtControlWidget;
class LabControlWidget;
class WardManagementWidget;
class OperatingRoomControlWidget;
class RecordingControlWidget;

/// RViz dockable panel that commands the hospital simulation.
///
/// A pure ROS 2 client: it issues goals only through the existing action
/// servers (``OperateDoor`` / ``MoveTable`` / ``RunScan``) and renders state
/// only from the ``*State`` topics plus live action feedback. Several clients
/// can therefore run side by side against the same controllers. The panel
/// reuses the node RViz already spins, so no extra executor thread is needed.
class HospitalControlPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit HospitalControlPanel(QWidget *parent = nullptr);
  ~HospitalControlPanel() override;

  void onInitialize() override;

  // Preferred startup size. Widens the docked panel by ~40% over the natural
  // content width so all controls are comfortably visible without manual
  // resizing when RViz first lays out the panel.
  QSize sizeHint() const override;

 private:
  friend class CtLiftStopTest;
  friend class LabPanelResultTest;

  using AnalyzeBlood = hospital_interfaces::action::AnalyzeBlood;
  using OperateDoor = hospital_interfaces::action::OperateDoor;
  using MoveTable = hospital_interfaces::action::MoveTable;
  using RunScan = hospital_interfaces::action::RunScan;
  using ExecuteTask = hospital_interfaces::action::ExecuteTask;
  using SetCleanliness = hospital_interfaces::srv::SetCleanliness;

  struct CallbackLifetime {
    std::mutex mutex;
    bool active{true};
  };

  template <typename Callback>
  auto guardCallback(Callback callback) {
    const auto lifetime = callback_lifetime_;
    return [lifetime, callback = std::move(callback)](auto &&...args) mutable {
      std::lock_guard lock(lifetime->mutex);
      if (lifetime->active) {
        std::invoke(callback, std::forward<decltype(args)>(args)...);
      }
    };
  }

  void buildUi();

  // Commanding (called from the GUI thread on button presses).
  void sendDoorGoal(const std::string &device_id, uint8_t command);
  void sendMoveGoal(uint8_t command);
  void sendScanGoal(uint8_t command);
  // Dispatch a ward mission to a specific robot. Each robot owns its own queue,
  // so the panel only fires the goal at the selected robot's action server and
  // tracks per-task feedback/result for the timing table.
  void sendExecuteTaskGoal(const std::string &robot_id, uint8_t value,
                           const std::string &target_location,
                           const std::string &task_id);
  void sendAnalyzeBloodGoal();
  // Stop a movement at its current position by cancelling the active goal.
  void cancelDoorGoal(const std::string &device_id);
  void cancelMoveGoal();
  // Abort a ward mission by cancelling its ExecuteTask goal. A running task is
  // stopped (the robot teleports back to its home waypoint); a still-queued
  // task is dropped from the robot's queue before it starts.
  void cancelWardGoal(const std::string &robot_id, const std::string &task_id);

  // Operating room controls. Cleanliness goes through the set_cleanliness
  // service; the X-ray is the controller's xray_active parameter, changed with
  // a parameter client. Both changes come back on the latched state topic.
  void sendSetCleanliness(uint8_t target_state);
  void setXrayActive(bool active);

  // Activity logging: toggle the room-dwell/task recorder via its SetBool
  // service and forward ward-task lifecycle events (start/finish) so the logged
  // task duration matches the Mission dispatch table exactly.
  void setRecording(bool enable);
  void publishTaskEvent(uint8_t kind, const std::string &robot_id,
                        const std::string &task_label,
                        const std::string &task_id, bool success,
                        float elapsed_seconds);

  // State subscriptions (run on RViz's ROS executor thread). They only cache
  // the latest telemetry; refresh_timer_ applies it to the widgets on the GUI
  // thread at a fixed rate (see update*Cache / refreshWidgets).
  // The aggregated, latched door snapshot: a late-opened panel receives every
  // door's current state at once.
  void onDoorStates(
      hospital_interfaces::msg::DoorStateArray::ConstSharedPtr msg);
  void onCtTableState(
      hospital_interfaces::msg::CtTableState::ConstSharedPtr msg);
  void onCtScanState(
      hospital_interfaces::msg::CtScannerState::ConstSharedPtr msg);

  // Latest laboratory state, including the cardiac markers of the most recent
  // analysis. Cached on the ROS executor thread; rendered from
  // refreshWidgets().
  void onLabState(hospital_interfaces::msg::LabState::ConstSharedPtr msg);

  // Stopwatch readings (door opening / lab analysis / CT scan). Cached on the
  // ROS executor thread; refresh_timer_ renders them on the GUI thread and
  // ticks the live elapsed time from start_stamp while an operation runs.
  void onStopwatch(hospital_interfaces::msg::Stopwatch::ConstSharedPtr msg);

  // Latest operating room state. The controller latches it and republishes on
  // every change (cleanliness or X-ray), so a late-opened panel is current.
  void onOperatingRoomState(
      hospital_interfaces::msg::OperatingRoomState::ConstSharedPtr msg);

  // Cache the latest telemetry coming from action feedback and the state
  // topics (called on the ROS executor thread), then apply it to the widgets
  // from refreshWidgets() at a fixed UI rate instead of once per message. This
  // decouples the UI from the controller publish rate so a flood of high-rate
  // feedback can never back up the Qt event queue and delay result/"Stopping"
  // updates behind the backlog.
  void updateDoorCache(const hospital_interfaces::msg::DoorState &state);
  void updateCtTableCache(const hospital_interfaces::msg::CtTableState &state);
  void updateCtScanCache(const hospital_interfaces::msg::CtScannerState &state);
  void updateLabCache(const hospital_interfaces::msg::LabState &state);
  void updateOperatingRoomCache(
      const hospital_interfaces::msg::OperatingRoomState &state);
  void refreshWidgets();

  // Render the latest operation message on the shared bottom status bar.
  void postStatus(const QString &message, bool error);

  std::shared_ptr<CallbackLifetime> callback_lifetime_{
      std::make_shared<CallbackLifetime>()};
  rclcpp::Node::SharedPtr node_;

  // A door may connect two rooms and therefore appear in more than one location
  // section, so each device_id maps to every widget mirroring it. The single CT
  // scanner has one widget.
  std::map<std::string, std::vector<DoorControlWidget *>> door_widgets_;
  CtControlWidget *ct_widget_{nullptr};
  LabControlWidget *lab_widget_{nullptr};
  WardManagementWidget *ward_widget_{nullptr};
  OperatingRoomControlWidget *operating_room_widget_{nullptr};
  RecordingControlWidget *recording_widget_{nullptr};
  QLabel *status_bar_{nullptr};

  rclcpp::Subscription<hospital_interfaces::msg::DoorStateArray>::SharedPtr
      door_states_sub_;
  rclcpp::Subscription<hospital_interfaces::msg::CtTableState>::SharedPtr
      ct_table_sub_;
  rclcpp::Subscription<hospital_interfaces::msg::CtScannerState>::SharedPtr
      ct_scan_sub_;
  rclcpp::Subscription<hospital_interfaces::msg::CtScanProcedureState>::
      SharedPtr ct_procedure_sub_;
  rclcpp::Subscription<hospital_interfaces::msg::LabState>::SharedPtr
      lab_state_sub_;
  rclcpp::Subscription<hospital_interfaces::msg::Stopwatch>::SharedPtr
      stopwatch_sub_;
  rclcpp::Subscription<hospital_interfaces::msg::OperatingRoomState>::SharedPtr
      operating_room_sub_;

  rclcpp_action::Client<OperateDoor>::SharedPtr door_action_;
  rclcpp_action::Client<MoveTable>::SharedPtr move_action_;
  rclcpp_action::Client<RunScan>::SharedPtr scan_action_;
  // One ExecuteTask action client per robot, keyed by robot_id. Each targets
  // /hospital/<robot_id>/ward/execute_task so several robots run concurrently.
  std::map<std::string, rclcpp_action::Client<ExecuteTask>::SharedPtr>
      ward_actions_;
  rclcpp_action::Client<AnalyzeBlood>::SharedPtr lab_action_;
  rclcpp::Client<SetCleanliness>::SharedPtr set_cleanliness_client_;
  // Activity logger controls: SetBool recording toggle + TaskEvent publisher.
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_recording_client_;
  rclcpp::Publisher<hospital_interfaces::msg::TaskEvent>::SharedPtr
      task_event_pub_;
  // Parameter client targeting operating_room_controller_node; the X-ray is a
  // parameter (xray_active) with no dedicated service.
  rclcpp::AsyncParametersClient::SharedPtr xray_param_client_;

  // Active goal handles, kept so a Stop press can cancel the in-flight goal
  // and freeze the device at its current position. These maps plus the
  // in-flight / pending-cancel sets below are shared between the ROS executor
  // thread (action callbacks) and the GUI thread (Stop button), so every
  // access is serialised by handles_mutex_. They are deliberately NOT routed
  // through the Qt event queue: doing so let a Stop press outrun the
  // not-yet-stored handle at high publish rates.
  std::mutex handles_mutex_;
  std::map<std::string, rclcpp_action::ClientGoalHandle<OperateDoor>::SharedPtr>
      door_goal_handles_;
  rclcpp_action::ClientGoalHandle<MoveTable>::SharedPtr move_goal_handle_;
  // Active ward goal handles keyed by task_id so an Abort press can cancel the
  // in-flight ExecuteTask goal. ward_in_flight_ holds task_ids that were sent
  // but not yet answered by the server; ward_cancel_pending_ holds the subset
  // whose Abort arrived before the goal handle did (they are cancelled as soon
  // as the handle lands). An Abort for a task that was never sent or already
  // finished is ignored, so neither set outlives its goal. Guarded by
  // handles_mutex_.
  std::map<std::string, rclcpp_action::ClientGoalHandle<ExecuteTask>::SharedPtr>
      ward_goal_handles_;
  std::set<std::string> ward_in_flight_;
  std::set<std::string> ward_cancel_pending_;
  // Goals that have been sent but whose acceptance (goal handle) has not yet
  // arrived, and goals for which a Stop was requested before the handle was
  // available (cancel as soon as it is).
  std::set<std::string> door_in_flight_;
  std::set<std::string> door_cancel_pending_;
  bool move_in_flight_{false};
  bool move_cancel_pending_{false};

  // Latest telemetry per device, written by the ROS callbacks and read by
  // refreshWidgets() on the GUI thread; serialised by state_mutex_. The dirty
  // sets mark which devices changed since the last UI refresh.
  std::mutex state_mutex_;
  std::map<std::string, hospital_interfaces::msg::DoorState> door_state_cache_;
  std::set<std::string> door_state_dirty_;
  hospital_interfaces::msg::CtTableState ct_table_cache_;
  bool ct_table_dirty_{false};
  hospital_interfaces::msg::CtScannerState ct_scan_cache_;
  bool ct_scan_dirty_{false};
  uint8_t ct_procedure_phase_{0};
  bool ct_procedure_dirty_{false};
  // Latest laboratory state (single analyser); dirty flag marks a pending UI
  // refresh.
  hospital_interfaces::msg::LabState lab_state_cache_;
  bool lab_state_dirty_{false};
  // Latest operating room state; dirty flag marks a pending UI refresh.
  hospital_interfaces::msg::OperatingRoomState operating_room_cache_;
  bool operating_room_dirty_{false};
  // Latest stopwatch reading per device_id. Unlike the telemetry caches there
  // is no dirty set: running stopwatches are re-rendered every refresh tick so
  // the elapsed time ticks up smoothly between controller messages.
  std::map<std::string, hospital_interfaces::msg::Stopwatch> stopwatch_cache_;
  // Monotonic time each stopwatch reading was received, used to time out a
  // running stopwatch whose terminal (running=false) message never arrived.
  std::map<std::string, std::chrono::steady_clock::time_point>
      stopwatch_received_;
  QTimer *refresh_timer_{nullptr};
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__HOSPITAL_PANEL_HPP_
