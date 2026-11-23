// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__WARD_MANAGEMENT_WIDGET_HPP_
#define HOSPITAL_RVIZ_PANEL__WARD_MANAGEMENT_WIDGET_HPP_

#include <QElapsedTimer>
#include <QGroupBox>
#include <QString>
#include <QStringList>

#include <cstdint>
#include <map>

class QComboBox;
class QPushButton;
class QTableWidget;

namespace hospital_rviz_panel {

/// Mission-dispatch controls for the hospital ward.
///
/// The operator selects a command (CLEAN_OR / COLLECT_LAB / ...) and a target
/// room, then submits it as an ``ExecuteTask`` goal. Each submitted task is
/// appended to a table with a live elapsed timer; the panel feeds phase and
/// result updates back through the setters below. Completed tasks are removed
/// from the table (their final duration is forwarded to the status bar).
///
/// The widget holds no ROS state: Submit only emits ``taskSubmitted`` with a
/// freshly generated ``task_id`` (a random UUID) and the panel owns the action
/// client.
class WardManagementWidget : public QGroupBox {
  Q_OBJECT

 public:
  explicit WardManagementWidget(QWidget* parent = nullptr);

  /// Populate the robot dropdown with the available robot ids. Each id maps to
  /// its own ExecuteTask action server (the panel owns the clients).
  void setRobots(const QStringList& robot_ids);

  /// Update the phase/progress cell of the task identified by `task_id`.
  void updateTaskPhase(const QString& task_id, uint8_t phase, float progress);

  /// Mark the task finished: remove its row and report the final duration.
  void setTaskResult(const QString& task_id, bool success,
                     const QString& message);

  /// Re-render the elapsed-time cell of every still-running task. Driven by the
  /// panel's shared refresh timer so the times tick up smoothly.
  void tickTimers();

 Q_SIGNALS:
  /// Emitted on Submit with the ExecuteTask goal value, the selected robot id,
  /// the target_location slug (e.g. "intervention_or") and a unique task_id.
  void taskSubmitted(uint8_t value, const QString& robot_id,
                     const QString& target_location, const QString& task_id);

  /// Emitted with a human-readable message for the panel's shared status bar.
  void statusPosted(const QString& message, bool error);

  /// Emitted when a task's stopwatch starts (the robot begins the task). The
  /// panel forwards this to the activity logger as a TaskEvent START.
  void taskStarted(const QString& robot_id, const QString& command_label,
                   const QString& task_id);

  /// Emitted when a task finishes, carrying the exact elapsed time shown in the
  /// table so the logged duration matches the display. `elapsed_seconds` is < 0
  /// when the task never started (no meaningful duration).
  void taskFinished(const QString& robot_id, const QString& command_label,
                    const QString& task_id, bool success,
                    float elapsed_seconds);

  /// Emitted when the operator presses a row's Abort button. The panel cancels
  /// the corresponding ExecuteTask goal, which stops a running task (the robot
  /// then teleports home) or drops it from the robot's queue if it had not yet
  /// started. `was_running` distinguishes the two cases for logging.
  void taskAbortRequested(const QString& robot_id, const QString& task_id,
                          bool was_running);

 private:
  // Locate the table row currently displaying `task_id`, or -1 if none.
  int rowForTask(const QString& task_id) const;

  // Handle a row's Abort press: cancel the goal via the panel. A queued task is
  // removed from the table immediately; a running task's row is frozen and its
  // Abort button removed once the cancelled result arrives.
  void abortTask(const QString& task_id);

  // Remove every finished (DONE/FAILED) row, leaving running/queued tasks whose
  // timers are still live untouched.
  void clearCompletedTasks();

  QComboBox* command_combo_;
  QComboBox* robot_combo_;
  QPushButton* submit_button_;
  QPushButton* clear_button_;
  QTableWidget* table_;

  // Per-task stopwatch, keyed by task_id. The timer is only started once the
  // robot begins the task (first active-phase feedback), not at submit time, so
  // queued tasks do not accumulate time while another mission runs.
  struct TaskTiming {
    QElapsedTimer timer;
    bool started{false};
  };
  std::map<QString, TaskTiming> timings_;
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__WARD_MANAGEMENT_WIDGET_HPP_
