// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__DOOR_CONTROL_WIDGET_HPP_
#define HOSPITAL_RVIZ_PANEL__DOOR_CONTROL_WIDGET_HPP_

#include <QString>
#include <QWidget>

class QLabel;
class QProgressBar;

namespace hospital_rviz_panel {

/// Open/Close controls plus a live status indicator for a single door.
///
/// Rendered as a single compact row so a long list of doors stays readable
/// without scrolling. The widget holds no ROS state: it emits
/// ``commandRequested`` when a button is pressed and exposes setters that the
/// panel calls from ROS callbacks. Operation outcomes are forwarded through
/// ``statusPosted`` to the panel's shared status bar instead of a per-row
/// message label.
class DoorControlWidget : public QWidget {
  Q_OBJECT

 public:
  /// `device_id` is the controller-facing identifier used for goals and state
  /// routing; `display_name` is the (possibly shortened) label shown in the UI.
  explicit DoorControlWidget(const QString& device_id,
                             const QString& display_name,
                             QWidget* parent = nullptr);

  /// Update the live state label and the open-percentage progress bar.
  void setState(int state, float percentage_open);

  /// Update the opening stopwatch readout. `seconds` is the elapsed time while
  /// `running`, or the final duration once the operation has finished.
  void setStopwatch(float seconds, bool running, bool interrupted);

  /// Show the outcome of the most recent OperateDoor goal.
  void setResult(bool success, const QString& message);

 Q_SIGNALS:
  /// Emitted with an OperateDoor command constant (0=OPEN, 1=CLOSE).
  void commandRequested(int command);

  /// Emitted when Stop is pressed; the panel cancels the active goal so the
  /// door freezes at its current position.
  void stopRequested();

  /// Emitted with a human-readable message for the panel's shared status bar.
  /// `error` selects the warning styling.
  void statusPosted(const QString& message, bool error);

 private:
  QString device_id_;
  QLabel* status_label_;
  QProgressBar* progress_;
  QLabel* stopwatch_label_;
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__DOOR_CONTROL_WIDGET_HPP_
