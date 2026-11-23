// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__CT_CONTROL_WIDGET_HPP_
#define HOSPITAL_RVIZ_PANEL__CT_CONTROL_WIDGET_HPP_

#include <QGroupBox>
#include <QString>

class QLabel;
class QProgressBar;

namespace hospital_rviz_panel {

/// Table movement + scan controls and live indicators for one CT scanner.
///
/// The widget holds no ROS state: it emits
/// ``moveRequested`` / ``scanRequested`` with the relevant action command
/// constant and exposes setters the panel calls from ROS callbacks.
class CtControlWidget : public QGroupBox {
  Q_OBJECT

 public:
  explicit CtControlWidget(QWidget* parent = nullptr);

  /// Update the vertical-lift label and progress bar (CtTableState).
  void setTableState(int state, float percentage_done);

  /// Update the table-top scan label, progress bar and radiation indicator.
  void setScanState(int state, float percentage_done, bool radiating);

  void setProcedurePhase(int phase);

  /// Update the scan stopwatch readout. `seconds` is the elapsed time while
  /// `running`, or the final duration once the scan has finished.
  void setStopwatch(float seconds, bool running, bool interrupted);

  /// Show the outcome of the most recent MoveTable goal.
  void setMoveResult(bool success, const QString& message);

  /// Show the outcome of the most recent RunScan goal.
  void setScanResult(bool success, const QString& message);

 Q_SIGNALS:
  /// Emitted with a MoveTable command constant (0=UP, 1=DOWN).
  void moveRequested(int command);

  /// Emitted for vertical Stop; the panel stops standalone lift movement or
  /// RunScan preparation, leaving the lift at its current position.
  void stopMoveRequested();

  /// Emitted with a RunScan command constant (0=START, 1=STOP, 2=RETRACT).
  void scanRequested(int command);

  /// Emitted with a human-readable message for the panel's shared status bar.
  /// `error` selects the warning styling.
  void statusPosted(const QString& message, bool error);

 private:
  void setRadiation(bool on);

  QLabel* table_status_;
  QLabel* procedure_status_;
  QProgressBar* table_progress_;
  QLabel* scan_status_;
  QLabel* radiation_label_;
  QProgressBar* scan_progress_;
  QLabel* stopwatch_label_;
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__CT_CONTROL_WIDGET_HPP_
