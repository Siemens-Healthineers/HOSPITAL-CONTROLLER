// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__LAB_CONTROL_WIDGET_HPP_
#define HOSPITAL_RVIZ_PANEL__LAB_CONTROL_WIDGET_HPP_

#include <QGroupBox>
#include <QString>

#include <cstdint>

class QLabel;
class QProgressBar;
class QPushButton;

namespace hospital_rviz_panel {

/// Controls and stopwatch readout for the single blood-analysis laboratory.
///
/// Emits ``startAnalysisRequested`` when the user presses Start; the panel
/// wires this to the ``AnalyzeBlood`` action. Also renders the analysis
/// stopwatch published on ``/hospital/stopwatch``.
class LabControlWidget : public QGroupBox {
  Q_OBJECT

 public:
  explicit LabControlWidget(QWidget* parent = nullptr);

  void setState(uint8_t state);

  /// Update the analysis stopwatch readout. `seconds` is the elapsed time while
  /// `running`, or the final duration once the analysis has finished.
  void setStopwatch(float seconds, bool running, bool interrupted);

  /// Update the slim progress bar with the analysis completion percentage.
  void setProgress(float percentage);

  /// Render the current cardiac-marker result. When `valid`
  /// is false the result line shows a placeholder. Values above their clinical
  /// reference limit are highlighted and the overall verdict is shown.
  void setResult(bool valid, float ctnt, float ctni, float ck_mb);

 Q_SIGNALS:
  /// Emitted when the user presses "Start Blood Analysis".
  void startAnalysisRequested();

 private:
  QLabel* status_label_;
  QLabel* stopwatch_label_;
  QProgressBar* progress_;
  QLabel* result_label_;
  QPushButton* start_button_{nullptr};
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__LAB_CONTROL_WIDGET_HPP_
