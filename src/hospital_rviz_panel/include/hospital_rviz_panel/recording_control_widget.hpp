// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__RECORDING_CONTROL_WIDGET_HPP_
#define HOSPITAL_RVIZ_PANEL__RECORDING_CONTROL_WIDGET_HPP_

#include <QGroupBox>
#include <QString>

class QLabel;
class QPushButton;

namespace hospital_rviz_panel {

/// Start/Stop controls for the room-dwell + task activity logger.
///
/// The widget holds no ROS state: pressing a button emits a request signal and
/// the panel calls the logger's ``std_srvs/SetBool`` service. The panel feeds
/// the outcome back through ``setRecording`` so the buttons and status label
/// reflect the logger's actual state.
class RecordingControlWidget : public QGroupBox {
  Q_OBJECT

 public:
  explicit RecordingControlWidget(QWidget* parent = nullptr);

  /// Reflect the logger state: enables/disables the buttons and updates the
  /// status label. `detail` is an optional message (e.g. the log file path).
  void setRecording(bool recording, const QString& detail = QString());

 Q_SIGNALS:
  /// Emitted when the user presses Start Recording.
  void startRecordingRequested();

  /// Emitted when the user presses Stop Recording.
  void stopRecordingRequested();

 private:
  QPushButton* start_button_{nullptr};
  QPushButton* stop_button_{nullptr};
  QLabel* status_label_{nullptr};
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__RECORDING_CONTROL_WIDGET_HPP_
