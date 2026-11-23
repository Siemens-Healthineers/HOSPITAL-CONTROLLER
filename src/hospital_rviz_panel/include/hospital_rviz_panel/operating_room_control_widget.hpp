// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__OPERATING_ROOM_CONTROL_WIDGET_HPP_
#define HOSPITAL_RVIZ_PANEL__OPERATING_ROOM_CONTROL_WIDGET_HPP_

#include <QGroupBox>
#include <QString>

class QLabel;
class QPushButton;

namespace hospital_rviz_panel {

class ToggleSwitch;

/// Controls for the single simulated operating room.
///
/// The room is either CLEAN or DIRTY (set through the ``set_cleanliness``
/// service) and reports whether its X-ray is radiating (the ``xray_active``
/// parameter on the controller). Cleanliness and the X-ray are each driven by a
/// sliding switch; the controller only publishes its state when asked, so the
/// widget also offers an explicit "Get State" refresh. The widget holds no ROS
/// state: it emits request signals when a switch is flipped and exposes
/// ``setState`` for the panel to call from ROS callbacks.
class OperatingRoomControlWidget : public QGroupBox {
  Q_OBJECT

 public:
  explicit OperatingRoomControlWidget(QWidget* parent = nullptr);

  /// Render the current room state: ``cleanliness`` uses the OperatingRoomState
  /// constants (DIRTY=0, CLEAN=1) and ``xray_active`` reflects the controller's
  /// ``xray_active`` parameter. Updates both switches without re-emitting.
  void setState(int cleanliness, bool xray_active);

 Q_SIGNALS:
  /// Emitted with a SetCleanliness target_state constant (0=DIRTY, 1=CLEAN).
  void setCleanlinessRequested(int target_state);

  /// Emitted when Get State is pressed; the panel calls the get_state service.
  // void getStateRequested();

  /// Emitted when the X-ray toggle is flipped; the panel sets the controller's
  /// ``xray_active`` parameter to ``active``.
  void xrayToggleRequested(bool active);

  /// Emitted with a human-readable message for the panel's shared status bar.
  /// ``error`` selects the warning styling.
  void statusPosted(const QString& message, bool error);

 private:
  void setRadiation(bool radiating);

  QLabel* cleanliness_label_;
  QLabel* xray_label_;
  ToggleSwitch* cleanliness_switch_;
  ToggleSwitch* xray_switch_;
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__OPERATING_ROOM_CONTROL_WIDGET_HPP_
