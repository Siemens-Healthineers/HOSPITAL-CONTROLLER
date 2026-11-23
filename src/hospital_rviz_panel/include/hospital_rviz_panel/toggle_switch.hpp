// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__TOGGLE_SWITCH_HPP_
#define HOSPITAL_RVIZ_PANEL__TOGGLE_SWITCH_HPP_

#include <QAbstractButton>
#include <QColor>
#include <QSize>

namespace hospital_rviz_panel {

/// A small sliding on/off switch, painted directly so it reads as a physical
/// toggle rather than a push button. Being a checkable ``QAbstractButton`` it
/// exposes the usual ``toggled(bool)`` / ``setChecked`` API, so callers use it
/// exactly like a checkable button. The track colour when ON is configurable so
/// different switches can carry their own accent (e.g. red for an X-ray).
class ToggleSwitch : public QAbstractButton {
  Q_OBJECT

 public:
  explicit ToggleSwitch(QWidget* parent = nullptr);

  /// Track colour shown when the switch is ON (checked). Defaults to the
  /// panel's teal secondary accent.
  void setOnColor(const QColor& color);

  QSize sizeHint() const override;

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  QColor on_color_;
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__TOGGLE_SWITCH_HPP_
