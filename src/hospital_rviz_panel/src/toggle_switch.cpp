// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/toggle_switch.hpp"

#include <QPainter>

namespace hospital_rviz_panel {

namespace {
// Fixed geometry for the switch. The knob is inset from the track edges by
// kMargin on all sides.
constexpr int kWidth = 46;
constexpr int kHeight = 24;
constexpr int kMargin = 3;
}  // namespace

ToggleSwitch::ToggleSwitch(QWidget* parent)
    : QAbstractButton(parent), on_color_(0x00, 0x99, 0x99) {
  setCheckable(true);
  setCursor(Qt::PointingHandCursor);
}

void ToggleSwitch::setOnColor(const QColor& color) {
  on_color_ = color;
  update();
}

QSize ToggleSwitch::sizeHint() const { return QSize(kWidth, kHeight); }

void ToggleSwitch::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const int track_height = kHeight;
  const int radius = track_height / 2;
  const QRect track(0, 0, kWidth, track_height);

  // Track: accent colour when ON, muted grey when OFF.
  painter.setPen(Qt::NoPen);
  painter.setBrush(isChecked() ? on_color_ : QColor(0xc8, 0xc8, 0xc8));
  painter.drawRoundedRect(track, radius, radius);

  // Knob: a white circle that slides to the right when ON.
  const int knob_diameter = track_height - 2 * kMargin;
  const int knob_x = isChecked() ? (kWidth - kMargin - knob_diameter) : kMargin;
  painter.setBrush(QColor(0xff, 0xff, 0xff));
  painter.drawEllipse(knob_x, kMargin, knob_diameter, knob_diameter);
}

}  // namespace hospital_rviz_panel
