// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/operating_room_control_widget.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "hospital_rviz_panel/toggle_switch.hpp"

namespace hospital_rviz_panel {

namespace {
// Mirrors hospital_interfaces/msg/OperatingRoomState constants.
constexpr int kCleanlinessDirty = 0;
constexpr int kCleanlinessClean = 1;
}  // namespace

OperatingRoomControlWidget::OperatingRoomControlWidget(QWidget* parent)
    : QGroupBox("Operating Room", parent) {
  // -- Cleanliness row: DIRTY (off) <-> CLEAN (on) switch and a state pill. --
  auto* cleanliness_caption = new QLabel("Cleanliness", this);

  cleanliness_switch_ = new ToggleSwitch(this);
  cleanliness_switch_->setToolTip(
      "Slide to mark the operating room CLEAN or DIRTY.");

  cleanliness_label_ = new QLabel("--", this);
  cleanliness_label_->setObjectName("statePill");
  cleanliness_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  // auto* get_state_button = new QPushButton("Get State", this);
  // get_state_button->setToolTip("Refresh the room state from the
  // controller.");

  // -- X-ray row: a red switch and a CT-style radiation pill. --
  auto* xray_caption = new QLabel("X-ray", this);

  xray_switch_ = new ToggleSwitch(this);
  xray_switch_->setOnColor(QColor(0xc0, 0x39, 0x2b));
  xray_switch_->setToolTip("Slide to start or stop the X-ray radiating.");

  xray_label_ = new QLabel(this);
  xray_label_->setObjectName("radiationPill");
  xray_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  connect(
      cleanliness_switch_, &ToggleSwitch::toggled, this, [this](bool checked) {
        Q_EMIT statusPosted(QString("Operating room: setting %1\xE2\x80\xA6")
                                .arg(checked ? "CLEAN" : "DIRTY"),
                            false);
        Q_EMIT setCleanlinessRequested(checked ? kCleanlinessClean
                                               : kCleanlinessDirty);
      });
  // connect(get_state_button, &QPushButton::clicked, this, [this]() {
  //   Q_EMIT statusPosted("Operating room: fetching state\xE2\x80\xA6", false);
  //   Q_EMIT getStateRequested();
  // });
  connect(xray_switch_, &ToggleSwitch::toggled, this, [this](bool checked) {
    Q_EMIT statusPosted(QString("Operating room: X-ray %1\xE2\x80\xA6")
                            .arg(checked ? "activating" : "deactivating"),
                        false);
    Q_EMIT xrayToggleRequested(checked);
  });

  // A grid keeps the two switches in the same column so they line up directly
  // above each other regardless of the differing caption widths.
  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(8);
  grid->setVerticalSpacing(6);
  grid->addWidget(cleanliness_caption, 0, 0);
  grid->addWidget(cleanliness_switch_, 0, 1);
  grid->addWidget(cleanliness_label_, 0, 2);
  grid->addWidget(xray_caption, 1, 0);
  grid->addWidget(xray_switch_, 1, 1);
  grid->addWidget(xray_label_, 1, 2);
  grid->setColumnStretch(2, 1);

  auto* layout = new QVBoxLayout(this);
  layout->setSpacing(6);
  layout->addLayout(grid);
  // layout->addWidget(get_state_button);

  setRadiation(false);
}

void OperatingRoomControlWidget::setState(int cleanliness, bool xray_active) {
  const bool clean = (cleanliness == kCleanlinessClean);

  // Reflect the confirmed state on both switches without re-triggering a
  // request.
  {
    const QSignalBlocker blocker(cleanliness_switch_);
    cleanliness_switch_->setChecked(clean);
  }
  cleanliness_label_->setText(clean ? "CLEAN" : "DIRTY");
  // CLEAN keeps the teal accent from the statePill style; DIRTY is
  // shown in brown to stand out.
  cleanliness_label_->setStyleSheet(
      clean ? QString() : "color: #8a5a2b; font-weight: 600;");

  {
    const QSignalBlocker blocker(xray_switch_);
    xray_switch_->setChecked(xray_active);
  }
  setRadiation(xray_active);
}

void OperatingRoomControlWidget::setRadiation(bool radiating) {
  if (radiating) {
    xray_label_->setText("\xE2\x98\xA2 RADIATING");
    xray_label_->setStyleSheet(
        "color: white; background-color: #c0392b; border-radius: 3px; "
        "padding: 1px 6px; font-weight: bold;");
  } else {
    xray_label_->setText("\xE2\x98\xA2 off");
    xray_label_->setStyleSheet("color: #9a9a9a; padding: 1px 6px;");
  }
}

}  // namespace hospital_rviz_panel
