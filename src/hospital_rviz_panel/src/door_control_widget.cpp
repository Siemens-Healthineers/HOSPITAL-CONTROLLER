// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/door_control_widget.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QSize>

#include <cmath>
#include <map>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hospital_rviz_panel {

namespace {
// Mirrors hospital_interfaces/msg/DoorState constants.
const std::map<int, QString> kStateNames = {
    {0, "CLOSED"},  {1, "OPENING"},    {2, "OPEN"},
    {3, "CLOSING"}, {4, "OBSTRUCTED"}, {5, "STOPPED"},
};

// Mirrors hospital_interfaces/action/OperateDoor command constants.
constexpr int kCommandOpen = 0;
constexpr int kCommandClose = 1;

// Load a pictogram from the package's installed resource directory. Returns a
// null icon (button simply shows no image) if the package or file is missing.
QIcon loadIcon(const QString& file_name) {
  try {
    const std::string path =
        ament_index_cpp::get_package_share_directory("hospital_rviz_panel") +
        "/resource/" + file_name.toStdString();
    return QIcon(QString::fromStdString(path));
  } catch (const std::exception&) {
    return QIcon();
  }
}
}  // namespace

DoorControlWidget::DoorControlWidget(const QString& device_id,
                                     const QString& display_name,
                                     QWidget* parent)
    : QWidget(parent), device_id_(device_id) {
  setObjectName("deviceRow");
  // A plain QWidget only paints stylesheet borders/backgrounds when it has the
  // styled-background attribute, so enable it for the boxed outline to show.
  setAttribute(Qt::WA_StyledBackground, true);

  auto* name_label = new QLabel(display_name, this);
  name_label->setObjectName("deviceName");
  name_label->setMinimumWidth(90);

  status_label_ = new QLabel("--", this);
  status_label_->setObjectName("statePill");
  status_label_->setMinimumWidth(80);

  stopwatch_label_ = new QLabel("--", this);
  stopwatch_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  stopwatch_label_->setMinimumWidth(64);

  // Slim, text-free bar: the fill alone communicates the open fraction so the
  // row stays low. The exact percentage is available via tooltip.
  progress_ = new QProgressBar(this);
  progress_->setObjectName("slimBar");
  progress_->setRange(0, 100);
  progress_->setValue(0);
  progress_->setTextVisible(false);

  // Icon-only command buttons: the pictograms (opening / closing door, stop
  // sign) carry the meaning. Each is a fixed square sized to the pictogram so
  // they never stretch with the panel and stay aligned across the section grid.
  const QSize kButtonSize(48, 48);
  const QSize kIconSize(48, 48);
  auto make_icon_button = [this, kButtonSize, kIconSize](
                              const QString& icon_file, const QString& tip,
                              bool is_stop) {
    auto* button = new QPushButton(this);
    button->setObjectName(is_stop ? "stopButton" : "iconButton");
    button->setIcon(loadIcon(icon_file));
    button->setIconSize(kIconSize);
    button->setFixedSize(kButtonSize);
    button->setToolTip(tip);
    return button;
  };
  auto* open_button = make_icon_button("pictogramm_open.png", "Open", false);
  auto* close_button = make_icon_button("pictogramm_close.png", "Close", false);
  auto* stop_button = make_icon_button("pictogramm_stop.png", "Stop", true);
  connect(open_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("%1: opening\xE2\x80\xA6").arg(device_id_),
                        false);
    Q_EMIT commandRequested(kCommandOpen);
  });
  connect(close_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("%1: closing\xE2\x80\xA6").arg(device_id_),
                        false);
    Q_EMIT commandRequested(kCommandClose);
  });
  connect(stop_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("%1: stopping\xE2\x80\xA6").arg(device_id_),
                        false);
    Q_EMIT stopRequested();
  });

  // Row 0: identity + live state + stopwatch. Row 1: slim bar above the
  // command buttons, which keep their fixed size but are spread evenly across
  // the row so they sit symmetrically beneath the progress bar.
  auto* button_row = new QHBoxLayout();
  button_row->setSpacing(6);
  button_row->addStretch(1);
  button_row->addWidget(open_button);
  button_row->addStretch(1);
  button_row->addWidget(close_button);
  button_row->addStretch(1);
  button_row->addWidget(stop_button);
  button_row->addStretch(1);

  auto* layout = new QGridLayout(this);
  layout->setContentsMargins(2, 4, 2, 4);
  layout->setHorizontalSpacing(8);
  layout->setVerticalSpacing(3);
  layout->addWidget(name_label, 0, 0);
  layout->addWidget(status_label_, 0, 1);
  layout->addWidget(stopwatch_label_, 0, 2);
  layout->addWidget(progress_, 1, 0, 1, 3);
  layout->addLayout(button_row, 2, 0, 1, 3);
  layout->setColumnStretch(0, 1);
}

void DoorControlWidget::setState(int state, float percentage_open) {
  auto it = kStateNames.find(state);
  const QString name =
      (it != kStateNames.end()) ? it->second : QString::number(state);
  const int pct = static_cast<int>(std::lround(percentage_open));
  status_label_->setText(name);
  progress_->setValue(pct);
  progress_->setToolTip(QString("%1% open").arg(pct));
}

void DoorControlWidget::setResult(bool success, const QString& message) {
  const QString prefix = success ? "OK" : "FAILED";
  Q_EMIT statusPosted(
      QString("%1 \xE2\x80\x94 %2: %3").arg(device_id_, prefix, message),
      !success);
}

void DoorControlWidget::setStopwatch(float seconds, bool running,
                                     bool interrupted) {
  const QString time = QString::number(seconds, 'f', 1) + " s";
  if (running) {
    stopwatch_label_->setText(QString("%1").arg(time));
  } else if (interrupted) {
    stopwatch_label_->setText(QString("%1 (interrupted)").arg(time));
  } else {
    stopwatch_label_->setText(QString("%1 (done)").arg(time));
  }
}

}  // namespace hospital_rviz_panel
