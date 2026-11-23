// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/recording_control_widget.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace hospital_rviz_panel {

RecordingControlWidget::RecordingControlWidget(QWidget* parent)
    : QGroupBox("Activity recording", parent) {
  start_button_ = new QPushButton("Start Recording", this);
  stop_button_ = new QPushButton("Stop Recording", this);
  status_label_ = new QLabel("Idle", this);
  status_label_->setWordWrap(true);

  auto* buttons = new QHBoxLayout();
  buttons->addWidget(start_button_);
  buttons->addWidget(stop_button_);

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(buttons);
  layout->addWidget(status_label_);

  connect(start_button_, &QPushButton::clicked, this,
          [this]() { Q_EMIT startRecordingRequested(); });
  connect(stop_button_, &QPushButton::clicked, this,
          [this]() { Q_EMIT stopRecordingRequested(); });

  setRecording(false);
}

void RecordingControlWidget::setRecording(bool recording,
                                          const QString& detail) {
  start_button_->setEnabled(!recording);
  stop_button_->setEnabled(recording);
  QString text = recording ? QString("Recording") : QString("Idle");
  if (!detail.isEmpty()) {
    text += " - " + detail;
  }
  status_label_->setText(text);
}

}  // namespace hospital_rviz_panel
