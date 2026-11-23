// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/lab_control_widget.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "hospital_interfaces/msg/lab_state.hpp"

namespace hospital_rviz_panel {

namespace {
// Upper reference limits for the cardiac markers; a value above the limit is
// "out of norm" and indicative of cardiac injury (mirrors the lab controller).
constexpr float kCtntUpperLimit = 14.0f;  // cTnT in ng/L
constexpr float kCtniUpperLimit = 20.0f;  // cTnI in ng/L
constexpr float kCkMbUpperLimit = 5.0f;   // CK-MB in ng/mL

// Format one marker, highlighting the value in red when it exceeds its limit.
QString formatMarker(const QString& name, float value, float limit,
                     const QString& unit) {
  const QString text =
      QString("%1 %2 %3").arg(name, QString::number(value, 'f', 1), unit);
  if (value > limit) {
    return QString("<span style=\"color:#d33;font-weight:bold;\">%1</span>")
        .arg(text);
  }
  return text;
}
}  // namespace

LabControlWidget::LabControlWidget(QWidget* parent)
    : QGroupBox("Laboratory: blood analysis", parent) {
  status_label_ = new QLabel("State: --", this);
  stopwatch_label_ = new QLabel("Time: --", this);
  stopwatch_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  result_label_ = new QLabel("Result: --", this);
  result_label_->setTextFormat(Qt::RichText);
  result_label_->setWordWrap(true);

  // Top row: state on the left, stopwatch on the right; result line below.
  auto* top_row = new QHBoxLayout();
  top_row->addWidget(status_label_, 1);
  top_row->addWidget(stopwatch_label_);

  // Slim, text-free bar: the fill communicates the analysis completion so the
  // section stays low. The exact percentage is available via tooltip.
  progress_ = new QProgressBar(this);
  progress_->setObjectName("slimBar");
  progress_->setRange(0, 100);
  progress_->setValue(0);
  progress_->setTextVisible(false);

  start_button_ = new QPushButton("Start Blood Analysis", this);
  start_button_->setEnabled(false);
  start_button_->setToolTip("Send a START_BLOODTEST goal to the lab analyser.");
  connect(start_button_, &QPushButton::clicked, this,
          [this]() { Q_EMIT startAnalysisRequested(); });

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(top_row);
  layout->addWidget(progress_);
  layout->addWidget(result_label_);
  layout->addWidget(start_button_);
}

void LabControlWidget::setState(uint8_t state) {
  using LabState = hospital_interfaces::msg::LabState;
  switch (state) {
    case LabState::READY:
      status_label_->setText("State: READY");
      break;
    case LabState::ANALYSING:
      status_label_->setText("State: ANALYSING");
      break;
    case LabState::DONE:
      status_label_->setText("State: DONE");
      break;
    default:
      status_label_->setText("State: --");
      break;
  }
  start_button_->setEnabled(state == LabState::READY);
}

void LabControlWidget::setStopwatch(float seconds, bool running,
                                    bool interrupted) {
  const QString time = QString::number(seconds, 'f', 1) + " s";
  if (running) {
    stopwatch_label_->setText(QString("Time: %1").arg(time));
  } else if (interrupted) {
    stopwatch_label_->setText(QString("Time: %1 (interrupted)").arg(time));
  } else {
    stopwatch_label_->setText(QString("Time: %1 (done)").arg(time));
  }
}

void LabControlWidget::setProgress(float percentage) {
  const int pct =
      static_cast<int>(std::lround(std::clamp(percentage, 0.0f, 100.0f)));
  progress_->setValue(pct);
  progress_->setToolTip(QString("%1% complete").arg(pct));
}

void LabControlWidget::setResult(bool valid, float ctnt, float ctni,
                                 float ck_mb) {
  if (!valid) {
    result_label_->setText("Result: --");
    return;
  }

  const bool abnormal = ctnt > kCtntUpperLimit || ctni > kCtniUpperLimit ||
                        ck_mb > kCkMbUpperLimit;
  const QString verdict =
      abnormal
          ? QString(
                "<span style=\"color:#d33;font-weight:bold;\">ABNORMAL</span>")
          : QString("<span style=\"color:#2a8000;\">normal</span>");

  result_label_->setText(
      QString("Result: %1 &middot; %2 &middot; %3 &mdash; %4")
          .arg(formatMarker("cTnT", ctnt, kCtntUpperLimit, "ng/L"),
               formatMarker("cTnI", ctni, kCtniUpperLimit, "ng/L"),
               formatMarker("CK-MB", ck_mb, kCkMbUpperLimit, "ng/mL"),
               verdict));
}

}  // namespace hospital_rviz_panel
