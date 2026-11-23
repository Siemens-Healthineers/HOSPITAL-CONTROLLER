// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/ct_control_widget.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QSize>
#include <QVBoxLayout>

#include <cmath>
#include <map>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hospital_interfaces/msg/ct_scan_procedure_state.hpp"

namespace hospital_rviz_panel {

namespace {
// Mirrors hospital_interfaces/msg/CtTableState constants (vertical lift).
const std::map<int, QString> kTableStateNames = {
    {0, "BOTTOM"}, {1, "MOVING_UP"}, {2, "MOVING_DOWN"},
    {3, "TOP"},    {4, "PAUSED"},
};

// Mirrors hospital_interfaces/msg/CtScannerState constants (gantry table-top).
const std::map<int, QString> kScanStateNames = {
    {0, "IDLE"},
    {1, "MOVING IN"},
    {2, "MOVING OUT"},
    {3, "PAUSED"},
};

// Mirrors hospital_interfaces/action/MoveTable command constants.
constexpr int kCommandUp = 0;
constexpr int kCommandDown = 1;

// Mirrors hospital_interfaces/action/RunScan command constants.
constexpr int kCommandStart = 0;
constexpr int kCommandStop = 1;
constexpr int kCommandRetract = 2;

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

CtControlWidget::CtControlWidget(QWidget* parent)
    : QGroupBox("CT Scanner", parent) {
  procedure_status_ = new QLabel("Procedure: --", this);
  procedure_status_->setWordWrap(true);
  // Table (vertical lift) section.
  table_status_ = new QLabel("Table: --", this);
  table_progress_ = new QProgressBar(this);
  table_progress_->setObjectName("slimBar");
  table_progress_->setRange(0, 100);
  table_progress_->setValue(0);
  table_progress_->setTextVisible(false);

  // Table lift uses the up/down arrow pictograms; the stop sign freezes it.
  // Fixed square icon buttons so they never stretch with the panel.
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
  auto* up_button = make_icon_button("arrow_up.png", "Move table up", false);
  auto* down_button =
      make_icon_button("arrow_down.png", "Move table down", false);
  connect(up_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("CT Scanner: moving table up\xE2\x80\xA6"),
                        false);
    Q_EMIT moveRequested(kCommandUp);
  });
  connect(down_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("CT Scanner: moving table down\xE2\x80\xA6"),
                        false);
    Q_EMIT moveRequested(kCommandDown);
  });

  auto* stop_table_button =
      make_icon_button("pictogramm_stop.png", "Stop vertical movement", true);
  connect(stop_table_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("CT Scanner: stopping table\xE2\x80\xA6"),
                        false);
    Q_EMIT stopMoveRequested();
  });

  auto* table_buttons = new QHBoxLayout();
  table_buttons->setSpacing(6);
  table_buttons->addStretch(1);
  table_buttons->addWidget(up_button);
  table_buttons->addStretch(1);
  table_buttons->addWidget(down_button);
  table_buttons->addStretch(1);
  table_buttons->addWidget(stop_table_button);
  table_buttons->addStretch(1);

  // Scan (gantry table-top) section.
  scan_status_ = new QLabel("Scan: --", this);
  radiation_label_ = new QLabel("RADIATION: off", this);
  radiation_label_->setObjectName("radiationPill");
  radiation_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  scan_progress_ = new QProgressBar(this);
  scan_progress_->setObjectName("slimBar");
  scan_progress_->setRange(0, 100);
  scan_progress_->setValue(0);
  scan_progress_->setTextVisible(false);

  stopwatch_label_ = new QLabel("--", this);
  stopwatch_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  auto* scan_button = new QPushButton("Start Scan", this);
  auto* stop_scan_button =
      make_icon_button("pictogramm_stop.png", "Stop scan", true);
  auto* recover_button = new QPushButton("Recover", this);
  connect(scan_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("CT Scanner: starting scan\xE2\x80\xA6"),
                        false);
    Q_EMIT scanRequested(kCommandStart);
  });
  connect(stop_scan_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("CT Scanner: stopping scan\xE2\x80\xA6"),
                        false);
    Q_EMIT scanRequested(kCommandStop);
  });
  connect(recover_button, &QPushButton::clicked, this, [this]() {
    Q_EMIT statusPosted(QString("CT Scanner: retracting table-top\xE2\x80\xA6"),
                        false);
    Q_EMIT scanRequested(kCommandRetract);
  });

  auto* scan_buttons = new QHBoxLayout();
  scan_buttons->addWidget(scan_button);
  scan_buttons->addWidget(stop_scan_button);
  scan_buttons->addWidget(recover_button);

  // Compact two-section stack: a status line, its slim bar, then the buttons.
  auto* table_header = new QHBoxLayout();
  table_header->addWidget(table_status_, 1);

  auto* scan_header = new QHBoxLayout();
  scan_header->addWidget(scan_status_, 1);
  scan_header->addWidget(radiation_label_);
  scan_header->addWidget(stopwatch_label_);

  auto* layout = new QVBoxLayout(this);
  layout->setSpacing(4);
  layout->addWidget(procedure_status_);
  layout->addLayout(table_header);
  layout->addWidget(table_progress_);
  layout->addLayout(table_buttons);
  layout->addLayout(scan_header);
  layout->addWidget(scan_progress_);
  layout->addLayout(scan_buttons);

  setRadiation(false);
}

void CtControlWidget::setTableState(int state, float percentage_done) {
  auto it = kTableStateNames.find(state);
  const QString name =
      (it != kTableStateNames.end()) ? it->second : QString::number(state);
  const int pct = static_cast<int>(std::lround(percentage_done));
  table_status_->setText(QString("Table: %1 \xC2\xB7 %2%").arg(name).arg(pct));
  table_progress_->setValue(pct);
}

void CtControlWidget::setScanState(int state, float percentage_done,
                                   bool radiating) {
  auto it = kScanStateNames.find(state);
  const QString name =
      (it != kScanStateNames.end()) ? it->second : QString::number(state);
  const int pct = static_cast<int>(std::lround(percentage_done));
  scan_status_->setText(QString("Scan: %1 \xC2\xB7 %2%").arg(name).arg(pct));
  scan_progress_->setValue(pct);
  setRadiation(radiating);
}

void CtControlWidget::setMoveResult(bool success, const QString& message) {
  const QString prefix = success ? "OK" : "FAILED";
  Q_EMIT statusPosted(
      QString("CT Scanner \xE2\x80\x94 Table %1: %2").arg(prefix, message),
      !success);
}

void CtControlWidget::setProcedurePhase(int phase) {
  using ProcedureState = hospital_interfaces::msg::CtScanProcedureState;
  static const std::map<int, QString> names = {
      {ProcedureState::PHASE_INACTIVE, "Inactive"},
      {ProcedureState::PHASE_PREPARING_LIFT, "Preparing lift"},
      {ProcedureState::PHASE_SWEEP_IN, "Sweeping in"},
      {ProcedureState::PHASE_SWEEP_OUT, "Sweeping out"},
      {ProcedureState::PHASE_RETRACTING, "Retracting"},
      {ProcedureState::PHASE_COMPLETED, "Completed"},
      {ProcedureState::PHASE_INTERRUPTED, "Interrupted"},
  };
  const auto found = names.find(phase);
  const QString name =
      found != names.end() ? found->second : QString::number(phase);
  procedure_status_->setText(QString("Procedure: %1").arg(name));
}

void CtControlWidget::setScanResult(bool success, const QString& message) {
  const QString prefix = success ? "OK" : "FAILED";
  Q_EMIT statusPosted(
      QString("CT Scanner \xE2\x80\x94 Scan %1: %2").arg(prefix, message),
      !success);
}

void CtControlWidget::setStopwatch(float seconds, bool running,
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

void CtControlWidget::setRadiation(bool on) {
  if (on) {
    radiation_label_->setText("\xE2\x98\xA2 RADIATION");
    radiation_label_->setStyleSheet(
        "color: white; background-color: #c0392b; border-radius: 3px; "
        "padding: 1px 6px; font-weight: bold;");
  } else {
    radiation_label_->setText("\xE2\x98\xA2 off");
    radiation_label_->setStyleSheet("color: #9a9a9a; padding: 1px 6px;");
  }
}

}  // namespace hospital_rviz_panel
