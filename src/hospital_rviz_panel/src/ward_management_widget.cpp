// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/ward_management_widget.hpp"

#include <QColor>
#include <QComboBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QUuid>
#include <QVariant>

#include "hospital_interfaces/action/execute_task.hpp"

namespace hospital_rviz_panel {

namespace {
using ExecuteTask = hospital_interfaces::action::ExecuteTask;

// Table columns.
enum Column {
  kCommand = 0,
  kRobot = 1,
  kTime = 2,
  kAbort = 3,
  kColumnCount = 4
};

// task_id is stored on the command cell so a row survives removals/reorders.
constexpr int kTaskIdRole = Qt::UserRole;

// Format an elapsed millisecond count as a one-decimal seconds string.
QString formatSeconds(qint64 ms) {
  return QString::number(static_cast<double>(ms) / 1000.0, 'f', 1) + " s";
}

// Display color for a robot, keyed by its position in the roster. MUST match
// hospital_robot::colorForIndex (robot_server_node.cpp) so a robot's row here
// uses the same color as its 3D arrow + id label in RViz.
QColor colorForRobotIndex(int index) {
  static const QColor kPalette[] = {
      QColor(0, 199, 0),     // green   -> robot_1
      QColor(26, 125, 255),  // blue    -> robot_2
      QColor(255, 140, 0),   // orange  -> robot_3
      QColor(199, 0, 199),   // magenta
      QColor(0, 199, 199),   // cyan
      QColor(255, 0, 0),     // red
      QColor(150, 99, 0),    // brown
      QColor(120, 0, 255),   // purple
  };
  constexpr int kCount =
      static_cast<int>(sizeof(kPalette) / sizeof(kPalette[0]));
  if (index < 0) {
    index = 0;
  }
  return kPalette[index % kCount];
}

// Pick black or white text so the robot id stays readable on its color tint.
QColor textColorFor(const QColor& background) {
  const double luminance = 0.299 * background.red() +
                           0.587 * background.green() +
                           0.114 * background.blue();
  return luminance > 140.0 ? QColor(Qt::black) : QColor(Qt::white);
}
}  // namespace

WardManagementWidget::WardManagementWidget(QWidget* parent)
    : QGroupBox("Mission dispatch", parent) {
  command_combo_ = new QComboBox(this);
  // Item data carries the ExecuteTask goal value so the UI never duplicates the
  // numeric constants.
  command_combo_->addItem("1. Guide CT", ExecuteTask::Goal::GUIDE_CT);
  command_combo_->addItem("2. Clean Operating Room",
                          ExecuteTask::Goal::CLEAN_OR);
  command_combo_->addItem("3. Collect Laboratory",
                          ExecuteTask::Goal::COLLECT_LAB);
  command_combo_->addItem("4. Surgical Assistance",
                          ExecuteTask::Goal::SURGICAL_ASSIST);
  command_combo_->addItem("5. Heart Attack", ExecuteTask::Goal::HEART_ATTACK);

  submit_button_ = new QPushButton("Submit", this);
  clear_button_ = new QPushButton("Clear List", this);

  robot_combo_ = new QComboBox(this);

  table_ = new QTableWidget(0, kColumnCount, this);
  table_->setHorizontalHeaderLabels({"Command", "Robot", "Time", "Abort"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionMode(QAbstractItemView::NoSelection);

  auto* layout = new QGridLayout(this);
  layout->addWidget(new QLabel("Command", this), 0, 0);
  layout->addWidget(command_combo_, 0, 1);
  layout->addWidget(new QLabel("Robot", this), 1, 0);
  layout->addWidget(robot_combo_, 1, 1);
  layout->addWidget(submit_button_, 2, 0, 1, 2);
  layout->addWidget(table_, 3, 0, 1, 2);
  layout->addWidget(clear_button_, 4, 0, 1, 2);

  connect(submit_button_, &QPushButton::clicked, this, [this]() {
    const auto value =
        static_cast<uint8_t>(command_combo_->currentData().toUInt());
    const QString command_label = command_combo_->currentText();
    const QString robot_id = robot_combo_->currentText();
    if (robot_id.isEmpty()) {
      Q_EMIT statusPosted("No robot available to dispatch to", true);
      return;
    }
    // A random UUID keeps task ids unique across RViz restarts, so robot logs
    // and the activity logger's START/END pairing never see a reused id.
    const QString task_id = QUuid::createUuid().toString(QUuid::WithoutBraces);

    const int row = table_->rowCount();
    table_->insertRow(row);
    auto* command_item = new QTableWidgetItem(command_label);
    command_item->setData(kTaskIdRole, task_id);
    table_->setItem(row, kCommand, command_item);
    // Tint the robot cell with the robot's palette color so the operator can
    // match this row to the correspondingly colored arrow + label in RViz.
    auto* robot_item = new QTableWidgetItem(robot_id);
    const QColor robot_color = colorForRobotIndex(robot_combo_->currentIndex());
    robot_item->setBackground(robot_color);
    robot_item->setForeground(textColorFor(robot_color));
    table_->setItem(row, kRobot, robot_item);
    table_->setItem(row, kTime, new QTableWidgetItem("--"));
    // Per-row Abort button. The task_id (not the row index) is captured so the
    // press still targets the right task after other rows are
    // removed/reordered.
    auto* abort_button = new QPushButton("Abort", this);
    connect(abort_button, &QPushButton::clicked, this,
            [this, task_id]() { abortTask(task_id); });
    table_->setCellWidget(row, kAbort, abort_button);

    // The stopwatch is created but not started: it begins ticking only once the
    // robot reports it has started the task (see updateTaskPhase).
    timings_.emplace(task_id, TaskTiming{});

    Q_EMIT taskSubmitted(value, robot_id, QString(), task_id);
    Q_EMIT statusPosted(
        QString("Submitted %1 to %2").arg(command_label, robot_id), false);
  });

  connect(clear_button_, &QPushButton::clicked, this,
          &WardManagementWidget::clearCompletedTasks);
}

void WardManagementWidget::setRobots(const QStringList& robot_ids) {
  const QString current = robot_combo_->currentText();
  robot_combo_->clear();
  robot_combo_->addItems(robot_ids);
  const int index = robot_combo_->findText(current);
  if (index >= 0) {
    robot_combo_->setCurrentIndex(index);
  }
}

void WardManagementWidget::clearCompletedTasks() {
  // Walk bottom-up so row removals do not shift indices we still need to visit.
  // A row is "finished" once its stopwatch has been dropped from `timings_`
  // (see setTaskResult); running/queued tasks keep their live timer and row.
  int removed = 0;
  for (int row = table_->rowCount() - 1; row >= 0; --row) {
    const QTableWidgetItem* item = table_->item(row, kCommand);
    if (item == nullptr) {
      continue;
    }
    const QString task_id = item->data(kTaskIdRole).toString();
    if (timings_.find(task_id) == timings_.end()) {
      table_->removeRow(row);
      ++removed;
    }
  }
  Q_EMIT statusPosted(QString("Cleared %1 finished task%2")
                          .arg(removed)
                          .arg(removed == 1 ? "" : "s"),
                      false);
}

int WardManagementWidget::rowForTask(const QString& task_id) const {
  for (int row = 0; row < table_->rowCount(); ++row) {
    const QTableWidgetItem* item = table_->item(row, kCommand);
    if (item != nullptr && item->data(kTaskIdRole).toString() == task_id) {
      return row;
    }
  }
  return -1;
}

void WardManagementWidget::updateTaskPhase(const QString& task_id,
                                           uint8_t phase, float /*progress*/) {
  const int row = rowForTask(task_id);
  if (row < 0) {
    return;
  }
  // The robot has started this task once it reports any active phase; start the
  // stopwatch on that first transition (queued tasks stay at "--").
  if (phase != ExecuteTask::Goal::PHASE_QUEUED) {
    auto timing_it = timings_.find(task_id);
    if (timing_it != timings_.end() && !timing_it->second.started) {
      timing_it->second.timer.start();
      timing_it->second.started = true;
      // Announce the start so the panel can log it (authoritative timing lives
      // here in the widget, not in the logger).
      const auto* robot_item = table_->item(row, kRobot);
      const auto* command_item = table_->item(row, kCommand);
      Q_EMIT taskStarted(
          robot_item != nullptr ? robot_item->text() : QString(),
          command_item != nullptr ? command_item->text() : QString(), task_id);
    }
  }
}

void WardManagementWidget::abortTask(const QString& task_id) {
  const int row = rowForTask(task_id);
  if (row < 0) {
    return;
  }
  const auto* robot_item = table_->item(row, kRobot);
  const QString robot_id =
      robot_item != nullptr ? robot_item->text() : QString();
  auto timing_it = timings_.find(task_id);
  const bool running = timing_it != timings_.end() && timing_it->second.started;

  // Ask the panel to cancel the goal. For a running task this stops it and the
  // robot teleports back to its home position; for a queued task it is dropped
  // from the robot's queue before it ever starts.
  Q_EMIT taskAbortRequested(robot_id, task_id, running);

  if (running) {
    // Keep the row so the operator still sees the (now frozen) elapsed time.
    // Disable the button to avoid a second cancel; setTaskResult removes it and
    // freezes the time once the cancelled result arrives.
    if (auto* button = table_->cellWidget(row, kAbort)) {
      button->setEnabled(false);
    }
    Q_EMIT statusPosted(QString("Aborting task %1").arg(task_id), false);
  } else {
    // Queued task: drop its row and stopwatch immediately.
    timings_.erase(task_id);
    table_->removeRow(row);
    Q_EMIT statusPosted(QString("Removed queued task %1").arg(task_id), false);
  }
}

void WardManagementWidget::setTaskResult(const QString& task_id, bool success,
                                         const QString& message) {
  const int row = rowForTask(task_id);
  QString elapsed = "--";
  float elapsed_seconds = -1.0F;
  auto timing_it = timings_.find(task_id);
  if (timing_it != timings_.end()) {
    // Only report a duration if the task actually started running; a task that
    // failed before the robot started it has no meaningful elapsed time.
    if (timing_it->second.started) {
      const qint64 ms = timing_it->second.timer.elapsed();
      elapsed = formatSeconds(ms);
      elapsed_seconds = static_cast<float>(ms) / 1000.0F;
    }
    // Stop the live stopwatch for this task regardless of outcome.
    timings_.erase(timing_it);
  }
  // The row is kept in the table; freeze its final duration and drop the Abort
  // button (there is nothing left to abort once the task has finished).
  QString robot_id;
  QString command_label;
  if (row >= 0) {
    table_->item(row, kTime)->setText(elapsed);
    table_->removeCellWidget(row, kAbort);
    const auto* robot_item = table_->item(row, kRobot);
    const auto* command_item = table_->item(row, kCommand);
    robot_id = robot_item != nullptr ? robot_item->text() : QString();
    command_label = command_item != nullptr ? command_item->text() : QString();
  }
  // Forward the exact measured duration so the logged value matches the table.
  Q_EMIT taskFinished(robot_id, command_label, task_id, success,
                      elapsed_seconds);
  const QString outcome = success ? "completed" : "failed";
  Q_EMIT statusPosted(
      QString("Task %1 %2 in %3%4")
          .arg(task_id, outcome, elapsed,
               message.isEmpty() ? QString() : QString(": %1").arg(message)),
      !success);
}

void WardManagementWidget::tickTimers() {
  for (const auto& [task_id, timing] : timings_) {
    if (!timing.started) {
      continue;
    }
    const int row = rowForTask(task_id);
    if (row >= 0) {
      table_->item(row, kTime)->setText(formatSeconds(timing.timer.elapsed()));
    }
  }
}

}  // namespace hospital_rviz_panel
