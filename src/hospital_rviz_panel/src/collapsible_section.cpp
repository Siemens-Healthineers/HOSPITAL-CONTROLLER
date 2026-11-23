// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hospital_rviz_panel/collapsible_section.hpp"

#include <QLayout>
#include <QToolButton>
#include <QVBoxLayout>

namespace hospital_rviz_panel {

CollapsibleSection::CollapsibleSection(const QString& title, QWidget* parent)
    : QWidget(parent) {
  toggle_button_ = new QToolButton(this);
  toggle_button_->setObjectName("sectionHeader");
  toggle_button_->setText(title);
  toggle_button_->setCheckable(true);
  toggle_button_->setChecked(false);
  toggle_button_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  toggle_button_->setArrowType(Qt::RightArrow);
  toggle_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  content_area_ = new QWidget(this);
  content_area_->setVisible(false);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);
  outer->addWidget(toggle_button_);
  outer->addWidget(content_area_);

  connect(toggle_button_, &QToolButton::toggled, this,
          [this](bool checked) { setExpanded(checked); });
}

void CollapsibleSection::setContentLayout(QLayout* layout) {
  delete content_area_->layout();
  content_area_->setLayout(layout);
}

void CollapsibleSection::setExpanded(bool expanded) {
  toggle_button_->setChecked(expanded);
  toggle_button_->setArrowType(expanded ? Qt::DownArrow : Qt::RightArrow);
  content_area_->setVisible(expanded);
}

}  // namespace hospital_rviz_panel
