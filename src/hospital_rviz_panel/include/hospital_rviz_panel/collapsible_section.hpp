// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef HOSPITAL_RVIZ_PANEL__COLLAPSIBLE_SECTION_HPP_
#define HOSPITAL_RVIZ_PANEL__COLLAPSIBLE_SECTION_HPP_

#include <QString>
#include <QWidget>

class QLayout;
class QToolButton;

namespace hospital_rviz_panel {

/// A titled section whose body can be folded away by clicking its header.
///
/// Qt has no built-in collapsible group box, so this wraps a checkable header
/// button (with a disclosure arrow) above a content area that is shown or
/// hidden as the button is toggled. The panel groups its controls by hospital
/// location into one of these each so the long list of devices reads as a few
/// compact, foldable blocks instead of one overwhelming column.
class CollapsibleSection : public QWidget {
  Q_OBJECT

 public:
  explicit CollapsibleSection(const QString& title, QWidget* parent = nullptr);

  /// Install the section body. Takes ownership of `layout`.
  void setContentLayout(QLayout* layout);

  /// Expand (true) or collapse (false) the body.
  void setExpanded(bool expanded);

 private:
  QToolButton* toggle_button_;
  QWidget* content_area_;
};

}  // namespace hospital_rviz_panel

#endif  // HOSPITAL_RVIZ_PANEL__COLLAPSIBLE_SECTION_HPP_
