// Copyright (c) 2026 Siemens Healthineers AG
// SPDX-License-Identifier: GPL-3.0-or-later

#include <QApplication>

#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"
#include "rviz_common/panel.hpp"

TEST(PluginLoading, InstantiatesExportedPanel) {
  qputenv("QT_QPA_PLATFORM", "offscreen");
  int argc = 1;
  char application_name[] = "test_plugin_loading";
  char* argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common",
                                                    "rviz_common::Panel");
  constexpr char kClassName[] = "hospital_rviz_panel/HospitalControlPanel";
  ASSERT_EQ(loader.getClassPackage(kClassName), "hospital_rviz_panel");
  const auto panel = loader.createSharedInstance(kClassName);
  ASSERT_NE(panel, nullptr);
}
