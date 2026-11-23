# A ward-task lifecycle event emitted by the RViz panel's Mission dispatch so
# the activity logger can record task durations that EXACTLY match the panel's
# timing table. The panel is the sole authoritative timer for a ward task (it
# starts on the first active-phase feedback and stops on the action result), so
# it forwards the elapsed value it already measured rather than have the logger
# re-measure and drift out of sync with the on-screen table.

# START when the robot begins the task, END when its result arrives.
uint8 START=0
uint8 END=1
uint8 kind

# Robot executing the task, a human-readable command label (e.g.
# "Clean Operating Room"), and the unique id correlating START with END.
string robot_id
string task_label
string task_id

# ROS time of the event.
builtin_interfaces/Time stamp

# END only: task outcome and the authoritative duration measured by the panel.
bool success
float32 elapsed_seconds

# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later
