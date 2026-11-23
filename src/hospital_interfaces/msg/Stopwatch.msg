# A single stopwatch reading published by a controller while it performs a
# timed operation (door opening, blood analysis, CT scan). The controller is
# the authoritative source of the measurement: it captures the duration with a
# monotonic clock on the worker thread that owns the operation, so the value is
# race-free and immune to UI lag or message-rate jitter. Consumers (e.g. the
# RViz panel) only render it; for a smoothly ticking display they may compute
# `now - start_stamp` locally while `running` is true and snap to
# `final_seconds` once it turns false.

# What is being timed.
uint8 KIND_DOOR_OPEN=0
uint8 KIND_LAB_ANALYSIS=1
uint8 KIND_CT_SCAN=2

# Device the stopwatch belongs to ("clinical_corridor_1", "lab", "ct_scanner", ...).
string device_id
uint8 kind

# ROS time at which the operation started. A display can compute the live
# elapsed time as (now - start_stamp) while `running` is true.
builtin_interfaces/Time start_stamp

# Monotonic elapsed time since the operation started, refreshed on every tick.
float32 elapsed_seconds

# Authoritative total duration, 0 while running and set once the operation
# ends (whether it completed or was interrupted).
float32 final_seconds

# Expected total duration of the operation, or 0 when unknown. A display can
# clamp its live `now - start_stamp` estimate to this value so a running timer
# never visibly overshoots the nominal time while it waits for the authoritative
# terminal message, and can time out a stopwatch whose terminal message was lost.
float32 nominal_seconds

# True while the operation is in progress, false once it has finished.
bool running

# True if the operation ended early (cancelled / stopped / obstructed) instead
# of completing normally.
bool interrupted

# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later
