uint8 BOTTOM=0
uint8 MOVING_UP=1
uint8 MOVING_DOWN=2
uint8 TOP=3
uint8 PAUSED=4

uint8 state
float32 percentage_done
# `state` describes the vertical patient-table lift. Anchors: BOTTOM (patient
# boards here) and TOP (raised for the gantry sweep). `state` is PAUSED when a
# movement was frozen between the two anchors by a cancel; the table holds its
# physical position and a new movement can resume from there.
# `percentage_done` is the lift position along the travel axis:
# 0 = BOTTOM, 100 = TOP. It therefore fully describes the physical position
# together with `state` (e.g. PAUSED at 20 = 20% of the way from BOTTOM to
# TOP).

# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later