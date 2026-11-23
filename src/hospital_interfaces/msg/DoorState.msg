uint8 CLOSED=0
uint8 OPENING=1
uint8 OPEN=2
uint8 CLOSING=3
uint8 OBSTRUCTED=4
uint8 STOPPED=5

string device_id
uint8 state
float32 percentage_open
# STOPPED means stationary between 0 (CLOSED) and 100 (OPEN).
# A new OPEN or CLOSE goal moves from the held position; no automatic resume.

# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later
