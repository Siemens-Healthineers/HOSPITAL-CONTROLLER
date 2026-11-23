uint8 PHASE_INACTIVE=0
uint8 PHASE_PREPARING_LIFT=1
uint8 PHASE_SWEEP_IN=2
uint8 PHASE_SWEEP_OUT=3
uint8 PHASE_RETRACTING=4
uint8 PHASE_COMPLETED=5
uint8 PHASE_INTERRUPTED=6

# Last RunScan procedure snapshot, not an admission/resource lock.
# Standalone MoveTable operations do not update this snapshot.
uint8 phase
# Absolute vertical position: 0 = BOTTOM, 100 = TOP.
hospital_interfaces/CtTableState table_telemetry
# Horizontal sweep progress: 0 = OUT, 50 = IN, 100 = OUT.
hospital_interfaces/CtScannerState telemetry

# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later