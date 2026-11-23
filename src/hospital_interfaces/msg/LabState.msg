uint8 READY=0
uint8 ANALYSING=1
uint8 DONE=2

uint8 state
float32 progress_percentage

# Current analysis result. Every accepted new test clears the markers to zero
# and sets result_valid=false; cancellation/abort leaves them cleared.
# Success sets result_valid=true and retains measurements until the next test.
# Zero markers with result_valid=false mean unavailable, not a normal result.
bool result_valid
# Troponin T (cTnT) in ng/L.
float32 ctnt
# Troponin I (cTnI) in ng/L.
float32 ctni
# Creatine kinase MB (CK-MB) in %.
float32 ck_mb

# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later
