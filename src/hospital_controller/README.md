# hospital_controller

The authoritative smart-building simulation for the Hospital Controller project.
Four composable C++20 nodes model doors, the CT scanner, a blood analyser, and
an operating room. Clinical processes and radiation are simulated states and
timed operations, not real medical procedures.

## Project Guide

See the [Hospital Controller project guide](../hospital_bringup/README.md) for
the introduction, architecture, full-workspace setup, and application launch.
This README is the detailed device-behavior reference.


## Role in the System

The controllers own device state and expose actions, services, and telemetry
from [hospital_interfaces](../hospital_interfaces/README.md).
[hospital_robot](../hospital_robot/README.md) and
[hospital_rviz_panel](../hospital_rviz_panel/README.md) are clients of that API.
[hospital_arena](../hospital_arena/README.md) converts telemetry into joints for
the [hospital_description](../hospital_description/README.md) model.
[hospital_bringup](../hospital_bringup/README.md) launches and configures the full
system. Ward missions are served by robots, not by these four device nodes.


## Controllers and Behavior

### Doors

`hospital_controller::DoorControllerNode` manages multiple named doors through
the `/hospital/doors/operate` action (`OperateDoor`).

**Commands and concurrency.** Each goal specifies a `device_id` and a command:
OPEN (`0`) or CLOSE (`1`). Goals are rejected for unknown door IDs, invalid
commands, a door reserved by an existing operation, or an already reached
target (OPEN at 100% or CLOSE at 0%). Different doors can move concurrently
on independent workers.

The launch-time `door_ids` list must contain unique IDs. A duplicate fails node
construction with an error identifying the duplicate; valid lists retain their
configured order in the aggregated state.

**Cancellation and restart.** Cancellation holds the last completed opening
percentage. A stopped door reports CLOSED at 0%, OPEN at 100%, or STOPPED (`5`)
between those endpoints. Once the previous worker releases its reservation,
a new OPEN or CLOSE goal can continue or reverse travel from the retained
percentage. There is no autonomous closing, reversal, or retry after cancellation.

**State reporting.** `DoorState` contains the state (CLOSED, OPENING, OPEN,
CLOSING, or STOPPED) and `percentage_open`. State is available through:

- `/hospital/doors/state`: individual `DoorState` updates.
- `/hospital/doors/states`: the full `DoorStateArray`.
- `/hospital/doors/get_states`: a service returning a full snapshot.

Both state topics use transient-local durability.

### CT Scanner

`hospital_controller::CtScannerControllerNode` models a vertical table lift and
a horizontal tabletop that moves into and out of the gantry. Radiation is
simulated and enabled only during the scan sweep.

**Commands and concurrency.** Two actions control CT movement:

- `/hospital/ct/table/move` (`MoveTable`): UP (`0`) or DOWN (`1`).
- `/hospital/ct/scan/run` (`RunScan`): START (`0`), STOP (`1`), or RETRACT (`2`).

Standalone lift movement and scan/retraction are mutually exclusive. START can
begin from any stationary lift height. **Inserted-tabletop interlock:** MoveTable
rejects both UP and DOWN unless the tabletop is fully OUT, even after a stopped
scan worker exits. Complete RETRACT before moving the lift.

**Scan sequence.** A successful START performs three phases:

1. **PREPARING_LIFT:** raise the lift from its current height to TOP without
    radiation. Feedback and the procedure topic report lift state and height.
    Horizontal scanner state can remain IDLE or PAUSED during preparation;
    this does not mean CT is available.
2. **SWEEP_IN:** move the tabletop toward fully IN with radiation enabled.
    Scanner state is MOVING_IN; sweep progress runs from 0 to 50.
3. **SWEEP_OUT:** move the tabletop fully OUT with radiation enabled.
    Scanner state is MOVING_OUT; sweep progress runs from 50 to 100.

Completion resets scanner state to IDLE and sweep progress to 0, turns radiation
off, and leaves the lift at TOP. Lowering requires a separate DOWN goal. The
scan stopwatch times only the horizontal sweep, excluding preparation and recovery.

START after a halt begins a new cycle, not a resumed action: raise to TOP,
sweep toward IN from the retained insertion, then sweep OUT.

**Stopping and recovery.** Stopping is not retraction. Canceling START or RETRACT,
or sending RunScan STOP (`COMMAND_STOP`), holds the last completed positions
with radiation off. Neither axis returns automatically. STOP is accepted only
while the scan/retraction reservation is held; its result acknowledges the
request, not completion of the halt or recovery. It does not stop a standalone
MoveTable goal; cancel that goal instead.

Canceling standalone lift movement holds its height, reporting PAUSED between
endpoints or BOTTOM / TOP at an endpoint. A halt during PREPARING_LIFT uses the
same resting states and can leave the lift below TOP; the table topic,
INTERRUPTED procedure snapshot, and action feedback report the held height.
The active scan/retraction worker publishes INTERRUPTED when it handles the halt.

To recover after a halted scan:

1. Wait for the original action to terminate and its reservation to clear.
    The STOP acknowledgment alone is not sufficient.
2. Send RETRACT (`COMMAND_RETRACT`) to pull the tabletop fully OUT with radiation
    off, leaving the lift unchanged. Already OUT is an accepted no-op.
3. After successful retraction and reservation release, send DOWN
    (`COMMAND_DOWN`) if the lift should return to BOTTOM.

**State reporting.** RunScan feedback is published at phase boundaries and each
completed motion step. It contains three independent fields:

- `phase`: procedure phase, using `CtScanProcedureState` constants.
- `table_telemetry` (`CtTableState`): vertical lift state and absolute height.
  `percentage_done` is 0 at BOTTOM and 100 at TOP, including during preparation.
- `telemetry` (`CtScannerState`): horizontal sweep progress and radiation flag.
  Progress maps 0 to OUT, 50 to IN, and 100 to OUT.

Neither percentage represents overall procedure completion. The same fields
are published as `CtScanProcedureState` on `/hospital/ct/scan/procedure_state`,
so observers can follow procedures started by other clients. This topic uses
transient-local durability and KeepLast depth 1. Its phase constants have the
`PHASE_` prefix:

| Phase | Value | Meaning |
| --- | --- | --- |
| INACTIVE | 0 | Startup; no procedure has run. |
| PREPARING_LIFT | 1 | Raising the lift to TOP before the sweep. |
| SWEEP_IN | 2 | Inward scan sweep. |
| SWEEP_OUT | 3 | Outward scan sweep. |
| RETRACTING | 4 | Radiation-free tabletop recovery. |
| COMPLETED | 5 | Successful START or RETRACT, including already-OUT recovery. |
| INTERRUPTED | 6 | Active worker has handled a halt. |

The procedure topic retains the last snapshot, not an activity lock or event
history. Standalone MoveTable goals do not update it. Use the transient-local
axis topics `/hospital/ct/table/state` and `/hospital/ct/scan/state` for current
positions and model animation, particularly after later standalone movement.
Instantaneous phases can be superseded before a subscriber reads them, and
early feedback can precede client-side goal registration. Determine an action's
outcome from its original result, not a phase transition alone.



### Laboratory

`hospital_controller::LabControllerNode` exposes the `/hospital/lab/analyze`
action (`AnalyzeBlood`).

**Commands and concurrency.** The analyser accepts one goal at a time. The only
supported command is the case-sensitive string `START_BLOODTEST`; other commands
and requests made while busy are rejected without changing state or results.

**Simulated results.** The launch-time `cardiac_abnormal` parameter selects
whether randomly generated marker values are normal or abnormal. Results contain:

- Troponin T (`ctnt`, ng/L): abnormal above 14 ng/L.
- Troponin I (`ctni`, ng/L): abnormal above 20 ng/L.
- CK-MB (`ck_mb`, percent): abnormal above 5 percent.

**State reporting.** `/hospital/lab/state` publishes the current snapshot with
transient-local durability and KeepLast depth 1. Action feedback carries live
progress, but no measurements (`result_valid=false`, zero markers). Completed
measurements arrive in the action result and on the state topic.

**Lifecycle and cancellation.** The analyser starts READY at 0%, without a valid
result. An accepted test clears previous measurements and enters ANALYSING at
0%. Success briefly publishes DONE at 100%, then READY at 0%, retaining valid
measurements until the next accepted test. Cancellation returns `success=false`
and zero markers, leaving the analyser READY at 0% with no valid result; aborted
tests also leave measurements invalid. Consumers must check `result_valid`:
zero markers with invalid results mean unavailable, not normal. Each new test
starts from zero; there is no resume or automatic retest.


### Operating Room

`hospital_controller::OperatingRoomControllerNode` starts CLEAN and exposes:

- `/hospital/operating_room/set_cleanliness` (`SetCleanliness`): target `0` = DIRTY, `1` = CLEAN; invalid values return `accepted=false`.
- `/hospital/operating_room/get_state` (`GetOperatingRoomState`): returns current cleanliness and the simulated `xray_active` flag without publishing.
- `/hospital/operating_room/state` (`OperatingRoomState`): latched (`transient_local`, depth 1). Published at startup, on accepted cleanliness requests, and whenever `xray_active` changes, so a late subscriber always receives the current state.

`xray_active` is a runtime-settable node parameter for the simulated radiation
state, not a robot action. A change publishes the state topic.
There is no periodic operating-room state publisher.


## Stop and Cancellation Reference


### Admission Matrix

| Request | Accepted when | Rejection or execution edge case |
| --- | --- | --- |
| Door OPEN (0) / CLOSE (1) | Door ID exists, that door is not reserved, and target differs from current percentage. | Unknown ID, unknown command, busy door, OPEN at 100%, or CLOSE at 0% is rejected. Duplicate and opposite goals do not replace the active goal. |
| MoveTable UP (0) / DOWN (1) | Neither standalone lift nor scan/retraction is reserved, table-top is fully OUT, and target differs from current lift position. | Unknown command, busy CT, inserted table-top, UP at TOP, or DOWN at BOTTOM is rejected. Duplicate and opposite goals do not replace the active goal. |
| RunScan START (0) | Neither scan/retraction nor standalone lift is reserved. | Busy CT or unknown command is rejected. A paused lift or inserted table-top does not prevent START. |
| RunScan STOP (1) | The scan reservation is held | Rejected when no scan worker owns the resource, even if a standalone MoveTable is running. Success acknowledges a stop request only. |
| RunScan RETRACT (2) | Neither scan/retraction nor standalone lift is reserved. | Busy CT is rejected. Already OUT is an accepted no-op. |
| AnalyzeBlood `START_BLOODTEST` | Analyser is not reserved. | Any other case-sensitive command string, including a STOP string, or another running test is rejected. |
| SetCleanliness DIRTY (0) / CLEAN (1) | Target value is valid, regardless of current cleanliness or X-ray flag. | Invalid value returns `accepted=false` without publishing or changing state. Repeating the current value succeeds and republishes. |


There is **no global emergency-halt subscription**, persistent brake state,
or reset-to-enable procedure in these four controllers. A fresh valid goal can
run after the old reservation clears. Cancellation targets an action goal;
RunScan STOP is the only separate stop command and can be sent by any client.

| Operation / stop route | Motion and state after worker handles stop | Original operation outcome |
| --- | --- | --- |
| Door action cancel | Hold last completed opening percentage; CLOSED / OPEN at endpoints, otherwise STOPPED. No retraction or reversal. | CANCELED, `success=false`, `"Operation canceled"`. |
| Standalone MoveTable cancel | Hold last completed lift position; BOTTOM / TOP at anchors, PAUSED between them. | CANCELED, `success=false`, `"Movement canceled"`; `final_position` is a state enum, not a percentage. |
| START cancel during preparatory lift | Lift holds its last completed height: PAUSED between anchors, otherwise BOTTOM / TOP. Scanner becomes IDLE if table-top OUT, otherwise PAUSED; radiation off. Procedure phase INTERRUPTED. | CANCELED, **`success=true`**, `"Scan halted; table-top holding position"`. |
| START cancel during sweep | Hold both axes; lift at TOP, scanner PAUSED if inserted, otherwise IDLE; radiation off. No automatic retraction. | CANCELED, **`success=true`**, same halt message. |
| RunScan STOP during START | Same holding behavior as cancel for the active phase. | START becomes ABORTED with **`success=true`**. The separate STOP goal immediately SUCCEEDS with `"Stop requested"`. |
| Lab action cancel | READY, 0% progress; zero markers and `result_valid=false` on topic state. | CANCELED, `success=false`, `"Analysis canceled"`, zero result markers. |
| RETRACT cancel or RunScan STOP | Hold last completed table-top position; PAUSED while inserted, otherwise IDLE. Radiation off, lift unchanged. | RETRACT becomes CANCELED for cancel or ABORTED for STOP, with `success=false`, `"Retraction halted; table-top holding position"`. STOP itself SUCCEEDS with `"Stop requested"`. |

Door, standalone lift, and lab workers use ABORTED with `success=false` when
they observe a worker stop without the goal being in CANCELING. They report
`"Operation stopped"`, `"Movement halted"`, or `"Analysis stopped"` respectively.
This is not a public alternate stop command or a guaranteed shutdown result.

For CT recovery, wait for the original START result, allow its reservation to
clear, send RETRACT, wait for its successful result, then send DOWN. The STOP
goal's result alone is not a barrier: immediate RETRACT may still be rejected.
If RETRACT is stopped or canceled, wait for that goal to terminate and its
reservation to clear, then send another RETRACT to complete recovery before DOWN.


## Usage in the Application

Follow the central [build](../hospital_bringup/README.md#build) and
[run](../hospital_bringup/README.md#run) instructions. Bringup loads all four
components into `hospital_controller_container` and supplies shared parameters
alongside the clients, robot fleet, and live scene. These components are not
standalone executables for `ros2 run`.

Do not start a second controller launch while the application is running:
duplicate servers on the same absolute endpoints can produce conflicting state
and action results.

## Command-Line Examples

With the full application running, use another terminal sourced as described
in the [project run guide](../hospital_bringup/README.md#run). Execute commands
individually and wait for completion before issuing a dependent operation:

```bash
ros2 action send_goal --feedback /hospital/doors/operate \
    hospital_interfaces/action/OperateDoor "{device_id: 'clinical_corridor_1', command: 0}"
ros2 service call /hospital/doors/get_states hospital_interfaces/srv/GetDoorStates "{}"

ros2 action send_goal --feedback /hospital/ct/table/move \
    hospital_interfaces/action/MoveTable "{command: 0}"
ros2 action send_goal --feedback /hospital/ct/scan/run \
    hospital_interfaces/action/RunScan "{command: 0}"
# In another terminal while the scan is active:
ros2 action send_goal /hospital/ct/scan/run hospital_interfaces/action/RunScan "{command: 1}"
# After the scan has stopped:
ros2 action send_goal /hospital/ct/scan/run hospital_interfaces/action/RunScan "{command: 2}"
ros2 action send_goal /hospital/ct/table/move hospital_interfaces/action/MoveTable "{command: 1}"

ros2 action send_goal --feedback /hospital/lab/analyze \
    hospital_interfaces/action/AnalyzeBlood "{command: 'START_BLOODTEST'}"
ros2 service call /hospital/operating_room/set_cleanliness \
    hospital_interfaces/srv/SetCleanliness "{target_state: 0}"
ros2 service call /hospital/operating_room/get_state \
    hospital_interfaces/srv/GetOperatingRoomState "{}"
```

CT and lab goals have no `device_id`. Use IDs from the active door configuration;
clients can discover them with `get_states`.

## Configuration and Checks

| Node | Main parameters |
| --- | --- |
| `door_controller_node` | Launch-time `door_ids`; `open_duration`, `close_duration`, `publish_rate`. |
| `ct_scanner_controller_node` | `table_move_duration`, `scan_duration`, `publish_rate`. |
| `lab_controller_node` | `analysis_duration`, `publish_rate`; launch-time `cardiac_abnormal`. |
| `operating_room_controller_node` | `xray_active`. |

Durations are seconds and rates are Hz. Durations must lie in
[0.001, 3600] s and `publish_rate` in [0.1, 1000] Hz. The ranges are parameter
descriptors, so rclcpp rejects out-of-range values both at startup (the node
fails to construct) and at runtime. A runtime `set_parameters_atomically`
batch is applied all-or-nothing. Integrated values come from
[the shared YAML](../hospital_bringup/config/hospital_params.yaml), which can
differ from compiled defaults. Keep door IDs aligned with robot protocols,
panel controls, and model mappings.

All timed controller animations use steady-clock deadlines anchored to the
start of each movement. Publishing and scheduling overhead do not accumulate
as an extra delay per step. A stop request wakes the current wait immediately;
the device retains its last completed step, as described above.


## License

Copyright (c) 2026 Siemens Healthineers AG

SPDX-License-Identifier: GPL-3.0-or-later

The package's first-party code and configuration are licensed under the
[GNU General Public License, version 3 or later](LICENSE).
