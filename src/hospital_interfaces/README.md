# hospital_interfaces

The shared ROS 2 API for the Hospital Controller project. This package defines
messages, services, and actions and generates their language bindings with
`rosidl`. It contains no running nodes or simulation logic.

## Project Guide

See the [Hospital Controller project guide](../hospital_bringup/README.md) for
the introduction, architecture, full-workspace setup, and application launch.
This README is the shared ROS 2 interface reference.

## Role in the System

[hospital_controller](../hospital_controller/README.md) serves device commands
and publishes state using these types.
[hospital_robot](../hospital_robot/README.md) consumes the device API and serves
ward missions. [hospital_rviz_panel](../hospital_rviz_panel/README.md) is an
operator client, and [hospital_arena](../hospital_arena/README.md) converts device
telemetry into model joints. External teams can use the same API without
depending on those clients or the hospital meshes.

The controllers are authoritative for device states. Long-running operations
use actions with feedback and cancellation; queries and simple changes use
services. Topic names belong to the implementing nodes, not the type definitions.

## Interface Reference

### Actions

| Definition | Server endpoint in the integrated system | Goal and feedback |
| --- | --- | --- |
| [OperateDoor](action/OperateDoor.action) | `/hospital/doors/operate` | `device_id`, `command`: open `0`, close `1`; `DoorState` feedback. |
| [MoveTable](action/MoveTable.action) | `/hospital/ct/table/move` | `command`: up `0`, down `1`; `CtTableState` feedback. |
| [RunScan](action/RunScan.action) | `/hospital/ct/scan/run` | `command`: start `0`, stop `1`, retract `2`; feedback: `phase`, `CtTableState table_telemetry`, `CtScannerState telemetry`. |
| [AnalyzeBlood](action/AnalyzeBlood.action) | `/hospital/lab/analyze` | `command: START_BLOODTEST`; `LabState` feedback and cardiac marker results. |
| [ExecuteTask](action/ExecuteTask.action) | `/hospital/<robot_id>/ward/execute_task` | `value`, `target_location`, `task_id`; phase, progress, and correlation ID feedback. |

Only door goals need a `device_id`: the CT scanner and laboratory are single
devices. Mission values are `CLEAN_OR=0`, `COLLECT_LAB=1`, `GUIDE_CT=2`,
`HEART_ATTACK=3`, and `SURGICAL_ASSIST=4`. Feedback phases are
`PHASE_QUEUED=5`, `PHASE_NAVIGATING=6`, `PHASE_EXECUTING=7`, and `PHASE_DONE=8`.
The robot, not a device controller, hosts `ExecuteTask`.

### Messages

| Definition | Purpose / usual topic |
| --- | --- |
| [DoorState](msg/DoorState.msg) | Named door status and opening percentage; `/hospital/doors/state`. |
| [DoorStateArray](msg/DoorStateArray.msg) | Snapshot of all doors; `/hospital/doors/states`. |
| [CtTableState](msg/CtTableState.msg) | Vertical lift status and position: `0` percent = BOTTOM, `100` = TOP; `/hospital/ct/table/state`. |
| [CtScannerState](msg/CtScannerState.msg) | Horizontal scan-cycle progress and simulated `radiating` flag; `/hospital/ct/scan/state`. |
| [CtScanProcedureState](msg/CtScanProcedureState.msg) | Last RunScan phase and both axis snapshots; `/hospital/ct/scan/procedure_state` (transient-local, KeepLast 1). |
| [LabState](msg/LabState.msg) | Analysis progress and current result validity; `/hospital/lab/state`. |
| [OperatingRoomState](msg/OperatingRoomState.msg) | `DIRTY=0` / `CLEAN=1` and `xray_active`; `/hospital/operating_room/state`. |
| [Stopwatch](msg/Stopwatch.msg) | Controller-owned operation timing; `/hospital/stopwatch`. |
| [TaskEvent](msg/TaskEvent.msg) | Panel-emitted mission START/END events and measured duration; `/hospital/task_events`. |

Door, lift, scanner, and laboratory progress fields use percentages (`0` to
`100`). `ExecuteTask.progress` is separate mission progress; the bundled robot
uses `0` to `1`. Scanner progress describes an out-in-out cycle, not a single
absolute insertion axis: `0` and `100` are out, `50` is fully in.

RunScan phase constants are defined in `CtScanProcedureState`:
`PHASE_INACTIVE=0`, `PHASE_PREPARING_LIFT=1`, `PHASE_SWEEP_IN=2`,
`PHASE_SWEEP_OUT=3`, `PHASE_RETRACTING=4`, `PHASE_COMPLETED=5`, and
`PHASE_INTERRUPTED=6`. Action feedback and the procedure topic use the same
fields and meanings. Lift height is not sweep progress or total completion.
The procedure topic is a latest snapshot, not a phase event log or resource
lock, and is not updated by standalone MoveTable goals. Use the axis topics
for current positions after standalone movement. The result remains the
goal-specific outcome; the STOP result acknowledges a request only.

Laboratory action feedback deliberately has `result_valid=false` and no marker
results. Each accepted test immediately clears state-topic markers to zero and
sets `result_valid=false`; cancellation/abort leaves them cleared. Success sets
new measurements valid until the next accepted test. Rejection changes nothing.
Zero with invalid status means unavailable, not normal. Read completed markers
from a successful action result or a valid state-topic message. The lab state
publisher retains only the current snapshot (transient-local, KeepLast depth 1).
`Stopwatch` distinguishes live, final, and nominal durations, with
`running` and `interrupted` flags. `TaskEvent.elapsed_seconds` carries the
panel's mission timer so recordings match the displayed time.

### Services

| Definition | Endpoint | Purpose |
| --- | --- | --- |
| [GetDoorStates](srv/GetDoorStates.srv) | `/hospital/doors/get_states` | Empty request; returns `door_states`. |
| [GetOperatingRoomState](srv/GetOperatingRoomState.srv) | `/hospital/operating_room/get_state` | Empty request; returns `state`. |
| [SetCleanliness](srv/SetCleanliness.srv) | `/hospital/operating_room/set_cleanliness` | `target_state` (`0` dirty, `1` clean); returns `accepted`. |

Recording uses the standard `std_srvs/srv/SetBool` type, not a custom service
here. The operating-room X-ray flag is changed through a node parameter.
See the [controller README](../hospital_controller/README.md) for behavior and
topic durability; do not assume every state topic is latched.

## Inspect the Interfaces

Build and source the workspace using the central
[build guide](../hospital_bringup/README.md#build). Interface inspection does not
require running controllers:

```bash
ros2 interface show hospital_interfaces/action/RunScan
ros2 interface show hospital_interfaces/msg/Stopwatch
ros2 interface show hospital_interfaces/srv/GetDoorStates
```

For live device calls, start the full application and use the central
[command-line examples](../hospital_bringup/README.md#command-line-examples).
When changing interface fields or constants, rebuild dependent packages and
update both servers and clients.

## License

Copyright (c) 2026 Siemens Healthineers AG

SPDX-License-Identifier: GPL-3.0-or-later

The package's first-party code and configuration are licensed under the
[GNU General Public License, version 3 or later](LICENSE).

License notices are at the end of ROS interface files to preserve their
generated interface descriptions.
