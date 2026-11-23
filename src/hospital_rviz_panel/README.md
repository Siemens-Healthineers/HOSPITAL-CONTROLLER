# hospital_rviz_panel

The native C++/Qt5 operator panel for the Hospital Controller application.
`hospital_rviz_panel/HospitalControlPanel` is a `rviz_common::Panel` plugin
loaded into the `rviz2` process. It provides device controls, live telemetry,
Ward Management mission dispatch, and activity-recording controls alongside
the hospital's 3D scene.

## Project Guide

See the [Hospital Controller project guide](../hospital_bringup/README.md) for
the introduction, architecture, full-workspace setup, and application launch.
This README describes the operator panel and its integration.

## Role in the System

The panel is a ROS 2 client, not a simulator or device-state authority. It sends
actions and service requests to
[hospital_controller](../hospital_controller/README.md) and renders published
state and action feedback using
[hospital_interfaces](../hospital_interfaces/README.md).

Ward missions go to [hospital_robot](../hospital_robot/README.md) task servers.
The panel also controls that package's activity logger and publishes mission
timing events for recordings. The animated RobotModel is supplied separately
by [hospital_arena](../hospital_arena/README.md) and
[hospital_description](../hospital_description/README.md).
[hospital_bringup](../hospital_bringup/README.md) starts and configures all of
these pieces together.

## Controls and Interfaces

| Area | Operator functions | ROS interface |
| --- | --- | --- |
| Doors | Open, close, stop; state, opening percentage, duration | `/hospital/doors/operate`, `/hospital/doors/states` |
| CT | Lift up/down/cancel; scan start/stop/retract; state and duration | `/hospital/ct/table/move`, `/hospital/ct/scan/run`, corresponding state topics |
| Laboratory | Start blood analysis; progress, duration, completed cardiac markers | `/hospital/lab/analyze`, `/hospital/lab/state` |
| Operating room | Set CLEAN/DIRTY; toggle simulated X-ray state | `/hospital/operating_room/set_cleanliness`, `/hospital/operating_room/state`, and the controller's `xray_active` parameter |
| Ward Management | Choose robot and mission, submit, inspect phase/progress/time, abort | `/hospital/<robot_id>/ward/execute_task` |
| Recording | Start/stop activity recording | `/hospital/logging/set_recording` (`std_srvs/srv/SetBool`) |

Doors are grouped by location; shared doors appear in more than one section but
still address the same device. The layout has one CT scanner and fixed door
controls. A `door_ids` list in a parameter file does not generate a new layout.

Device durations come from controller-owned `/hospital/stopwatch` messages.
Mission durations are measured by the Ward Management widget, starting at the
first active phase and ending on the result; queued time is excluded. The panel
publishes START/END `TaskEvent` messages on `/hospital/task_events` so the logger
records the same durations. Robot task serialization belongs to the robot,
not the mission table.

Laboratory state and Start availability come only from `/hospital/lab/state`;
Start is enabled only in READY. The transient-local subscription uses KeepLast
depth 1 to receive the current snapshot, not historical results. Each accepted
analysis clears previous measurements, shown as `Result: --` while invalid;
cancellation leaves them empty and only success supplies new measurements.
Lab stopwatch updates affect timing text only. Thus READY after completion can
appear alongside a time marked `(done)` or `(interrupted)` without overriding
the device state.

CT stop freezes the scan with simulated radiation off; use Retract separately
to pull the tabletop out. The panel's vertical Stop button stops lift motion
regardless of its origin: it cancels standalone UP/DOWN goals (including goals
from another client), or sends RunScan STOP while the procedure reports
PREPARING_LIFT. This interrupts scan preparation without retraction. During
gantry sweeps or recovery, use scan Stop; vertical Stop does not stop horizontal
movement. Routing uses the latest received procedure and table telemetry.
The CT procedure label reads `/hospital/ct/scan/procedure_state` with
transient-local KeepLast depth 1, including scans initiated by robots or other
clients. It distinguishes preparation, inward/outward sweep, recovery, and
terminal status without changing the separate lift-height and sweep-progress
bars. Axis topics remain the current-position source; the retained procedure
snapshot is not reused as live position after a standalone lift movement.
RunScan feedback also supplies separate lift and scanner telemetry through
the existing throttled GUI cache. The CT stopwatch still excludes lifting
and recovery. A terminal procedure label describes the last RunScan, not
whether a subsequent standalone MoveTable is running or CT can accept a goal.
The X-ray toggle sets `/operating_room_controller_node`'s parameter; the
controller republishes the latched room state on every change, so the panel
needs no state query. Door, CT and operating-room state topics are subscribed
with `transient_local` durability, so a panel opened late shows current state.
These are simulation/operator controls, not real clinical safety interlocks.

## Usage in the Application

Follow the central [build](../hospital_bringup/README.md#build) and
[run](../hospital_bringup/README.md#run) instructions. Bringup opens RViz with
the panel and starts the controllers, fleet, logger, and model bridge it uses.
Launching RViz alone does not provide those servers or the live hospital model.

Build dependencies include Qt5 Widgets, `rviz_common`, `pluginlib`, ROS 2
action/service support, and the hospital interfaces. Running RViz requires a
graphical session.

The plugin can also be added to another RViz layout through **Panels > Add New
Panel**, selecting `hospital_rviz_panel/HospitalControlPanel` after sourcing
the workspace. A compatible live scene uses fixed frame `world`, a RobotModel
on `/robot_description`, and robot markers on `/hospital/robot_markers`.

### Plugin-loading regression test

After building, verify that RViz's plugin loader can discover and instantiate
the installed panel without a graphical session:

```bash
colcon test --packages-select hospital_rviz_panel --ctest-args -R '^test_plugin_loading$'
colcon test-result --test-result-base build/hospital_rviz_panel --verbose
```

Keep the `xml-model` instruction in [package.xml](package.xml) immediately after
the XML declaration, before the copyright comment. Pluginlib's TinyXML2 parser
rejects it after a comment, preventing RViz from loading the panel even though
the package builds successfully.

## License

Copyright (c) 2026 Siemens Healthineers AG

SPDX-License-Identifier: GPL-3.0-or-later

The package's first-party code and configuration are licensed under the
[GNU General Public License, version 3 or later](LICENSE). The license does
not grant trademark rights; see the [trademark notice](../../NOTICE).
