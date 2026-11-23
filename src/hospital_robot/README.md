# hospital_robot

Protocol-driven robot simulation and robot-independent activity recording for
the Hospital Controller project. Two composable C++20 components are provided:
`hospital_robot::RobotServerNode` and `hospital_robot::ActivityLoggerNode`.

The simulated robot is an API/workflow demonstration, not an autonomous
navigation stack. It waits for a configured travel time and teleports between
fixed room waypoints. Patient interaction, cleaning, and object handling are
represented by timers and controller calls rather than physical manipulation.

## Project Guide

See the [Hospital Controller project guide](../hospital_bringup/README.md) for
the introduction, architecture, full-workspace setup, and application launch.
This README describes robot protocols, fleet interfaces, and recording behavior.

## Role in the System

Ward Management in [hospital_rviz_panel](../hospital_rviz_panel/README.md)
dispatches `ExecuteTask` goals to robots. Each robot serves those missions and
acts as a client of [hospital_controller](../hospital_controller/README.md) for
doors, CT, blood analysis, and room cleanliness. Types come from
[hospital_interfaces](../hospital_interfaces/README.md).

[hospital_bringup](../hospital_bringup/README.md) launches the fleet and logger
in separate component containers and supplies shared configuration. The
[hospital_description](../hospital_description/README.md) model supplies the
visual setting; [hospital_arena](../hospital_arena/README.md) animates devices,
while robots publish their own poses, TF, and colored markers.

## Robot API and Missions

For a nonempty `robot_id`, interfaces are derived as follows:

| Entity | Name / type |
| --- | --- |
| Mission action server | `/hospital/<robot_id>/ward/execute_task`, `hospital_interfaces/action/ExecuteTask` |
| Pose topic | `/hospital/<robot_id>/pose`, `geometry_msgs/msg/PoseStamped` (transient-local) |
| Robot TF child frame | `<robot_id>_base_link`, under `pose_frame_id` (normally `world`) |
| Shared marker topic | `/hospital/robot_markers`, `visualization_msgs/msg/MarkerArray` (transient-local) |
| Fleet node name | `<robot_id>_server` |

Missions carry `value`, `target_location`, and a client-generated `task_id`.
The bundled protocols use fixed named rooms; `target_location` does not select
an arbitrary route. Feedback echoes the task ID, phase (QUEUED, NAVIGATING,
EXECUTING, DONE), and progress on `0..1`.

| Mission | Value | Implemented protocol |
| --- | --- | --- |
| `CLEAN_OR` | `0` | Visit the intervention OR; if dirty, fetch supplies via the storage and set CLEAN; return to parking. |
| `COLLECT_LAB` | `1` | Collect a simulated blood sample in the phlebotomy room, analyse it in the laboratory, and return. |
| `GUIDE_CT` | `2` | Greet and guide a patient through changing and CT, run the scan, lower the table, and return. |
| `HEART_ATTACK` | `3` | Guide phlebotomy and scanning room visits, analyse blood, and prepare the intervention OR if markers are abnormal. The CT visit simulates a patient-managed scan rather than commanding the scanner. |
| `SURGICAL_ASSIST` | `4` | Carry two simulated non-sterile waste objects from the intervention OR to the storage, then return. |

Door passage generally opens the door, changes room, and closes it behind the
robot. Each robot executes only one mission at a time; additional accepted
missions wait in a FIFO queue and start in acceptance order. Cancelling a
queued mission removes it from the queue (CANCELED). Timed waits are
interruptible by cancellation, node destruction, or rclcpp shutdown; discovery
and response waits can take longer to interrupt. Separate robots can execute
concurrently but share the same devices.
The demonstration protocols warn and continue for several rejected or failed
device operations, so mission success alone is not proof that every physical
precondition or nested operation succeeded.

## Usage in the Application

Follow the central [build](../hospital_bringup/README.md#build) and
[run](../hospital_bringup/README.md#run) instructions. Bringup starts the
configured fleet and logger together with the device servers needed by the
protocols. No additional robot or logger launch is required.

Dispatch missions from the panel's Ward Management section so task durations
are also available to the logger. For direct action-client integration, see
the central [command-line examples](../hospital_bringup/README.md#command-line-examples).
Use standard action cancellation to cancel missions; the panel offers Abort.
Avoid a second robot instance serving the same mission endpoint.

## Fleet and Configuration

The fleet roster is configured once as `robot_ids` under
`robot_server_node.ros__parameters` in
[`hospital_params.yaml`](../hospital_bringup/config/hospital_params.yaml).
The robot launch, activity logger, and RViz panel all read this list. Robots
are offset along the y-axis by roster position.

Add a robot ID once, then rebuild and relaunch:

```yaml
robot_server_node:
  ros__parameters:
    robot_ids: ["robot_1", "robot_2", "robot_3"]
```

The launch derives each robot's action, pose, and frame names from its ID.
Shared robot settings remain in the same `robot_server_node` section. Each
robot's waypoints are offset by 0.8 m per roster position along the y-axis;
there is no x-axis offset.

The change is launch/config only, so reinstall the bringup package and relaunch:

```bash
colcon build --packages-select hospital_bringup
source install/setup.bash
ros2 launch hospital_bringup bringup.launch.py
```

Robot parameters include parallel `room_names`, `room_x`, `room_y`, `room_yaw`
arrays; `parking_room`; `travel_duration`, `interaction_duration` (seconds);
`pose_publish_rate` (Hz); pose frame; and device action/service endpoints.
The pose topic, base frame, and mission action name are derived from each
`robot_id`.
Durations must lie in [0, 3600] s and the rate in [0.1, 1000] Hz; an
out-of-range value fails node construction.

The robot waits for a device action result for
`controller_timeout_factor * duration + controller_timeout_margin` seconds
(defaults 1.5 and 10 s). The duration is read from the controller's parameters
(`open_duration` / `close_duration`, `analysis_duration`,
`table_move_duration` (+ `scan_duration` for a scan)) on the nodes named by
`door_controller_node`, `lab_controller_node` and `ct_scanner_controller_node`
after a goal is accepted, so the timeout accounts for the values read at that
time. If they cannot be read, `door_fallback_timeout` (30 s),
`lab_fallback_timeout` (60 s), `scan_fallback_timeout` (60 s) or
`table_fallback_timeout` (30 s) is used.
The fleet launch adds `robot_id`, `robot_index`, and metre-valued
`pose_offset_x` / `pose_offset_y`. Offsets shift every waypoint, not the building
or logger regions; keep shifted poses inside the intended rooms.

## Activity Recording

`ActivityLoggerNode` detects room occupancy geometrically from each roster
robot's pose, independently of its mission implementation. It subscribes to
`/hospital/<id>/pose` with transient-local durability and reads XY coordinates
directly. Third-party robots must publish compatible poses in the same `world`
coordinate system; the logger does not transform arbitrary pose frames.

Room rectangles and the clinical corridor polygon come from the `activity_logger_node`
configuration. A region change must remain stable for `debounce_seconds`
(default `0.5`); poses outside configured rooms use `in_transit`.

The full application starts the logger. Recording starts disabled and is
toggled by the panel or, with the application running, the standard service:

```bash
ros2 service call /hospital/logging/set_recording std_srvs/srv/SetBool "{data: true}"
# Stop to finalize the session and JSON report
ros2 service call /hospital/logging/set_recording std_srvs/srv/SetBool "{data: false}"
```

`log_directory` defaults to `$HOME/hospital_logs` when empty. Each session writes:

```text
run_<YYYYMMDD_HHMMSS>/
    robot_1/
        rooms.csv
        tasks.csv
    run.json
```

There is one subdirectory per robot ID. Room re-entry creates a separate visit,
not an accumulated room total. Stopping closes the current visits and writes
the JSON report. Task records depend on START/END `TaskEvent` messages on
`/hospital/task_events`, normally emitted by the panel; the logger preserves
the panel's measured duration. A task sent only via `ros2 action send_goal`
does not automatically generate those events, so room recording can work while
task CSVs remain empty. External mission clients must publish matching events
to participate in task recording. At most 1024 STARTs awaiting their END are
kept; beyond that the oldest is forgotten and its task record falls back to the
END's duration for the start time.

## License

Copyright (c) 2026 Siemens Healthineers AG

SPDX-License-Identifier: GPL-3.0-or-later

The package's first-party code and configuration are licensed under the
[GNU General Public License, version 3 or later](LICENSE).
