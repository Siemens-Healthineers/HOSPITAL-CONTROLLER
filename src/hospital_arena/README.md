# hospital_arena

The live visualization bridge between Hospital Controller telemetry and the
hospital's articulated 3D model. Its C++ executable `joint_bridge_node` converts
device percentages into `sensor_msgs/msg/JointState` positions. It does not
simulate devices, accept device commands, or implement robot navigation.

## Project Guide

See the [Hospital Controller project guide](../hospital_bringup/README.md) for
the introduction, architecture, full-workspace setup, and application launch.
This README describes the visualization bridge in detail.

## Role in the System

```text
hospital_controller -> device state topics -> joint_bridge_node -> /joint_states
hospital_description -> xacro model -> robot_state_publisher -> TF -> RViz RobotModel
```

[hospital_controller](../hospital_controller/README.md) owns device state using
[hospital_interfaces](../hospital_interfaces/README.md).
[hospital_description](../hospital_description/README.md) owns geometry and joint
definitions. [hospital_rviz_panel](../hospital_rviz_panel/README.md) sends commands
to the controllers and displays the scene in RViz.
[hospital_bringup](../hospital_bringup/README.md) starts the bridge and RViz
together when `use_rviz:=true`.

## Inputs and Outputs

| Direction | Topic | Type / purpose |
| --- | --- | --- |
| Subscribe | `/hospital/doors/states` | `hospital_interfaces/msg/DoorStateArray`: opening of every door (latched). |
| Subscribe | `/hospital/ct/table/state` | `hospital_interfaces/msg/CtTableState`: vertical table position (latched). |
| Subscribe | `/hospital/ct/scan/state` | `hospital_interfaces/msg/CtScannerState`: horizontal scan-cycle progress (latched). |
| Publish | `/joint_states` | `sensor_msgs/msg/JointState`: door and CT joint positions. |

The accompanying `robot_state_publisher` publishes the expanded robot description
and TF transforms. The hospital model is rooted in `world`. Laboratory and
operating-room status are shown by the panel, not converted to joints here.
Robot poses and colored markers are published separately by
[hospital_robot](../hospital_robot/README.md).

## Joint Mapping

[src/joint_bridge_node.cpp](src/joint_bridge_node.cpp) contains the mapping:

- Ten hinged doors use `<device_id>_hinge`. Their meshes are baked open, so joint position `0` is open and the signed quarter-turn limit is closed.
- `intervention_or_1_slide` and `scanning_room_1_slide` are sliding doors with open travel of `1.2588 m` and `1.71951 m` respectively.
- `ct_lift` maps BOTTOM (`0` percent) to `-0.479147 m` and TOP (`100` percent) to `0 m`.
- `ct_table_slide` maps scanner progress `0 -> 50 -> 100` percent to `0 -> 1.8 -> 0 m` insertion.

The bridge interpolates door positions and clamps received percentages to
`0..100`. CT shroud joints mimic `ct_lift` in the URDF and are therefore not
published independently. Before telemetry arrives, the bridge seeds doors
closed and the CT table at BOTTOM with the tabletop out.

Mappings are compiled into the bridge, not configured by `door_ids`. When
changing device names, joint names, hinge directions, or travel limits, update
both this source file and the model, plus the shared device configuration.
The bridge subscribes with transient-local durability to the aggregated door
snapshot and the CT state topics. A late-starting bridge receives the latest
latched states and updates the model without waiting for another movement.

## Usage and Configuration

Follow the central [build](../hospital_bringup/README.md#build) and
[run](../hospital_bringup/README.md#run) instructions. System bringup starts the
bridge and `robot_state_publisher` with RViz, the model, and the controllers.
There is no need to launch the bridge separately for normal operation.

RViz uses a RobotModel display on `/robot_description` and fixed frame `world`.
Avoid another joint publisher driving the same model, such as the description
package's joint sliders, while the live bridge is running.

`joint_bridge_node.ros__parameters.publish_rate` defaults to `30.0 Hz`; the
effective timer rate is at least `1 Hz`. This is the visualization refresh rate,
not a controller movement rate. Add that node section to the shared parameter
file to configure it through bringup. Geometry and mappings remain compiled
into the bridge and must agree with `hospital_description`.

## License

Copyright (c) 2026 Siemens Healthineers AG

SPDX-License-Identifier: GPL-3.0-or-later

The package's first-party code and configuration are licensed under the
[GNU General Public License, version 3 or later](LICENSE).
