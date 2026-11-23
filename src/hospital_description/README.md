# hospital_description

The hospital's 3D geometry and articulated ROS 2 model: rooms, doors, equipment,
and the CT patient table. This asset package installs meshes, URDF/xacro, a
model-inspection launch file, and an RViz configuration. It does not contain
device controllers or a physics/navigation simulator.

The published assets are available only in a reduced form.
Teams taking part in the competition may obtain the original assets directly from Siemens Healthineers.

## Project Guide

See the [Hospital Controller project guide](../hospital_bringup/README.md) for
the introduction, architecture, full-workspace setup, and application launch.
This README describes the hospital model and its assets.

## Role in the System

[hospital_arena](../hospital_arena/README.md) expands this model and supplies
joint positions from live [hospital_controller](../hospital_controller/README.md)
telemetry. `robot_state_publisher` converts those joints into TF, and RViz's
RobotModel display renders the result.
[hospital_rviz_panel](../hospital_rviz_panel/README.md) commands the devices
through ROS interfaces, not by modifying the model.
[hospital_bringup](../hospital_bringup/README.md) connects these pieces in the
full application.

The model is rooted in `world`; positions and travel distances are in metres,
and revolute joint angles are in radians. Collision meshes describe geometry
only: their presence does not provide collision checking or prevent simulated
robots from teleporting through the scene.

## Package Contents

| Location | Purpose |
| --- | --- |
| [urdf/hospital.urdf.xacro](urdf/hospital.urdf.xacro) | Hospital links, door joints, CT lift/slide, and mimic joints. |
| [meshes/visual](meshes/visual) | Visual mesh assets used by the model. |
| [meshes/collision](meshes/collision) | Collision geometry. |
| [launch/display.launch.py](launch/display.launch.py) | Standalone model inspection with RViz and optional joint sliders. |
| [rviz/hospital.rviz](rviz/hospital.rviz) | Model-viewing RViz configuration. |

Articulated elements include hinged and sliding doors, `ct_lift`, and
`ct_table_slide`. The telescopic CT shrouds mimic the lift. The arena bridge
must use the same joint names, limits, and sign conventions as this model.
Some mesh transforms are baked into the exported assets; changing a joint
origin without accounting for the visual/collision offsets can misplace them.

Door link names match the current device IDs in the
[door naming table](../hospital_bringup/README.md#doors). Hinged joints append
`_hinge`; sliding joints append `_slide`. The door macros take separate `name`
and `mesh` parameters: `name` identifies the link/joint, while `mesh` selects the
existing visual and collision asset basenames. Legacy names inside mesh files
are asset metadata, not controller IDs, and are intentionally preserved.

CT meshes use matching `ct_` basenames in the visual and collision directories
(for example, `ct_gantry.gltf` and `ct_gantry.stl`). The CT links are named
`ct_scanner_<part>` (for example, `ct_scanner_gantry` and
`ct_scanner_tabletop`), so no link shares its name with a joint such as
`ct_shroud_01`. Alternate mesh packages that use this xacro must provide the
same basenames.

## Usage in the Application

Follow the central [build](../hospital_bringup/README.md#build) and
[run](../hospital_bringup/README.md#run) instructions. Bringup starts the arena
bridge, which expands this model and supplies live joint positions for RViz.
No separate model-viewer launch is needed for normal use.

Visual meshes are stored as glTF with external files (`.gltf`, `.bin`, and
base-color `<name>_tex<N>.png` textures), because RViz cannot read textures
embedded in a `.glb`. To update a visual mesh, export it from Blender as `.glb`
into `meshes/visual` with **+Y Up disabled** (RViz uses glTF coordinates as-is,
so a default Y-up export lies on its side and no longer matches the Z-up STL
collision mesh), run
`python3 scripts/unpack_glb_textures.py meshes/visual` from this package
directory, and then delete the `.glb`: binary glTF files are not kept in the
repository.

Furniture such as the hospital bed and the trolley (`trolly` link) is fixed to
`world`. The trolley's link frame is its footprint centre on the floor; to move
or turn it, edit the `xyz`/`rpy` of its `static_part` in the xacro and rebuild.

After modifying meshes or xacro, rebuild this package and relaunch the full
application. If joint names, limits, or directions change, update
[the arena mapping](../hospital_arena/README.md#joint-mapping) as well. Inspect
door motion, CT lift/slide, and mesh placement in the live RViz scene.

## License

Copyright (c) 2026 Siemens Healthineers AG

SPDX-License-Identifier: GPL-3.0-or-later

The package's code, configuration, and mesh assets are licensed under the
[GNU General Public License, version 3 or later](LICENSE). Meshes and
textures cannot carry a license header, so they are annotated in the root
[REUSE.toml](../../REUSE.toml).
