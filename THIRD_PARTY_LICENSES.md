# Third-Party Licenses and Notices

The first-party code in the public Hospital Controller ROS 2 packages is licensed
under the GNU General Public License, version 3 or later (`GPL-3.0-or-later`).
Each public package contains the full license text in its own `LICENSE` file
(for example [`hospital_controller/LICENSE`](src/hospital_controller/LICENSE)).

This source release does **not** include third-party source code or libraries.
ROS 2, Qt, and their transitive dependencies are external dependencies, which
users install separately (e.g. via `apt` or `rosdep`). Each is used under its
own license. The 3D meshes and textures in `hospital_description/meshes/` are
licensed under `GPL-3.0-or-later` like the code (see the root
[`REUSE.toml`](REUSE.toml)). The included AI-generated assets are described
below.

This is a summary, not a complete license manifest. Anyone who distributes
binaries, container images, or other bundles that include these dependencies
must comply with those dependencies' licenses (notices, license texts, and,
where applicable, corresponding source).

The [combined CycloneDX SBOM](sbom/hospital-controller.cdx.json) inventories the
seven first-party packages and their direct dependencies, resolved to installed
APT package versions on the recorded build host. This does not mean these
dependencies are included in the source release. External licenses and
transitive dependencies remain unresolved;

---

## ROS 2 (Robot Operating System 2)

This project targets [**ROS 2 Jazzy Jalisco**](https://www.ros.org/). It
builds against and runs with the following ROS 2 packages, which are installed
separately:

- **Apache-2.0:** `rclcpp`, `rclcpp_action`, `rclcpp_components`,
  `rclpy`, `ament_index_cpp`, `ament_index_python`, `std_msgs`, `std_srvs`,
  `geometry_msgs`, `sensor_msgs`, `diagnostic_msgs`, `rcl_interfaces`,
  `visualization_msgs`, `builtin_interfaces`, `action_msgs`, `rosidl_*`,
  `ament_cmake`, `ament_cmake_gtest`, `ament_cmake_clang_format`,
  `ament_lint_*`, `launch`, and `launch_ros`.
  [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
- **BSD-family licenses:** `rviz2`, `rviz_common`, `pluginlib`, `tf2_ros`,
  `urdf`, `xacro`, `robot_state_publisher`, and `joint_state_publisher_gui`.

Copyright belongs to Open Source Robotics Foundation, Inc. and the respective
contributors. For exact terms, see the `package.xml` and license files of each
package (installed under `/opt/ros/jazzy/share/<package>/`).

RViz uses the OGRE 3D rendering engine (MIT License), supplied by the ROS
package `rviz_ogre_vendor`.

---

## Qt 5

The `hospital_rviz_panel` plugin uses the **Qt 5** modules Widgets, GUI, and
Core and is dynamically linked against them. Qt is not part of this source
release; on ROS 2 Jazzy / Ubuntu 24.04 it comes from the distribution's system
packages (`qtbase5-dev`, `libqt5widgets5t64`, ...), which RViz depends on.

Qt is copyright The Qt Company Ltd. and other contributors. The open-source
distribution is available under GNU LGPL v3 and GNU GPL v2 or v3. The LGPL v3
and GPL v3 options are compatible with this project's `GPL-3.0-or-later`
license when using GPL v3. See
[Qt 5 licensing](https://doc.qt.io/archives/qt-5.15/licensing.html).

---

## AI-Generated Assets

The following pictograms in `hospital_rviz_panel/resource/` were created with
generative AI tools:

| File | Generator |
| --- | --- |
| `pictogramm_open.png` | Google Generative AI (with a SynthID watermark) |
| `pictogramm_close.png` | Google Generative AI (with a SynthID watermark) |
| `pictogramm_stop.png` | Azure OpenAI ImageGen |
| `arrow_up.png` | Generative AI tool (not recorded) |
| `arrow_down.png` | Generative AI tool (not recorded) |

The REUSE metadata for these files is in the root [`REUSE.toml`](REUSE.toml).

They are distributed with this project as part of the RViz panel resources.

---

## Build / Lint Tooling

**colcon**, **ament_lint**, **ament_cmake_gtest**/**GoogleTest**,
**ament_cmake_clang_format**/**clang-format** (LLVM, Apache-2.0 with LLVM
exception), **flake8**, **pep257**, and **pytest** are used at build/test time
only and are not redistributed by this project. Each is available under its
own license from its upstream. The declared Python dependencies `python3-yaml`
(MIT) and `python3-numpy` (BSD-3-Clause; used by the source-only collision repair
utility) are installed separately and are not included in this source release.

