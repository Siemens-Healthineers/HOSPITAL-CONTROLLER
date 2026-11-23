# Contributing to Hospital Controller (ROS 2)

Thanks for your interest in this project.

**Maintenance notice:** Siemens Healthineers will not maintain this project.
Contributions and issues are not guaranteed to be reviewed or receive a
response.

## Project Description

Hospital Controller is a ROS 2 simulation of a hospital environment for the
Healthcare Challenge at RoboCup 2027. It provides simulated device controllers
(doors, CT scanner, blood analyser, operating room), protocol-driven robots, an
RViz operator panel, and a 3D hospital model.

### Technology Stack

* **Middleware**: ROS 2 Jazzy Jalisco
* **Languages**: C++20, Python 3.12
* **GUI**: RViz 2 plugin with Qt 5
* **Build**: colcon, ament_cmake, ament_python
* **OS**: Ubuntu 24.04 LTS

For more information about the project, see:

* [README.md](README.md)
* [Project guide](src/hospital_bringup/README.md)

## Project License

The public packages of this project are licensed under the
**GNU General Public License, version 3 or later** (`GPL-3.0-or-later`).
See the root [LICENSE](LICENSE) file; each package also contains a copy, for example
[src/hospital_bringup/LICENSE](src/hospital_bringup/LICENSE). The repository
follows the [REUSE specification](https://reuse.software/): license texts live
in the root [LICENSES/](LICENSES) directory, and files that cannot carry a
license header are annotated in the root [REUSE.toml](REUSE.toml).

By contributing to this project, you agree that your contributions will be
licensed under the same license.

## Getting Started

1. **Fork the repository** and clone it to your local machine.
2. **Set up your environment** as described in the
   [project guide](src/hospital_bringup/README.md).
3. **Create a feature branch** for your contribution.
4. **Make your changes** and test them.
5. **Submit a pull request** with a clear description of your changes.

## Development Process

### Coding Standards

* **C++**: C++20, Google style as defined by each package's `.clang-format`.
  `ament_lint_common` runs with `uncrustify` replaced by `clang-format`; check a
  package with, for example:

  ```bash
  ament_clang_format --config src/hospital_controller/.clang-format src/hospital_controller
  ```
* **Python**: PEP 8, checked by `flake8` and `pep257`.
* **License headers**: Every new source file must start with the same copyright
  and SPDX header as the existing files (use `#` instead of `//` in Python):

  ```cpp
  // Copyright (c) 2026 Siemens Healthineers AG
  // SPDX-License-Identifier: GPL-3.0-or-later
  ```

  Files that cannot carry a header (images, meshes) must be annotated in the
  root [REUSE.toml](REUSE.toml). Check the whole repository from its root with
  [`reuse`](https://reuse.software/), which CI also runs:

  ```bash
  pipx run reuse lint
  ```

* **Interfaces**: Changes to messages, services, or actions in
  `hospital_interfaces` affect all clients. Describe the impact in your pull
  request.

### Testing

Build and test from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
colcon test
colcon test-result --verbose
```

* Make sure existing functionality is not broken.
* Add unit tests for new features where applicable.
* Before opening a pull request, run the build and tests above on ROS 2
  Jazzy and check licensing compliance with `reuse lint`.

### Commit Messages

* Use clear, descriptive commit messages.
* Reference issue numbers when applicable.
* Use the format `[package] Brief description`, for example:

```text
[hospital_controller] Add timeout to CT scan action
[hospital_rviz_panel] Fix door button state after reconnect
```

### Pull Request Process

1. Update the documentation for any changed functionality.
2. Make sure your code follows the coding standards and passes the linters.
3. Test your changes in a ROS 2 Jazzy environment.
4. Give a clear description of the changes in your pull request.
5. Link to any relevant issues.

### Third-Party Content

Do not add third-party source code, meshes, images, or other assets.
## Contributor Agreement

By contributing to this project, you certify that:

1. The contribution was created in whole or in part by you and you have the
   right to submit it under the open source license indicated in the file; or
2. The contribution is based upon previous work that, to the best of your
   knowledge, is covered under an appropriate open source license and you have
   the right under that license to submit that work with modifications, whether
   created in whole or in part by you, under the same open source license
   (unless you are permitted to submit under a different license); or
3. The contribution was provided directly to you by some other person who
   certified (1), (2) or (3) and you have not modified it.
4. You understand and agree that this project and the contribution are public
   and that a record of the contribution (including all personal information
   you submit with it) is maintained indefinitely and may be redistributed
   consistent with this project or the open source license(s) involved.

## Contact

* **Issues**: Use the GitHub issue tracker.
* **Email**: Contact the maintainers listed in [Maintainer.md](Maintainer.md).

## Code of Conduct

All contributors are expected to follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## Security Vulnerabilities

Do **not** report security vulnerabilities in public issues. Follow the
instructions in [SECURITY.md](SECURITY.md).

## Thank You

Thank you for contributing to Hospital Controller and to the RoboCup
Healthcare Challenge!
