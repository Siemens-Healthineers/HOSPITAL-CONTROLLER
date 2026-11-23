# Security Policy

**Maintenance notice:** Siemens Healthineers will not maintain this project.
Security reports are not guaranteed to receive a response or a fix.

## Scope

Hospital Controller is a simulation and training environment for the
Healthcare Challenge at RoboCup 2027. It does not control real medical devices
or hardware, and it does not process patient data. All clinical processes,
devices, and radiation are simulated.

Because of this, the project has no dedicated security process. Reports are
still welcome, but there is no guarantee they will be reviewed.

Do not use this software to control real hospital equipment or in
safety-critical environments.

## How To Report a Vulnerability

**Do not report security vulnerabilities through public issues, discussions,
or pull requests.**

Send reports by email to the project maintainers at
[robocup@siemens-healthineers.com](mailto:robocup@siemens-healthineers.com).
This channel is not encrypted, so do not include confidential information.

Include as much of the following as you can:

* **Type of issue**: for example a crash, denial of service, or unsafe input handling
* **Affected package(s)**: for example `hospital_controller` or `hospital_rviz_panel`
* **Affected version**: tag, branch, or commit
* **Reproduction steps**: including the launch command and parameters used
* **Impact**: what an attacker could achieve
* **Suggested fix**: optional

## Thank You

Thank you for helping to improve this project and for disclosing issues
responsibly.
