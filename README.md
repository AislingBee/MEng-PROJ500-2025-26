# MEng-PROJ500-2025-26

Main repository for the PROJ500 humanoid robot project.

Project timeline: December 2025 to May 2026.

## Start Here

1. Read this file for the project map.
2. For simulation and Isaac workflow, read [simulation/README.md](simulation/README.md).
3. For ROS2 motor stack, read [Software/src/motor_test/README.md](Software/src/motor_test/README.md).
4. For STM32 Nucleo CAN bridge firmware/docs, read [Software/nucleo_can_bridge/docs/README.md](Software/nucleo_can_bridge/docs/README.md).

## Top-Level Repository Overview

### Core folders

- [simulation](simulation): Isaac Sim assets, tools, and RL workspace.
  - Further docs:
  - [simulation/README.md](simulation/README.md)
  - [simulation/docs/README.md](simulation/docs/README.md)
  - [simulation/isaac/README.md](simulation/isaac/README.md)
  - [simulation/isaac/rl/README.md](simulation/isaac/rl/README.md)

- [Software](Software): software stack for firmware, bridge tools, ROS2 motor test package, and configs.
  - ROS2 motor package:
  - [Software/src/motor_test/README.md](Software/src/motor_test/README.md)
  - STM32/bridge documentation:
  - [Software/nucleo_can_bridge/docs/README.md](Software/nucleo_can_bridge/docs/README.md)
  - Integration guide:
  - [Software/ROS_STM32_INTEGRATION.md](Software/ROS_STM32_INTEGRATION.md)

- [nucleo_can_bridge](nucleo_can_bridge): legacy or alternate Nucleo bridge workspace at repository root.

### Top-level files

- [README.md](README.md): this overview.
- [joint_limit_results.csv](joint_limit_results.csv): saved joint limit validation/test output.
- [thor_policy_runner.py](thor_policy_runner.py): policy runtime script at repo root.
- [power_shell_commands](power_shell_commands): command notes and helper snippets.

## Software Folder Details

Within [Software](Software):

- [Software/config](Software/config): shared config files (including JSON config used by launch files).
- [Software/nucleo](Software/nucleo): motor protocol notes/manual assets and experimental scripts.
- [Software/nucleo_can_bridge](Software/nucleo_can_bridge): STM32 firmware, bridge utilities, tests, and docs.
- [Software/src/motor_test](Software/src/motor_test): ROS2 package for command generation, bridges, listeners, and launch files.

## Important Readme and Documentation Index

### Simulation and RL

- [simulation/README.md](simulation/README.md)
- [simulation/docs/README.md](simulation/docs/README.md)
- [simulation/isaac/README.md](simulation/isaac/README.md)
- [simulation/isaac/rl/README.md](simulation/isaac/rl/README.md)

### Firmware and Bridge

- [Software/nucleo_can_bridge/docs/README.md](Software/nucleo_can_bridge/docs/README.md)
- [Software/nucleo_can_bridge/docs/explanations/QUICKSTART.md](Software/nucleo_can_bridge/docs/explanations/QUICKSTART.md)
- [Software/nucleo_can_bridge/docs/explanations/MOTOR_CONTROLLER_README.md](Software/nucleo_can_bridge/docs/explanations/MOTOR_CONTROLLER_README.md)
- [Software/nucleo_can_bridge/docs/explanations/ARCHITECTURE.md](Software/nucleo_can_bridge/docs/explanations/ARCHITECTURE.md)
- [Software/ROS_STM32_INTEGRATION.md](Software/ROS_STM32_INTEGRATION.md)

### ROS2 Motor Test

- [Software/src/motor_test/README.md](Software/src/motor_test/README.md)
- Launch files are in [Software/src/motor_test/launch](Software/src/motor_test/launch).
- Test and bridge nodes are in [Software/src/motor_test/motor_test](Software/src/motor_test/motor_test).

## Contribution Rules

- Do not push directly to main.
- Use feature branches and open a pull request.
- Include a clear summary of changes and test steps in each PR.
- Merge to main only after review.
- Delete merged branches.

## Commit and PR Conventions

- Use descriptive commit titles (example: Add ROS2 joint command bridge).
- Include test evidence in PR descriptions.
- Provide exact run steps and expected output for reviewers.

Example test flow:

```bash
cd Software/src/motor_test
colcon build --packages-select motor_test
source install/setup.bash
ros2 launch motor_test multi_state_launch.py
```

## Team

- Mechanical Lead (Project Lead): Brendan
- Electrical Lead: Charlie
- Simulation Lead: Joe
- Software/ROS Lead: Ash
