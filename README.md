# MEng-PROJ500-2025-26

Main repository for the PROJ500 humanoid robot project.

Project timeline: December 2025 to May 2026.

## Start Here

1. Read this file for the project map.
2. Read [README_PROJECT_MAP.md](README_PROJECT_MAP.md) for a top-level file/folder map.
3. Read [README_IMPORTANT_PATHS.md](README_IMPORTANT_PATHS.md) for key entrypoints and files.
4. Read [README_QUICKSTART.md](README_QUICKSTART.md) for launch order and runtime checks.
5. For simulation and Isaac workflow, read [simulation/README.md](simulation/README.md).
6. For ROS2 motor stack, read [Software/src/motor_control/README.md](Software/src/motor_control/README.md).
7. For the RCU UDP protocol handover, read [Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md](Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md).

## Top-Level Repository Overview

## New STM32 Addition (RCU UDP Bridge)

The current hardware path now supports a direct STM32 RCU UDP binary bridge.

- New bridge node: [Software/src/motor_control/motor_control/bridges/rcu_udp_bridge.py](Software/src/motor_control/motor_control/bridges/rcu_udp_bridge.py)
- RCU launch entrypoint: [Software/src/motor_control/launch/rcu_launch.py](Software/src/motor_control/launch/rcu_launch.py)
- Full RL launch (with RCU transport): [Software/src/motor_control/launch/rl_robot_launch.py](Software/src/motor_control/launch/rl_robot_launch.py)
- Protocol reference and handover: [Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md](Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md)

RCU network defaults:

- RCU IP: `192.168.100.10`
- Thor host IP: `192.168.100.20/24`
- Telemetry in: UDP `7700`
- Command out: UDP `7701`
- Supervision out: UDP `7702`

The Software Nucleo bridge path has been removed; RCU UDP is now the primary integration path.

### Core folders

- [simulation](simulation): Isaac Sim assets, tools, and RL workspace.
  - Further docs:
  - [simulation/README.md](simulation/README.md)
  - [simulation/docs/README.md](simulation/docs/README.md)
  - [simulation/isaac/README.md](simulation/isaac/README.md)
  - [simulation/isaac/rl/README.md](simulation/isaac/rl/README.md)

- [Software](Software): software stack for firmware, bridge tools, ROS2 motor test package, and configs.
  - ROS2 motor package:
  - [Software/src/motor_control/README.md](Software/src/motor_control/README.md)
  - Integration guide:
  - [Software/ROS_STM32_INTEGRATION.md](Software/ROS_STM32_INTEGRATION.md)

### Top-level files

- [README.md](README.md): this overview.
- [joint_limit_results.csv](joint_limit_results.csv): saved joint limit validation/test output.
- [hardware/thor/thor_policy_runner.py](hardware/thor/thor_policy_runner.py): policy runtime script.
- [power_shell_commands](power_shell_commands): command notes and helper snippets.

## Software Folder Details

Within [Software](Software):

- [Software/config](Software/config): shared config files (including JSON config used by launch files).
- [Software/nucleo](Software/nucleo): motor protocol notes/manual assets and experimental scripts.
- [Software/src/motor_control](Software/src/motor_control): ROS2 package for command generation, bridges, listeners, and launch files.

## Important Readme and Documentation Index

### Simulation and RL

- [simulation/README.md](simulation/README.md)
- [simulation/docs/README.md](simulation/docs/README.md)
- [simulation/isaac/README.md](simulation/isaac/README.md)
- [simulation/isaac/rl/README.md](simulation/isaac/rl/README.md)

### Firmware and Bridge

- [Software/ROS_STM32_INTEGRATION.md](Software/ROS_STM32_INTEGRATION.md)
- [Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md](Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md)

### ROS2 Motor Test

- [Software/src/motor_control/README.md](Software/src/motor_control/README.md)
- Launch files are in [Software/src/motor_control/launch](Software/src/motor_control/launch).
- Test and bridge nodes are in [Software/src/motor_control/motor_control](Software/src/motor_control/motor_control).

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
cd Software/src/motor_control
colcon build --packages-select motor_control
source install/setup.bash
ros2 launch motor_control rl_robot_launch.py
```

## Team

- Mechanical Lead (Project Lead): Brendan
- Electrical Lead: Charlie
- Simulation Lead: Joe
- Software/ROS Lead: Ash
