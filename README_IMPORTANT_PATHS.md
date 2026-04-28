# Important Files and Entry Points

Use this index when you need to quickly find the right place to edit or run.

## Core Docs

- `README.md`: Main project overview.
- `README_PROJECT_MAP.md`: Top-level structure map.
- `README_QUICKSTART.md`: Startup order and run commands.
- `Software/TERMINAL_COMMANDS.md`: Command reference.
- `Software/ROS_STM32_INTEGRATION.md`: ROS <-> STM32 protocol/integration notes.

## Policy Runner and Hardware Integration

- `hardware/thor/thor_policy_runner.py`: Thor standing policy runtime script.
- `simulation/isaac/rl/interface/ros2_robot_bridge.py`: ROS2 bridge used by policy runner hooks.
- `simulation/isaac/rl/interface/robot_hardware_interface.py`: Hardware interface contract.

## ROS2 Launch and Nodes (motor_control)

- `Software/src/motor_control/launch/rl_robot_launch.py`: Main RL-to-hardware launch.
- `Software/src/motor_control/launch/single_motor_launch.py`: Single motor serial test.
- `Software/src/motor_control/launch/multi_motor_launch.py`: Multi-motor serial test.
- `Software/src/motor_control/launch/multi_state_launch.py`: Multi-state command test.

- `Software/src/motor_control/motor_control/bridges/robot_command_bridge.py`: RobotCommand -> packed motor CAN payload.
- `Software/src/motor_control/motor_control/bridges/ethernet_can_bridge.py`: UDP bridge to STM32, publishes feedback and IMU.
- `Software/src/motor_control/motor_control/bridges/serial_can_bridge.py`: Serial bridge alternative.
- `Software/src/motor_control/motor_control/bridges/robot_observation_bridge.py`: Combines motor feedback + IMU into RobotObservation.
- `Software/src/motor_control/motor_control/sensors/motor_feedback_listener.py`: Decodes packed feedback topic.

## Shared Configuration

- `simulation/isaac/configuration/joint_limits_config.json`: Source of truth for joint names and limits.
- `Software/config/json/motor_names.json`: Legacy motor-name list format.
- `Software/config/json/MotorParams.json`: Motor parameter configuration.
- `Software/config/json/multi_state_test_config.json`: Multi-state test sequence config.

## Simulation and RL

- `simulation/README.md`: Simulation section overview.
- `simulation/isaac/README.md`: Isaac environment overview.
- `simulation/isaac/rl/README.md`: RL workspace overview.
- `simulation/isaac/runtime/thor_policy_runner_testbench.py`: Policy-runner testbench harness.

## Firmware and Bridge Docs

- `Software/nucleo_can_bridge/docs/README.md`: Firmware docs root.
- `Software/nucleo_can_bridge/docs/explanations/QUICKSTART.md`: Firmware quickstart.
- `Software/nucleo_can_bridge/platformio.ini`: PlatformIO target/build configuration.
