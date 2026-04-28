# ROS <-> STM32 RCU Integration

## Status

The legacy Nucleo serial bridge path has been removed from `Software/`.
The active hardware integration is the STM32H723 RCU UDP binary bridge.

## Active Runtime Path

```text
thor_policy_runner.py (Ros2RobotBridge)
  -> /robot_command
  -> rcu_udp_bridge.py
  -> UDP 7701 (Thor -> RCU)

RCU
  -> UDP 7700 (RCU -> Thor)
  -> rcu_udp_bridge.py
  -> /motor_can_feedback + /imu0 + /imu1
  -> motor_feedback_listener.py + robot_observation_bridge.py
  -> /robot_observation
  -> thor_policy_runner.py
```

## Canonical Files

- `Software/src/motor_control/motor_control/bridges/rcu_udp_bridge.py`
- `Software/src/motor_control/launch/rl_robot_launch.py`
- `Software/src/motor_control/launch/rcu_launch.py`
- `Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md`

## Network Defaults

- `rcu_ip`: `192.168.100.10`
- Thor host IP: `192.168.100.20/24`
- Telemetry input port: `7700`
- Command output port: `7701`
- Supervision output port: `7702`

## Build and Launch

```bash
source /opt/ros/jazzy/setup.bash
cd Software
colcon build --packages-select motor_control
source install/setup.bash

# Full RL-to-hardware pipeline
ros2 launch motor_control rl_robot_launch.py

# RCU bridge + observation only
ros2 launch motor_control rcu_launch.py
```

## Safety Services

```bash
# Enable motors
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: true}"

# Full e-stop (disable motors + assert PDU fault)
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: false}"

# Clear PDU fault
ros2 service call /rcu_pdu_fault std_srvs/srv/SetBool "{data: false}"
```

## Notes

- This document supersedes previous serial bridge instructions in `Software/`.
- For protocol-level packet details, use the handover file in `Charlie/STM32Cube/Tools/ROS2/`.
