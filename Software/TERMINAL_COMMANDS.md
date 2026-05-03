# Terminal Commands Reference

All commands assume repo root: `c:\git\MEng-PROJ500-2025-26`

## 1. ROS 2 Build (motor_control)

```bash
source /opt/ros/jazzy/setup.bash
cd Software
colcon build --packages-select motor_control
source install/setup.bash
```

## 2. Launch Files (RCU Path)

```bash
# Full 12-motor one-command launch (RCU stack + startup ramp + standing hold + policy)
ros2 launch motor_control thor_12_motor_pipeline_launch.py

# RL-ready pipeline only (run startup_then_policy_runner.py manually in a second terminal)
ros2 launch motor_control rl_robot_launch.py

# Minimal RCU bridge bring-up
ros2 launch motor_control rcu_launch.py
```

> **Note:** `thor_12_motor_pipeline_launch.py` launches `startup_then_policy_runner.py` which
> handles ramp → hold → policy in one process. `PYTHONPATH` is set automatically so `simulation`
> is importable. No extra export needed.

## 3. Runtime Inspection

```bash
ros2 topic list
ros2 topic echo /robot_command
ros2 topic echo /motor_can_feedback
ros2 topic echo /robot_observation
ros2 topic hz /imu0
ros2 node list
```

## 4. Safety Services

```bash
# Enable all motors
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: true}"

# Full e-stop (disable + PDU fault assert)
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: false}"

# Assert PDU fault
ros2 service call /rcu_pdu_fault std_srvs/srv/SetBool "{data: true}"

# Clear PDU fault
ros2 service call /rcu_pdu_fault std_srvs/srv/SetBool "{data: false}"
```

## 5. Network Setup (Thor Host)

```bash
sudo ip addr add 192.168.100.20/24 dev eth0
sudo ip link set eth0 up
```

## 6. Troubleshooting Quick Checks

```bash
# Confirm RCU is reachable
ping 192.168.100.10

# Check IMU stream rate
ros2 topic hz /imu0

# Check motor feedback stream
ros2 topic echo /motor_can_feedback --no-arr
```

## 7. Reference

- `Software/src/motor_control/README.md`
- `Software/ROS_STM32_INTEGRATION.md`
- `Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md`
