# Software
This folder contains the ROS2 software used to communicate with hardware controllers and sensors.

## Folder Contents:

<img width="759" height="343" alt="ros2 motor test 1 drawio" src="https://github.com/user-attachments/assets/6acf4cdb-a618-4b3c-b4a8-d8a31d1f741e" />

## Launch Files:
### Main launch entrypoints

- `rl_robot_launch.py`: full RL-to-hardware pipeline using the STM32 RCU UDP bridge.
- `rcu_launch.py`: minimal RCU-focused launch entrypoint for bring-up and diagnostics.

### New STM32 RCU addition

The project now supports direct UDP binary communication with the STM32H723 RCU.

- Transport node: `motor_control/bridges/rcu_udp_bridge.py`
- Command topic: `/robot_command`
- Feedback topic: `/motor_can_feedback`
- IMU topics: `/imu0` and `/imu1`
- Observation output: `/robot_observation`

Default RCU network settings:

- `rcu_ip`: `192.168.100.10`
- `rcu_cmd_port`: `7701`
- `telem_port`: `7700`
- `ctrl_mode`: `1` (CSP phase)

Reference protocol and handover document:

- `Charlie/STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md`

Params that may be changed as needed:

- UDP transport settings (IP and ports)
- joint/CAN mapping settings
- gains (`kp`, `kd`) and test profile settings

Safety-critical controls exposed by the RCU bridge:

- `/rcu_motor_estop` (`std_srvs/SetBool`)
	- `true`: enable motors
	- `false`: full e-stop (motor disable plus PDU fault assert)
- `/rcu_pdu_fault` (`std_srvs/SetBool`)
	- `true`: assert PDU fault
	- `false`: clear PDU fault

## How to Run ROS2:

```bash
source /opt/ros/jazzy/setup.bash
```

```
cd Software
colcon build --packages-select motor_control
source install/setup.bash
ros2 launch motor_control rl_robot_launch.py
```

Optional RCU-only launch:

```bash
ros2 launch motor_control rcu_launch.py
```

For additional launch examples and runtime commands, see `Software/TERMINAL_COMMANDS.md`.
