# Software
This folder contains all the ROS2 software for communicating with the various comoputers and external sensors.

## Folder Contents:

<img width="759" height="343" alt="ros2 motor test 1 drawio" src="https://github.com/user-attachments/assets/6acf4cdb-a618-4b3c-b4a8-d8a31d1f741e" />

## Launch Files:
### Main launch entrypoints

- `rl_robot_launch.py`: full RL-to-hardware pipeline (command bridge, transport bridge, feedback listener, observation bridge).
- `single_motor_launch.py`: single motor step-test pipeline.
- `multi_motor_launch.py`: multi-motor step-test pipeline.
- `multi_state_launch.py`: configurable multi-state command test.

Params that may be changed as needed:

- serial/UDP transport settings (port, IP, baud)
- joint/CAN mapping settings
- gains (`kp`, `kd`) and test profile settings

Params that may be changed as needed:

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

For additional launch examples and runtime commands, see `Software/TERMINAL_COMMANDS.md`.
