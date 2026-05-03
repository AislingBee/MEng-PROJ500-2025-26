# Quickstart (Linux + ROS2 + Policy Runner)

This guide gives a practical order for bringing up the software stack and the Thor policy runner.

## 1) Source ROS2, build, and source workspace

From repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd Software
colcon build --packages-select motor_control --symlink-install
source install/setup.bash
```

## 2) Launch ROS hardware pipeline

### Option A — One command (recommended)

Starts RCU stack + policy runner together, with auto_enable and startup gate:

```bash
ros2 launch motor_control thor_12_motor_pipeline_launch.py
```

### Option B — Two terminals (manual control)

Terminal A — RCU stack:
```bash
ros2 launch motor_control rl_robot_launch.py
```

Terminal B — Policy runner:
```bash
source /opt/ros/jazzy/setup.bash
cd Software
source install/setup.bash
cd ..
export PYTHONPATH=$(pwd):$PYTHONPATH
python3 hardware/thor/thor_policy_runner.py
```

This starts:
- Command bridge (`/robot_command` -> `/motor_can_tx`)
- Ethernet bridge (STM32 comms, motor feedback, STM32 IMU passthrough)
- Motor feedback listener (`/motor_can_feedback` -> `/motor_feedback`)
- Robot observation bridge (`/motor_feedback` + `/imu0` -> `/robot_observation`)

## 3) Runtime checks

```bash
ros2 topic list
ros2 topic echo /robot_observation
ros2 topic echo /imu0
ros2 topic echo /robot_command
```

## Notes

- Joint names are sourced from `simulation/isaac/configuration/joint_limits_config.json`.
- Per-joint CAN IDs are expected to map by joint order (first joint -> ID 1).
