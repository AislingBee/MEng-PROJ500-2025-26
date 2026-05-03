# THOR Linux Startup Checklist (RL Policy Runner)

Use this checklist when bringing up the full THOR RL-to-hardware loop on Linux.

## 0. Preflight (one-time or after reboot)

- [ ] Thor host and RCU are on the same subnet.
- [ ] Expected network defaults:
  - RCU IP: `192.168.100.10`
  - Thor host IP: `192.168.100.20/24`
  - Telemetry UDP: `7700`
  - Command UDP: `7701`
- [ ] Robot is in a safe stance with mechanical clearance.
- [ ] E-stop path is physically reachable.

Optional quick network check:

```bash
ping -c 3 192.168.100.10
```

## 1. Build and source ROS workspace

From repo root:

```bash
source /opt/ros/jazzy/setup.bash
cd Software
colcon build --packages-select motor_control --symlink-install
source install/setup.bash
```

## 2. Launch hardware ROS pipeline (Terminal A)

```bash
source /opt/ros/jazzy/setup.bash
cd Software
source install/setup.bash
ros2 launch motor_control rl_robot_launch.py
```

This should start:
- `rcu_udp_bridge`
- `motor_feedback_listener`
- `robot_observation_bridge`

## 3. Verify topics are alive (Terminal B)

```bash
source /opt/ros/jazzy/setup.bash
cd Software
source install/setup.bash

ros2 topic list | grep -E "robot_command|robot_observation|motor_can_feedback|imu0"
ros2 topic hz /robot_observation
ros2 topic hz /imu0
```

Expected:
- `/robot_observation` and `/imu0` publish steadily.
- No repeated timeout/errors from bridge nodes.

## 4. Start policy runner (Terminal C)

From repo root:

```bash
source /opt/ros/jazzy/setup.bash
cd Software
source install/setup.bash
cd ..
python3 hardware/thor/thor_policy_runner.py
```

Policy runner defaults:
- Command topic: `/robot_command`
- Observation topic: `/robot_observation`
- Node: `thor_policy_runner`
- Policy file: `exports/standing_policy.pt`

## 5. Runtime checks

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd Software
source install/setup.bash

ros2 topic hz /robot_command
ros2 topic echo /robot_command --once
ros2 topic echo /robot_observation --once
```

Expected:
- `/robot_command` publishes continuously while runner is active.
- Observation messages update and contain sensible values.

## 6. Safe stop

1. Stop policy runner first (`Ctrl+C` in Terminal C).
2. Stop launch pipeline (`Ctrl+C` in Terminal A).
3. Confirm command stream stops:

```bash
ros2 topic hz /robot_command
```

## 7. Fault recovery shortcuts

Use if needed after startup failures:

```bash
source /opt/ros/jazzy/setup.bash
cd Software
source install/setup.bash

# Enable motors
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: true}"

# Full e-stop (disable motors + assert PDU fault)
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: false}"

# Clear PDU fault
ros2 service call /rcu_pdu_fault std_srvs/srv/SetBool "{data: false}"
```

## 8. Common issues

- `ROS2 bridge dependencies are unavailable`:
  - Ensure ROS2 and `Software/install/setup.bash` are sourced in that terminal.
- No `/robot_observation`:
  - Check `rcu_udp_bridge` telemetry, RCU IP, and UDP port routing.
- Policy file load error:
  - Confirm `exports/standing_policy.pt` exists and is a deployable actor module (not raw training checkpoint dict).
