# Thor Motor Control — Hardware Testing Guide

Step-by-step guide for bringing up and testing the 12-motor Thor hardware stack,
from individual motor validation through to full policy deployment.

---

## 0. Connect and SSH into the Jetson

```bash
ssh jetson@192.168.100.20
```

> If the Jetson hostname is not resolved, use the IP directly.
> Default credentials depend on your setup — check with the team if unsure.

### Network check (confirm RCU is reachable)

```bash
ping -c 3 192.168.100.10
```

If ping fails, set the network interface manually:

```bash
sudo ip addr add 192.168.100.20/24 dev eth0
sudo ip link set eth0 up
ping -c 3 192.168.100.10
```

---

## 1. Pull latest code and build

```bash
cd ~/git/MEng-PROJ500-2025-26
git fetch origin
git pull origin main

source /opt/ros/jazzy/setup.bash
cd Software
colcon build --packages-select motor_control --symlink-install
source install/setup.bash
cd ..
```

> Run `source install/setup.bash` in **every new terminal** before using `ros2` commands.

---

## 2. STAGE A — Verify RCU connection (no motors commanded)

Open **one terminal**.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/git/MEng-PROJ500-2025-26/Software && source install/setup.bash

ros2 launch motor_control rcu_launch.py \
    auto_enable:=False \
    active_motor_ids:="[1,2,3,4,5,6,7,8,9,10,11,12]" \
    left_bus_motor_ids:="[1,3,5,7,9,11]" \
    right_bus_motor_ids:="[2,4,6,8,10,12]" \
    scan_motor_can_ids:=True
```

**Expected output:**
- `rcu_udp_bridge ready: RCU=192.168.100.10`
- `CAN ID scan enabled`
- `OBSERVED CAN IDs` log lines listing all 12 joints once motors are powered

**Check topics in a second terminal:**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/git/MEng-PROJ500-2025-26/Software && source install/setup.bash

ros2 topic list
ros2 topic hz /imu0
ros2 topic hz /motor_can_feedback
ros2 topic echo /motor_can_feedback --no-arr
```

Stop with **Ctrl+C** before proceeding.

---

## 3. STAGE B — IMU frame validation

With the RCU stack running (from Stage A or a fresh launch):

```bash
ros2 topic echo /imu0_remapped
```

With the robot **upright and stationary**, verify:

| Field | Expected value |
|---|---|
| `linear_acceleration.x` | ≈ 0.0 |
| `linear_acceleration.y` | ≈ 0.0 |
| `linear_acceleration.z` | ≈ −9.81 |
| `angular_velocity.*` | ≈ 0.0, 0.0, 0.0 |

If the gravity vector is wrong (e.g. `[0, 0, +9.81]` or on the wrong axis) stop and
report to the simulation team — do not proceed to motor testing.

---

## 4. STAGE C — Sequential motor zero (one motor at a time)

This automatically ramps each motor to `0.0 rad` in CAN ID order (1 → 12),
holding already-zeroed motors at zero throughout.

**Terminal A — RCU stack:**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/git/MEng-PROJ500-2025-26/Software && source install/setup.bash

ros2 launch motor_control rcu_launch.py \
    auto_enable:=True \
    active_motor_ids:="[1,2,3,4,5,6,7,8,9,10,11,12]" \
    left_bus_motor_ids:="[1,3,5,7,9,11]" \
    right_bus_motor_ids:="[2,4,6,8,10,12]" \
    scan_motor_can_ids:=True
```

**Terminal B — Sequential zero node:**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/git/MEng-PROJ500-2025-26/Software && source install/setup.bash

ros2 run motor_control sequential_motor_zero.py
```

**Default parameters** (override with `--ros-args -p name:=value`):

| Parameter | Default | Description |
|---|---|---|
| `ramp_time_s` | `4.0` | Seconds to ramp each motor to zero |
| `hold_time_s` | `1.0` | Seconds to hold before next motor |
| `kp` | `20.0` | Position gain (Nm/rad) |
| `kd` | `1.5` | Velocity gain (Nm·s/rad) |
| `rate_hz` | `200.0` | Command publish rate |

Example with slower ramp for first test:

```bash
ros2 run motor_control sequential_motor_zero.py \
    --ros-args -p ramp_time_s:=6.0 -p kp:=15.0 -p hold_time_s:=2.0
```

**Watch each motor:**
- It should move gently to `0.0 rad` with no sudden snap
- The already-zeroed motors should remain still
- Check for any unusual sounds or resistance

Stop Terminal B with **Ctrl+C** once all 12 are confirmed at zero.
Stop Terminal A with **Ctrl+C**.

---

## 5. STAGE D — Startup ramp to standing pose (no policy)

This verifies the robot can ramp from `0.0 rad` to the standing pose and hold it.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/git/MEng-PROJ500-2025-26/Software && source install/setup.bash

ros2 launch motor_control thor_12_motor_pipeline_launch.py
```

**Keyboard controls once running:**

| Key | Action |
|---|---|
| `Enter` or `p` | Advance from STANDING_HOLD to policy |
| `h` | Return to STANDING_HOLD |
| `x` or Ctrl+C | Safe exit (sends final standing pose first) |

**Expected sequence:**
1. All 12 CAN IDs appear online in the log
2. `Ros2RobotBridge ready` log line appears
3. Robot ramps to standing pose over ~8 s
4. `Startup ramp complete. Entering STANDING_HOLD.` printed
5. Robot holds the standing pose stably

**Do NOT press `p` yet** — just verify the standing hold is stable.

Stop with **`x`** or **Ctrl+C**.

---

## 6. STAGE E — Full pipeline (standing ramp → hold → policy)

Only attempt this after Stage D is confirmed stable.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/git/MEng-PROJ500-2025-26/Software && source install/setup.bash

ros2 launch motor_control thor_12_motor_pipeline_launch.py
```

1. Wait for `Startup ramp complete. Entering STANDING_HOLD.`
2. Visually confirm the robot is stable in the standing pose
3. Press **`Enter`** or **`p`** to hand over to the policy
4. Monitor the `/robot_observation` topic in a second terminal:

```bash
ros2 topic echo /robot_observation
```

Stop with **`x`** or **Ctrl+C** at any time.

---

## 7. Runtime inspection (any stage)

Open a spare terminal alongside any running stage:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/git/MEng-PROJ500-2025-26/Software && source install/setup.bash

# All active topics
ros2 topic list

# Message rates
ros2 topic hz /imu0
ros2 topic hz /imu0_remapped
ros2 topic hz /motor_can_feedback
ros2 topic hz /robot_observation
ros2 topic hz /robot_command

# Message content
ros2 topic echo /robot_observation
ros2 topic echo /imu0_remapped
ros2 topic echo /motor_can_feedback --no-arr

# Active nodes
ros2 node list
```

---

## 8. Emergency stop

```bash
# Software e-stop — disables all motors immediately
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: false}"

# Assert PDU fault (full power cut)
ros2 service call /rcu_pdu_fault std_srvs/srv/SetBool "{data: true}"
```

> Always have the physical e-stop within reach during any motor test.

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ping 192.168.100.10` fails | Wrong subnet | Re-run `sudo ip addr add 192.168.100.20/24 dev eth0` |
| `No module named 'simulation'` | PYTHONPATH not set | Use `thor_12_motor_pipeline_launch.py` (sets PYTHONPATH automatically) |
| `Startup gate blocking TX: online=[]` | Motors not powered or CAN not connected | Check PDU, CAN cables, motor IDs |
| `Joint target exceeds policy contract joint limits` | Encoder noise at zero start | Expected — fixed in startup ramp (no limit check during ramp) |
| `TimeoutError: No RobotObservation received` | `robot_observation_bridge` not receiving IMU or feedback | Check `/imu0_remapped` and `/motor_feedback` are publishing |
| `imu_publisher` exit code 1 on shutdown | ROS2 context already shut down | Harmless — ignore |

---

## Reference

- Launch files: `Software/src/motor_control/launch/LAUNCH_FILES_GUIDE.md`
- Network and build commands: `Software/TERMINAL_COMMANDS.md`
- Startup checklist: `hardware/thor/LINUX_STARTUP_CHECKLIST.md`
- Joint limits: `simulation/isaac/configuration/joint_limits_config.json`
- Standing pose targets: `simulation/isaac/configuration/standing_pose.py`
- CAN ID → joint mapping: `Software/src/motor_control/motor_control/rcu_protocol.py`
