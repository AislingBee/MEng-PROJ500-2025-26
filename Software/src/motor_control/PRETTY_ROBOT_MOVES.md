# Pretty Robot Moves

Suspended-leg choreography runner for Thor. Smoothly interpolates between named
joint-space poses using cosine ease-in/ease-out PD control. No RL policy.

---

## 1. Build (run once after any code change)

```bash
cd ~/ros2_ws          # or wherever the workspace root is
colcon build --packages-select motor_control
source Software/install/setup.bash
```

---

## 2. Run

### Full launch — recommended (starts RCU bridge automatically)

```bash
ros2 launch motor_control pretty_robot_moves_launch.py
```

### With options

```bash
# Choose a sequence
ros2 launch motor_control pretty_robot_moves_launch.py sequence:=taps
ros2 launch motor_control pretty_robot_moves_launch.py sequence:=shapes
ros2 launch motor_control pretty_robot_moves_launch.py sequence:=kicks

# Adjust speed (default 0.3 — slow and safe)
ros2 launch motor_control pretty_robot_moves_launch.py speed:=0.5

# Play once then stop
ros2 launch motor_control pretty_robot_moves_launch.py no_loop:=True

# Custom RCU IP
ros2 launch motor_control pretty_robot_moves_launch.py rcu_ip:=192.168.100.10

# Combine options
ros2 launch motor_control pretty_robot_moves_launch.py sequence:=taps speed:=0.6 no_loop:=True
```

### Script only — if the RCU bridge is already running separately

```bash
ros2 run motor_control pretty_robot_moves.py
ros2 run motor_control pretty_robot_moves.py --sequence taps --speed 0.5
```

### Direct Python (no colcon build required)

```bash
cd ~/ros2_ws
source Software/install/setup.bash
python3 Software/src/motor_control/motor_control/pretty_robot_moves.py --sequence kicks
```

---

## 3. Available sequences

| Name | Description |
|------|-------------|
| `kicks` | Stand → extend → kick left → kick right → stand *(default)* |
| `taps` | Stand → 6 rapid alternating ankle-point taps → stand |
| `shapes` | Stand → knees up → extend → legs back → stand |

---

## 4. Keyboard controls (while running — type command then press Enter)

| Key | Action |
|-----|--------|
| `q` | Safe stop — ramps back to standing then exits |
| `p` | Pause / resume |
| `n` | Skip to next move immediately |
| `l` | Toggle looping on/off |
| `+` | Increase speed by 10 % |
| `-` | Decrease speed by 10 % |

---

## 5. All launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `sequence` | `kicks` | Which sequence to run (`kicks`, `taps`, `shapes`) |
| `speed` | `0.3` | Global speed multiplier (1.0 = as written, 0.3 = 3× slower) |
| `no_loop` | `False` | Set `True` to play once then exit |
| `rcu_ip` | `192.168.100.10` | IP address of the RCU |
| `kp_scale` | `0.20` | Stiffness gain scale (keep ≤ 0.25) |
| `kd_scale` | `1.00` | Damping gain scale |
| `loop_hz` | `66.67` | Control loop frequency (Hz) |
| `max_position_error_rad` | `0.90` | Abort threshold for joint tracking error (rad) |
| `auto_enable` | `True` | Auto-enable motors on startup |
| `device` | `cpu` | Torch device |

---

## 6. Safety

- **Soft limits** — defined in `pretty_robot_moves_joint_limits.json`.
  Any commanded position that crosses a soft limit sends **zero kp/kd/torque**
  to all motors before raising an error.
- **Warn zone** — 0.05 rad before the soft limit; a warning is printed to console.
- **Tracking error abort** — if any joint falls more than `max_position_error_rad`
  behind its target, motors are zeroed and the script exits safely.
- **Exit** — any quit path (keyboard `q`, Ctrl-C, or error) ramps back to the
  standing pose before shutting down.
