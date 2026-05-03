# motor_control Launch Files Guide

This file explains the purpose of each launch file in this folder and when to use it.

## Quick selection

- Use `rcu_launch.py` for the base RCU-to-ROS pipeline.
- Use `rl_robot_launch.py` for RL-ready full 12-motor topic wiring (runner started separately).
- Use `thor_12_motor_pipeline_launch.py` for one-command startup of both RCU stack and THOR policy runner.
- Use `rcu_bench_test_launch.py` for two-motor bench command testing.
- Use `rcu_diagnosis_launch.py` for CAN-ID bring-up/diagnosis with feedback monitoring.

## 1) rcu_launch.py

Purpose:
- Core hardware bridge pipeline for RCU communication and observation topics.

Starts:
- `rcu_udp_bridge.py`
- `motor_feedback_listener.py`
- `robot_observation_bridge.py`

Main outputs:
- `/motor_can_feedback`
- `/motor_feedback`
- `/imu0`
- `/robot_observation`

Main inputs:
- `/robot_command`

Notable defaults:
- Bench-oriented motor set by default:
  - `active_motor_ids="[1,2]"`
  - `left_bus_motor_ids="[1,2]"`
- `auto_enable=False`
- startup gate disabled by default (`wait_for_expected_online_ids=False`)

When to use:
- You want the base RCU pipeline and will provide commands from another node/tool.

Example:
```bash
ros2 launch motor_control rcu_launch.py
```

## 2) rl_robot_launch.py

Purpose:
- RL-oriented pipeline setup with full 12-motor defaults and explicit bus split.

Starts:
- `rcu_udp_bridge.py`
- `motor_feedback_listener.py`
- `robot_observation_bridge.py`

Main outputs:
- `/motor_can_feedback`
- `/motor_feedback`
- `/imu0`
- `/robot_observation`

Main inputs:
- `/robot_command`

Notable defaults:
- Full robot IDs enabled in command path:
  - `active_motor_ids="[1,2,3,4,5,6,7,8,9,10,11,12]"`
- Bus split defaults:
  - `left_bus_motor_ids="[1,3,5,7,9,11]"`
  - `right_bus_motor_ids="[2,4,6,8,10,12]"`
- `auto_enable=False`
- strict startup gate remains opt-in (`wait_for_expected_online_ids=False`)

When to use:
- You are running THOR policy runner manually in another terminal and want RL-ready topic wiring.

Example:
```bash
ros2 launch motor_control rl_robot_launch.py
```

## 3) thor_12_motor_pipeline_launch.py

Purpose:
- End-to-end orchestrator for 12-motor THOR policy execution.

Starts:
- Includes `rcu_launch.py` with full 12-motor arguments.
- Launches `hardware/thor/startup_then_policy_runner.py` via `ExecuteProcess`.

Main behavior:
- Brings up the RCU stack, ramps the robot from zero pose to standing, holds
  standing pose, then runs the standing policy — all in one process.
- Keyboard controls during runtime:
  - `p` — advance from STANDING_HOLD to POLICY
  - `h` — return to STANDING_HOLD from POLICY
  - `x` / Ctrl+C — safe exit (sends final standing pose)

Notable defaults:
- `auto_enable=True`
- `scan_motor_can_ids=True`
- startup gate enabled (`wait_for_expected_online_ids=True`)
- expected IDs set to all 12 motors

When to use:
- You want one launch command for full startup-to-policy execution.

Example:
```bash
ros2 launch motor_control thor_12_motor_pipeline_launch.py
```

## 4) rcu_bench_test_launch.py

Purpose:
- Bench workflow for commanding a small motor subset without full policy inference.

Starts:
- Includes `rcu_launch.py` configured for two-motor bench IDs.
- Launches `rcu_bench_command_test.py` command publisher.

Notable defaults:
- `active_motor_ids="[9,10]"`
- `left_bus_motor_ids="[9]"`
- startup gate enabled with expected IDs `[9,10]`
- command generator publishes on `/robot_command`

When to use:
- You are validating command path and motor response on bench hardware.

Example:
```bash
ros2 launch motor_control rcu_bench_test_launch.py
```

## 5) rcu_diagnosis_launch.py

Purpose:
- Bring-up/diagnosis mode focused on online CAN ID visibility and feedback monitoring.

Starts:
- `rcu_udp_bridge.py` (diagnosis parameters)
- `motor_feedback_listener.py`

Does not start:
- `robot_observation_bridge.py`

Notable defaults:
- `auto_enable=True`
- CAN-ID scan enabled
- startup gate enabled with expected IDs `[9,10]`
- targets diagnosis pair IDs 9 and 10 with bus overrides

When to use:
- You need to verify RCU link, detected motor IDs, and basic feedback stream before full control runs.

Example:
```bash
ros2 launch motor_control rcu_diagnosis_launch.py
```

## Relationship summary

- `rcu_launch.py` is the base reusable stack.
- `rcu_bench_test_launch.py` and `thor_12_motor_pipeline_launch.py` build on that base for specific workflows.
- `rl_robot_launch.py` is a direct RL-focused stack launch with broader parameter exposure and full 12-motor defaults.
- `rcu_diagnosis_launch.py` is intentionally diagnostic-first rather than full pipeline.
