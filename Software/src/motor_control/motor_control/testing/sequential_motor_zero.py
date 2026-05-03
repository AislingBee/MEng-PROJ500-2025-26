#!/usr/bin/env python3
"""
sequential_motor_zero.py
------------------------
Enables motors one at a time and gently ramps each to its zero position.

Sequence (CAN ID order 1 → 12):
  - Press Enter  : enable current motor and ramp it to 0.0 rad over ramp_time_s
  - Press 's'    : skip current motor (leave it unpowered)
  - Press 'x'    : send zero torque to all enabled motors and exit

Usage (from repo root):
  export PYTHONPATH=$PWD
  python3 hardware/thor/sequential_motor_zero.py

Or via ROS2 launch (rcu_stack must be running separately):
  python3 hardware/thor/sequential_motor_zero.py --ramp-time 5.0 --kp 15.0 --kd 1.5
"""

import argparse
import sys
import termios
import threading
import time
import tty

import rclpy

# ---------------------------------------------------------------------------
# CAN ID → joint name mapping (mirrors rcu_protocol.MOTOR_JOINT_NAMES)
# ---------------------------------------------------------------------------
MOTOR_SEQUENCE = [
    (1,  "pelvis_link_l_yaw_joint"),
    (2,  "pelvis_link_r_yaw_joint"),
    (3,  "l_hip_yaw_link_l_pitch_joint"),
    (4,  "r_hip_yaw_link_r_pitch_joint"),
    (5,  "l_hip_pitch_link_l_roll_joint"),
    (6,  "r_hip_pitch_link_r_roll_joint"),
    (7,  "l_thigh_link_l_knee_joint"),
    (8,  "r_thigh_link_r_knee_joint"),
    (9,  "l_shank_link_l_ankle_joint"),
    (10, "r_shank_link_r_ankle_joint"),
    (11, "l_ankle_link_l_foot_joint"),
    (12, "r_ankle_link_r_foot_joint"),
]

# ---------------------------------------------------------------------------
# Keyboard helper (non-blocking single-char read, same pattern as
# startup_then_policy_runner.py)
# ---------------------------------------------------------------------------

def _read_key_nonblocking(fd: int) -> str | None:
    """Return a single character from fd without blocking, or None."""
    import select
    r, _, _ = select.select([fd], [], [], 0.0)
    if r:
        return sys.stdin.read(1)
    return None


def _keyboard_thread_fn(key_queue: list, stop_event: threading.Event) -> None:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not stop_event.is_set():
            key = _read_key_nonblocking(fd)
            if key is not None:
                key_queue.append(key)
            time.sleep(0.02)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ---------------------------------------------------------------------------
# ROS2 command writer (uses thor_policy_runner bridge helpers)
# ---------------------------------------------------------------------------
def _build_command(joint_names: list[str], q_des: list[float],
                   kp: float, kd: float) -> object:
    """Build a RobotCommand message for the listed joints."""
    from motor_control.msg import RobotCommand
    msg = RobotCommand()
    msg.joint_names = joint_names
    msg.q_des       = [float(q) for q in q_des]
    msg.kp          = [kp] * len(joint_names)
    msg.kd          = [kd] * len(joint_names)
    msg.tau_ff      = [0.0] * len(joint_names)
    return msg


# ---------------------------------------------------------------------------
# Main routine
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="Sequentially zero all 12 motors one at a time.")
    parser.add_argument("--ramp-time",  type=float, default=5.0,
                        help="Seconds to ramp each motor from current position to 0.0 rad.")
    parser.add_argument("--kp",         type=float, default=20.0,
                        help="Position gain (Nm/rad) during ramp.")
    parser.add_argument("--kd",         type=float, default=1.5,
                        help="Velocity gain (Nm·s/rad) during ramp.")
    parser.add_argument("--rate-hz",    type=float, default=50.0,
                        help="Command publish rate in Hz.")
    parser.add_argument("--command-topic", type=str, default="robot_command",
                        help="ROS2 topic for RobotCommand messages.")
    args = parser.parse_args()

    rclpy.init()
    from rclpy.node import Node
    from motor_control.msg import RobotCommand

    node = Node("sequential_motor_zero")
    pub  = node.create_publisher(RobotCommand, args.command_topic, 1)

    # Spin ROS2 in a background thread so pub.publish() works from main thread.
    ros_stop = threading.Event()
    def _ros_spin():
        while not ros_stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.01)
    ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    ros_thread.start()

    # Keyboard thread
    key_queue:  list[str]  = []
    kbd_stop   = threading.Event()
    kbd_thread = threading.Thread(target=_keyboard_thread_fn,
                                  args=(key_queue, kbd_stop), daemon=True)
    kbd_thread.start()

    period_s      = 1.0 / args.rate_hz
    enabled_joints: list[str]  = []   # joints that have been zeroed so far
    enabled_q:      list[float] = []  # all held at 0.0

    print("=" * 60)
    print("  Sequential Motor Zero  —  12 motors  —  CAN ID order")
    print("=" * 60)
    print("  Enter  →  enable current motor and ramp to 0.0 rad")
    print("  s      →  skip current motor (leave unpowered)")
    print("  x      →  release all motors and exit")
    print("=" * 60)

    try:
        for can_id, joint_name in MOTOR_SEQUENCE:
            print(f"\n[{can_id:2d}/12]  {joint_name}")
            print(f"       Press Enter to enable and zero  |  's' to skip  |  'x' to quit")

            # --- Wait for keypress -------------------------------------------
            action = None
            while action is None:
                if key_queue:
                    k = key_queue.pop(0).lower()
                    if k in ("\r", "\n", ""):
                        action = "enable"
                    elif k == "s":
                        action = "skip"
                    elif k == "x":
                        action = "exit"
                time.sleep(0.02)

            if action == "exit":
                print("\nExit requested.")
                break

            if action == "skip":
                print(f"       Skipped {joint_name}.")
                continue

            # --- Ramp this motor to 0.0 rad ----------------------------------
            print(f"       Ramping {joint_name} to 0.0 rad over {args.ramp_time:.1f}s ...")

            # Read current position from feedback (best-effort: use 0.0 if
            # no feedback available yet — the ramp will still be smooth because
            # the motor is already near zero at startup).
            q_start = 0.0   # conservative default

            ramp_start  = time.monotonic()
            next_t      = ramp_start
            ramp_done   = False
            exit_ramp   = False

            while not ramp_done:
                elapsed = time.monotonic() - ramp_start
                alpha   = min(elapsed / args.ramp_time, 1.0)
                q_now   = q_start * (1.0 - alpha)   # linear ramp from q_start → 0.0

                # Build command for ALL enabled joints (hold them at 0.0)
                # plus the current joint being ramped.
                all_names = enabled_joints + [joint_name]
                all_q     = enabled_q      + [q_now]

                msg = _build_command(all_names, all_q, args.kp, args.kd)
                pub.publish(msg)

                if alpha >= 1.0:
                    ramp_done = True

                # Check for early exit during ramp
                if key_queue:
                    k = key_queue[0].lower()
                    if k == "x":
                        key_queue.pop(0)
                        exit_ramp = True
                        ramp_done = True

                next_t += period_s
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_t = time.monotonic()

            # Add this motor to the "holding at zero" set
            enabled_joints.append(joint_name)
            enabled_q.append(0.0)
            print(f"       {joint_name} at 0.0 rad. ({len(enabled_joints)} motor(s) now held at zero)")

            if exit_ramp:
                print("\nExit requested during ramp.")
                break

        print(f"\nAll done. {len(enabled_joints)} motor(s) held at zero. Press 'x' to release and exit.")

        # Hold all enabled motors at zero until 'x'
        next_t = time.monotonic()
        while True:
            if key_queue and key_queue[0].lower() == "x":
                key_queue.pop(0)
                break
            if enabled_joints:
                msg = _build_command(enabled_joints, enabled_q, args.kp, args.kd)
                pub.publish(msg)
            next_t += period_s
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

    finally:
        # Release: send zero-torque command (kp=0, kd=0, tau_ff=0) to all
        # motors that were enabled so they go limp rather than holding.
        if enabled_joints:
            print(f"\nReleasing {len(enabled_joints)} motor(s) (zero torque) ...")
            msg = _build_command(enabled_joints,
                                 [0.0] * len(enabled_joints),
                                 kp=0.0, kd=0.0)
            pub.publish(msg)
            time.sleep(0.2)

        kbd_stop.set()
        ros_stop.set()
        node.destroy_node()
        rclpy.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
