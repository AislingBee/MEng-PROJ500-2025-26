#!/usr/bin/env python3
"""
sequential_motor_zero.py
------------------------
Moves all 12 motors to their zero positions one at a time in CAN ID order.
Already-zeroed motors are held at 0.0 rad while the next one is ramped.

SAFETY: Reads each motor's current position via /motor_feedback before ramping.
q_des ramps from q_actual → 0.0 rad while kp ramps 0 → full simultaneously.
This keeps the position error near zero throughout, preventing violent jolts.

Intended to run AFTER the RCU stack is up and motors are enabled (e.g. from
thor_12_motor_pipeline_launch.py before startup_then_policy_runner starts).

Parameters (overridable via --ros-args -p):
    ramp_time_s        (float)  default 4.0   — seconds to ramp each motor to 0
    hold_time_s        (float)  default 1.0   — seconds to hold at 0 before next motor
    kp                 (float)  default 20.0  — position gain [Nm/rad]
    kd                 (float)  default 1.5   — velocity gain [Nm·s/rad]
    rate_hz            (float)  default 200.0 — command publish rate
    motor_ids          (string) default "[1,2,3,4,5,6,7,8,9,10,11,12]" — order to zero
    command_topic      (string) default "robot_command"
    feedback_timeout_s (float)  default 5.0   — max wait for first feedback message

Usage:
    ros2 run motor_control sequential_motor_zero.py
    ros2 run motor_control sequential_motor_zero.py --ros-args -p ramp_time_s:=6.0 -p kp:=15.0
"""

import time

import rclpy
from rclpy.node import Node

from motor_control.msg import RobotCommand, MotorFeedback
from motor_control import rcu_protocol as rp


class SequentialMotorZero(Node):
    def __init__(self) -> None:
        super().__init__("sequential_motor_zero")

        self.declare_parameter("ramp_time_s",        4.0)
        self.declare_parameter("hold_time_s",        1.0)
        self.declare_parameter("kp",                 20.0)
        self.declare_parameter("kd",                 1.5)
        self.declare_parameter("rate_hz",            200.0)
        self.declare_parameter("motor_ids",          "[1,2,3,4,5,6,7,8,9,10,11,12]")
        self.declare_parameter("command_topic",      "robot_command")
        self.declare_parameter("feedback_timeout_s", 5.0)

        self._ramp_time_s        = float(self.get_parameter("ramp_time_s").value)
        self._hold_time_s        = float(self.get_parameter("hold_time_s").value)
        self._kp                 = float(self.get_parameter("kp").value)
        self._kd                 = float(self.get_parameter("kd").value)
        self._rate_hz            = float(self.get_parameter("rate_hz").value)
        self._period_s           = 1.0 / self._rate_hz
        self._feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
        cmd_topic                = str(self.get_parameter("command_topic").value)

        raw_ids = str(self.get_parameter("motor_ids").value)
        self._motor_ids: list[int] = []
        for tok in raw_ids.replace("[", "").replace("]", "").split(","):
            tok = tok.strip()
            if tok.isdigit():
                mid = int(tok)
                if 1 <= mid <= 12:
                    self._motor_ids.append(mid)

        if not self._motor_ids:
            self.get_logger().error("No valid motor IDs configured — exiting.")
            raise SystemExit(1)

        self._pub = self.create_publisher(RobotCommand, cmd_topic, 1)

        # Latest encoder positions keyed by joint name, updated via /motor_feedback.
        self._latest_q: dict[str, float] = {}
        self._sub_fb = self.create_subscription(
            MotorFeedback, "/motor_feedback", self._feedback_cb, 10
        )

        self.get_logger().info(
            f"SequentialMotorZero: {len(self._motor_ids)} motors, "
            f"ramp={self._ramp_time_s:.1f}s, hold={self._hold_time_s:.1f}s, "
            f"kp={self._kp}, kd={self._kd}, rate={self._rate_hz:.0f}Hz"
        )
        self.get_logger().info(
            "Motor order: " +
            ", ".join(f"{mid}:{rp.MOTOR_JOINT_NAMES[mid]}" for mid in self._motor_ids)
        )

    def _feedback_cb(self, msg: MotorFeedback) -> None:
        for entry in msg.motors:
            self._latest_q[entry.name] = entry.q

    # ------------------------------------------------------------------
    def _publish(self, names: list[str], q_list: list[float],
                 kp_list: list[float], kd_list: list[float]) -> None:
        msg = RobotCommand()
        msg.joint_names = names
        msg.q_des       = [float(q) for q in q_list]
        msg.qd_des      = [0.0] * len(names)
        msg.kp          = kp_list
        msg.kd          = kd_list
        msg.tau_ff      = [0.0] * len(names)
        msg.kp_gains    = kp_list[:]
        msg.kd_gains    = kd_list[:]
        self._pub.publish(msg)

    def _hold(self, names: list[str], q_list: list[float], duration_s: float) -> None:
        """Publish a fixed hold command at full gains for duration_s seconds."""
        kp = [self._kp] * len(names)
        kd = [self._kd] * len(names)
        end_t  = time.monotonic() + duration_s
        next_t = time.monotonic()
        while time.monotonic() < end_t:
            self._publish(names, q_list, kp, kd)
            next_t += self._period_s
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

    # ------------------------------------------------------------------
    def run(self) -> None:
        # Wait for the first feedback message so we know actual motor positions.
        self.get_logger().info(
            f"Waiting up to {self._feedback_timeout_s:.1f}s for /motor_feedback..."
        )
        deadline = time.monotonic() + self._feedback_timeout_s
        while not self._latest_q and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self._latest_q:
            self.get_logger().error(
                "No /motor_feedback received before timeout. "
                "Is motor_feedback_listener running? Aborting."
            )
            raise SystemExit(1)
        self.get_logger().info(
            f"Feedback received for {len(self._latest_q)} joints."
        )

        # Send one zero-torque packet to all motors before any ramp starts.
        # This ensures motors receive a safe kp=0/kd=0/tau=0 command the
        # moment they come up, rather than whatever was in the bridge cache.
        all_names = [rp.MOTOR_JOINT_NAMES[mid] for mid in self._motor_ids]
        self.get_logger().info("Sending zero-torque safety packet to all motors...")
        self._publish(
            all_names,
            [0.0] * len(all_names),
            [0.0] * len(all_names),
            [0.0] * len(all_names),
        )
        time.sleep(self._period_s * 5)  # let a few TX cycles flush before ramping

        zeroed_names: list[str]   = []
        zeroed_q:     list[float] = []

        for idx, mid in enumerate(self._motor_ids):
            joint_name = rp.MOTOR_JOINT_NAMES[mid]

            # Read the current position so we ramp from here, not from 0.0.
            # This prevents violent movement if the motor is far from encoder zero.
            rclpy.spin_once(self, timeout_sec=0.0)
            q_start = self._latest_q.get(joint_name, 0.0)

            self.get_logger().info(
                f"[{idx + 1}/{len(self._motor_ids)}]  Motor {mid}: {joint_name}"
                f" — current={q_start:+.3f} rad, ramping to 0.0 rad over {self._ramp_time_s:.1f}s"
            )

            # Ramp this motor:
            #   q_des   : q_start → 0.0  (linear)
            #   kp, kd  : 0       → full (linear)
            # Both ramp together so position error stays near zero throughout,
            # preventing the large-error / high-gain torque spike that caused
            # dangerous movement when q_des was fixed at 0.0 while kp climbed.
            ramp_start = time.monotonic()
            next_t     = ramp_start

            while True:
                elapsed = time.monotonic() - ramp_start
                alpha   = min(elapsed / self._ramp_time_s, 1.0)

                # q_des moves from q_start toward 0.0 at the same rate kp climbs,
                # so the commanded position error = (1-alpha)*q_start which shrinks
                # as gains grow — torque ≈ kp * (1-alpha)*q_start stays bounded.
                q_this  = q_start * (1.0 - alpha)
                eff_kp  = self._kp * alpha
                eff_kd  = self._kd * alpha

                names   = zeroed_names + [joint_name]
                q_list  = zeroed_q     + [q_this]
                kp_list = [self._kp] * len(zeroed_names) + [eff_kp]
                kd_list = [self._kd] * len(zeroed_names) + [eff_kd]

                self._publish(names, q_list, kp_list, kd_list)

                if alpha >= 1.0:
                    break

                next_t += self._period_s
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_t = time.monotonic()

            zeroed_names.append(joint_name)
            zeroed_q.append(0.0)
            self.get_logger().info(
                f"  Motor {mid} at 0.0 rad  "
                f"({len(zeroed_names)}/{len(self._motor_ids)} done). "
                f"Holding {self._hold_time_s:.1f}s before next motor."
            )

            # Hold all zeroed motors before moving to the next one.
            if idx < len(self._motor_ids) - 1:
                self._hold(zeroed_names, zeroed_q, self._hold_time_s)

        self.get_logger().info(
            f"All {len(zeroed_names)} motors at 0.0 rad. "
            "Holding position — Ctrl+C or kill this node when ready to proceed."
        )

        # Hold all motors at zero indefinitely until killed.
        while rclpy.ok():
            self._hold(zeroed_names, zeroed_q, 1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SequentialMotorZero()
    try:
        node.run()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
