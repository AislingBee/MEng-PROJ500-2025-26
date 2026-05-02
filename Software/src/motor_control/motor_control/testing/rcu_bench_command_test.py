#!/usr/bin/env python3
"""
Publish simple two-motor RobotCommand traffic for the RCU UDP bridge.

Default behavior targets motor_1 and motor_2 at a fixed rate and sends
step position commands with MIT gains.

Typical usage:
  1) Start bridge stack:
       ros2 launch motor_control rcu_launch.py active_motor_ids:="[1,2]" left_bus_motor_ids:="[1,2]"
  2) In another terminal:
       ros2 run motor_control rcu_bench_command_test.py
"""

import rclpy
from rclpy.node import Node

from motor_control.msg import RobotCommand


class RcuBenchCommandTest(Node):
    def __init__(self):
        super().__init__("rcu_bench_command_test")

        self.declare_parameter("command_topic", "/robot_command")
        self.declare_parameter("joint_names", "motor_1,motor_2")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("match_plymouth_bench_mode", True)
        self.declare_parameter("target_q_rad", 0.20)
        self.declare_parameter("step_duration_s", 2.5)
        self.declare_parameter("out_of_phase", True)
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("tau_ff", 1.0)
        self.declare_parameter("max_q_slew_rad_s", 1.2)
        self.declare_parameter("send_velocity_commands", True)
        self.declare_parameter("use_fixed_velocity_command", True)
        self.declare_parameter("velocity_command_rad_s", 1.0)
        self.declare_parameter("command_log_hz", 2.0)

        topic = str(self.get_parameter("command_topic").value)
        names_raw = str(self.get_parameter("joint_names").value)
        self._joint_names = [s.strip() for s in names_raw.split(",") if s.strip()]
        if not self._joint_names:
            self._joint_names = ["motor_1", "motor_2"]

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._match_plymouth_bench_mode = bool(
            self.get_parameter("match_plymouth_bench_mode").value
        )
        self._target_q = float(self.get_parameter("target_q_rad").value)
        self._step_duration_s = max(0.05, float(self.get_parameter("step_duration_s").value))
        self._out_of_phase = bool(self.get_parameter("out_of_phase").value)
        self._kp = float(self.get_parameter("kp").value)
        self._kd = float(self.get_parameter("kd").value)
        self._tau = float(self.get_parameter("tau_ff").value)
        self._max_q_slew = max(0.0, float(self.get_parameter("max_q_slew_rad_s").value))
        self._send_velocity_commands = bool(self.get_parameter("send_velocity_commands").value)
        self._use_fixed_velocity_command = bool(self.get_parameter("use_fixed_velocity_command").value)
        self._velocity_command_rad_s = float(self.get_parameter("velocity_command_rad_s").value)
        self._command_log_hz = max(0.0, float(self.get_parameter("command_log_hz").value))

        self._pub = self.create_publisher(RobotCommand, topic, 10)
        self._t0 = self.get_clock().now()

        period = 1.0 / max(1e-3, self._rate_hz)
        self._dt = period
        self._timer = self.create_timer(period, self._tick)
        self._q_cmd = [0.0 for _ in self._joint_names]
        self._tx_count = 0
        self._last_log_t = -1.0

        self.get_logger().info(
            f"Publishing step RobotCommand on {topic} for joints {self._joint_names} at {self._rate_hz:.1f} Hz"
        )
        if self._match_plymouth_bench_mode:
            self.get_logger().info("Plymouth-compatible fixed command mode enabled")
        if self._send_velocity_commands:
            if self._use_fixed_velocity_command:
                self.get_logger().info(
                    f"Velocity commands enabled: fixed qd_des={self._velocity_command_rad_s:+.3f} rad/s"
                )
            else:
                self.get_logger().info("Velocity commands enabled: qd_des follows slew-limited transitions")
        if self._command_log_hz > 0.0:
            self.get_logger().info(
                f"Command TX logging enabled at {self._command_log_hz:.2f} Hz"
            )

    def _tick(self):
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9

        if self._match_plymouth_bench_mode:
            n = len(self._joint_names)
            q_des = [self._target_q] * n
            qd_val = self._velocity_command_rad_s if self._send_velocity_commands else 0.0
            qd_des = [qd_val] * n

            msg = RobotCommand()
            msg.joint_names = self._joint_names
            msg.q_des = q_des
            msg.qd_des = qd_des
            msg.kp = [self._kp] * n
            msg.kd = [self._kd] * n
            msg.tau_ff = [self._tau] * n
            msg.kp_gains = [self._kp] * n
            msg.kd_gains = [self._kd] * n
            self._pub.publish(msg)
            self._tx_count += 1

            if self._command_log_hz > 0.0:
                log_period = 1.0 / self._command_log_hz
                if self._last_log_t < 0.0 or (t - self._last_log_t) >= log_period:
                    self._last_log_t = t
                    cmd_parts = []
                    for i, name in enumerate(self._joint_names):
                        cmd_parts.append(
                            f"{name}: q={q_des[i]:+.3f} qd={qd_des[i]:+.3f} tau={self._tau:+.3f} kp={self._kp:.1f} kd={self._kd:.1f}"
                        )
                    self.get_logger().info(
                        f"TX #{self._tx_count}: " + " | ".join(cmd_parts)
                    )
            return

        phase = int(t / self._step_duration_s) % 2
        q_base = self._target_q if phase == 0 else -self._target_q
        q_other = -q_base if self._out_of_phase else q_base

        q_targets = []
        for i, _ in enumerate(self._joint_names):
            if i % 2 == 0:
                q_targets.append(q_base)
            else:
                q_targets.append(q_other)

        # Slew-limit position setpoint transitions to avoid aggressive fault-triggering jumps.
        q_des = []
        qd_des = []
        max_delta = self._max_q_slew * self._dt
        for i, q_tgt in enumerate(q_targets):
            q_prev = self._q_cmd[i]
            if max_delta <= 0.0:
                q_new = q_tgt
                qd_cmd = 0.0
            else:
                delta = max(-max_delta, min(max_delta, q_tgt - q_prev))
                q_new = q_prev + delta
                qd_cmd = delta / self._dt if self._dt > 0.0 else 0.0
            self._q_cmd[i] = q_new
            q_des.append(q_new)
            if not self._send_velocity_commands:
                qd_out = 0.0
            elif self._use_fixed_velocity_command:
                qd_out = self._velocity_command_rad_s if i % 2 == 0 else -self._velocity_command_rad_s
            else:
                qd_out = qd_cmd
            qd_des.append(qd_out)

        n = len(self._joint_names)
        msg = RobotCommand()
        msg.joint_names = self._joint_names
        msg.q_des = q_des
        msg.qd_des = qd_des
        msg.kp = [self._kp] * n
        msg.kd = [self._kd] * n
        msg.tau_ff = [self._tau] * n
        msg.kp_gains = [self._kp] * n
        msg.kd_gains = [self._kd] * n
        self._pub.publish(msg)
        self._tx_count += 1

        if self._command_log_hz > 0.0:
            log_period = 1.0 / self._command_log_hz
            if self._last_log_t < 0.0 or (t - self._last_log_t) >= log_period:
                self._last_log_t = t
                cmd_parts = []
                for i, name in enumerate(self._joint_names):
                    cmd_parts.append(
                        f"{name}: q={q_des[i]:+.3f} qd={qd_des[i]:+.3f}"
                    )
                self.get_logger().info(
                    f"TX #{self._tx_count}: " + " | ".join(cmd_parts)
                )


def main(args=None):
    rclpy.init(args=args)
    node = RcuBenchCommandTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
