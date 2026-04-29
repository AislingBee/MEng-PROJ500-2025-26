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
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("target_q_rad", 0.25)
        self.declare_parameter("step_duration_s", 2.0)
        self.declare_parameter("out_of_phase", True)
        self.declare_parameter("kp", 20.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("tau_ff", 0.0)

        topic = str(self.get_parameter("command_topic").value)
        names_raw = str(self.get_parameter("joint_names").value)
        self._joint_names = [s.strip() for s in names_raw.split(",") if s.strip()]
        if not self._joint_names:
            self._joint_names = ["motor_1", "motor_2"]

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._target_q = float(self.get_parameter("target_q_rad").value)
        self._step_duration_s = max(0.05, float(self.get_parameter("step_duration_s").value))
        self._out_of_phase = bool(self.get_parameter("out_of_phase").value)
        self._kp = float(self.get_parameter("kp").value)
        self._kd = float(self.get_parameter("kd").value)
        self._tau = float(self.get_parameter("tau_ff").value)

        self._pub = self.create_publisher(RobotCommand, topic, 10)
        self._t0 = self.get_clock().now()

        period = 1.0 / max(1e-3, self._rate_hz)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Publishing step RobotCommand on {topic} for joints {self._joint_names} at {self._rate_hz:.1f} Hz"
        )

    def _tick(self):
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9

        phase = int(t / self._step_duration_s) % 2
        q_base = self._target_q if phase == 0 else -self._target_q
        q_other = -q_base if self._out_of_phase else q_base

        q_des = []
        qd_des = []
        for i, _ in enumerate(self._joint_names):
            if i % 2 == 0:
                q_des.append(q_base)
                qd_des.append(0.0)
            else:
                q_des.append(q_other)
                qd_des.append(0.0)

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


def main(args=None):
    rclpy.init(args=args)
    node = RcuBenchCommandTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
