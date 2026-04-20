#!/usr/bin/env python3
# Subscribes: None
# Publishes: /motor_params (MotorParam)
# TODO: add a short explanation of what this node does and why

import rclpy
from rclpy.node import Node

try:
    from motor_test.common import clamp_rate
except ModuleNotFoundError:
    from common import clamp_rate

from motor_test.msg import MotorParam


class MotorPub(Node):
    def __init__(self):
        super().__init__('motor_pub')
        # TODO: explain the parameters used by this node
        self.declare_parameter('topic', 'motor_params')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('q', 0.0)
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('tau', 0.0)
        self.declare_parameter('all_logging_info', False)

        self.topic = self.get_parameter('topic').value
        self.rate = clamp_rate(self.get_parameter('rate').value)
        self.q = self.get_parameter('q').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.tau = self.get_parameter('tau').value
        self.all_logging_info = bool(self.get_parameter('all_logging_info').value)

        # TODO: explain the publisher, topic, and timer setup
        self.publisher = self.create_publisher(MotorParam, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        if self.all_logging_info:
            self.get_logger().info(
                f'Ready. publishing MotorParam on "{self.topic}" at {self.rate} Hz '
                f'(q={self.q}, kp={self.kp}, kd={self.kd}, tau={self.tau})'
            )

    def timer_callback(self):
        msg = MotorParam()
        msg.q = self.q
        msg.kp = self.kp
        msg.kd = self.kd
        msg.tau = self.tau
        self.publisher.publish(msg)
        if self.all_logging_info:
            self.get_logger().info(
                f'Published MotorParam on "{self.topic}" (q={self.q}, kp={self.kp}, kd={self.kd}, tau={self.tau})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MotorPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
