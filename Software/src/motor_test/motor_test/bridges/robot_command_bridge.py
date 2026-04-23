#!/usr/bin/env python3
"""Converts RobotCommand to a packed CAN byte payload (UInt8MultiArray).

Each joint is packed as 4 × float32 LE: [q_des, kp, kd, tau_ff] (16 bytes).
"""

import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from motor_test.msg import RobotCommand


class RobotCommandBridge(Node):
    """Converts a full-robot RobotCommand into a packed CAN byte payload."""

    def __init__(self):
        super().__init__('robot_command_bridge')

        self.declare_parameter('command_topic', 'robot_command')
        self.declare_parameter('can_tx_topic', 'motor_can_tx')
        self.declare_parameter('all_logging_info', False)

        self.command_topic = self.get_parameter('command_topic').value
        self.can_tx_topic = self.get_parameter('can_tx_topic').value
        self.all_logging_info = bool(self.get_parameter('all_logging_info').value)

        # QoS depth 1: always forward the newest command.
        self.subscription = self.create_subscription(
            RobotCommand,
            self.command_topic,
            self._command_callback,
            1,
        )
        self.publisher = self.create_publisher(UInt8MultiArray, self.can_tx_topic, 1)

        self.get_logger().info(
            f'RobotCommandBridge ready: '
            f'"{self.command_topic}" → "{self.can_tx_topic}"'
        )

    # ------------------------------------------------------------------
    def _command_callback(self, msg: RobotCommand) -> None:
        n = len(msg.joint_names)
        if n == 0:
            self.get_logger().warn('Received RobotCommand with no joints; ignoring.')
            return

        # Validate array lengths.
        lengths = {
            'q_des':   len(msg.q_des),
            'qd_des':  len(msg.qd_des),
            'kp':      len(msg.kp),
            'kd':      len(msg.kd),
            'tau_ff':  len(msg.tau_ff),
        }
        bad = {k: v for k, v in lengths.items() if v != n}
        if bad:
            self.get_logger().error(
                f'RobotCommand length mismatch for {n} joints: {bad}'
            )
            return

        # Pack each joint as 4 × float32 LE (16 bytes): q kp kd tau.
        payload = bytearray()
        for i in range(n):
            payload += struct.pack(
                '<ffff',
                float(msg.q_des[i]),
                float(msg.kp[i]),
                float(msg.kd[i]),
                float(msg.tau_ff[i]),
            )

        out = UInt8MultiArray()
        out.data = list(payload)
        self.publisher.publish(out)

        if self.all_logging_info:
            self.get_logger().info(
                f'Packed {n} joints into {len(payload)} CAN bytes'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
