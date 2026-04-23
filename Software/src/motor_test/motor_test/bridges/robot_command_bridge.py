#!/usr/bin/env python3
"""Converts RobotCommand to a packed CAN byte payload (UInt8MultiArray).

Each joint is packed as 4 x float32 LE: [q_des, kp, kd, tau_ff] (16 bytes).

For multi-joint robot operation, this node can enforce a configured joint order
from motor_names.json so the downstream frame chunk index always maps to the
expected motor/CAN ID.
"""

import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from motor_test.common import load_motor_names
from motor_test.msg import RobotCommand


class RobotCommandBridge(Node):
    """Converts a full-robot RobotCommand into a packed CAN byte payload."""

    def __init__(self):
        super().__init__('robot_command_bridge')

        self.declare_parameter('command_topic', 'robot_command')
        self.declare_parameter('can_tx_topic', 'motor_can_tx')
        self.declare_parameter('all_logging_info', False)
        self.declare_parameter('enforce_joint_order', False)
        self.declare_parameter('strict_joint_names', True)
        self.declare_parameter('names_file', 'motor_names.json')

        self.command_topic = self.get_parameter('command_topic').value
        self.can_tx_topic = self.get_parameter('can_tx_topic').value
        self.all_logging_info = bool(self.get_parameter('all_logging_info').value)
        self.enforce_joint_order = bool(self.get_parameter('enforce_joint_order').value)
        self.strict_joint_names = bool(self.get_parameter('strict_joint_names').value)
        self.names_file = str(self.get_parameter('names_file').value)

        self.expected_joint_names = load_motor_names(
            self.names_file,
            motor_count=12,
            logger=self.get_logger(),
        )

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
        if self.enforce_joint_order:
            self.get_logger().info(
                f'Joint order enforcement enabled ({len(self.expected_joint_names)} joints) '
                f'using names file "{self.names_file}"'
            )

    # ------------------------------------------------------------------
    def _command_callback(self, msg: RobotCommand) -> None:
        incoming_joint_names = [str(name) for name in msg.joint_names]
        n = len(incoming_joint_names)
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

        if len(set(incoming_joint_names)) != n:
            self.get_logger().error('RobotCommand contains duplicate joint names; refusing to pack')
            return

        order_indices = list(range(n))
        ordered_joint_names = incoming_joint_names
        if self.enforce_joint_order:
            incoming_index_by_name = {name: i for i, name in enumerate(incoming_joint_names)}
            expected = list(self.expected_joint_names)
            missing = [name for name in expected if name not in incoming_index_by_name]
            extras = [name for name in incoming_joint_names if name not in expected]

            if self.strict_joint_names and (missing or extras):
                self.get_logger().error(
                    f'Joint-name mismatch. Missing expected={missing}, unexpected={extras}'
                )
                return

            if missing and not self.strict_joint_names:
                self.get_logger().warn(
                    f'Missing expected joints in command, skipping those joints: {missing}'
                )

            if extras and not self.strict_joint_names:
                self.get_logger().warn(
                    f'Ignoring unexpected joints not in configured order: {extras}'
                )

            ordered_joint_names = [name for name in expected if name in incoming_index_by_name]
            if not ordered_joint_names:
                self.get_logger().error('No joints left after joint-order filtering; ignoring command')
                return

            order_indices = [incoming_index_by_name[name] for name in ordered_joint_names]

        # Pack each joint as 4 × float32 LE (16 bytes): q kp kd tau.
        payload = bytearray()
        for i in order_indices:
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
                f'Packed {len(order_indices)} joints into {len(payload)} CAN bytes '
                f'using order: {ordered_joint_names}'
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
