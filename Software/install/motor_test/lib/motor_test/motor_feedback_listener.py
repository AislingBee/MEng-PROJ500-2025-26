#!/usr/bin/env python3
# Subscribes: /motor_can (UInt8MultiArray)
# Publishes: /motor_feedback (MotorFeedback)
# TODO: add a short description of what this does

import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from motor_test.common import load_motor_names
from motor_test.msg import MotorFeedback, MotorFeedbackEntry


class MotorFeedbackListener(Node):
    def __init__(self):
        super().__init__('motor_feedback_listener')

        self.declare_parameter('input', 'motor_can')
        self.declare_parameter('motor_count', 13)
        self.declare_parameter('names_file', 'motor_names.json')

        self.input_topic = self.get_parameter('input').value
        self.motor_count = int(self.get_parameter('motor_count').value)
        self.names_file = self.get_parameter('names_file').value
        self.all_logging_info = bool(self.get_parameter_or('all_logging_info', False))
        self.record_format = '<ff'
        self.record_size = struct.calcsize(self.record_format)

        # TODO: sort out motor names in config file
        self.motor_names = self.load_motor_names()
        if self.all_logging_info:
            self.get_logger().info(
                f'Loaded {len(self.motor_names)} motor names from "{self.names_file}"'
            )

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            self.input_topic,
            self.motor_can_callback,
            10,
        )
        self.publisher = self.create_publisher(MotorFeedback, 'motor_feedback', 10)

        if self.all_logging_info:
            self.get_logger().info(
                f'Listening for {self.motor_count} motor packets on "{self.input_topic}"'
            )

    def load_motor_names(self):
        return load_motor_names(self.names_file, self.motor_count, self.get_logger())

    def motor_can_callback(self, msg: UInt8MultiArray):
        # TODO:  check that this works?? it may need editing to be the right file time
        payload = bytes(msg.data)
        packet_count = len(payload) // self.record_size

        if packet_count == 0:
            self.get_logger().warn('Empty CAN packet received')
            return

        if packet_count != self.motor_count:
            self.get_logger().warn(
                f'Received {packet_count} records, expected {self.motor_count}'
            )

        actual_count = min(packet_count, self.motor_count)
        values = struct.unpack(self.record_format * actual_count, payload[:actual_count * self.record_size])

        q_values = values[0::2]
        q_dot_values = values[1::2]

        feedback_msg = MotorFeedback()
        feedback_msg.motors = []
        for index in range(actual_count):
            entry = MotorFeedbackEntry()
            entry.name = self.motor_names[index] if index < len(self.motor_names) else f'motor_{index + 1}'
            entry.q = q_values[index]
            entry.q_dot = q_dot_values[index]
            feedback_msg.motors.append(entry)

        self.publisher.publish(feedback_msg)
        if self.all_logging_info:
            self.get_logger().info(
                f'Sent feedback for {actual_count}/{self.motor_count} motors'
            )

            for index in range(actual_count):
                self.get_logger().info(
                    f'{feedback_msg.motors[index].name}: q={q_values[index]:.4f}, dq={q_dot_values[index]:.4f}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = MotorFeedbackListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
