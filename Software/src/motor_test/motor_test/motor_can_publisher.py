#!/usr/bin/env python3
# Subscribes: /test_motor_params (MotorParam)
# Publishes: /motor_can (UInt8MultiArray)
# TODO: add a short descrption

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

try:
    from motor_test.common import bytes_to_uint8_list, pack_floats
except ModuleNotFoundError:
    from common import bytes_to_uint8_list, pack_floats

from motor_test.msg import MotorParam


class MotorCanPublisher(Node):
    def __init__(self):
        super().__init__('motor_can_publisher')

        self.declare_parameter('input', 'test_motor_params')
        self.declare_parameter('output', 'motor_can')
        self.declare_parameter('frame', 0x201)
        self.declare_parameter('all_logging_info', False)

        self.input_topic = self.get_parameter('input').value
        self.output_topic = self.get_parameter('output').value
        self.frame = self.get_parameter('frame').value
        self.all_logging_info = bool(self.get_parameter('all_logging_info').value)

        # Set up subscriber and publisher
        self.subscription = self.create_subscription(
            MotorParam,
            self.input_topic,
            self.motor_param_callback,
            10,
        )
        self.publisher = self.create_publisher(UInt8MultiArray, self.output_topic, 10)

        if self.all_logging_info:
            self.get_logger().info(
                f'Ready. receiving MotorParam on "{self.input_topic}" and publishing CAN on "{self.output_topic}"'
            )

    def motor_param_callback(self, msg: MotorParam):
        can_data = pack_floats((msg.q, msg.kp, msg.kd, msg.tau))
        can_msg = UInt8MultiArray()
        can_msg.data = bytes_to_uint8_list(can_data)

        self.publisher.publish(can_msg)
        if self.all_logging_info:
            self.get_logger().info(
                f'Sent CAN message on frame 0x{self.frame:X}; {len(can_msg.data)} bytes'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MotorCanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
