#!/usr/bin/env python3

# This code creates a node that converts ROS messages to CAN
# Subscribes: /motor_params (MotorParam)
# Publishes: /motor_can (UInt8MultiArray)
# TODO: add description of this 

#Defs
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from motor_test.common import bytes_to_uint8_list, pack_floats
from motor_test.msg import MotorParam

# Class defintion
class MotorParamsCanBridge(Node):
    def __init__(self):
        super().__init__('motor_params_can_bridge')

        self.declare_parameter('inputs', 'motor_params')
        self.declare_parameter('output', 'motor_can')
        self.declare_parameter('frame', 0x201)

        self.inputs = self.get_parameter('inputs').value
        self.output = self.get_parameter('output').value
        self.frame = self.get_parameter('frame').value
        self.all_logging_info = bool(self.get_parameter_or('all_logging_info', False))

        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            MotorParam,
            self.inputs,
            self.motor_params_callback,
            10,
        )
        self.publisher = self.create_publisher(UInt8MultiArray, self.output, 10)

        # Log Current State
        if self.all_logging_info:
            self.get_logger().info(
                f'Bridge ready: listening on "{self.inputs}" and sending CAN bytes on "{self.output}" '
                f'(frame 0x{self.frame:X})'
            )

    def motor_params_callback(self, msg: MotorParam):
        can_data = pack_floats((msg.q, msg.kp, msg.kd, msg.tau))
        can_msg = UInt8MultiArray()
        can_msg.data = bytes_to_uint8_list(can_data)

        # Publish the CAN message
        self.publisher.publish(can_msg)
        if self.all_logging_info:
            self.get_logger().info(
                f'Sent CAN message on frame 0x{self.frame:X}; {len(can_msg.data)} bytes'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MotorParamsCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
