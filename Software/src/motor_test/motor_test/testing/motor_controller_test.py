#!/usr/bin/env python3
# Publishes: /motor_params (MotorParam)
# Subscribes: /motor_feedback (MotorFeedback)
# This node generates a simple step command for one motor and logs the feedback.

import rclpy
from rclpy.node import Node
from motor_test.msg import MotorParam, MotorFeedback


class MotorControllerTest(Node):
    def __init__(self):
        super().__init__('motor_controller_test')

        self.declare_parameter('command_topic', 'motor_params')
        self.declare_parameter('feedback_topic', 'motor_feedback')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('target_q', 0.4)
        self.declare_parameter('kp', 20.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('tau', 0.0)
        self.declare_parameter('step_duration', 4.0)
        self.declare_parameter('motor_index', 0)
        self.declare_parameter('all_logging_info', False)

        self.command_topic = self.get_parameter('command_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.rate = float(self.get_parameter('rate').value)
        self.target_q = float(self.get_parameter('target_q').value)
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.tau = float(self.get_parameter('tau').value)
        self.step_duration = float(self.get_parameter('step_duration').value)
        self.motor_index = int(self.get_parameter('motor_index').value)
        self.all_logging_info = bool(self.get_parameter('all_logging_info').value)

        self.publisher = self.create_publisher(MotorParam, self.command_topic, 10)
        self.subscription = self.create_subscription(
            MotorFeedback,
            self.feedback_topic,
            self.feedback_callback,
            10,
        )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self.publish_command)

        if self.all_logging_info:
            self.get_logger().info(
                f'Ready. publishing MotorParam on "{self.command_topic}" at {self.rate:.1f} Hz '
                f'and listening on "{self.feedback_topic}"'
            )

    def publish_command(self):
        elapsed = self.get_clock().now() - self.start_time
        elapsed_s = elapsed.nanoseconds / 1e9
        step = int(elapsed_s / self.step_duration) % 2
        target_q = self.target_q if step == 0 else -self.target_q

        msg = MotorParam()
        msg.q = target_q
        msg.kp = self.kp
        msg.kd = self.kd
        msg.tau = self.tau

        self.publisher.publish(msg)
        if self.all_logging_info:
            self.get_logger().info(
                f'Published command: q={msg.q:.4f}, kp={msg.kp:.4f}, kd={msg.kd:.4f}, tau={msg.tau:.4f}'
            )

    def feedback_callback(self, msg: MotorFeedback):
        count = len(msg.motors)
        if count == 0:
            self.get_logger().warn('Received empty MotorFeedback message')
            return

        index = min(self.motor_index, count - 1)
        motor = msg.motors[index]
        self.get_logger().info(
            f'Feedback[{index}] {motor.name}: q={motor.q:.4f}, q_dot={motor.q_dot:.4f} ' 
            f'({count} motors total)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
