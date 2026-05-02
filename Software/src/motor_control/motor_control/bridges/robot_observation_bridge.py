#!/usr/bin/env python3
"""Reads motor feedback and IMU into a RobotObservation message."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from motor_control.msg import MotorFeedback, RobotObservation


class RobotObservationBridge(Node):
    """Reads motor feedback and IMU into a RobotObservation message."""

    def __init__(self):
        super().__init__('robot_observation_bridge')

        self.declare_parameter('feedback_topic', 'motor_feedback')
        self.declare_parameter('imu_topic', 'imu0')
        self.declare_parameter('observation_topic', 'robot_observation')
        self.declare_parameter('all_logging_info', False)
        self.declare_parameter('log_hz', 2.0)

        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.observation_topic = self.get_parameter('observation_topic').value
        self.all_logging_info = bool(self.get_parameter('all_logging_info').value)
        self.log_hz = max(0.1, float(self.get_parameter('log_hz').value))
        self._log_period_s = 1.0 / self.log_hz
        self._last_log_t = -1.0
        self._latest_imu = None

        self.feedback_sub = self.create_subscription(
            MotorFeedback,
            self.feedback_topic,
            self._feedback_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_callback,
            10,
        )
        self.publisher = self.create_publisher(RobotObservation, self.observation_topic, 1)

        self.get_logger().info(
            f'RobotObservationBridge ready: '
            f'{self.feedback_topic} + {self.imu_topic} → {self.observation_topic}'
        )

    def _imu_callback(self, msg: Imu) -> None:
        self._latest_imu = msg

    # ------------------------------------------------------------------
    def _feedback_callback(self, msg: MotorFeedback) -> None:
        obs = RobotObservation()
        obs.timestamp_s = self.get_clock().now().nanoseconds * 1e-9

        # Joint state from motor feedback (read-only)
        obs.joint_pos_rad = [float(m.q) for m in msg.motors]
        obs.joint_vel_rad_s = [float(m.q_dot) for m in msg.motors]
        obs.joint_effort_nm = []  # not available from the current CAN feedback

        imu = self._latest_imu
        if imu is None:
            obs.projected_gravity_b = [0.0, 0.0, -1.0]
            obs.imu_gyro_b = [0.0, 0.0, 0.0]
        else:
            ax = float(imu.linear_acceleration.x)
            ay = float(imu.linear_acceleration.y)
            az = float(imu.linear_acceleration.z)
            norm = math.sqrt(ax * ax + ay * ay + az * az)
            if norm > 1e-6:
                # IMU linear acceleration includes gravity with opposite sign.
                obs.projected_gravity_b = [-ax / norm, -ay / norm, -az / norm]
            else:
                obs.projected_gravity_b = [0.0, 0.0, -1.0]
            obs.imu_gyro_b = [
                float(imu.angular_velocity.x),
                float(imu.angular_velocity.y),
                float(imu.angular_velocity.z),
            ]

        self.publisher.publish(obs)

        if self.all_logging_info:
            now_s = obs.timestamp_s
            if self._last_log_t < 0.0 or (now_s - self._last_log_t) >= self._log_period_s:
                self._last_log_t = now_s
                n = len(obs.joint_pos_rad)
                grav = obs.projected_gravity_b
                self.get_logger().info(
                    f'Published RobotObservation: {n} joints, '
                    f'grav=[{grav[0]:.3f},{grav[1]:.3f},{grav[2]:.3f}] '
                    f'({self.log_hz:.1f} Hz log)'
                )


def main(args=None):
    rclpy.init(args=args)
    node = RobotObservationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
