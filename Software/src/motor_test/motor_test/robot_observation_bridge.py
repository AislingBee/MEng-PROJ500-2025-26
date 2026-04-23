#!/usr/bin/env python3
"""ROS2 bridge: MotorFeedback + IMU → RobotObservation.

Subscribes:
  /{feedback_topic}     (motor_test/MotorFeedback)  – joint positions & velocities
  /{imu_topic}          (sensor_msgs/Imu)            – orientation & angular velocity

Publishes:
  /{observation_topic}  (motor_test/RobotObservation)

The node publishes a new RobotObservation each time a MotorFeedback message
arrives.  If an IMU message has been received at least once the latest cached
value is used; otherwise gravity and gyro default to [0, 0, -1] and [0, 0, 0].

IMU orientation (quaternion) is used to project the gravity vector into the
body frame – matching the convention used by IsaacHardwareInterface and
RobotHardwareInterface in simulation/isaac/rl/interface/.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from motor_test.msg import MotorFeedback, RobotObservation


def _quat_rotate_inverse(
    qw: float, qx: float, qy: float, qz: float,
    vx: float, vy: float, vz: float,
) -> tuple[float, float, float]:
    """Rotate vector v by the *inverse* (conjugate) of unit quaternion q.

    Implements: v' = q* ⊗ v ⊗ q  (passive rotation / body-frame projection).
    Equivalent to IsaacLab's quat_rotate_inverse with scalar-first convention.
    """
    # Cross product  q_vec × v  where q_vec = (qx, qy, qz)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # v' = v + qw * (q_vec × v) + q_vec × (q_vec × v)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class RobotObservationBridge(Node):
    """Aggregates motor feedback and IMU data into a RobotObservation message."""

    def __init__(self):
        super().__init__('robot_observation_bridge')

        self.declare_parameter('feedback_topic', 'motor_feedback')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('observation_topic', 'robot_observation')
        self.declare_parameter('all_logging_info', False)

        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.observation_topic = self.get_parameter('observation_topic').value
        self.all_logging_info = bool(self.get_parameter('all_logging_info').value)

        self._lock = threading.Lock()
        self._latest_imu: Imu | None = None

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
            f'({self.feedback_topic} + {self.imu_topic}) → {self.observation_topic}'
        )

    # ------------------------------------------------------------------
    def _imu_callback(self, msg: Imu) -> None:
        with self._lock:
            self._latest_imu = msg

    # ------------------------------------------------------------------
    def _feedback_callback(self, msg: MotorFeedback) -> None:
        with self._lock:
            imu = self._latest_imu

        obs = RobotObservation()
        obs.timestamp_s = self.get_clock().now().nanoseconds * 1e-9

        # Joint state from motor feedback
        obs.joint_pos_rad = [float(m.q) for m in msg.motors]
        obs.joint_vel_rad_s = [float(m.q_dot) for m in msg.motors]
        obs.joint_effort_nm = []  # not available from the current CAN feedback

        # IMU-derived gravity and gyro in body frame
        if imu is not None:
            qw = imu.orientation.w
            qx = imu.orientation.x
            qy = imu.orientation.y
            qz = imu.orientation.z

            # Normalise quaternion to guard against near-zero edge cases
            norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if norm > 1e-6:
                qw /= norm
                qx /= norm
                qy /= norm
                qz /= norm
            else:
                qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

            gx, gy, gz = _quat_rotate_inverse(qw, qx, qy, qz, 0.0, 0.0, -1.0)
            obs.projected_gravity_b = [gx, gy, gz]
            obs.imu_gyro_b = [
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            ]
        else:
            # Safe fallback: robot assumed upright, no rotation
            obs.projected_gravity_b = [0.0, 0.0, -1.0]
            obs.imu_gyro_b = [0.0, 0.0, 0.0]

        self.publisher.publish(obs)

        if self.all_logging_info:
            n = len(obs.joint_pos_rad)
            grav = obs.projected_gravity_b
            self.get_logger().info(
                f'Published RobotObservation: {n} joints, '
                f'grav=[{grav[0]:.3f},{grav[1]:.3f},{grav[2]:.3f}]'
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
