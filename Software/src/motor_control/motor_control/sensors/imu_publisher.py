#!/usr/bin/env python3
# Subscribes: /<rcu_imu_topic>  (sensor_msgs/Imu)  — raw IMU from rcu_udp_bridge
# Publishes:  /<imu_topic>      (sensor_msgs/Imu)  — remapped into policy frame

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def imu_to_policy_frame(v_imu):
    """
    Convert IMU-frame vector into policy/root frame.

    IMU frame (from CAD/datasheet):
        +X = robot +X
        +Y = robot +Z
        +Z = robot -Y

    Mapping:
        policy_x = -imu_z
        policy_y =  imu_x
        policy_z = -imu_y

    Validation (upright, stationary robot):
        gravity_b  ≈ [0, 0, -1]
        gyro_b     ≈ [0, 0,  0]
    """
    x, y, z = float(v_imu[0]), float(v_imu[1]), float(v_imu[2])
    return (
        -z,
         x,
        -y,
    )


def deg_s_to_rad_s(v):
    """Convert angular velocity tuple from deg/s to rad/s."""
    k = math.pi / 180.0
    return tuple(a * k for a in v)


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('rcu_imu_topic', 'imu0')  # raw topic from rcu_udp_bridge
        self.declare_parameter('imu_topic', 'imu')       # remapped output topic
        self.declare_parameter('frame_id', 'imu_link')

        rcu_imu_topic  = self.get_parameter('rcu_imu_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.frame_id  = self.get_parameter('frame_id').value

        self.publisher  = self.create_publisher(Imu, self.imu_topic, 10)
        self.subscriber = self.create_subscription(
            Imu, rcu_imu_topic, self._imu_callback, 10)

        self.get_logger().info(
            f'ImuPublisher ready: {rcu_imu_topic} → remap → {self.imu_topic}'
        )

    # ------------------------------------------------------------------
    def _imu_callback(self, raw: Imu) -> None:
        msg = Imu()
        msg.header          = raw.header
        msg.header.frame_id = self.frame_id

        # Orientation is passed through unchanged — quaternion remapping
        # requires a different transform and is handled downstream if needed.
        msg.orientation = raw.orientation

        # Angular velocity: remap from IMU frame to policy frame.
        gx, gy, gz = imu_to_policy_frame((
            raw.angular_velocity.x,
            raw.angular_velocity.y,
            raw.angular_velocity.z,
        ))
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Linear acceleration: remap from IMU frame to policy frame.
        ax, ay, az = imu_to_policy_frame((
            raw.linear_acceleration.x,
            raw.linear_acceleration.y,
            raw.linear_acceleration.z,
        ))
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.orientation_covariance         = raw.orientation_covariance
        msg.angular_velocity_covariance    = raw.angular_velocity_covariance
        msg.linear_acceleration_covariance = raw.linear_acceleration_covariance

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
