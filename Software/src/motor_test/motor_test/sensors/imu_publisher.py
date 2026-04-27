#!/usr/bin/env python3
# Subscribes: None (reads directly from IMU hardware)
# Publishes: /imu (sensor_msgs/Imu)
#
# Template IMU publisher. Fill in the three TODO sections with your
# hardware-specific read calls, then this node feeds robot_observation_bridge.py.

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100.0)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.rate_hz   = float(self.get_parameter('rate_hz').value)

        self.publisher = self.create_publisher(Imu, self.imu_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_callback)

        # TODO: initialise your IMU hardware here.
        # Examples:
        #   self.imu = smbus2.SMBus(1)          # I2C on Raspberry Pi
        #   self.imu = serial.Serial('/dev/ttyUSB0', 115200)  # UART
        #   self.imu = YourDriverClass()

        self.get_logger().info(
            f'ImuPublisher ready: publishing on "{self.imu_topic}" at {self.rate_hz} Hz'
        )

    # ------------------------------------------------------------------
    def _timer_callback(self) -> None:
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # ----------------------------------------------------------------
        # TODO: replace the zero values below with real reads from the IMU.
        #
        # Orientation — unit quaternion (w, x, y, z), scalar-first.
        #   If your driver returns Euler angles (roll, pitch, yaw) convert with:
        #     cy, sy = cos(yaw/2), sin(yaw/2)
        #     cp, sp = cos(pitch/2), sin(pitch/2)
        #     cr, sr = cos(roll/2), sin(roll/2)
        #     qw = cr*cp*cy + sr*sp*sy
        #     qx = sr*cp*cy - cr*sp*sy
        #     qy = cr*sp*cy + sr*cp*sy
        #     qz = cr*cp*sy - sr*sp*cy
        #
        # Angular velocity — rad/s in body frame (x = roll-rate, y = pitch-rate, z = yaw-rate).
        # Linear acceleration — m/s² in body frame, including gravity.
        # ----------------------------------------------------------------

        # raw_qw, raw_qx, raw_qy, raw_qz = self.imu.read_quaternion()
        raw_qw, raw_qx, raw_qy, raw_qz = 1.0, 0.0, 0.0, 0.0

        # raw_gx, raw_gy, raw_gz = self.imu.read_gyro()  # rad/s
        raw_gx, raw_gy, raw_gz = 0.0, 0.0, 0.0

        # raw_ax, raw_ay, raw_az = self.imu.read_accel()  # m/s²
        raw_ax, raw_ay, raw_az = 0.0, 0.0, -9.81

        # ----------------------------------------------------------------
        # Normalise quaternion (protects against sensor drift / scale errors).
        norm = math.sqrt(raw_qw**2 + raw_qx**2 + raw_qy**2 + raw_qz**2)
        if norm > 1e-6:
            raw_qw /= norm
            raw_qx /= norm
            raw_qy /= norm
            raw_qz /= norm
        else:
            raw_qw, raw_qx, raw_qy, raw_qz = 1.0, 0.0, 0.0, 0.0

        msg.orientation.w = raw_qw
        msg.orientation.x = raw_qx
        msg.orientation.y = raw_qy
        msg.orientation.z = raw_qz

        msg.angular_velocity.x = raw_gx
        msg.angular_velocity.y = raw_gy
        msg.angular_velocity.z = raw_gz

        msg.linear_acceleration.x = raw_ax
        msg.linear_acceleration.y = raw_ay
        msg.linear_acceleration.z = raw_az

        # Covariance matrices (9-element row-major 3×3).
        # Set diagonal to a small positive value; -1 means "unknown".
        # TODO: tune these to your IMU's datasheet noise figures.
        msg.orientation_covariance        = [1e-4, 0.0, 0.0,
                                             0.0, 1e-4, 0.0,
                                             0.0, 0.0, 1e-4]
        msg.angular_velocity_covariance   = [1e-4, 0.0, 0.0,
                                             0.0, 1e-4, 0.0,
                                             0.0, 0.0, 1e-4]
        msg.linear_acceleration_covariance = [1e-3, 0.0, 0.0,
                                              0.0, 1e-3, 0.0,
                                              0.0, 0.0, 1e-3]

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
