#!/usr/bin/env python3
"""
hold_position_test.py — Publish a fixed hold-position RobotCommand for all 12 motors.

Holds each joint at the position captured during the diagnosis run on 2026-05-02:
  pelvis_link_l_yaw_joint       : 4.2437 rad
  pelvis_link_r_yaw_joint       : 3.4239 rad
  l_hip_yaw_link_l_pitch_joint  : 3.6871 rad
  r_hip_yaw_link_r_pitch_joint  : 4.8276 rad
  l_hip_pitch_link_l_roll_joint : 2.0337 rad
  r_hip_pitch_link_r_roll_joint : 3.7780 rad
  l_thigh_link_l_knee_joint     : 3.2755 rad
  r_thigh_link_r_knee_joint     : 0.1026 rad
  l_shank_link_l_ankle_joint    : 2.9816 rad
  r_shank_link_r_ankle_joint    : 1.7341 rad
  l_ankle_link_l_foot_joint     : 2.7925 rad
  r_ankle_link_r_foot_joint     : 6.0962 rad

Parameters (all overridable via --ros-args -p):
  rate_hz   (float) default 200.0  — publish rate
  kp        (float) default 20.0   — position gain  [Nm/rad]
  kd        (float) default 1.0    — velocity gain  [Nm·s/rad]

Usage:
  ros2 run motor_control hold_position_test.py
  ros2 run motor_control hold_position_test.py --ros-args -p kp:=10.0 -p kd:=0.5
"""

import rclpy
from rclpy.node import Node
from motor_control.msg import RobotCommand

# Joint names in motor_id order (1 → 12), matching the bridge's JOINT_TO_MOTOR_ID map.
JOINT_NAMES = [
    "pelvis_link_l_yaw_joint",        # motor 1
    "pelvis_link_r_yaw_joint",        # motor 2
    "l_hip_yaw_link_l_pitch_joint",   # motor 3
    "r_hip_yaw_link_r_pitch_joint",   # motor 4
    "l_hip_pitch_link_l_roll_joint",  # motor 5
    "r_hip_pitch_link_r_roll_joint",  # motor 6
    "l_thigh_link_l_knee_joint",      # motor 7
    "r_thigh_link_r_knee_joint",      # motor 8
    "l_shank_link_l_ankle_joint",     # motor 9
    "r_shank_link_r_ankle_joint",     # motor 10
    "l_ankle_link_l_foot_joint",      # motor 11
    "r_ankle_link_r_foot_joint",      # motor 12
]

# Positions captured from the 2026-05-02 diagnosis run [rad].
HOLD_POSITIONS = [
    4.2437,  # pelvis_link_l_yaw_joint
    3.4239,  # pelvis_link_r_yaw_joint
    3.6871,  # l_hip_yaw_link_l_pitch_joint
    4.8276,  # r_hip_yaw_link_r_pitch_joint
    2.0337,  # l_hip_pitch_link_l_roll_joint
    3.7780,  # r_hip_pitch_link_r_roll_joint
    3.2755,  # l_thigh_link_l_knee_joint
    0.1026,  # r_thigh_link_r_knee_joint
    2.9816,  # l_shank_link_l_ankle_joint
    1.7341,  # r_shank_link_r_ankle_joint
    2.7925,  # l_ankle_link_l_foot_joint
    6.0962,  # r_ankle_link_r_foot_joint
]

N = len(JOINT_NAMES)


class HoldPositionTest(Node):
    def __init__(self):
        super().__init__("hold_position_test")

        self.declare_parameter("rate_hz", 200.0)
        self.declare_parameter("kp", 20.0)
        self.declare_parameter("kd", 1.0)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._kp = float(self.get_parameter("kp").value)
        self._kd = float(self.get_parameter("kd").value)

        self._pub = self.create_publisher(RobotCommand, "/robot_command", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"hold_position_test: holding {N} joints at diagnosis positions "
            f"at {rate_hz:.0f} Hz (kp={self._kp}, kd={self._kd})"
        )

    def _tick(self):
        msg = RobotCommand()
        msg.joint_names = JOINT_NAMES
        msg.q_des    = HOLD_POSITIONS
        msg.qd_des   = [0.0] * N
        msg.tau_ff   = [0.0] * N
        msg.kp_gains = [self._kp] * N
        msg.kd_gains = [self._kd] * N
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HoldPositionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
