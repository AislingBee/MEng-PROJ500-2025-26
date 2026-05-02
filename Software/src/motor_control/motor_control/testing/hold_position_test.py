#!/usr/bin/env python3
"""
hold_position_test.py — Publish fixed hold-position RobotCommand traffic.

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
    motor_ids (string) default "9,10"  — motor IDs to command (subset of 1..12)
  rate_hz   (float) default 200.0  — publish rate
  kp        (float) default 20.0   — position gain  [Nm/rad]
  kd        (float) default 1.0    — velocity gain  [Nm·s/rad]

Usage:
  ros2 run motor_control hold_position_test.py
    ros2 run motor_control hold_position_test.py --ros-args -p motor_ids:="9,10"
  ros2 run motor_control hold_position_test.py --ros-args -p kp:=10.0 -p kd:=0.5
"""

import rclpy
from rclpy.node import Node
from motor_control.msg import RobotCommand

# Position references captured from the 2026-05-02 diagnosis run [rad].
HOLD_REF_BY_MOTOR_ID = {
    1: ("pelvis_link_l_yaw_joint", 4.2437),
    2: ("pelvis_link_r_yaw_joint", 3.4239),
    3: ("l_hip_yaw_link_l_pitch_joint", 3.6871),
    4: ("r_hip_yaw_link_r_pitch_joint", 4.8276),
    5: ("l_hip_pitch_link_l_roll_joint", 2.0337),
    6: ("r_hip_pitch_link_r_roll_joint", 3.7780),
    7: ("l_thigh_link_l_knee_joint", 3.2755),
    8: ("r_thigh_link_r_knee_joint", 0.1026),
    9: ("l_shank_link_l_ankle_joint", 1.2),
    10: ("r_shank_link_r_ankle_joint", 3.0),
    11: ("l_ankle_link_l_foot_joint", 2.7925),
    12: ("r_ankle_link_r_foot_joint", 6.0962),
}


class HoldPositionTest(Node):
    def __init__(self):
        super().__init__("hold_position_test")

        self.declare_parameter("motor_ids", "9,10")
        self.declare_parameter("rate_hz", 200.0)
        self.declare_parameter("kp", 20.0)
        self.declare_parameter("kd", 1.0)

        motor_ids_raw = str(self.get_parameter("motor_ids").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self._kp = float(self.get_parameter("kp").value)
        self._kd = float(self.get_parameter("kd").value)

        requested_ids = []
        for token in motor_ids_raw.replace("[", "").replace("]", "").split(","):
            tok = token.strip()
            if not tok:
                continue
            try:
                mid = int(tok)
            except ValueError:
                continue
            if 1 <= mid <= 12 and mid not in requested_ids:
                requested_ids.append(mid)
        if not requested_ids:
            requested_ids = [9, 10]

        self._joint_names = [HOLD_REF_BY_MOTOR_ID[mid][0] for mid in requested_ids]
        self._q_des = [HOLD_REF_BY_MOTOR_ID[mid][1] for mid in requested_ids]
        self._n = len(self._joint_names)

        self._pub = self.create_publisher(RobotCommand, "/robot_command", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"hold_position_test: holding motor IDs {requested_ids} "
            f"at {rate_hz:.0f} Hz (kp={self._kp}, kd={self._kd})"
        )

    def _tick(self):
        msg = RobotCommand()
        msg.joint_names = self._joint_names
        msg.q_des = self._q_des
        msg.qd_des = [0.0] * self._n
        msg.tau_ff = [0.0] * self._n
        msg.kp_gains = [self._kp] * self._n
        msg.kd_gains = [self._kd] * self._n
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
