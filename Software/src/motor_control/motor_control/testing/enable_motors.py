#!/usr/bin/env python3
"""
Send one-shot motor enable commands to the RCU and exit.

This utility is intentionally narrow in scope: it only enables selected motors.
It sends, in order:
  1) bus standby control (DBGCMD_MOTOR_BUS_CTRL)
  2) supervisory enable mask (Type 0x11)
  3) per-motor enables (DBGCMD_MOTOR_ENABLE)

Usage examples:
  ros2 run motor_control enable_motors.py
  ros2 run motor_control enable_motors.py --ros-args -p active_motor_ids:="[9,10]" -p left_bus_motor_ids:="[9]" -p right_bus_motor_ids:="[10]"
"""

import socket
import time

import rclpy
from rclpy.node import Node

from motor_control import rcu_protocol as rp


class EnableMotors(Node):
    def __init__(self):
        super().__init__("enable_motors")

        self.declare_parameter("rcu_ip", rp.RCU_IP)
        self.declare_parameter("rcu_cmd_port", rp.PORT_CMD)
        self.declare_parameter("ctrl_mode", 0)
        self.declare_parameter("active_motor_ids", "[1,2,3,4,5,6,7,8,9,10,11,12]")
        self.declare_parameter("left_bus_motor_ids", "[1,3,5,7,9,11]")
        self.declare_parameter("right_bus_motor_ids", "[2,4,6,8,10,12]")
        self.declare_parameter("retries", 3)
        self.declare_parameter("retry_delay_s", 0.01)

        self._rcu_ip = str(self.get_parameter("rcu_ip").value)
        self._cmd_port = int(self.get_parameter("rcu_cmd_port").value)
        self._ctrl_mode = int(self.get_parameter("ctrl_mode").value)
        self._retries = max(1, int(self.get_parameter("retries").value))
        self._retry_delay_s = max(0.0, float(self.get_parameter("retry_delay_s").value))

        active_raw = self.get_parameter("active_motor_ids").value
        left_raw = self.get_parameter("left_bus_motor_ids").value
        right_raw = self.get_parameter("right_bus_motor_ids").value

        self._active_ids = self._parse_motor_ids(active_raw)
        self._left_ids = self._parse_motor_ids(left_raw)
        self._right_ids = self._parse_motor_ids(right_raw)

        if not self._active_ids:
            self._active_ids = list(range(1, 13))

        self._bus_map = list(rp.MOTOR_BUS_MAP)
        for mid in self._right_ids:
            self._bus_map[mid] = 0
        for mid in self._left_ids:
            self._bus_map[mid] = 1

        self._rcu_addr = (self._rcu_ip, self._cmd_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @staticmethod
    def _parse_motor_ids(raw):
        if isinstance(raw, str):
            txt = raw.strip().strip("[]")
            if not txt:
                return []
            parts = [p.strip() for p in txt.split(",") if p.strip()]
        elif isinstance(raw, (list, tuple)):
            parts = list(raw)
        else:
            parts = [raw]

        ids = []
        for p in parts:
            try:
                mid = int(p)
            except (TypeError, ValueError):
                continue
            if 1 <= mid <= 12 and mid not in ids:
                ids.append(mid)
        return ids

    def _send_with_retries(self, pkt: bytes):
        for i in range(self._retries):
            self._sock.sendto(pkt, self._rcu_addr)
            if i < (self._retries - 1):
                time.sleep(self._retry_delay_s)

    def run(self):
        left_active = any(self._bus_map[mid] == 1 for mid in self._active_ids)
        right_active = any(self._bus_map[mid] == 0 for mid in self._active_ids)
        standby_mask = 0
        if not left_active:
            standby_mask |= 0b01
        if not right_active:
            standby_mask |= 0b10

        self.get_logger().info(
            f"Enabling motors {self._active_ids} on {self._rcu_ip}:{self._cmd_port} "
            f"(ctrl_mode={self._ctrl_mode})"
        )
        self.get_logger().info(
            f"Bus standby mask 0b{standby_mask:02b} "
            f"(left_active={left_active}, right_active={right_active})"
        )

        self._send_with_retries(
            rp.encode_debug_cmd(rp.DBGCMD_MOTOR_BUS_CTRL, bytes([standby_mask]))
        )

        self._send_with_retries(
            rp.encode_motor_supervisory(
                enable_mask=0x0FFF,
                clear_fault_mask=0x0FFF,
                ctrl_mode=self._ctrl_mode,
            )
        )

        for mid in self._active_ids:
            bus = self._bus_map[mid]
            payload = bytes([bus & 0xFF, mid & 0xFF, 1, 1])
            self._send_with_retries(rp.encode_debug_cmd(rp.DBGCMD_MOTOR_ENABLE, payload))

        self.get_logger().info("Enable sequence complete")

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EnableMotors()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
