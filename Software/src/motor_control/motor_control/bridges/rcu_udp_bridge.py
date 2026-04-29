#!/usr/bin/env python3
"""
rcu_udp_bridge.py — ROS2 node bridging RCU UDP binary protocol ↔ ROS2 topics

Replaces ethernet_can_bridge.py and nucleo_can_bridge in the PROJ500 Thor stack.

TX (Thor → RCU, port 7701):
  Subscribes /robot_command  → encodes as Type 0x10 motor cmd packets at 200 Hz
  Accepts Type 0x11 supervisory via ROS2 service /rcu_motor_estop

RX (RCU → Thor, port 7700):
  Type 0x02 motor feedback (200 Hz) → /motor_can_feedback (UInt8MultiArray)
  Type 0x04 fast IMU        (200 Hz) → /imu0, /imu1 (sensor_msgs/Imu)
  Type 0x01 slow telem      (10  Hz) → /rcu_pdu_telem (JSON String) + CSV log

Services:
  /rcu_motor_estop  (std_srvs/SetBool)
      True  → enable all motors  (Type 0x11 enable_mask=0x0FFF)
      False → FULL E-STOP: Type 0x11 enable_mask=0 + PDU fault assert
  /rcu_pdu_fault    (std_srvs/SetBool)
      True  → assert PDU fault (cuts power rails via debug cmd 0x07)
      False → clear PDU fault

Parameters:
  rcu_ip         (str)  default "192.168.100.10"
  rcu_cmd_port   (int)  default 7701
  telem_port     (int)  default 7700
  ctrl_mode      (int)  default 0  (0=MIT impedance Phase 2, 1=CSP pos Phase 1)
  auto_enable    (bool) default False
  log_dir        (str)  default "~/rcu_logs"
  loop_rate_hz   (float) default 200.0  (motor command TX rate)

Usage:
  ros2 run motor_control rcu_udp_bridge
  or from the launch file: ros2 launch motor_control rcu_launch.py
"""
import os
import csv
import socket
import struct
import threading
import time
import json
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import UInt8MultiArray, String
from std_srvs.srv import SetBool
from motor_control.msg import RobotCommand
HAS_ROBOT_CMD = True

from motor_control import rcu_protocol as rp

# ---------------------------------------------------------------------------
# IMU conversion constants (from rcu_protocol.py)
# ---------------------------------------------------------------------------
ACCEL_M_S2  = rp.IMU_ACCEL_SCALE * 9.80665  # → m/s²
GYRO_RAD_S  = rp.IMU_GYRO_SCALE  * (math.pi / 180.0)  # → rad/s

# ---------------------------------------------------------------------------
# Motor feedback format for /motor_can_feedback
# Compatible with robot_observation_bridge.py "FBK" format:
# 8 bytes/motor: pos_rad (float32 LE) + vel_rads (float32 LE)
# Ordered by motor_id 1→12
# ---------------------------------------------------------------------------
FB_ENTRY_FMT = "<ff"
FB_ENTRY_SIZE = struct.calcsize(FB_ENTRY_FMT)  # 8 bytes


class RcuUdpBridge(Node):
    def __init__(self):
        super().__init__("rcu_udp_bridge")

        # ----- Parameters -----
        self.declare_parameter("rcu_ip",       rp.RCU_IP)
        self.declare_parameter("rcu_cmd_port", rp.PORT_CMD)
        self.declare_parameter("telem_port",   rp.PORT_TELEM)
        self.declare_parameter("ctrl_mode",    0)
        self.declare_parameter("auto_enable",  False)
        self.declare_parameter("log_dir",      os.path.expanduser("~/rcu_logs"))
        self.declare_parameter("loop_rate_hz", 200.0)

        rcu_ip      = self.get_parameter("rcu_ip").value
        cmd_port    = self.get_parameter("rcu_cmd_port").value
        telem_port  = self.get_parameter("telem_port").value
        self._ctrl_mode  = self.get_parameter("ctrl_mode").value
        auto_enable = self.get_parameter("auto_enable").value
        log_dir     = self.get_parameter("log_dir").value
        rate_hz     = self.get_parameter("loop_rate_hz").value

        # ----- Sockets -----
        self._rcu_addr = (rcu_ip, cmd_port)
        self._tx_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.bind(("", telem_port))
        self._rx_sock.settimeout(0.05)

        # ----- Cached motor state -----
        # motor_id → (pos_rad, vel_rads); initialised to zeros
        self._motor_fb = {mid: (0.0, 0.0) for mid in range(1, 13)}
        self._motor_fb_lock = threading.Lock()

        # Latest robot command (joint positions) from /robot_command
        self._cmd_positions = {mid: 0.0 for mid in range(1, 13)}
        self._cmd_lock = threading.Lock()

        # ----- QoS -----
        best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Publishers -----
        self._pub_motor_fb = self.create_publisher(
            UInt8MultiArray, "/motor_can_feedback", 10)
        self._pub_pdu_telem = self.create_publisher(String, "/rcu_pdu_telem", 10)

        # ----- Subscribers -----
        if HAS_ROBOT_CMD:
            self._sub_cmd = self.create_subscription(
                RobotCommand, "/robot_command",
                self._robot_command_cb, best_effort)
        else:
            self.get_logger().warn(
                "custom_msgs.msg.RobotCommand not found — "
                "/robot_command subscription disabled")

        # ----- Services -----
        self._svc_estop = self.create_service(
            SetBool, "/rcu_motor_estop", self._estop_cb)
        self._svc_fault = self.create_service(
            SetBool, "/rcu_pdu_fault", self._pdu_fault_cb)

        # ----- CSV log -----
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(log_dir, f"rcu_telem_{stamp}.csv")
        self._csv_file   = open(log_path, "w", newline="")
        self._csv_writer = None  # created on first row (need field names)
        self.get_logger().info(f"Logging PDU telem to {log_path}")

        # ----- RX thread -----
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # ----- TX timer (motor commands at rate_hz) -----
        period = 1.0 / rate_hz
        self._tx_timer = self.create_timer(period, self._tx_tick)

        # ----- Auto-enable -----
        if auto_enable:
            self.get_logger().info("auto_enable=True — enabling all motors")
            self._send_enable_all()

        self.get_logger().info(
            f"rcu_udp_bridge ready: RCU={rcu_ip}, ctrl_mode={self._ctrl_mode}, "
            f"{rate_hz:.0f} Hz TX")

    # -----------------------------------------------------------------------
    # TX path
    # -----------------------------------------------------------------------
    def _tx_tick(self):
        """Called at loop_rate_hz.  Build and send motor command packet."""
        with self._cmd_lock:
            entries = [
                {"motor_id": mid, "pos_rad": self._cmd_positions[mid]}
                for mid in range(1, 13)
            ]
        pkt = rp.encode_motor_cmd_packet(entries)
        self._tx_sock.sendto(pkt, self._rcu_addr)

    def _robot_command_cb(self, msg):
        """Map /robot_command joint positions → internal commanded positions."""
        if not hasattr(msg, "joint_names") or not hasattr(msg, "q_des"):
            return
        with self._cmd_lock:
            for name, pos in zip(msg.joint_names, msg.q_des):
                mid = rp.JOINT_TO_MOTOR_ID.get(name)
                if mid is not None:
                    self._cmd_positions[mid] = float(pos)

    def _send(self, data: bytes):
        self._tx_sock.sendto(data, self._rcu_addr)

    def _send_enable_all(self):
        self._send(rp.encode_motor_supervisory(
            enable_mask=0x0FFF, clear_fault_mask=0x0FFF,
            ctrl_mode=self._ctrl_mode))

    def _send_full_estop(self):
        """FULL E-STOP: CAN stop all motors + assert PDU power fault."""
        self._send(rp.encode_motor_supervisory(enable_mask=0x0000))
        self._send(rp.encode_debug_cmd(rp.DBGCMD_ASSERT_PDU_FAULT, bytes([1])))
        self.get_logger().warn("FULL E-STOP: motors disabled + PDU fault asserted")

    # -----------------------------------------------------------------------
    # Services
    # -----------------------------------------------------------------------
    def _estop_cb(self, request, response):
        if request.data:
            self._send_enable_all()
            response.success = True
            response.message = "Motors enabled"
        else:
            self._send_full_estop()
            response.success = True
            response.message = "FULL E-STOP: CAN stop + PDU fault"
        return response

    def _pdu_fault_cb(self, request, response):
        self._send(rp.encode_debug_cmd(
            rp.DBGCMD_ASSERT_PDU_FAULT, bytes([1 if request.data else 0])))
        response.success = True
        response.message = "PDU fault asserted" if request.data else "PDU fault cleared"
        return response

    # -----------------------------------------------------------------------
    # RX path
    # -----------------------------------------------------------------------
    def _rx_loop(self):
        while self._running:
            try:
                data, _ = self._rx_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as exc:
                self.get_logger().error(f"RX error: {exc}")
                break
            hdr = rp.parse_header(data)
            if not hdr:
                continue
            pkt_type, _seq, _plen = hdr
            payload = data[rp.HDR_SIZE:]

            if pkt_type == rp.PKT_MOTOR_FB:
                self._handle_motor_fb(payload)
            # IMU disabled — feedback-only observation
            # elif pkt_type == rp.PKT_IMU_FAST:
            #     self._handle_imu_fast(payload)
            elif pkt_type == rp.PKT_SLOW_TELEM:
                self._handle_slow_telem(payload)
            elif pkt_type == rp.PKT_DEBUG_REPLY:
                self.get_logger().debug("Debug reply received")

    def _handle_motor_fb(self, payload: bytes):
        """Decode motor feedback and publish /motor_can_feedback."""
        slots = rp.decode_motor_fb(payload)
        with self._motor_fb_lock:
            for s in slots:
                mid = s["motor_id"]
                if 1 <= mid <= 12:
                    self._motor_fb[mid] = (s["pos_rad"], s["vel_rads"])

        # Publish ordered 8-byte entries for motor_ids 1–12
        buf = b""
        with self._motor_fb_lock:
            for mid in range(1, 13):
                pos, vel = self._motor_fb[mid]
                buf += struct.pack(FB_ENTRY_FMT, pos, vel)
        msg = UInt8MultiArray()
        msg.data = list(buf)
        self._pub_motor_fb.publish(msg)

    # IMU publishing disabled — feedback-only observation
    # def _handle_imu_fast(self, payload: bytes):
    #     """Decode fast IMU and publish /imu0 + /imu1."""
    #     d = rp.decode_imu_fast(payload)
    #     if not d:
    #         return
    #     now = self.get_clock().now().to_msg()
    #
    #     def make_imu(accel_g, gyro_dps):
    #         msg = Imu()
    #         msg.header.stamp    = now
    #         msg.header.frame_id = "imu_link"
    #         msg.linear_acceleration.x = accel_g[0] * 9.80665
    #         msg.linear_acceleration.y = accel_g[1] * 9.80665
    #         msg.linear_acceleration.z = accel_g[2] * 9.80665
    #         msg.angular_velocity.x    = gyro_dps[0] * (math.pi / 180.0)
    #         msg.angular_velocity.y    = gyro_dps[1] * (math.pi / 180.0)
    #         msg.angular_velocity.z    = gyro_dps[2] * (math.pi / 180.0)
    #         # Orientation unknown — set covariance to -1 flag
    #         msg.orientation_covariance[0] = -1.0
    #         return msg
    #
    #     self._pub_imu0.publish(make_imu(d["imu0_accel_g"], d["imu0_gyro_dps"]))
    #     self._pub_imu1.publish(make_imu(d["imu1_accel_g"], d["imu1_gyro_dps"]))

    def _handle_slow_telem(self, payload: bytes):
        """Decode slow telem, publish JSON to /rcu_pdu_telem, log to CSV."""
        d = rp.decode_slow_telem(payload)
        if not d:
            return
        raw = d.pop("_raw", {})

        # Publish JSON string
        msg = String()
        msg.data = json.dumps(d)
        self._pub_pdu_telem.publish(msg)

        # CSV log
        row = {"timestamp": time.time()}
        row.update(d)
        if self._csv_writer is None:
            self._csv_writer = csv.DictWriter(
                self._csv_file, fieldnames=list(row.keys()))
            self._csv_writer.writeheader()
        self._csv_writer.writerow(row)
        self._csv_file.flush()

    # -----------------------------------------------------------------------
    # Shutdown
    # -----------------------------------------------------------------------
    def destroy_node(self):
        self._running = False
        try:
            self._csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RcuUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down — sending motor disable...")
        node._send(rp.encode_motor_supervisory(enable_mask=0x0000))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
