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
import queue
import socket
import struct
import threading
import time
import json
import math
from collections import defaultdict
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
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
        self._shutting_down = False

        def _parse_bool(value):
            if isinstance(value, bool):
                return value
            if isinstance(value, (int, float)):
                return bool(value)
            if isinstance(value, str):
                return value.strip().lower() in ("1", "true", "yes", "on")
            return False

        def _parse_int(value, default):
            try:
                return int(value)
            except (TypeError, ValueError):
                return default

        def _parse_float(value, default):
            try:
                return float(value)
            except (TypeError, ValueError):
                return default

        def _parse_motor_id_list(raw):
            values = raw
            if isinstance(raw, str):
                txt = raw.strip()
                if not txt:
                    values = []
                else:
                    try:
                        values = json.loads(txt)
                    except Exception:
                        # Accept simple CSV-like input: "1,2,3" or "[1,2,3]"
                        txt = txt.strip("[]")
                        values = [part.strip() for part in txt.split(",") if part.strip()]
            if isinstance(values, (int, float, str)):
                values = [values]
            if not isinstance(values, (list, tuple)):
                return []

            parsed = []
            for mid_raw in values:
                try:
                    mid = int(mid_raw)
                except (TypeError, ValueError):
                    continue
                if 1 <= mid <= 12 and mid not in parsed:
                    parsed.append(mid)
            return parsed

        # ----- Parameters -----
        self.declare_parameter("rcu_ip",       rp.RCU_IP)
        self.declare_parameter("rcu_cmd_port", rp.PORT_CMD)
        self.declare_parameter("telem_port",   rp.PORT_TELEM)
        self.declare_parameter("ctrl_mode",    0)
        self.declare_parameter("auto_enable",  False)
        self.declare_parameter("log_dir",      os.path.expanduser("~/rcu_logs"))
        self.declare_parameter("loop_rate_hz", 200.0)
        self.declare_parameter("scan_motor_can_ids", False)
        self.declare_parameter("can_id_online_timeout_s", 1.0)
        self.declare_parameter("can_id_scan_log_period_s", 1.0)
        self.declare_parameter("wait_for_expected_online_ids", False)
        self.declare_parameter("expected_online_motor_ids", [])
        self.declare_parameter("startup_gate_error_after_s", 5.0)
        # Keep these as strings in launch-facing APIs so IncludeLaunchDescription
        # and CLI argument overrides remain compatible on Jazzy.
        self.declare_parameter("left_bus_motor_ids", "[1,3,5,7,9,11]")
        self.declare_parameter("right_bus_motor_ids", "[2,4,6,8,10,12]")
        self.declare_parameter("active_motor_ids", "[1,2,3,4,5,6,7,8,9,10,11,12]")

        rcu_ip_raw = self.get_parameter("rcu_ip").value
        cmd_port_raw = self.get_parameter("rcu_cmd_port").value
        telem_port_raw = self.get_parameter("telem_port").value
        ctrl_mode_raw = self.get_parameter("ctrl_mode").value
        auto_enable_raw = self.get_parameter("auto_enable").value
        log_dir_raw = self.get_parameter("log_dir").value
        rate_hz_raw = self.get_parameter("loop_rate_hz").value
        scan_motor_can_ids_raw = self.get_parameter("scan_motor_can_ids").value
        can_id_online_timeout_s_raw = self.get_parameter("can_id_online_timeout_s").value
        can_id_scan_log_period_s_raw = self.get_parameter("can_id_scan_log_period_s").value
        wait_for_expected_online_ids_raw = self.get_parameter("wait_for_expected_online_ids").value
        expected_online_motor_ids_raw = self.get_parameter("expected_online_motor_ids").value
        startup_gate_error_after_s_raw = self.get_parameter("startup_gate_error_after_s").value
        left_bus_motor_ids_raw = self.get_parameter("left_bus_motor_ids").value
        right_bus_motor_ids_raw = self.get_parameter("right_bus_motor_ids").value
        active_motor_ids_raw = self.get_parameter("active_motor_ids").value

        rcu_ip = str(rcu_ip_raw)
        cmd_port = _parse_int(cmd_port_raw, rp.PORT_CMD)
        telem_port = _parse_int(telem_port_raw, rp.PORT_TELEM)
        self._ctrl_mode = _parse_int(ctrl_mode_raw, 0)
        auto_enable = _parse_bool(auto_enable_raw)
        log_dir = os.path.expanduser(str(log_dir_raw))
        rate_hz = _parse_float(rate_hz_raw, 200.0)
        self._scan_motor_can_ids = _parse_bool(scan_motor_can_ids_raw)
        self._can_id_online_timeout_s = max(0.1, _parse_float(can_id_online_timeout_s_raw, 1.0))
        self._can_id_scan_log_period_s = max(0.2, _parse_float(can_id_scan_log_period_s_raw, 1.0))
        self._wait_for_expected_online_ids = _parse_bool(wait_for_expected_online_ids_raw)
        self._expected_online_motor_ids = _parse_motor_id_list(expected_online_motor_ids_raw)
        self._startup_gate_error_after_s = max(0.0, _parse_float(startup_gate_error_after_s_raw, 5.0))
        left_bus_motor_ids = _parse_motor_id_list(left_bus_motor_ids_raw)
        right_bus_motor_ids = _parse_motor_id_list(right_bus_motor_ids_raw)
        active_motor_ids = _parse_motor_id_list(active_motor_ids_raw)

        self._active_motor_ids = []
        if active_motor_ids:
            self._active_motor_ids.extend(active_motor_ids)
        if not self._active_motor_ids:
            self._active_motor_ids = [i for i in range(1, 13)]

        # Per-motor bus map (index=motor_id): default from protocol, with optional
        # overrides for bench wiring (e.g. motor IDs 1 and 2 both on left bus).
        self._motor_bus_map = list(rp.MOTOR_BUS_MAP)
        for mid in right_bus_motor_ids:
            self._motor_bus_map[mid] = 0
        for mid in left_bus_motor_ids:
            self._motor_bus_map[mid] = 1

        # ----- Sockets -----
        self._rcu_addr = (rcu_ip, cmd_port)
        self._tx_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx_sock.bind(("", telem_port))
        self._rx_sock.settimeout(0.05)

        # ----- Cached motor state -----
        # motor_id → (pos_rad, vel_rads); initialised to zeros
        self._motor_fb = {mid: (0.0, 0.0) for mid in range(1, 13)}
        self._motor_fb_lock = threading.Lock()
        self._last_seen_motor_id = defaultdict(float)
        self._last_can_scan_log_t = 0.0
        self._startup_gate_ready = not self._wait_for_expected_online_ids
        self._last_gate_log_t = 0.0
        self._startup_gate_started_t = time.monotonic()

        # Latest robot command cache from /robot_command
        # Each entry carries full MIT fields so Type-1 control is effective.
        self._cmd_state = {
            mid: {
                "pos_rad": 0.0,
                "vel_rads": 0.0,
                "torque_nm": 0.0,
                "kp": 0.0,
                "kd": 0.0,
            }
            for mid in range(1, 13)
        }
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
        self._pub_imu0 = self.create_publisher(Imu, "/imu0", 10)
        self._pub_imu1 = self.create_publisher(Imu, "/imu1", 10)
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

        # Queue for passing RX decoded data to the main ROS thread
        self._rx_queue: queue.Queue = queue.Queue(maxsize=200)

        # ----- RX thread -----
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # ----- TX timer (motor commands at rate_hz) -----
        period = 1.0 / rate_hz
        self._tx_timer = self.create_timer(period, self._tx_tick)

        # ----- RX drain timer (publish on main thread at same rate) -----
        self._rx_timer = self.create_timer(period, self._rx_drain)

        # Ensure all selected buses are taken out of standby at startup.
        self._send_motor_bus_ctrl_for_active_ids()

        # ----- Auto-enable -----
        if auto_enable:
            self.get_logger().info("auto_enable=True — enabling all motors")
            self._send_enable_all()

        self.get_logger().info(
            f"rcu_udp_bridge ready: RCU={rcu_ip}, ctrl_mode={self._ctrl_mode}, "
            f"{rate_hz:.0f} Hz TX")
        self.get_logger().info(
            f"Active motor IDs for TX: {self._active_motor_ids}")
        id_map = ", ".join(
            f"{mid}:{rp.MOTOR_JOINT_NAMES[mid]}"
            for mid in self._active_motor_ids
            if 1 <= mid <= 12
        )
        if id_map:
            self.get_logger().info(f"CAN ID -> joint mapping: {id_map}")
        if left_bus_motor_ids:
            self.get_logger().info(
                f"Left-bus overrides active for motor IDs: {left_bus_motor_ids}")
        if right_bus_motor_ids:
            self.get_logger().info(
                f"Right-bus overrides active for motor IDs: {right_bus_motor_ids}")
        if self._scan_motor_can_ids:
            self.get_logger().info(
                "CAN ID scan enabled "
                f"(period={self._can_id_scan_log_period_s:.1f}s, "
                f"online_timeout={self._can_id_online_timeout_s:.1f}s)"
            )
        if self._wait_for_expected_online_ids:
            if not self._expected_online_motor_ids:
                self._expected_online_motor_ids = list(self._active_motor_ids)
            self.get_logger().info(
                "Startup gate enabled: waiting for online motor IDs "
                f"{self._expected_online_motor_ids} before TX"
            )

    # -----------------------------------------------------------------------
    # TX path
    # -----------------------------------------------------------------------
    def _tx_tick(self):
        """Called at loop_rate_hz.  Build and send motor command packet."""
        if not self._startup_gate_ready:
            now = time.monotonic()
            if (now - self._last_gate_log_t) >= 1.0:
                self._last_gate_log_t = now
                online = self._online_motor_ids(now)
                missing = [mid for mid in self._expected_online_motor_ids if mid not in online]
                elapsed = now - self._startup_gate_started_t
                self.get_logger().error(
                    "Startup gate blocking TX: "
                    f"online={online}, missing={missing}, elapsed={elapsed:.1f}s"
                )
                if self._startup_gate_error_after_s > 0.0 and elapsed >= self._startup_gate_error_after_s:
                    self.get_logger().error(
                        "Startup gate timeout exceeded. "
                        "Not all expected CAN IDs are online."
                    )
            return
        with self._cmd_lock:
            entries = [
                {"motor_id": mid, "bus": self._motor_bus_map[mid], **self._cmd_state[mid]}
                for mid in self._active_motor_ids
            ]
        pkt = rp.encode_motor_cmd_packet(entries)
        self._tx_sock.sendto(pkt, self._rcu_addr)

    def _robot_command_cb(self, msg):
        """Map /robot_command joint positions → internal commanded positions."""
        if not hasattr(msg, "joint_names") or not hasattr(msg, "q_des"):
            return

        q_des = list(getattr(msg, "q_des", []))
        qd_des = list(getattr(msg, "qd_des", []))
        tau_ff = list(getattr(msg, "tau_ff", []))

        # Prefer kp_gains/kd_gains if present, otherwise fall back to kp/kd.
        kp_src = list(getattr(msg, "kp_gains", [])) or list(getattr(msg, "kp", []))
        kd_src = list(getattr(msg, "kd_gains", [])) or list(getattr(msg, "kd", []))

        def _pick(arr, idx, default=0.0):
            if idx < len(arr):
                return float(arr[idx])
            return default

        with self._cmd_lock:
            for i, name in enumerate(msg.joint_names):
                mid = rp.JOINT_TO_MOTOR_ID.get(name)
                if mid is None and isinstance(name, str) and name.startswith("motor_"):
                    suffix = name.split("motor_", 1)[1]
                    if suffix.isdigit():
                        mid_val = int(suffix)
                        if 1 <= mid_val <= 12:
                            mid = mid_val
                if mid is not None:
                    self._cmd_state[mid]["pos_rad"] = _pick(q_des, i, self._cmd_state[mid]["pos_rad"])
                    self._cmd_state[mid]["vel_rads"] = _pick(qd_des, i, 0.0)
                    self._cmd_state[mid]["torque_nm"] = _pick(tau_ff, i, 0.0)
                    self._cmd_state[mid]["kp"] = _pick(kp_src, i, 0.0)
                    self._cmd_state[mid]["kd"] = _pick(kd_src, i, 0.0)

    def _send(self, data: bytes):
        self._tx_sock.sendto(data, self._rcu_addr)

    def _send_with_retries(self, data: bytes, retries: int = 3, delay_s: float = 0.01):
        for i in range(max(1, retries)):
            self._send(data)
            if i < (retries - 1):
                time.sleep(delay_s)

    def _send_motor_enable(self, bus: int, motor_id: int,
                           enable: bool = True, clear_fault: bool = False):
        payload = bytes([
            bus & 0xFF,
            motor_id & 0xFF,
            1 if enable else 0,
            1 if clear_fault else 0,
        ])
        self._send_with_retries(rp.encode_debug_cmd(rp.DBGCMD_MOTOR_ENABLE, payload), retries=3, delay_s=0.005)

    def _send_motor_bus_ctrl_for_active_ids(self):
        left_active = any(self._motor_bus_map[mid] == 1 for mid in self._active_motor_ids)
        right_active = any(self._motor_bus_map[mid] == 0 for mid in self._active_motor_ids)

        standby_mask = 0
        if not left_active:
            standby_mask |= 0b01
        if not right_active:
            standby_mask |= 0b10

        self._send_with_retries(
            rp.encode_debug_cmd(rp.DBGCMD_MOTOR_BUS_CTRL, bytes([standby_mask])),
            retries=3,
            delay_s=0.005,
        )
        self.get_logger().info(
            "Configured motor bus standby mask for active IDs: "
            f"0b{standby_mask:02b} (left_active={left_active}, right_active={right_active})"
        )

    def _send_enable_all(self):
        self._send_motor_bus_ctrl_for_active_ids()
        self._send_with_retries(
            rp.encode_motor_supervisory(
                enable_mask=0x0FFF, clear_fault_mask=0x0FFF,
                ctrl_mode=self._ctrl_mode,
            ),
            retries=3,
            delay_s=0.005,
        )
        for mid in self._active_motor_ids:
            self._send_motor_enable(
                bus=self._motor_bus_map[mid], motor_id=mid,
                enable=True, clear_fault=True)

    def _send_disable_all(self):
        """Disable active motors without asserting PDU fault (limp mode)."""
        self._send_motor_bus_ctrl_for_active_ids()
        self._send_with_retries(
            rp.encode_motor_supervisory(enable_mask=0x0000),
            retries=3,
            delay_s=0.005,
        )
        for mid in self._active_motor_ids:
            self._send_motor_enable(
                bus=self._motor_bus_map[mid], motor_id=mid,
                enable=False, clear_fault=False)

    def _send_full_estop(self):
        """FULL E-STOP: CAN stop all motors + assert PDU power fault."""
        self._send_with_retries(
            rp.encode_motor_supervisory(enable_mask=0x0000),
            retries=3,
            delay_s=0.005,
        )
        for mid in self._active_motor_ids:
            self._send_motor_enable(
                bus=self._motor_bus_map[mid], motor_id=mid,
                enable=False, clear_fault=False)
        self._send_with_retries(
            rp.encode_debug_cmd(rp.DBGCMD_ASSERT_PDU_FAULT, bytes([1])),
            retries=3,
            delay_s=0.005,
        )
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
                if not self._shutting_down:
                    self.get_logger().error(f"RX error: {exc}")
                break
            try:
                hdr = rp.parse_header(data)
                if not hdr:
                    continue
                pkt_type, _seq, plen = hdr
                if plen < 0:
                    continue
                payload = data[rp.HDR_SIZE:rp.HDR_SIZE + plen]

                if pkt_type == rp.PKT_MOTOR_FB:
                    slots = rp.decode_motor_fb(payload)
                    if slots:
                        try:
                            self._rx_queue.put_nowait(('motor_fb', slots))
                        except queue.Full:
                            pass
                elif pkt_type == rp.PKT_IMU_FAST:
                    d = rp.decode_imu_fast(payload)
                    if d:
                        try:
                            self._rx_queue.put_nowait(('imu_fast', d))
                        except queue.Full:
                            pass
                elif pkt_type == rp.PKT_SLOW_TELEM:
                    d = rp.decode_slow_telem(payload)
                    if d:
                        try:
                            self._rx_queue.put_nowait(('slow_telem', d))
                        except queue.Full:
                            pass
                elif pkt_type == rp.PKT_DEBUG_REPLY:
                    pass  # nothing to publish
            except Exception as exc:
                if not self._shutting_down:
                    self.get_logger().warn(f"Dropped malformed RX packet: {exc}")
                continue

    def _rx_drain(self):
        """Called on the main ROS thread — drains the RX queue and publishes."""
        while True:
            try:
                kind, data = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            if kind == 'motor_fb':
                self._publish_motor_fb(data)
            elif kind == 'imu_fast':
                self._publish_imu_fast(data)
            elif kind == 'slow_telem':
                self._publish_slow_telem(data)

    def _handle_motor_fb(self, payload: bytes):
        """Decode motor feedback payload and publish /motor_can_feedback."""
        slots = rp.decode_motor_fb(payload)
        self._publish_motor_fb(slots)

    def _publish_motor_fb(self, slots: list):
        """Publish decoded motor feedback on the main ROS thread."""
        now = time.monotonic()
        with self._motor_fb_lock:
            for s in slots:
                mid = s["motor_id"]
                if 1 <= mid <= 12:
                    self._motor_fb[mid] = (s["pos_rad"], s["vel_rads"])
                    self._last_seen_motor_id[mid] = now

        # Publish ordered 8-byte entries for motor_ids 1–12
        buf = b""
        with self._motor_fb_lock:
            for mid in range(1, 13):
                pos, vel = self._motor_fb[mid]
                buf += struct.pack(FB_ENTRY_FMT, pos, vel)
        msg = UInt8MultiArray()
        msg.data = list(buf)
        self._pub_motor_fb.publish(msg)

        # Always evaluate startup gate from fresh feedback, even if scan logging is disabled.
        self._update_startup_gate(now, self._online_motor_ids(now))

        if self._scan_motor_can_ids:
            self._log_online_can_ids(now)

    def _log_online_can_ids(self, now: float):
        if (now - self._last_can_scan_log_t) < self._can_id_scan_log_period_s:
            return
        self._last_can_scan_log_t = now
        online = self._online_motor_ids(now)
        if online:
            online_named = [f"{mid}:{rp.MOTOR_JOINT_NAMES[mid]}" for mid in online]
            self.get_logger().info(f"CAN scan online motor IDs: {online_named}")
        else:
            self.get_logger().error("CAN scan online motor IDs: []")

        if self._expected_online_motor_ids:
            missing = [mid for mid in self._expected_online_motor_ids if mid not in online]
            if missing:
                self.get_logger().error(
                    f"Missing expected CAN IDs: {missing}"
                )

        self._update_startup_gate(now, online)

    def _online_motor_ids(self, now: float):
        online = []
        for mid in range(1, 13):
            last = self._last_seen_motor_id.get(mid, 0.0)
            if last > 0.0 and (now - last) <= self._can_id_online_timeout_s:
                online.append(mid)
        return online

    def _update_startup_gate(self, now: float, online_ids: list):
        if self._startup_gate_ready or not self._wait_for_expected_online_ids:
            return
        missing = [mid for mid in self._expected_online_motor_ids if mid not in online_ids]
        if not missing:
            self._startup_gate_ready = True
            self.get_logger().info(
                "Startup gate released: all expected motor IDs online "
                f"{self._expected_online_motor_ids}"
            )

    def _publish_imu_fast(self, d: dict):
        """Publish decoded fast IMU data on /imu0 and /imu1."""
        if not d:
            return
        stamp = self.get_clock().now().to_msg()

        def make_imu(accel_g, gyro_dps):
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = "imu_link"
            msg.linear_acceleration.x = float(accel_g[0]) * ACCEL_M_S2
            msg.linear_acceleration.y = float(accel_g[1]) * ACCEL_M_S2
            msg.linear_acceleration.z = float(accel_g[2]) * ACCEL_M_S2
            msg.angular_velocity.x = float(gyro_dps[0]) * GYRO_RAD_S
            msg.angular_velocity.y = float(gyro_dps[1]) * GYRO_RAD_S
            msg.angular_velocity.z = float(gyro_dps[2]) * GYRO_RAD_S
            # Orientation is not present in Type 0x04 packets.
            msg.orientation_covariance[0] = -1.0
            return msg

        self._pub_imu0.publish(
            make_imu(d.get("imu0_accel_g", [0.0, 0.0, 0.0]), d.get("imu0_gyro_dps", [0.0, 0.0, 0.0]))
        )
        self._pub_imu1.publish(
            make_imu(d.get("imu1_accel_g", [0.0, 0.0, 0.0]), d.get("imu1_gyro_dps", [0.0, 0.0, 0.0]))
        )

    def _handle_slow_telem(self, payload: bytes):
        """Decode slow telem payload and publish."""
        d = rp.decode_slow_telem(payload)
        if d:
            self._publish_slow_telem(d)

    def _publish_slow_telem(self, d: dict):
        """Publish decoded telem on the main ROS thread."""
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
        self._shutting_down = True
        self._running = False
        try:
            # Fallback limp command in case shutdown did not come from KeyboardInterrupt.
            self._send_disable_all()
        except Exception:
            pass
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
        node._shutting_down = True
        if rclpy.ok():
            node.get_logger().info("Shutting down — sending motor disable...")
        try:
            node._send_disable_all()
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
