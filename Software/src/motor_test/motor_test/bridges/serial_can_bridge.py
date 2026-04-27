#!/usr/bin/env python3
"""Serial bridge between ROS and STM32 Motor Controller (AT-frame binary protocol).

Translates 16-byte packed motor commands from /motor_can_tx into point-to-point
position control AT-frames sent over USART3, and parses incoming AT-frames to publish
motor feedback to /motor_can_feedback.

AT-frame format (nucleo_can_bridge / motor_controller.c protocol):
    0x41 0x54  [4-byte big-endian (ext_id << 3 | 0x04)]  [1-byte DLC]  [data]  0x0D 0x0A

Handshake:  host sends  AT+AT\r\n  (0x41 0x54 0x2B 0x41 0x54 0x0D 0x0A)
            firmware replies OK\r\n

Motor Control Protocol:
- COMM_ENABLE (3):             Enable motor, set to position control mode
- COMM_DISABLE (4):            Disable motor
- COMM_SET_ZERO (6):           Zero the position
- COMM_OPERATION_STATUS (2):   Motor telemetry feedback
- PARAM_MODE (0x7005):         Motor control mode
- PARAM_POSITION_TARGET (0x7016): Target position in radians
"""

import csv
import datetime
import math
import struct
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from motor_test.common import get_software_log_dir

try:
    import serial
except ImportError as exc:
    raise RuntimeError(
        'pyserial is required. Install with `pip install pyserial`.' ) from exc

# ── STM32 Motor Controller Protocol ─────────────────────────────────────────
_COMM_OPERATION_STATUS  = 2   # Motor telemetry (firmware → host)
_COMM_ENABLE            = 3   # Enable motor
_COMM_DISABLE           = 4   # Disable motor
_COMM_SET_ZERO          = 6   # Zero position
_HOST_ID                = 0xFD   # Host CAN ID

# Parameter IDs (for parameter write commands)
_PARAM_MODE             = 0x7005
_PARAM_KP               = 0x7014   # Proportional gain
_PARAM_KD               = 0x7015   # Damping gain
_PARAM_POSITION_TARGET  = 0x7016
_PARAM_PP_SPEED_LIMIT   = 0x7024
_PARAM_PP_ACCEL         = 0x7025
_PARAM_FEEDFORWARD_TORQUE = 0x7026

# Motor modes
_MODE_DISABLED          = 0
_MODE_POSITION_CONTROL  = 1
_MODE_VELOCITY_JOG      = 7

# Motor parameter ranges
_POS_RANGE              = 4.0 * math.pi   # ±2π radians
_VEL_RANGE              = 15.0            # ±15 rad/s
_TORQUE_RANGE           = 120.0           # ±120 Nm
_TEMP_SCALE             = 0.1             # °C per unit


def _at_frame(ext_id: int, data: bytes) -> bytes:
    """Wrap a CAN extended frame as an AT-frame for the nucleo bridge."""
    reg32 = (ext_id << 3) | 0x04
    return (b'\x41\x54'
            + struct.pack('>I', reg32)
            + bytes([len(data)])
            + data
            + b'\x0D\x0A')


def _encode_float_le(value: float) -> bytes:
    """Encode float as little-endian 4 bytes."""
    return struct.pack('<f', value)


def _position_target_frame(motor_id: int, position_rad: float) -> bytes:
    """Build a position target parameter frame."""
    # Data format: param_id (LE uint16), param_value (LE float), reserved (2 bytes)
    param_id_bytes = struct.pack('<H', _PARAM_POSITION_TARGET)
    param_val_bytes = _encode_float_le(position_rad)
    payload = param_id_bytes + param_val_bytes + b'\x00\x00'
    
    ext_id = (0x10 << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    return _at_frame(ext_id, payload)


def _kp_frame(motor_id: int, kp: float) -> bytes:
    """Build a Kp (proportional gain) parameter frame."""
    param_id_bytes = struct.pack('<H', _PARAM_KP)
    param_val_bytes = _encode_float_le(kp)
    payload = param_id_bytes + param_val_bytes + b'\x00\x00'
    
    ext_id = (0x10 << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    return _at_frame(ext_id, payload)


def _kd_frame(motor_id: int, kd: float) -> bytes:
    """Build a Kd (damping gain) parameter frame."""
    param_id_bytes = struct.pack('<H', _PARAM_KD)
    param_val_bytes = _encode_float_le(kd)
    payload = param_id_bytes + param_val_bytes + b'\x00\x00'
    
    ext_id = (0x10 << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    return _at_frame(ext_id, payload)


def _feedforward_torque_frame(motor_id: int, tau: float) -> bytes:
    """Build a feedforward torque parameter frame."""
    param_id_bytes = struct.pack('<H', _PARAM_FEEDFORWARD_TORQUE)
    param_val_bytes = _encode_float_le(tau)
    payload = param_id_bytes + param_val_bytes + b'\x00\x00'
    
    ext_id = (0x10 << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    return _at_frame(ext_id, payload)


def _enable_frame(motor_id: int) -> bytes:
    """Build an enable motor command frame."""
    ext_id = (_COMM_ENABLE << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    return _at_frame(ext_id, b'\x00' * 8)


def _disable_frame(motor_id: int) -> bytes:
    """Build a disable motor command frame."""
    ext_id = (_COMM_DISABLE << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    return _at_frame(ext_id, b'\x00' * 8)


def _zero_frame(motor_id: int) -> bytes:
    """Build a zero position command frame."""
    ext_id = (_COMM_SET_ZERO << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    return _at_frame(ext_id, b'\x01' + b'\x00' * 7)


def _u16_to_float(val_u16: int, range_min: float, range_max: float) -> float:
    """Decode uint16 to float using normalization."""
    normalized = (float(val_u16) / 32767.0 - 1.0)
    return normalized * (range_max - range_min) / 2.0


# ── ROS2 node ──────────────────────────────────────────────────────────────────

class SerialCanBridge(Node):
    def __init__(self):
        super().__init__('serial_can_bridge')

        self.declare_parameter('serial_port',      '/dev/ttyACM0')
        self.declare_parameter('baud_rate',        921600)
        self.declare_parameter('command_topic',    'motor_can_tx')
        self.declare_parameter('feedback_topic',   'motor_can_feedback')
        self.declare_parameter('can_id_per_joint', True)
        self.declare_parameter('can_id_base',      1)
        self.declare_parameter('can_id',           1)
        self.declare_parameter('all_logging_info', True)
        self.declare_parameter('log_dir', str(get_software_log_dir()))

        self.port             = self.get_parameter('serial_port').value
        self.baud_rate        = int(self.get_parameter('baud_rate').value)
        self.command_topic    = self.get_parameter('command_topic').value
        self.feedback_topic   = self.get_parameter('feedback_topic').value
        self.can_id_per_joint = bool(self.get_parameter('can_id_per_joint').value)
        self.can_id_base      = int(self.get_parameter('can_id_base').value)
        self.can_id           = int(self.get_parameter('can_id').value)
        self.all_logging      = bool(self.get_parameter('all_logging_info').value)

        # -- CAN log setup ---------------------------------------------------
        log_base  = self.get_parameter('log_dir').value
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir   = Path(log_base) / timestamp
        log_dir.mkdir(parents=True, exist_ok=True)
        self._t0         = time.monotonic()
        self._log_lock   = threading.Lock()
        self._can_csv    = open(log_dir / 'can_log.csv', 'w', newline='', encoding='utf-8')
        self._can_log    = open(log_dir / 'can_log.log', 'w', encoding='utf-8')
        self._can_writer = csv.writer(self._can_csv)
        self._can_writer.writerow(
            ['time_s', 'direction', 'motor_id', 'comm_type',
             'q_rad', 'kp', 'kd', 'tau_ff_Nm', 'q_dot_rad_s'])
        self._can_csv.flush()
        self.get_logger().info(f'CAN log: {log_dir / "can_log.csv"}  +  can_log.log')
        # --------------------------------------------------------------------

        self._enabled_motors: set = set()
        self._rx_buf         = bytearray()
        self._serial_lock    = threading.Lock()

        self.publisher = self.create_publisher(UInt8MultiArray, self.feedback_topic, 10)
        self.subscription = self.create_subscription(
            UInt8MultiArray, self.command_topic, self.command_callback, 10)

        self.serial        = None
        self.running       = False
        self.reader_thread = None

        self._open_serial_port()

        if self.serial is not None:
            self.running = True
            self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.reader_thread.start()
            self._do_handshake()

        if self.all_logging:
            self.get_logger().info(
                f'SerialCanBridge ready: {self.port}@{self.baud_rate} '
                f'cmd="{self.command_topic}" fbk="{self.feedback_topic}"')

    # ------------------------------------------------------------------
    def _open_serial_port(self):
        try:
            self.serial = serial.Serial(
                self.port, self.baud_rate, timeout=0.05, write_timeout=0.5)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Failed to open {self.port}: {exc}')
            self.serial = None

    def _do_handshake(self):
        """Send the AT+AT handshake; firmware replies OK\r\n."""
        try:
            self.serial.write(b'\x41\x54\x2B\x41\x54\x0D\x0A')
            self.serial.flush()
        except Exception as exc:
            self.get_logger().warning(f'Handshake write failed: {exc}')

    def _serial_write(self, data: bytes):
        with self._serial_lock:
            try:
                self.serial.write(data)
                self.serial.flush()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Serial write error: {exc}')

    # ------------------------------------------------------------------
    def command_callback(self, msg: UInt8MultiArray):
        """Process motor command. Expects 16-byte chunks: [q, kp, kd, tau]"""
        if self.serial is None or not self.serial.is_open:
            return

        payload = bytes(msg.data)
        if len(payload) == 0 or len(payload) % 16 != 0:
            self.get_logger().warning(f'Bad command payload length {len(payload)}')
            return

        for chunk_idx, offset in enumerate(range(0, len(payload), 16)):
            chunk = payload[offset:offset + 16]
            q, kp, kd, tau = struct.unpack('<ffff', chunk)

            motor_id = (self.can_id_base + chunk_idx) if self.can_id_per_joint else self.can_id

            # Send enable frame on first command for this motor ID
            if motor_id not in self._enabled_motors:
                enable_frame = _enable_frame(motor_id)
                self._serial_write(enable_frame)
                with self._log_lock:
                    _t = time.monotonic() - self._t0
                    self._can_writer.writerow([
                        f'{_t:.4f}', 'TX', motor_id,
                        'ENABLE', '', '', '', '', ''])
                    self._can_csv.flush()
                    self._can_log.write(f'[{_t:.4f}]  TX   motor={motor_id}  ENABLE\n')
                    self._can_log.flush()
                time.sleep(0.01)
                self._enabled_motors.add(motor_id)
                if self.all_logging:
                    self.get_logger().info(f'Enabled motor id={motor_id}')

            # Send control gains (Kp, Kd)
            if kp > 0:
                kp_frame = _kp_frame(motor_id, kp)
                self._serial_write(kp_frame)
                time.sleep(0.005)
            
            if kd > 0:
                kd_frame = _kd_frame(motor_id, kd)
                self._serial_write(kd_frame)
                time.sleep(0.005)
            
            # Send feedforward torque
            if tau != 0:
                tau_frame = _feedforward_torque_frame(motor_id, tau)
                self._serial_write(tau_frame)
                time.sleep(0.005)

            # Send position target command
            pos_frame = _position_target_frame(motor_id, q)
            self._serial_write(pos_frame)
            with self._log_lock:
                _t = time.monotonic() - self._t0
                self._can_writer.writerow([
                    f'{_t:.4f}', 'TX', motor_id, 'POSITION_TARGET',
                    f'{q:.6f}', f'{kp:.4f}', f'{kd:.4f}', f'{tau:.6f}', ''])
                self._can_csv.flush()
                self._can_log.write(
                    f'[{_t:.4f}]  TX   motor={motor_id}  POSITION_TARGET'
                    f'  q={q:.6f}  kp={kp:.4f}  kd={kd:.4f}  tau={tau:.6f}\n')
                self._can_log.flush()

            if self.all_logging:
                self.get_logger().debug(
                    f'CMD motor={motor_id} q={q:.4f} kp={kp:.1f} kd={kd:.2f} tau={tau:.3f}')

    # ------------------------------------------------------------------
    def _read_loop(self):
        while rclpy.ok() and self.running:
            try:
                incoming = self.serial.read(256)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                break
            if incoming:
                self._rx_buf.extend(incoming)
                self._parse_buffer()
        self.running = False

    def _parse_buffer(self):
        """Extract complete AT-frames from self._rx_buf and dispatch them."""
        buf = self._rx_buf

        while True:
            # Locate next 0x41 0x54 header
            idx = -1
            for i in range(len(buf) - 1):
                if buf[i] == 0x41 and buf[i + 1] == 0x54:
                    idx = i
                    break

            if idx < 0:
                # No header: keep last byte in case it starts the next frame
                self._rx_buf = buf[-1:] if buf else bytearray()
                return

            if idx > 0:
                buf = buf[idx:]   # discard junk before header
                continue

            # Need at least 7 bytes to read DLC:  AT(2) + reg32(4) + dlc(1)
            if len(buf) < 7:
                break

            dlc = buf[6]
            if dlc > 8:
                buf = buf[2:]   # bad DLC — skip this header
                continue

            frame_len = 9 + dlc   # AT(2) + reg32(4) + dlc(1) + data(dlc) + CRLF(2)
            if len(buf) < frame_len:
                break   # wait for more bytes

            if buf[frame_len - 2] != 0x0D or buf[frame_len - 1] != 0x0A:
                buf = buf[2:]   # terminator mismatch — skip header
                continue

            # Valid AT-frame — parse it
            reg32  = struct.unpack('>I', bytes(buf[2:6]))[0]
            ext_id = (reg32 >> 3) & 0x1FFFFFFF
            data   = bytes(buf[7:7 + dlc])
            buf    = buf[frame_len:]
            self._handle_can_frame(ext_id, data, dlc)

        self._rx_buf = buf

    def _handle_can_frame(self, ext_id: int, data: bytes, dlc: int):
        """Decode STM32 motor telemetry frame and publish position/velocity."""
        comm_type = (ext_id >> 24) & 0x3F
        motor_id  = ext_id & 0xFF

        if comm_type == _COMM_OPERATION_STATUS and dlc >= 8:
            # Telemetry format: [pos(2), vel(2), torq(2), temp(2)] - all big-endian uint16
            pos_u16  = (data[0] << 8) | data[1]
            vel_u16  = (data[2] << 8) | data[3]
            torq_u16 = (data[4] << 8) | data[5]
            temp_u16 = (data[6] << 8) | data[7]

            # Decode normalized uint16 back to float
            q     = _u16_to_float(pos_u16, -_POS_RANGE / 2.0, _POS_RANGE / 2.0)
            q_dot = _u16_to_float(vel_u16, -_VEL_RANGE, _VEL_RANGE)
            torque = _u16_to_float(torq_u16, -_TORQUE_RANGE, _TORQUE_RANGE)
            temp = float(temp_u16) * _TEMP_SCALE

            # Publish position and velocity
            out = UInt8MultiArray()
            out.data = list(struct.pack('<ff', q, q_dot))
            self.publisher.publish(out)

            with self._log_lock:
                _t = time.monotonic() - self._t0
                self._can_writer.writerow([
                    f'{_t:.4f}', 'RX', motor_id, 'TELEMETRY',
                    f'{q:.6f}', '', '', '', f'{q_dot:.6f}'])
                self._can_csv.flush()
                self._can_log.write(
                    f'[{_t:.4f}]  RX   motor={motor_id}  TELEMETRY'
                    f'  q={q:.6f}  q_dot={q_dot:.6f}  tau={torque:.3f}  temp={temp:.1f}C\n')
                self._can_log.flush()

            if self.all_logging:
                self.get_logger().info(
                    f'FBK motor={motor_id} q={q:.4f} q_dot={q_dot:.4f} torque={torque:.2f} temp={temp:.1f}')

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.running = False
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        if hasattr(self, '_can_csv'):
            self._can_csv.close()
        if hasattr(self, '_can_log'):
            self._can_log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
