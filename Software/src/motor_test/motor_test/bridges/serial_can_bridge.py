#!/usr/bin/env python3
"""Serial bridge between ROS and the STM32 CAN firmware (AT-frame binary protocol).

Translates 16-byte packed motor commands from /motor_can_tx into RS04 MIT
control AT-frames sent over USART3, and parses incoming AT-frames to publish
motor feedback to /motor_can_feedback.

AT-frame format (nucleo_can_bridge / test_motor.py protocol):
    0x41 0x54  [4-byte big-endian (ext_id << 3 | 0x04)]  [1-byte DLC]  [data]  0x0D 0x0A

Handshake:  host sends  AT+AT\r\n  (0x41 0x54 0x2B 0x41 0x54 0x0D 0x0A)
            firmware replies OK\r\n
"""

import csv
import datetime
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

# ── RS04 MIT protocol ──────────────────────────────────────────────────────────
_COMM_MOTION_CONTROL    = 0x01
_COMM_MOTOR_FEEDBACK    = 0x02
_COMM_MOTOR_ENABLE      = 0x03
_COMM_SET_SINGLE_PARAM  = 0x12
_RUN_MODE_INDEX         = 0x7005  # param index for run-mode register
_CONTROL_MODE_MIT       = 0x00    # MIT position-velocity-torque mode
_HOST_ID                = 0x00   # master CAN node ID

_P_MIN,  _P_MAX  = -12.5,  12.5   # position [rad]
_V_MIN,  _V_MAX  = -44.0,  44.0   # velocity [rad/s]
_KP_MIN, _KP_MAX =   0.0, 500.0   # position gain [Nm/rad]
_KD_MIN, _KD_MAX =   0.0,   5.0   # damping gain  [Nm·s/rad]
_T_MIN,  _T_MAX  = -17.0,  17.0   # torque [Nm]


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _f2u(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Float to unsigned integer (RS04 encoding)."""
    x = _clamp(x, x_min, x_max)
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def _u2f(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """Unsigned integer to float (RS04 decoding)."""
    return x_int / ((1 << bits) - 1) * (x_max - x_min) + x_min


def _at_frame(ext_id: int, data: bytes) -> bytes:
    """Wrap a CAN extended frame as an AT-frame for the nucleo bridge."""
    reg32 = (ext_id << 3) | 0x04
    return (b'\x41\x54'
            + struct.pack('>I', reg32)
            + bytes([len(data)])
            + data
            + b'\x0D\x0A')


def _motion_frame(motor_id: int, q: float, kp: float, kd: float, tau: float) -> bytes:
    """Build an RS04 MIT motion-control AT-frame."""
    p_int  = _f2u(q,   _P_MIN,  _P_MAX,  16)
    v_int  = _f2u(0.0, _V_MIN,  _V_MAX,  16)   # velocity setpoint = 0
    kp_int = _f2u(kp,  _KP_MIN, _KP_MAX, 16)
    kd_int = _f2u(kd,  _KD_MIN, _KD_MAX, 16)
    t_int  = _f2u(tau, _T_MIN,  _T_MAX,  16)
    ext_id = (_COMM_MOTION_CONTROL << 24) | (t_int << 8) | (motor_id & 0xFF)
    payload = bytes([
        (p_int  >> 8) & 0xFF, p_int  & 0xFF,
        (v_int  >> 8) & 0xFF, v_int  & 0xFF,
        (kp_int >> 8) & 0xFF, kp_int & 0xFF,
        (kd_int >> 8) & 0xFF, kd_int & 0xFF,
    ])
    return _at_frame(ext_id, payload)


def _enable_frames(motor_id: int) -> list:
    """Return [mode_frame, enable_frame] to put motor into MIT control mode."""
    mode_data = bytes([
        _RUN_MODE_INDEX & 0xFF,
        (_RUN_MODE_INDEX >> 8) & 0xFF,
        0x00, 0x00,
        _CONTROL_MODE_MIT,
        0x00, 0x00, 0x00,
    ])
    mode_ext_id   = (_COMM_SET_SINGLE_PARAM << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    enable_ext_id = (_COMM_MOTOR_ENABLE     << 24) | (_HOST_ID << 8) | (motor_id & 0xFF)
    return [
        _at_frame(mode_ext_id,   mode_data),
        _at_frame(enable_ext_id, bytes(8)),
    ]


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

            # Send enable sequence on first command for this motor ID
            if motor_id not in self._enabled_motors:
                mode_frame, enable_frame = _enable_frames(motor_id)
                self._serial_write(mode_frame)
                with self._log_lock:
                    _t = time.monotonic() - self._t0
                    self._can_writer.writerow([
                        f'{_t:.4f}', 'TX', motor_id,
                        'SET_PARAM(MIT_MODE)', '', '', '', '', ''])
                    self._can_csv.flush()
                    self._can_log.write(f'[{_t:.4f}]  TX   motor={motor_id}  SET_PARAM(MIT_MODE)\n')
                    self._can_log.flush()
                time.sleep(0.01)
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

            self._serial_write(_motion_frame(motor_id, q, kp, kd, tau))
            with self._log_lock:
                _t = time.monotonic() - self._t0
                self._can_writer.writerow([
                    f'{_t:.4f}', 'TX', motor_id, 'MOTION',
                    f'{q:.6f}', f'{kp:.4f}', f'{kd:.4f}', f'{tau:.6f}', ''])
                self._can_csv.flush()
                self._can_log.write(
                    f'[{_t:.4f}]  TX   motor={motor_id}  MOTION'
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
        """Decode an RS04 motor feedback frame and publish q/q_dot."""
        comm_type = (ext_id >> 24) & 0x3F
        motor_id  = ext_id & 0xFF

        if comm_type == _COMM_MOTOR_FEEDBACK and dlc >= 4:
            p_int = (data[0] << 8) | data[1]
            v_int = (data[2] << 8) | data[3]
            q     = _u2f(p_int, _P_MIN, _P_MAX, 16)
            q_dot = _u2f(v_int, _V_MIN, _V_MAX, 16)

            out = UInt8MultiArray()
            out.data = list(struct.pack('<ff', q, q_dot))
            self.publisher.publish(out)

            with self._log_lock:
                _t = time.monotonic() - self._t0
                self._can_writer.writerow([
                    f'{_t:.4f}', 'RX', motor_id, 'FEEDBACK',
                    f'{q:.6f}', '', '', '', f'{q_dot:.6f}'])
                self._can_csv.flush()
                self._can_log.write(
                    f'[{_t:.4f}]  RX   motor={motor_id}  FEEDBACK'
                    f'  q={q:.6f}  q_dot={q_dot:.6f}\n')
                self._can_log.flush()

            if self.all_logging:
                self.get_logger().info(
                    f'FBK motor={motor_id} q={q:.4f} q_dot={q_dot:.4f}')

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
