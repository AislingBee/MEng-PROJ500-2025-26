#!/usr/bin/env python3
"""UDP bridge between ROS and the STM32 CAN firmware.

Sends CMD lines to the STM32 and receives FBK/ID/ERR replies over UDP.
Default STM32 address: 192.168.1.100:7777.

Logs are written to <Software>/logs/<timestamp>/udp_events.csv
  time_s, direction, can_id_hex, data
"""

import csv
import datetime
import socket
import struct
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from motor_test.common import get_software_log_dir


class EthernetCanBridge(Node):
    def __init__(self):
        super().__init__('ethernet_can_bridge')

        self.declare_parameter('stm32_ip',           '192.168.1.100')
        self.declare_parameter('stm32_port',         7777)
        self.declare_parameter('listen_port',        7777)
        self.declare_parameter('command_topic',      'motor_can_tx')
        self.declare_parameter('feedback_topic',     'motor_can_feedback')
        self.declare_parameter('can_id',             0x7F)
        # When True, assigns sequential CAN IDs per joint (can_id_base, base+1, ...).
        # When False, all joints use the same can_id.
        self.declare_parameter('can_id_per_joint',   True)
        self.declare_parameter('can_id_base',        0x201)
        self.declare_parameter('all_logging_info',   True)
        self.declare_parameter('log_dir', str(get_software_log_dir()))

        self.stm32_ip         = self.get_parameter('stm32_ip').value
        self.stm32_port       = int(self.get_parameter('stm32_port').value)
        self.listen_port      = int(self.get_parameter('listen_port').value)
        self.command_topic    = self.get_parameter('command_topic').value
        self.feedback_topic   = self.get_parameter('feedback_topic').value
        self.can_id           = int(self.get_parameter('can_id').value)
        self.can_id_per_joint = bool(self.get_parameter('can_id_per_joint').value)
        self.can_id_base      = int(self.get_parameter('can_id_base').value)
        self.all_logging      = bool(self.get_parameter('all_logging_info').value)
        log_base              = self.get_parameter('log_dir').value

        # -- Log file setup --------------------------------------------------
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = Path(log_base) / timestamp
        log_dir.mkdir(parents=True, exist_ok=True)
        self._t0 = time.monotonic()
        self._udp_csv  = open(log_dir / 'udp_events.csv', 'w', newline='', encoding='utf-8')
        self._udp_log  = open(log_dir / 'udp_events.log', 'w', encoding='utf-8')
        self._udp_writer = csv.writer(self._udp_csv)
        self._udp_writer.writerow(['time_s', 'direction', 'can_id_hex', 'data'])
        self._udp_csv.flush()
        self.get_logger().info(f'UDP event log: {log_dir / "udp_events.csv"}  +  udp_events.log')
        # --------------------------------------------------------------------

        self.publisher = self.create_publisher(UInt8MultiArray, self.feedback_topic, 10)
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            self.command_topic,
            self.command_callback,
            10,
        )

        self.sock = None
        self.running = False
        self.reader_thread = None

        self._open_socket()

        if self.sock is not None:
            self.running = True
            self.reader_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.reader_thread.start()

        if self.all_logging:
            self.get_logger().info(
                f'Ethernet bridge ready: STM32={self.stm32_ip}:{self.stm32_port} '
                f'listen_port={self.listen_port} '
                f'command_topic="{self.command_topic}" '
                f'feedback_topic="{self.feedback_topic}"'
            )

    # ------------------------------------------------------------------
    def _open_socket(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('', self.listen_port))
            self.sock.settimeout(0.5)
        except OSError as exc:
            self.get_logger().error(f'Failed to open UDP socket: {exc}')
            self.sock = None

    # ------------------------------------------------------------------
    def command_callback(self, msg: UInt8MultiArray):
        if self.sock is None:
            self.get_logger().error('UDP socket is not available')
            return

        payload = bytes(msg.data)
        if len(payload) == 0:
            return

        if len(payload) % 16 != 0:
            self.get_logger().warning(
                f'Unexpected command payload length {len(payload)}; expected multiple of 16'
            )

        commands_sent = 0
        chunk_index = 0
        for offset in range(0, len(payload) - (len(payload) % 16), 16):
            chunk = payload[offset:offset + 16]
            q, kp, kd, tau = struct.unpack('<ffff', chunk)

            # Choose CAN ID: sequential per-joint or uniform single-motor mode.
            if self.can_id_per_joint:
                can_id = self.can_id_base + chunk_index
            else:
                can_id = self.can_id

            line = f'CMD 0x{can_id:X} {q:.6f} {kp:.6f} {kd:.6f} {tau:.6f}\n'
            try:
                self.sock.sendto(line.encode('ascii'), (self.stm32_ip, self.stm32_port))
                _t = time.monotonic() - self._t0
                self._udp_writer.writerow(
                    [f'{_t:.4f}', 'TX',
                     f'0x{can_id:X}', f'q={q:.6f} kp={kp:.6f} kd={kd:.6f} tau={tau:.6f}'])
                self._udp_csv.flush()
                self._udp_log.write(
                    f'[{_t:.4f}]  TX   CAN=0x{can_id:X}'
                    f'  q={q:.6f}  kp={kp:.6f}  kd={kd:.6f}  tau={tau:.6f}\n')
                self._udp_log.flush()
                commands_sent += 1
                chunk_index += 1
                if self.all_logging:
                    self.get_logger().info(f'Sent UDP command: {line.strip()}')
            except OSError as exc:
                self.get_logger().error(f'UDP send failed: {exc}')
                break

        if commands_sent == 0 and self.all_logging:
            self.get_logger().warn('No valid 16-byte commands were sent over UDP')

    # ------------------------------------------------------------------
    def _recv_loop(self):
        recv_buf = b''
        while rclpy.ok() and self.running:
            try:
                data, _addr = self.sock.recvfrom(2048)
                recv_buf += data
            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().error(f'UDP receive failure: {exc}')
                break

            # Process all complete lines in the buffer
            while b'\n' in recv_buf:
                line_bytes, recv_buf = recv_buf.split(b'\n', 1)
                line = line_bytes.decode('ascii', errors='ignore').strip()
                if line:
                    if self.all_logging:
                        self.get_logger().info(f'Received UDP line: {line}')
                    self._handle_line(line)

        self.running = False

    # ------------------------------------------------------------------
    def _handle_line(self, line: str):
        parts = line.split()

        if parts[0] == 'FBK' and len(parts) >= 4:
            try:
                can_id = int(parts[1], 16)
                q      = float(parts[2])
                q_dot  = float(parts[3])
            except ValueError:
                self.get_logger().warning(f'Invalid FBK line: {line}')
                return

            payload = struct.pack('<ff', q, q_dot)
            out = UInt8MultiArray()
            out.data = list(payload)
            self.publisher.publish(out)

            _t = time.monotonic() - self._t0
            self._udp_writer.writerow(
                [f'{_t:.4f}', 'RX_FBK',
                 f'0x{can_id:02X}', f'q={q:.6f} q_dot={q_dot:.6f}'])
            self._udp_csv.flush()
            self._udp_log.write(
                f'[{_t:.4f}]  RX_FBK  CAN=0x{can_id:02X}'
                f'  q={q:.6f}  q_dot={q_dot:.6f}\n')
            self._udp_log.flush()

            if self.all_logging:
                self.get_logger().info(
                    f'Published feedback: id=0x{can_id:02X} q={q:.6f} q_dot={q_dot:.6f}'
                )

        elif parts[0] == 'ID' and len(parts) >= 2:
            self.get_logger().info(f'Motor ID reported: {parts[1]}')

        elif parts[0] == 'ERR':
            self.get_logger().error(f'STM32 error: {line}')
            _t = time.monotonic() - self._t0
            self._udp_writer.writerow(
                [f'{_t:.4f}', 'RX_ERR', '', line])
            self._udp_csv.flush()
            self._udp_log.write(f'[{_t:.4f}]  RX_ERR  {line}\n')
            self._udp_log.flush()

        else:
            if self.all_logging:
                self.get_logger().warn(f'Unsupported UDP line: {line}')

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.running = False
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if self.sock is not None:
            self.sock.close()
        self._udp_csv.close()
        self._udp_log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EthernetCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
