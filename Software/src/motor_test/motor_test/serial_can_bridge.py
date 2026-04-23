#!/usr/bin/env python3
"""ROS serial bridge for STM32 on COM3.

This bridge listens for ROS CAN command frames on `motor_can_tx`, sends them
as ASCII commands over a serial port, and publishes incoming STM32 motor
feedback back onto a ROS CAN-like topic `motor_can_feedback`.

The STM firmware should expose the following serial protocol:
  CMD 0xNN <q> <kp> <kd> <tau>\n
and reply as:
  FBK 0xNN <q> <q_dot>\n
The ROS side can then feed `motor_can_feedback` into the existing
`motor_feedback_listener.py` node.
"""

import threading
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

try:
    import serial
except ImportError as exc:
    raise RuntimeError(
        'pyserial is required for serial_can_bridge.py. Install with `pip install pyserial`.'
    ) from exc


class SerialCanBridge(Node):
    def __init__(self):
        super().__init__('serial_can_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('command_topic', 'motor_can_tx')
        self.declare_parameter('feedback_topic', 'motor_can_feedback')
        self.declare_parameter('can_id', 0x7F)
        # When can_id_per_joint=True the bridge increments the CAN ID for each
        # 16-byte chunk (joint 0 → can_id_base, joint 1 → can_id_base+1, …).
        self.declare_parameter('can_id_per_joint', True)
        self.declare_parameter('can_id_base', 0x201)
        self.declare_parameter('all_logging_info', True)

        self.port             = self.get_parameter('serial_port').value
        self.baud_rate        = int(self.get_parameter('baud_rate').value)
        self.timeout          = float(self.get_parameter('timeout').value)
        self.command_topic    = self.get_parameter('command_topic').value
        self.feedback_topic   = self.get_parameter('feedback_topic').value
        self.can_id           = int(self.get_parameter('can_id').value)
        self.can_id_per_joint = bool(self.get_parameter('can_id_per_joint').value)
        self.can_id_base      = int(self.get_parameter('can_id_base').value)
        self.all_logging_info = bool(self.get_parameter('all_logging_info').value)

        self.publisher = self.create_publisher(UInt8MultiArray, self.feedback_topic, 10)
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            self.command_topic,
            self.command_callback,
            10,
        )

        self.serial = None
        self.running = False
        self.reader_thread = None

        self.open_serial_port()

        if self.serial is not None:
            self.running = True
            self.reader_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.reader_thread.start()

        if self.all_logging_info:
            self.get_logger().info(
                f'Bridge ready: serial={self.port}@{self.baud_rate}, '
                f'command_topic="{self.command_topic}", feedback_topic="{self.feedback_topic}"'
            )

    def open_serial_port(self):
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Failed to open serial port {self.port}: {exc}')
            self.serial = None

    def command_callback(self, msg: UInt8MultiArray):
        if self.serial is None or not self.serial.is_open:
            self.get_logger().error('Serial port is not available')
            return

        payload = bytes(msg.data)
        if len(payload) == 0:
            return

        if len(payload) % 16 != 0:
            self.get_logger().warning(
                f'Received unexpected command payload length {len(payload)}; expected multiple of 16'
            )

        commands_sent = 0
        chunk_index = 0
        for offset in range(0, len(payload) - (len(payload) % 16), 16):
            chunk = payload[offset:offset + 16]
            q, kp, kd, tau = struct.unpack('<ffff', chunk)

            if self.can_id_per_joint:
                can_id = self.can_id_base + chunk_index
            else:
                can_id = self.can_id

            line = f'CMD 0x{can_id:X} {q:.6f} {kp:.6f} {kd:.6f} {tau:.6f}\n'
            try:
                self.serial.write(line.encode('ascii'))
                self.serial.flush()
                commands_sent += 1
                chunk_index += 1
                if self.all_logging_info:
                    self.get_logger().info(f'Sent serial command: {line.strip()}')
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Failed to write serial command: {exc}')
                break

        if commands_sent == 0 and self.all_logging_info:
            self.get_logger().warn('No valid 16-byte commands were sent to serial')

    def serial_read_loop(self):
        while rclpy.ok() and self.running:
            try:
                raw_line = self.serial.readline()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Serial read failure: {exc}')
                break

            if not raw_line:
                continue

            try:
                line = raw_line.decode('ascii', errors='ignore').strip()
            except Exception:
                continue

            if not line:
                continue

            if self.all_logging_info:
                self.get_logger().info(f'Received serial line: {line}')

            self.handle_serial_line(line)

        self.running = False

    def handle_serial_line(self, line: str):
        parts = line.split()
        if len(parts) < 4 or parts[0] != 'FBK':
            if self.all_logging_info:
                self.get_logger().warn(f'Unsupported serial line: {line}')
            return

        try:
            can_id = int(parts[1], 16)
            q = float(parts[2])
            q_dot = float(parts[3])
        except ValueError:
            self.get_logger().warning(f'Invalid serial feedback format: {line}')
            return

        payload = struct.pack('<ff', q, q_dot)
        msg = UInt8MultiArray()
        msg.data = list(payload)
        self.publisher.publish(msg)

        if self.all_logging_info:
            self.get_logger().info(
                f'Published feedback on {self.feedback_topic}: id=0x{can_id:03X} q={q:.6f} q_dot={q_dot:.6f}'
            )

    def destroy_node(self):
        self.running = False
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
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
