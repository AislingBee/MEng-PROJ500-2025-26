"""
RobStride Motor Control — Keyboard Console via Nucleo CAN Bridge (COM8)
========================================================================
Keybindings:
  A  = Jog left (negative velocity)
  D  = Jog right (positive velocity)
  Z  = Zero (set current position as zero)
  1  = Go to  90°
  2  = Go to  180°
  3  = Go to -90°
  4  = Go to -180°
  Q  = Quit

This sends the EXACT same CAN frames as app.py, routed through the
Nucleo F429ZI serial-to-CAN bridge on COM8.
"""

import serial
import struct
import math
import time
import threading
import sys
import os

# Windows-specific non-blocking keyboard input
if os.name == 'nt':
    import msvcrt

# ─── Motor Constants (same as app.py) ──────────────────────
POS_RANGE    = 4 * math.pi
VEL_RANGE    = 15.0
TORQUE_RANGE = 120.0
KP_RANGE     = 5000.0
KD_RANGE     = 100.0
HOST_ID      = 0xFD

# Communication types
COMM_OPERATION_CONTROL = 1
COMM_OPERATION_STATUS  = 2
COMM_ENABLE            = 3
COMM_DISABLE           = 4
COMM_SET_ZERO          = 6
COMM_READ_PARAMETER    = 17
COMM_WRITE_PARAMETER   = 18

# Parameter IDs
PARAM_MODE             = 0x7005
PARAM_VELOCITY_TARGET  = 0x700A
PARAM_POSITION_TARGET  = 0x7016
PARAM_VELOCITY_LIMIT   = 0x7017
PARAM_MECH_POS         = 0x7019
PARAM_PP_SPEED_LIMIT   = 0x7024
PARAM_PP_ACCEL         = 0x7025

# ─── Configuration ─────────────────────────────────────────
COM_PORT   = 'COM6'
BAUD_RATE  = 921600
MOTOR_ID   = 127
JOG_SPEED  = 1.0     # rad/s
PP_SPEED   = 10.0    # rad/s for position moves
PP_ACCEL   = 10.0    # rad/s² for position moves


class NucleoMotorController:
    def __init__(self, port=COM_PORT, baud=BAUD_RATE, motor_id=MOTOR_ID):
        self.port = port
        self.baud = baud
        self.motor_id = motor_id
        self.ser = None
        self.running = False
        self.jogging = False
        self.jog_dir = 0
        self.telemetry = {'pos_deg': 0, 'vel': 0, 'torq': 0, 'temp': 0}
        self._rx_buffer = bytearray()

    # ═══════════════════════════════════════════════════════
    #  FRAME BUILDERS (identical to app.py)
    # ═══════════════════════════════════════════════════════

    def _build_frame(self, motor_id, param_id, value_bytes):
        """Constructs the raw 32-bit CAN ID AT frame — SAME as app.py."""
        ext_id = (0x12 << 24) | (0xFD << 8) | motor_id
        reg32 = (ext_id << 3) | 0x04
        id_bytes = struct.pack(">I", reg32)
        param_bytes = struct.pack("<H", param_id)
        data = param_bytes + b'\x00\x00' + value_bytes
        frame = b'\x41\x54' + id_bytes + bytes([len(data)]) + data + b'\x0D\x0A'
        return frame

    def _build_can_frame(self, comm_type, motor_id, data, extra_data=HOST_ID):
        """General CAN frame builder — SAME as app.py."""
        ext_id = (comm_type << 24) | (extra_data << 8) | motor_id
        reg32 = (ext_id << 3) | 0x04
        id_bytes = struct.pack(">I", reg32)
        frame = b'\x41\x54' + id_bytes + bytes([len(data)]) + data + b'\x0D\x0A'
        return frame

    def _build_read_frame(self, motor_id, param_id):
        """Build a READ_PARAMETER frame (CommType 17) — SAME as app.py."""
        data = struct.pack("<HHL", param_id, 0x00, 0x00)
        return self._build_can_frame(COMM_READ_PARAMETER, motor_id, data)

    # ═══════════════════════════════════════════════════════
    #  CONNECT / DISCONNECT
    # ═══════════════════════════════════════════════════════

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            # Send init handshake (same as USB-CAN dongle)
            self.ser.write(bytes.fromhex("41542b41540d0a"))
            self.ser.flush()
            time.sleep(0.3)
            response = self.ser.read_all()
            print(f"  Bridge response: {response}")

            self.running = True
            # Start RX thread
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            return True
        except Exception as e:
            print(f"  ERROR: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ═══════════════════════════════════════════════════════
    #  MOTOR COMMANDS
    # ═══════════════════════════════════════════════════════

    def enable(self):
        """Enable motor in jog mode (same as app.py enable_motors)."""
        if not self.ser or not self.ser.is_open:
            return
        frame = self._build_frame(self.motor_id, PARAM_MODE,
                                   b'\x00\x00\x00\x00')
        self.ser.write(frame)
        self.ser.flush()
        time.sleep(0.02)

    def zero(self):
        """Zero the motor position — CommType 6."""
        if not self.ser or not self.ser.is_open:
            return
        frame = self._build_can_frame(COMM_SET_ZERO, self.motor_id,
                                       b'\x01' + b'\x00' * 7)
        self.ser.write(frame)
        self.ser.flush()

    def jog_start(self, direction):
        """Start jogging: direction = -1 (left) or +1 (right)."""
        if not self.ser or not self.ser.is_open:
            return
        vel = JOG_SPEED * direction
        vel_u16 = int(((vel / 15.0) + 1.0) * 32767)
        vel_u16 = max(0, min(65535, vel_u16))
        payload = bytes([0x07, 0x01]) + struct.pack(">H", vel_u16)
        frame = self._build_frame(self.motor_id, PARAM_MODE, payload)
        self.ser.write(frame)
        self.ser.flush()
        self.jogging = True
        self.jog_dir = direction

    def jog_stop(self):
        """Stop jogging."""
        if not self.ser or not self.ser.is_open:
            return
        payload = bytes([0x07, 0x00, 0x7F, 0xFF])
        frame = self._build_frame(self.motor_id, PARAM_MODE, payload)
        self.ser.write(frame)
        self.ser.flush()
        self.jogging = False
        self.jog_dir = 0

    def goto_position(self, angle_deg):
        """Go to position using PP mode — same sequence as app.py."""
        if not self.ser or not self.ser.is_open:
            return
        target_rad = math.radians(angle_deg)

        # Step 1: Set run_mode = 1 (PP mode)
        self.ser.write(self._build_frame(
            self.motor_id, PARAM_MODE, struct.pack('<I', 1)))
        self.ser.flush()
        time.sleep(0.02)

        # Step 2: CommType 3 ENABLE
        self.ser.write(self._build_can_frame(
            COMM_ENABLE, self.motor_id, b'\x00' * 8))
        self.ser.flush()
        time.sleep(0.02)

        # Step 3: Set speed limit (param 0x7024)
        self.ser.write(self._build_frame(
            self.motor_id, PARAM_PP_SPEED_LIMIT,
            struct.pack('<f', PP_SPEED)))
        self.ser.flush()
        time.sleep(0.02)

        # Step 4: Set acceleration (param 0x7025)
        self.ser.write(self._build_frame(
            self.motor_id, PARAM_PP_ACCEL,
            struct.pack('<f', PP_ACCEL)))
        self.ser.flush()
        time.sleep(0.02)

        # Step 5: Set target position in radians (param 0x7016)
        self.ser.write(self._build_frame(
            self.motor_id, PARAM_POSITION_TARGET,
            struct.pack('<f', target_rad)))
        self.ser.flush()
        time.sleep(0.02)

    # ═══════════════════════════════════════════════════════
    #  RX LOOP — Parse CAN responses from Nucleo
    # ═══════════════════════════════════════════════════════

    def _rx_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                n = self.ser.in_waiting
                if n > 0:
                    raw = self.ser.read(n)
                    if raw:
                        self._rx_buffer.extend(raw)
                        self._parse_rx_buffer()
                else:
                    time.sleep(0.01)
            except Exception:
                break

    def _parse_rx_buffer(self):
        """Extract AT-framed CAN responses — SAME parser as app.py."""
        while len(self._rx_buffer) >= 9:
            idx = self._rx_buffer.find(b'\x41\x54')
            if idx == -1:
                self._rx_buffer.clear()
                return
            if idx > 0:
                del self._rx_buffer[:idx]

            if len(self._rx_buffer) < 7:
                return

            dlc = self._rx_buffer[6]
            if dlc > 8:
                del self._rx_buffer[:2]
                continue

            frame_len = 2 + 4 + 1 + dlc + 2
            if len(self._rx_buffer) < frame_len:
                return

            id_bytes = bytes(self._rx_buffer[2:6])
            frame_data = bytes(self._rx_buffer[7:7 + dlc])

            reg32 = struct.unpack(">I", id_bytes)[0]
            ext_id = reg32 >> 3
            comm_type = (ext_id >> 24) & 0x1F

            # Update telemetry from status frames (comm_type 2)
            if comm_type == COMM_OPERATION_STATUS and len(frame_data) >= 8:
                try:
                    pos_u16, vel_u16, torq_u16, temp_u16 = struct.unpack(
                        ">HHHH", frame_data[:8])
                    self.telemetry['pos_deg'] = math.degrees(
                        (float(pos_u16) / 32767.0 - 1.0) * POS_RANGE)
                    self.telemetry['vel'] = (
                        (float(vel_u16) / 32767.0 - 1.0) * VEL_RANGE)
                    self.telemetry['torq'] = (
                        (float(torq_u16) / 32767.0 - 1.0) * TORQUE_RANGE)
                    self.telemetry['temp'] = float(temp_u16) * 0.1
                except Exception:
                    pass

            del self._rx_buffer[:frame_len]

    # ═══════════════════════════════════════════════════════
    #  TELEMETRY DISPLAY
    # ═══════════════════════════════════════════════════════

    def format_telemetry(self):
        t = self.telemetry
        jog_str = {-1: "<<< LEFT", 0: "STOPPED", 1: "RIGHT >>>"}[self.jog_dir]
        return (f"  Pos: {t['pos_deg']:+8.2f}°  |  "
                f"Vel: {t['vel']:+6.2f} r/s  |  "
                f"Torq: {t['torq']:+6.2f} Nm  |  "
                f"Temp: {t['temp']:5.1f}°C  |  "
                f"Jog: {jog_str}")


def clear_line():
    """Clear current terminal line."""
    sys.stdout.write('\r' + ' ' * 100 + '\r')
    sys.stdout.flush()


def main():
    print("=" * 68)
    print("  RobStride Motor Control — Nucleo CAN Bridge")
    print("=" * 68)
    print(f"  Port: {COM_PORT} | Motor ID: {MOTOR_ID}")
    print()

    ctrl = NucleoMotorController()

    # ── Connect ──
    print("[1/2] Connecting to Nucleo CAN Bridge on COM8...")
    if not ctrl.connect():
        print("FAILED to connect. Check COM8.")
        return
    print("  ✓ Connected!")
    time.sleep(0.2)

    # ── Enable motor ──
    print("[2/2] Enabling motor...")
    ctrl.enable()
    print("  ✓ Motor enabled in jog mode")
    time.sleep(0.1)

    # ── Print controls ──
    print()
    print("─" * 68)
    print("  CONTROLS:")
    print("    A = Jog Left     D = Jog Right     Z = Zero Position")
    print("    1 = 90°          2 = 180°")
    print("    3 = -90°         4 = -180°")
    print("    Q = Quit")
    print("─" * 68)
    print()

    # ── Main input loop ──
    jog_active = False
    last_jog_dir = 0
    last_display = 0

    try:
        while ctrl.running:
            key_pressed = False
            key = ''

            # Check for keypress (Windows)
            if msvcrt.kbhit():
                raw = msvcrt.getch()
                try:
                    key = raw.decode('utf-8', errors='ignore').lower()
                    key_pressed = True
                except Exception:
                    pass

            if key_pressed:
                if key == 'q':
                    print("\n  Stopping motor and quitting...")
                    if jog_active:
                        ctrl.jog_stop()
                    ctrl.running = False
                    break

                elif key == 'a':
                    if not jog_active or last_jog_dir != -1:
                        ctrl.jog_start(-1)
                        last_jog_dir = -1
                        jog_active = True

                elif key == 'd':
                    if not jog_active or last_jog_dir != 1:
                        ctrl.jog_start(1)
                        last_jog_dir = 1
                        jog_active = True

                elif key == 's' or key == ' ':
                    # Stop jog
                    if jog_active:
                        ctrl.jog_stop()
                        jog_active = False
                        last_jog_dir = 0

                elif key == 'z':
                    if jog_active:
                        ctrl.jog_stop()
                        jog_active = False
                        last_jog_dir = 0
                        time.sleep(0.05)
                    ctrl.zero()
                    clear_line()
                    print("  >>> ZERO SET <<<")

                elif key == '1':
                    if jog_active:
                        ctrl.jog_stop()
                        jog_active = False
                        last_jog_dir = 0
                        time.sleep(0.05)
                    ctrl.goto_position(90)
                    clear_line()
                    print("  >>> GOTO 90° <<<")

                elif key == '2':
                    if jog_active:
                        ctrl.jog_stop()
                        jog_active = False
                        last_jog_dir = 0
                        time.sleep(0.05)
                    ctrl.goto_position(180)
                    clear_line()
                    print("  >>> GOTO 180° <<<")

                elif key == '3':
                    if jog_active:
                        ctrl.jog_stop()
                        jog_active = False
                        last_jog_dir = 0
                        time.sleep(0.05)
                    ctrl.goto_position(-90)
                    clear_line()
                    print("  >>> GOTO -90° <<<")

                elif key == '4':
                    if jog_active:
                        ctrl.jog_stop()
                        jog_active = False
                        last_jog_dir = 0
                        time.sleep(0.05)
                    ctrl.goto_position(-180)
                    clear_line()
                    print("  >>> GOTO -180° <<<")

            else:
                # No key pressed — if jogging, stop after release
                if jog_active:
                    ctrl.jog_stop()
                    jog_active = False
                    last_jog_dir = 0

            # Update telemetry display ~10 Hz
            now = time.time()
            if now - last_display > 0.1:
                clear_line()
                sys.stdout.write(ctrl.format_telemetry())
                sys.stdout.flush()
                last_display = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n  Ctrl+C — stopping...")
        if jog_active:
            ctrl.jog_stop()
    finally:
        ctrl.disconnect()
        print("  Disconnected. Goodbye!")


if __name__ == "__main__":
    main()
