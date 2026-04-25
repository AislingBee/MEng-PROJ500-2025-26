"""
RobStride 2-Motor Control — Keyboard Console via Nucleo CAN Bridge
===================================================================
Controls can target Motor 1 only, Motor 2 only, or both.

Keybindings:
    [  = Select motor 1 only
    ]  = Select motor 2 only
    \\ = Select both motors
  A  = Jog left (negative velocity)
  D  = Jog right (positive velocity)
  S/Space = Stop jog
    Z  = Zero selected target(s)
    1  = Go to  90° (selected)
    2  = Go to  180° (selected)
    3  = Go to -90° (selected)
    4  = Go to -180° (selected)
  Q  = Quit
"""

import serial
import struct
import math
import time
import threading
import sys
import os

if os.name == "nt":
    import msvcrt

# Motor constants
POS_RANGE = 4 * math.pi
VEL_RANGE = 15.0
TORQUE_RANGE = 120.0
HOST_ID = 0xFD

# Communication types
COMM_OPERATION_STATUS = 2
COMM_ENABLE = 3
COMM_DISABLE = 4
COMM_SET_ZERO = 6

# Parameter IDs
PARAM_MODE = 0x7005
PARAM_POSITION_TARGET = 0x7016
PARAM_PP_SPEED_LIMIT = 0x7024
PARAM_PP_ACCEL = 0x7025

# Config
COM_PORT = "COM6"
BAUD_RATE = 921600
MOTOR_IDS = [127, 1]
JOG_SPEED = 1.0
PP_SPEED = 10.0
PP_ACCEL = 10.0


class DualMotorController:
    def __init__(self, port=COM_PORT, baud=BAUD_RATE, motor_ids=None):
        self.port = port
        self.baud = baud
        self.motor_ids = motor_ids or MOTOR_IDS
        self.ser = None
        self.running = False
        self.jog_dir = {mid: 0 for mid in self.motor_ids}
        self.telemetry = {
            mid: {"pos_deg": 0.0, "vel": 0.0, "torq": 0.0, "temp": 0.0}
            for mid in self.motor_ids
        }
        self._rx_buffer = bytearray()

    def _build_frame(self, motor_id, param_id, value_bytes):
        ext_id = (0x12 << 24) | (HOST_ID << 8) | motor_id
        reg32 = (ext_id << 3) | 0x04
        id_bytes = struct.pack(">I", reg32)
        param_bytes = struct.pack("<H", param_id)
        data = param_bytes + b"\x00\x00" + value_bytes
        return b"\x41\x54" + id_bytes + bytes([len(data)]) + data + b"\x0D\x0A"

    def _build_can_frame(self, comm_type, motor_id, data, extra=HOST_ID):
        ext_id = (comm_type << 24) | (extra << 8) | motor_id
        reg32 = (ext_id << 3) | 0x04
        id_bytes = struct.pack(">I", reg32)
        return b"\x41\x54" + id_bytes + bytes([len(data)]) + data + b"\x0D\x0A"

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.ser.write(bytes.fromhex("41542b41540d0a"))
            self.ser.flush()
            time.sleep(0.3)
            print(f"  Bridge response: {self.ser.read_all()}")
            self.running = True
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

    def _resolve_targets(self, targets=None):
        if targets is None:
            return self.motor_ids
        return [mid for mid in targets if mid in self.motor_ids]

    def enable_all(self):
        self.enable_targets(self.motor_ids)

    def enable_targets(self, targets=None):
        for mid in self._resolve_targets(targets):
            self.ser.write(self._build_frame(mid, PARAM_MODE, b"\x00\x00\x00\x00"))
            self.ser.flush()
            time.sleep(0.02)

    def disable_targets(self, targets=None):
        for mid in self._resolve_targets(targets):
            self.ser.write(self._build_can_frame(COMM_DISABLE, mid, b"\x00" * 8))
            self.ser.flush()
            time.sleep(0.02)

    def zero_all(self):
        self.zero_targets(self.motor_ids)

    def zero_targets(self, targets=None):
        for mid in self._resolve_targets(targets):
            self.ser.write(self._build_can_frame(COMM_SET_ZERO, mid, b"\x01" + b"\x00" * 7))
            self.ser.flush()
            time.sleep(0.02)

    def jog_all(self, direction):
        self.jog_targets(direction, self.motor_ids)

    def jog_targets(self, direction, targets=None):
        vel = JOG_SPEED * direction
        vel_u16 = int(((vel / 15.0) + 1.0) * 32767)
        vel_u16 = max(0, min(65535, vel_u16))
        payload = bytes([0x07, 0x01]) + struct.pack(">H", vel_u16)
        for mid in self._resolve_targets(targets):
            self.ser.write(self._build_frame(mid, PARAM_MODE, payload))
            self.ser.flush()
            self.jog_dir[mid] = direction

    def stop_all(self):
        self.stop_targets(self.motor_ids)

    def stop_targets(self, targets=None):
        payload = bytes([0x07, 0x00, 0x7F, 0xFF])
        for mid in self._resolve_targets(targets):
            self.ser.write(self._build_frame(mid, PARAM_MODE, payload))
            self.ser.flush()
            self.jog_dir[mid] = 0

    def goto_all(self, angle_deg):
        self.goto_targets(angle_deg, self.motor_ids)

    def goto_targets(self, angle_deg, targets=None):
        target_rad = math.radians(angle_deg)
        for mid in self._resolve_targets(targets):
            self.ser.write(self._build_frame(mid, PARAM_MODE, struct.pack("<I", 1)))
            self.ser.flush()
            time.sleep(0.01)

            self.ser.write(self._build_can_frame(COMM_ENABLE, mid, b"\x00" * 8))
            self.ser.flush()
            time.sleep(0.01)

            self.ser.write(self._build_frame(mid, PARAM_PP_SPEED_LIMIT, struct.pack("<f", PP_SPEED)))
            self.ser.flush()
            time.sleep(0.01)

            self.ser.write(self._build_frame(mid, PARAM_PP_ACCEL, struct.pack("<f", PP_ACCEL)))
            self.ser.flush()
            time.sleep(0.01)

            self.ser.write(self._build_frame(mid, PARAM_POSITION_TARGET, struct.pack("<f", target_rad)))
            self.ser.flush()
            time.sleep(0.01)

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

    def _extract_motor_id(self, ext_id):
        low = ext_id & 0xFF
        mid = (ext_id >> 8) & 0xFF
        if low in self.telemetry:
            return low
        if mid in self.telemetry:
            return mid
        return None

    def _parse_rx_buffer(self):
        while len(self._rx_buffer) >= 9:
            idx = self._rx_buffer.find(b"\x41\x54")
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

            if comm_type == COMM_OPERATION_STATUS and len(frame_data) >= 8:
                motor_id = self._extract_motor_id(ext_id)
                if motor_id is not None:
                    try:
                        pos_u16, vel_u16, torq_u16, temp_u16 = struct.unpack(
                            ">HHHH", frame_data[:8]
                        )
                        self.telemetry[motor_id]["pos_deg"] = math.degrees(
                            (float(pos_u16) / 32767.0 - 1.0) * POS_RANGE
                        )
                        self.telemetry[motor_id]["vel"] = (
                            (float(vel_u16) / 32767.0 - 1.0) * VEL_RANGE
                        )
                        self.telemetry[motor_id]["torq"] = (
                            (float(torq_u16) / 32767.0 - 1.0) * TORQUE_RANGE
                        )
                        self.telemetry[motor_id]["temp"] = float(temp_u16) * 0.1
                    except Exception:
                        pass

            del self._rx_buffer[:frame_len]

    def format_lines(self):
        lines = []
        for mid in self.motor_ids:
            t = self.telemetry[mid]
            jog_str = {-1: "<<< LEFT", 0: "STOPPED", 1: "RIGHT >>>"}[self.jog_dir[mid]]
            lines.append(
                f"M{mid:3d} | Pos:{t['pos_deg']:+8.2f} deg | Vel:{t['vel']:+6.2f} r/s | "
                f"Torq:{t['torq']:+6.2f} Nm | Temp:{t['temp']:5.1f} C | Jog:{jog_str}"
            )
        return lines


def target_label(targets):
    if len(targets) == 1:
        return f"M{targets[0]}"
    return "BOTH"


def apply_target_isolation(ctrl, selected_targets):
    non_selected = [mid for mid in ctrl.motor_ids if mid not in selected_targets]
    if non_selected:
        ctrl.disable_targets(non_selected)
    ctrl.enable_targets(selected_targets)


def main():
    print("=" * 72)
    print("  RobStride 2-Motor Control — Nucleo CAN Bridge")
    print("=" * 72)
    print(f"  Port: {COM_PORT} | Motors: {MOTOR_IDS}")
    print()

    ctrl = DualMotorController()

    print(f"[1/2] Connecting to Nucleo CAN Bridge on {COM_PORT}...")
    if not ctrl.connect():
        print(f"FAILED to connect. Check {COM_PORT}.")
        return
    print("  OK connected")

    print("[2/2] Enabling both motors...")
    ctrl.enable_all()
    print("  OK enabled")

    selected_targets = [ctrl.motor_ids[0], ctrl.motor_ids[1]]
    print("\nControls:")
    print("  [ = select motor 1, ] = select motor 2, \\ = select both")
    print("  A/D jog, S/Space stop, Z zero, 1/2/3/4 goto +/-90/180 on selected")
    print(f"  Active target: {target_label(selected_targets)}")
    apply_target_isolation(ctrl, selected_targets)

    last_display = 0
    jogging = False
    last_dir = 0

    try:
        while ctrl.running:
            key_pressed = False
            key = ""
            if msvcrt.kbhit():
                raw = msvcrt.getch()
                try:
                    key = raw.decode("utf-8", errors="ignore").lower()
                    key_pressed = True
                except Exception:
                    pass

            if key_pressed:
                if key == "q":
                    print("\nStopping and quitting...")
                    ctrl.stop_all()
                    ctrl.running = False
                    break
                if key == "[":
                    selected_targets = [ctrl.motor_ids[0]]
                    apply_target_isolation(ctrl, selected_targets)
                    print(f"\n>>> TARGET {target_label(selected_targets)} <<<")
                elif key == "]":
                    selected_targets = [ctrl.motor_ids[1]]
                    apply_target_isolation(ctrl, selected_targets)
                    print(f"\n>>> TARGET {target_label(selected_targets)} <<<")
                elif key == "\\":
                    selected_targets = [ctrl.motor_ids[0], ctrl.motor_ids[1]]
                    apply_target_isolation(ctrl, selected_targets)
                    print(f"\n>>> TARGET {target_label(selected_targets)} <<<")
                if key == "a":
                    if not jogging or last_dir != -1:
                        ctrl.jog_targets(-1, selected_targets)
                        jogging = True
                        last_dir = -1
                elif key == "d":
                    if not jogging or last_dir != 1:
                        ctrl.jog_targets(1, selected_targets)
                        jogging = True
                        last_dir = 1
                elif key == "s" or key == " ":
                    ctrl.stop_targets(selected_targets)
                    jogging = False
                    last_dir = 0
                elif key == "z":
                    ctrl.stop_targets(selected_targets)
                    jogging = False
                    last_dir = 0
                    time.sleep(0.03)
                    ctrl.zero_targets(selected_targets)
                    print(f"\n>>> ZERO {target_label(selected_targets)} <<<")
                elif key == "1":
                    ctrl.stop_targets(selected_targets)
                    jogging = False
                    last_dir = 0
                    time.sleep(0.03)
                    ctrl.goto_targets(90, selected_targets)
                    print(f"\n>>> GOTO {target_label(selected_targets)} 90 DEG <<<")
                elif key == "2":
                    ctrl.stop_targets(selected_targets)
                    jogging = False
                    last_dir = 0
                    time.sleep(0.03)
                    ctrl.goto_targets(180, selected_targets)
                    print(f"\n>>> GOTO {target_label(selected_targets)} 180 DEG <<<")
                elif key == "3":
                    ctrl.stop_targets(selected_targets)
                    jogging = False
                    last_dir = 0
                    time.sleep(0.03)
                    ctrl.goto_targets(-90, selected_targets)
                    print(f"\n>>> GOTO {target_label(selected_targets)} -90 DEG <<<")
                elif key == "4":
                    ctrl.stop_targets(selected_targets)
                    jogging = False
                    last_dir = 0
                    time.sleep(0.03)
                    ctrl.goto_targets(-180, selected_targets)
                    print(f"\n>>> GOTO {target_label(selected_targets)} -180 DEG <<<")
            else:
                if jogging:
                    ctrl.stop_targets(selected_targets)
                    jogging = False
                    last_dir = 0

            now = time.time()
            if now - last_display > 0.12:
                lines = ctrl.format_lines()
                sys.stdout.write("\r" + " " * 160 + "\r")
                sys.stdout.write(lines[0])
                if len(lines) > 1:
                    sys.stdout.write("\n" + lines[1])
                    sys.stdout.write(f"\nActive target: {target_label(selected_targets):<6}")
                    sys.stdout.write("\x1b[1A")
                    sys.stdout.write("\x1b[1A")
                sys.stdout.flush()
                last_display = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nCtrl+C - stopping...")
        ctrl.stop_all()
    finally:
        ctrl.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
