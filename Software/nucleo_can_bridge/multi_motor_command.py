#!/usr/bin/env python3
"""
Send simultaneous commands to multiple motors via Nucleo CAN bridge.

Usage:
    python multi_motor_command.py --motors 1,2 --positions 90,-90 --port COM6
    python multi_motor_command.py --motors 1,2 --velocities 1.0,-1.0
"""
import serial
import struct
import time
import argparse
import math


# Communication types
COMM_ENABLE = 3
COMM_DISABLE = 4

# Parameter IDs
PARAM_MODE = 0x7005
PARAM_POSITION_TARGET = 0x7016
PARAM_PP_SPEED_LIMIT = 0x7024
PARAM_PP_ACCEL = 0x7025
PARAM_VELOCITY_TARGET = 0x700A

# Motion settings
PP_SPEED = 10.0  # rad/s
PP_ACCEL = 10.0  # rad/s²
JOG_SPEED = 1.0  # rad/s
HOST_ID = 0xFD


def build_frame(motor_id, param_id, value_bytes, host_id=HOST_ID):
    """Build AT-frame parameter write."""
    ext_id = (0x12 << 24) | (host_id << 8) | motor_id
    reg32 = (ext_id << 3) | 0x04
    id_bytes = struct.pack(">I", reg32)
    param_bytes = struct.pack("<H", param_id)
    data = param_bytes + b'\x00\x00' + value_bytes
    frame = b'\x41\x54' + id_bytes + bytes([len(data)]) + data + b'\x0D\x0A'
    return frame


def build_can_frame(comm_type, motor_id, data, host_id=HOST_ID):
    """Build AT-frame CAN command."""
    ext_id = (comm_type << 24) | (host_id << 8) | motor_id
    reg32 = (ext_id << 3) | 0x04
    id_bytes = struct.pack(">I", reg32)
    frame = b'\x41\x54' + id_bytes + bytes([len(data)]) + data + b'\x0D\x0A'
    return frame


def send_position_command(ser, motor_id, position_deg):
    """Send full position command sequence."""
    target_rad = math.radians(position_deg)
    
    # 1. Set mode to positioning (1)
    ser.write(build_frame(motor_id, PARAM_MODE, struct.pack("<I", 1)))
    ser.flush()
    time.sleep(0.01)
    
    # 2. Enable motor
    ser.write(build_can_frame(COMM_ENABLE, motor_id, b"\x00" * 8))
    ser.flush()
    time.sleep(0.01)
    
    # 3. Set speed limit
    ser.write(build_frame(motor_id, PARAM_PP_SPEED_LIMIT, struct.pack("<f", PP_SPEED)))
    ser.flush()
    time.sleep(0.01)
    
    # 4. Set acceleration
    ser.write(build_frame(motor_id, PARAM_PP_ACCEL, struct.pack("<f", PP_ACCEL)))
    ser.flush()
    time.sleep(0.01)
    
    # 5. Send position target
    ser.write(build_frame(motor_id, PARAM_POSITION_TARGET, struct.pack("<f", target_rad)))
    ser.flush()
    time.sleep(0.01)


def send_velocity_command(ser, motor_id, velocity_rad_s):
    """Send full velocity (jog) command sequence."""
    # Encode velocity for jog mode: velocity range [-15, +15] maps to [0, 65535]
    vel_u16 = int(((velocity_rad_s / 15.0) + 1.0) * 32767)
    vel_u16 = max(0, min(65535, vel_u16))
    
    # Jog mode payload: [0x07, 0x01, velocity_hi, velocity_lo]
    payload = bytes([0x07, 0x01]) + struct.pack(">H", vel_u16)
    
    ser.write(build_frame(motor_id, PARAM_MODE, payload))
    ser.flush()
    time.sleep(0.01)


def send_stop_command(ser, motor_id):
    """Send stop command to motor."""
    # Stop payload: [0x07, 0x00, 0x7F, 0xFF]
    payload = bytes([0x07, 0x00, 0x7F, 0xFF])
    
    ser.write(build_frame(motor_id, PARAM_MODE, payload))
    ser.flush()
    time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser(description="Multi-motor command sender")
    parser.add_argument("--port", default="COM6", help="Serial port")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    parser.add_argument("--motors", required=True, help="Comma-separated motor IDs (e.g., 1,2)")
    parser.add_argument("--positions", help="Target positions in degrees (e.g., 90,-90)")
    parser.add_argument("--velocities", help="Target velocities in rad/s (e.g., 1.0,-1.0)")
    parser.add_argument("--duration", type=float, default=2.0, help="Duration for velocity commands (seconds)")
    
    args = parser.parse_args()
    
    # Parse motor IDs
    motor_ids = [int(m.strip()) for m in args.motors.split(",")]
    
    # Parse targets
    if args.positions:
        targets = [float(p.strip()) for p in args.positions.split(",")]
        command_type = "position"
    elif args.velocities:
        targets = [float(v.strip()) for v in args.velocities.split(",")]
        command_type = "velocity"
    else:
        parser.error("Must specify either --positions or --velocities")
    
    if len(targets) != len(motor_ids):
        parser.error("Number of targets must match number of motors")
    
    # Connect and send
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
        time.sleep(0.5)
        
        print(f"Sending {command_type} commands to motors {motor_ids}...")
        for motor_id, target in zip(motor_ids, targets):
            if command_type == "position":
                send_position_command(ser, motor_id, target)
                print(f"  Motor {motor_id}: {target:+7.1f}°")
            else:
                send_velocity_command(ser, motor_id, target)
                print(f"  Motor {motor_id}: {target:+6.2f} rad/s")
        
        if command_type == "velocity":
            print(f"Running for {args.duration:.1f} seconds...")
            time.sleep(args.duration)
            
            print("Stopping motors...")
            for motor_id in motor_ids:
                send_stop_command(ser, motor_id)
            print("✓ Motors stopped")
        else:
            print("✓ Position commands sent")
        
        ser.close()
        
    except Exception as e:
        print(f"ERROR: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
