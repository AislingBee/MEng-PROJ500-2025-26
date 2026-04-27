"""Set a RobStride motor CAN ID through the Nucleo serial bridge.

Safety notes:
- Power ONLY the motor you want to re-address.
- This tool requires the CAN-ID parameter address from your exact motor manual.
- Use probe_motor_ids.py before and after to confirm results.

Examples:
    # Manual Command 7 style (11-bit ID + FF FF FF FF FF FF CMD NEW_ID)
    python set_motor_id.py --current-id 127 --new-id 2 --method cmd7 --port COM6

    # Parameter write style (only if your manual confirms param address)
    python set_motor_id.py --current-id 127 --new-id 2 --method param --param-id 0x7010 --port COM6
"""

import argparse
import serial
import struct
import time

HOST_ID = 0xFD
BAUD_RATE = 921600
COMM_ENABLE = 3
COMM_READ_PARAMETER = 17
PARAM_MECH_POS = 0x7019


def build_can_frame(comm_type, motor_id, data, extra=HOST_ID):
    ext_id = (comm_type << 24) | (extra << 8) | motor_id
    reg32 = (ext_id << 3) | 0x04
    id_bytes = struct.pack(">I", reg32)
    return b"\x41\x54" + id_bytes + bytes([len(data)]) + data + b"\x0D\x0A"


def build_std_frame(std_id, data):
    # 11-bit standard CAN ID frame for Command 7/8 style commands.
    reg32 = (std_id & 0x7FF) << 3
    id_bytes = struct.pack(">I", reg32)
    return b"\x41\x54" + id_bytes + bytes([len(data)]) + data + b"\x0D\x0A"


def build_cmd7_change_id_frame(current_id, new_id, cmd_byte=0x07):
    payload = b"\xFF" * 6 + bytes([cmd_byte & 0xFF, new_id & 0xFF])
    return build_std_frame(current_id, payload)


def build_cmd7_ext_frame(current_id, new_id):
    """CommType 7 as extended frame: ext_id = (7<<24)|(new_id<<8)|current_id, data = 8 zeros.
    This matches the Seeed robstride_dynamics library write_id() behaviour."""
    ext_id = (7 << 24) | (new_id << 8) | (current_id & 0xFF)
    reg32 = (ext_id << 3) | 0x04
    id_bytes = struct.pack(">I", reg32)
    return b"\x41\x54" + id_bytes + bytes([8]) + b"\x00" * 8 + b"\x0D\x0A"


def build_save_frame(motor_id):
    """CommType 22 (SAVE_PARAMETERS): persists parameters to flash."""
    return build_can_frame(22, motor_id, b"\x00" * 8)


def build_param_write_frame(motor_id, param_id, value_u32):
    # CommType 18 write parameter frame encoded as 0x12 in the upper ID field.
    ext_id = (0x12 << 24) | (HOST_ID << 8) | motor_id
    reg32 = (ext_id << 3) | 0x04
    id_bytes = struct.pack(">I", reg32)
    payload = struct.pack("<H", param_id) + b"\x00\x00" + struct.pack("<I", value_u32)
    return b"\x41\x54" + id_bytes + bytes([len(payload)]) + payload + b"\x0D\x0A"


def build_read_frame(motor_id, param_id):
    data = struct.pack("<HHL", param_id, 0x00, 0x00)
    return build_can_frame(COMM_READ_PARAMETER, motor_id, data)


def parse_frames(raw):
    out = []
    i = 0
    n = len(raw)
    while i <= n - 9:
        if raw[i] == 0x41 and raw[i + 1] == 0x54:
            dlc = raw[i + 6]
            frame_len = 2 + 4 + 1 + dlc + 2
            if dlc <= 8 and i + frame_len <= n:
                reg32 = struct.unpack(">I", raw[i + 2:i + 6])[0]
                ext_id = reg32 >> 3
                comm_type = (ext_id >> 24) & 0x1F
                data = raw[i + 7:i + 7 + dlc]
                out.append((comm_type, ext_id, data))
                i += frame_len
                continue
        i += 1
    return out


def id_seen_in_frames(test_id, frames):
    for _, ext_id, _ in frames:
        low = ext_id & 0xFF
        mid = (ext_id >> 8) & 0xFF
        if low == test_id or mid == test_id:
            return True
    return False


def ping_id(ser, motor_id, tries=8):
    seen = False
    for _ in range(tries):
        ser.write(build_read_frame(motor_id, PARAM_MECH_POS))
        ser.flush()
        time.sleep(0.008)
        ser.write(build_can_frame(COMM_ENABLE, motor_id, b"\x00" * 8))
        ser.flush()
        time.sleep(0.012)
        frames = parse_frames(ser.read_all())
        if id_seen_in_frames(motor_id, frames):
            seen = True
            break
    return seen


def main():
    parser = argparse.ArgumentParser(description="Set motor CAN ID (requires manual param ID).")
    parser.add_argument("--current-id", type=int, required=True, help="Current motor ID (1..127)")
    parser.add_argument("--new-id", type=int, required=True, help="New motor ID (1..127)")
    parser.add_argument("--method", choices=["cmd7", "cmd7ext", "param"], default="cmd7ext",
                        help="ID change method: cmd7ext (CommType 7 extended frame, correct protocol), "
                             "cmd7 (legacy standard frame), or param write")
    parser.add_argument("--param-id", type=lambda x: int(x, 0),
                        help="CAN-ID parameter address from manual, e.g. 0x7010 (for --method param)")
    parser.add_argument("--cmd-byte", type=lambda x: int(x, 0), default=0x07,
                        help="Command byte for cmd7 payload (default: 0x07)")
    parser.add_argument("--port", default="COM6", help="Serial port (default: COM6)")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Baud rate (default: 921600)")
    parser.add_argument("--yes", action="store_true", help="Skip confirmation prompt")
    args = parser.parse_args()

    if not (1 <= args.current_id <= 127 and 1 <= args.new_id <= 127):
        raise SystemExit("IDs must be in range 1..127")
    if args.current_id == args.new_id:
        raise SystemExit("Current and new IDs are the same; nothing to do.")
    if args.method == "param" and args.param_id is None:
        raise SystemExit("--param-id is required when --method param is used.")

    if not args.yes:
        print("WARNING: Ensure ONLY one motor is powered/connected before ID change.")
        if args.method == "cmd7ext":
            ext_id = (7 << 24) | (args.new_id << 8) | args.current_id
            print(f"Will send CommType7 extended frame: {args.current_id} -> {args.new_id} on {args.port}")
            print(f"  ext_id=0x{ext_id:08X}, data=00*8")
        elif args.method == "cmd7":
            print(f"Will send Command 7 style write: {args.current_id} -> {args.new_id} on {args.port}")
            print(f"Payload: FF FF FF FF FF FF {args.cmd_byte:02X} {args.new_id:02X}")
        else:
            print(f"Will write param 0x{args.param_id:04X}: {args.current_id} -> {args.new_id} on {args.port}")
        conf = input("Type YES to continue: ").strip()
        if conf != "YES":
            print("Aborted.")
            return

    print(f"Opening {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=0.03)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        ser.write(bytes.fromhex("41542b41540d0a"))
        ser.flush()
        time.sleep(0.25)
        print(f"Bridge init: {ser.read_all()!r}")

        print(f"Checking current ID {args.current_id}...")
        old_seen_before = ping_id(ser, args.current_id)
        print(f"Current ID seen: {old_seen_before}")

        print("Writing new ID...")
        if args.method == "cmd7ext":
            ser.write(build_cmd7_ext_frame(args.current_id, args.new_id))
        elif args.method == "cmd7":
            ser.write(build_cmd7_change_id_frame(args.current_id, args.new_id, args.cmd_byte))
        else:
            ser.write(build_param_write_frame(args.current_id, args.param_id, args.new_id))
        ser.flush()
        time.sleep(0.1)
        _ = ser.read_all()

        print("Saving parameters to flash (CommType 22)...")
        ser.write(build_save_frame(args.current_id))
        ser.flush()
        time.sleep(0.3)
        _ = ser.read_all()

        print("Verifying IDs...")
        old_seen_after = ping_id(ser, args.current_id)
        new_seen_after = ping_id(ser, args.new_id)

        print("\n=== Result ===")
        print(f"Old ID {args.current_id} seen after write: {old_seen_after}")
        print(f"New ID {args.new_id} seen after write: {new_seen_after}")

        if new_seen_after and not old_seen_after:
            print("SUCCESS: ID change appears to have taken effect.")
        elif new_seen_after and old_seen_after:
            print("PARTIAL: Both IDs responded. Power-cycle and probe again.")
        else:
            print("UNCONFIRMED: New ID not detected. Verify --param-id from manual and retry.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
