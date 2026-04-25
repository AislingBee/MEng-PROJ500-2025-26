"""Probe likely RobStride motor CAN IDs through the Nucleo serial bridge."""

import serial
import struct
import time
from collections import Counter, defaultdict

COM_PORT = "COM6"
BAUD_RATE = 921600
HOST_ID = 0xFD
MAX_ID = 127

COMM_ENABLE = 3
COMM_READ_PARAMETER = 17
PARAM_MECH_POS = 0x7019


def build_can_frame(comm_type, motor_id, data, extra=HOST_ID):
    ext_id = (comm_type << 24) | (extra << 8) | motor_id
    reg32 = (ext_id << 3) | 0x04
    id_bytes = struct.pack(">I", reg32)
    return b"\x41\x54" + id_bytes + bytes([len(data)]) + data + b"\x0D\x0A"


def build_read_frame(motor_id, param_id):
    payload = struct.pack("<HHL", param_id, 0x00, 0x00)
    return build_can_frame(COMM_READ_PARAMETER, motor_id, payload)


def parse_frames(raw):
    frames = []
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
                frames.append((comm_type, ext_id, data))
                i += frame_len
                continue
        i += 1
    return frames


def extract_candidate_ids(ext_id):
    low = ext_id & 0xFF
    mid = (ext_id >> 8) & 0xFF
    out = []
    if 1 <= low <= MAX_ID:
        out.append(low)
    if 1 <= mid <= MAX_ID and mid != low:
        out.append(mid)
    return out


def main():
    print(f"Opening {COM_PORT} @ {BAUD_RATE}...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.02)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Initialize bridge transport.
        ser.write(bytes.fromhex("41542b41540d0a"))
        ser.flush()
        time.sleep(0.25)
        init_resp = ser.read_all()
        print(f"Bridge init response: {init_resp!r}")

        id_hits = Counter()
        id_comm_types = defaultdict(Counter)

        print("Probing IDs 1..127 (read + enable)...")
        for motor_id in range(1, MAX_ID + 1):
            ser.write(build_read_frame(motor_id, PARAM_MECH_POS))
            ser.flush()
            time.sleep(0.006)

            ser.write(build_can_frame(COMM_ENABLE, motor_id, b"\x00" * 8))
            ser.flush()
            time.sleep(0.008)

            raw = ser.read_all()
            if not raw:
                continue

            for comm_type, ext_id, _ in parse_frames(raw):
                for cid in extract_candidate_ids(ext_id):
                    id_hits[cid] += 1
                    id_comm_types[cid][comm_type] += 1

        print("\n=== Probe Results ===")
        if not id_hits:
            print("No responding IDs found.")
            return

        for motor_id, hits in id_hits.most_common():
            comm_str = ", ".join(
                f"ct{ct}:{count}" for ct, count in sorted(id_comm_types[motor_id].items())
            )
            print(f"ID {motor_id:3d} -> hits={hits:3d} [{comm_str}]")

    finally:
        ser.close()
        print("\nDone.")


if __name__ == "__main__":
    main()
