#!/usr/bin/env python3
"""Bridge ID scan: probe which motor IDs respond through Nucleo CAN bridge.

Usage examples:
  python test_bridge.py
  python test_bridge.py --port COM6 --max-id 127 --verbose
"""

import argparse
import struct
import time
from collections import Counter, defaultdict

import serial

HOST_ID = 0xFD
COMM_ENABLE = 3
COMM_READ_PARAMETER = 17
PARAM_MECH_POS = 0x7019


def build_can_frame(comm_type, motor_id, data, extra=HOST_ID):
	ext_id = (comm_type << 24) | (extra << 8) | motor_id
	reg32 = (ext_id << 3) | 0x04
	id_bytes = struct.pack(">I", reg32)
	return b"\x41\x54" + id_bytes + bytes([len(data)]) + data + b"\x0D\x0A"


def build_read_frame(motor_id, param_id):
	data = struct.pack("<HHL", param_id, 0x00, 0x00)
	return build_can_frame(COMM_READ_PARAMETER, motor_id, data)


def parse_frames(raw):
	results = []
	i = 0
	while i <= len(raw) - 9:
		if raw[i] == 0x41 and raw[i + 1] == 0x54:
			dlc = raw[i + 6]
			frame_len = 2 + 4 + 1 + dlc + 2
			if dlc <= 8 and i + frame_len <= len(raw):
				reg32 = struct.unpack(">I", raw[i + 2:i + 6])[0]
				ext_id = reg32 >> 3
				comm_type = (ext_id >> 24) & 0x1F
				data = raw[i + 7:i + 7 + dlc]
				results.append((comm_type, ext_id, data))
				i += frame_len
				continue
		i += 1
	return results


def candidate_ids_from_ext_id(ext_id, max_id):
	low = ext_id & 0xFF
	mid = (ext_id >> 8) & 0xFF
	out = []
	if 1 <= low <= max_id:
		out.append(low)
	if 1 <= mid <= max_id and mid != low:
		out.append(mid)
	return out


def main():
	parser = argparse.ArgumentParser(description="Probe motor IDs via Nucleo bridge")
	parser.add_argument("--port", default="COM6")
	parser.add_argument("--baud", type=int, default=921600)
	parser.add_argument("--max-id", type=int, default=127)
	parser.add_argument("--verbose", action="store_true")
	args = parser.parse_args()

	ser = serial.Serial(args.port, args.baud, timeout=0.03)
	try:
		time.sleep(0.2)
		ser.reset_input_buffer()
		ser.reset_output_buffer()

		ser.write(bytes.fromhex("41542b41540d0a"))
		ser.flush()
		time.sleep(0.25)
		print("Init:", repr(ser.read_all()))

		id_hits = Counter()
		comm_by_id = defaultdict(Counter)

		print(f"Scanning IDs 1..{args.max_id} ...")
		for motor_id in range(1, args.max_id + 1):
			ser.write(build_read_frame(motor_id, PARAM_MECH_POS))
			ser.flush()
			time.sleep(0.006)

			ser.write(build_can_frame(COMM_ENABLE, motor_id, b"\x00" * 8))
			ser.flush()
			time.sleep(0.010)

			raw = ser.read_all()
			if not raw:
				continue

			frames = parse_frames(raw)
			if args.verbose:
				print(f"  probe {motor_id:3d}: {len(frames)} frame(s)")

			for comm_type, ext_id, _ in frames:
				for cid in candidate_ids_from_ext_id(ext_id, args.max_id):
					id_hits[cid] += 1
					comm_by_id[cid][comm_type] += 1

		print("\n=== Bridge Probe Results ===")
		if not id_hits:
			print("No responding IDs found")
			return

		for motor_id, hits in id_hits.most_common():
			comm_summary = ", ".join(
				f"ct{ct}:{count}" for ct, count in sorted(comm_by_id[motor_id].items())
			)
			print(f"ID {motor_id:3d} -> hits={hits:3d} [{comm_summary}]")

	finally:
		ser.close()
		print("Done.")


if __name__ == "__main__":
	main()