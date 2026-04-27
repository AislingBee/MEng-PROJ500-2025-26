"""Change a RobStride motor CAN ID using a USB-CAN adapter (e.g. CANable).

This sends the confirmed Command 7 as a standard 11-bit CAN frame, which
cannot be done through the Nucleo bridge (it forces extended frames).

Requirements:
    pip install python-can

Hardware:
    - CANable (or similar slcan-compatible adapter) connected via USB
    - Motor CAN H/L wired to adapter CAN H/L
    - 120-ohm termination resistor across CAN H/L at each end
    - Power ONLY the motor you want to re-address

Usage:
    python change_id_usb_can.py --port COMX --current-id 127 --new-id 2

After running:
    - Power cycle the motor
    - Probe with: python probe_motor_ids.py --port COM6 --ids 2 127
"""

import argparse
import time

import can


def scan(bus: can.BusABC, start: int = 1, end: int = 127, timeout: float = 0.05):
    """Scan for responding motor IDs using CommType 0 (GET_DEVICE_ID) extended frames."""
    found = []
    host_id = 0xFF
    for motor_id in range(start, end + 1):
        ext_id = (0 << 24) | (host_id << 8) | motor_id
        msg = can.Message(arbitration_id=ext_id, is_extended_id=True,
                          data=b"\x00" * 8, check=True)
        bus.send(msg)
        resp = bus.recv(timeout=timeout)
        if resp is not None:
            found.append(motor_id)
    return found


def change_id(bus: can.BusABC, current_id: int, new_id: int):
    """Send Command 7 as a standard 11-bit frame to change the motor CAN ID."""
    payload = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, new_id & 0xFF])
    msg = can.Message(arbitration_id=current_id & 0x7FF,
                      is_extended_id=False,
                      data=payload,
                      check=True)
    bus.send(msg)
    print(f"Sent: std_id=0x{current_id:03X}  data={payload.hex(' ').upper()}")


def main():
    parser = argparse.ArgumentParser(description="Change RobStride motor CAN ID via USB-CAN adapter.")
    parser.add_argument("--port", required=True, help="Serial port of USB-CAN adapter, e.g. COM3")
    parser.add_argument("--current-id", type=int, required=True, help="Current motor ID (1..127)")
    parser.add_argument("--new-id", type=int, default=None, help="New motor ID (1..127); prompted if omitted")
    parser.add_argument("--interface", default="slcan",
                        help="python-can interface type (default: slcan for CANable)")
    parser.add_argument("--bitrate", type=int, default=1000000,
                        help="CAN bitrate in bps (default: 1000000)")
    parser.add_argument("--scan", action="store_true",
                        help="Scan for motors before and after the ID change")
    parser.add_argument("--yes", action="store_true", help="Skip confirmation prompt")
    args = parser.parse_args()

    if args.new_id is None:
        try:
            args.new_id = int(input(f"Enter new ID for motor {args.current_id} (1..127): ").strip())
        except ValueError:
            raise SystemExit("Invalid ID entered.")

    if not (1 <= args.current_id <= 127 and 1 <= args.new_id <= 127):
        raise SystemExit("IDs must be in range 1..127")
    if args.current_id == args.new_id:
        raise SystemExit("Current and new IDs are the same; nothing to do.")

    print(f"Interface : {args.interface}")
    print(f"Port      : {args.port}")
    print(f"Bitrate   : {args.bitrate}")
    print(f"ID change : {args.current_id} -> {args.new_id}")
    print()
    print("IMPORTANT: Power ONLY the motor you want to re-address.")
    print("           Disconnect the Nucleo bridge CAN lines first (share the same bus).")
    print()

    if not args.yes:
        conf = input("Type YES to continue: ").strip()
        if conf != "YES":
            print("Aborted.")
            return

    bus = can.interface.Bus(interface=args.interface, channel=args.port, bitrate=args.bitrate)
    try:
        if args.scan:
            print("Scanning for motors (this takes ~10s)...")
            found = scan(bus)
            print(f"Found IDs before change: {found}")
            if args.current_id not in found:
                print(f"WARNING: current ID {args.current_id} not found in scan. Proceeding anyway.")
            print()

        print("Sending Command 7 (standard 11-bit frame)...")
        change_id(bus, args.current_id, args.new_id)
        time.sleep(0.5)

        if args.scan:
            print("Scanning again after change...")
            found_after = scan(bus)
            print(f"Found IDs after change: {found_after}")

        print()
        print("Done. Now:")
        print(f"  1. Power cycle the motor")
        print(f"  2. Reconnect the Nucleo bridge")
        print(f"  3. Run: python probe_motor_ids.py --port COM6 --ids {args.new_id} {args.current_id}")

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
