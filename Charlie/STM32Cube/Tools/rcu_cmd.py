#!/usr/bin/env python3
"""
rcu_cmd.py — Send debug commands to the RCU board and display the reply

Usage:
  python rcu_cmd.py ping
  python rcu_cmd.py buzz
  python rcu_cmd.py led_blink
  python rcu_cmd.py can_loopback
  python rcu_cmd.py force_telem
  python rcu_cmd.py motor_cmd --bus 0 --id 1 --pos 0.0 --vel 0.0 --trq 0.0 --kp 10.0 --kd 1.0
  python rcu_cmd.py motor_enable  --bus 0 --id 1
  python rcu_cmd.py motor_stop    --bus 0 --id 1 [--clear-fault]

Options:
  --rcu-ip   RCU board IP    (default 192.168.100.10)
  --src-ip   Source IP for reply socket bind  (default 0.0.0.0)
  --timeout  Reply wait seconds  (default 3.0)
"""

import argparse
import socket
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Protocol constants (must match rcu_pkt.h)
# ---------------------------------------------------------------------------
PKT_MAGIC       = 0x5243
HDR_FMT         = "<HBBH"
HDR_SIZE        = struct.calcsize(HDR_FMT)

PKT_DEBUG_CMD   = 0x20
PKT_DEBUG_REPLY = 0x21
PKT_MOTOR_CMD   = 0x10

DBGCMD_PING         = 0x01
DBGCMD_BUZZ         = 0x02
DBGCMD_LED_BLINK    = 0x03
DBGCMD_CAN_LOOPBACK = 0x04
DBGCMD_FORCE_TELEM  = 0x05

RCU_IP_DEFAULT  = "192.168.100.10"
PORT_CMD_IN     = 7701   # RCU receives on this port
PORT_TELEM_OUT  = 7700   # RCU replies on this port

# rcu_debug_reply_t
DBG_REPLY_FMT  = "<II6Bxx I"
DBG_REPLY_SIZE = struct.calcsize(DBG_REPLY_FMT)

# rcu_motor_cmd_entry_t: bus u8, motor_id u8, pos u16, vel u16, trq u16, kp u8, kd u8
MOTOR_CMD_ENTRY_FMT  = "<BBHHHBBxx"   # 12 bytes with pad to align
MOTOR_CMD_ENTRY_SIZE = struct.calcsize(MOTOR_CMD_ENTRY_FMT)

# RS04 limits
RS04_POS_MAX = 12.57
RS04_VEL_MAX = 15.0
RS04_TRQ_MAX = 120.0
RS04_KP_MAX  = 5000.0
RS04_KD_MAX  = 100.0

_seq = 0

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def f_to_u16(v, lo, hi):
    v = max(lo, min(hi, v))
    return int((v - lo) / (hi - lo) * 65535 + 0.5)

def build_packet(pkt_type, payload_bytes):
    global _seq
    hdr = struct.pack(HDR_FMT, PKT_MAGIC, pkt_type, _seq & 0xFF, len(payload_bytes))
    _seq += 1
    return hdr + payload_bytes

def send_and_receive(sock, rcu_ip, data, timeout):
    sock.sendto(data, (rcu_ip, PORT_CMD_IN))
    sock.settimeout(timeout)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            resp, addr = sock.recvfrom(2048)
        except socket.timeout:
            break
        if len(resp) < HDR_SIZE:
            continue
        magic, pkt_type, seq, plen = struct.unpack_from(HDR_FMT, resp, 0)
        if magic != PKT_MAGIC:
            continue
        if pkt_type == PKT_DEBUG_REPLY:
            return resp[HDR_SIZE: HDR_SIZE + plen]
    return None

def print_debug_reply(payload):
    if payload is None:
        print("  [ERROR] No reply received (timeout).")
        return False

    if len(payload) < DBG_REPLY_SIZE:
        print(f"  [ERROR] Reply too short: {len(payload)} < {DBG_REPLY_SIZE}")
        return False

    (uptime, boot_rsr,
     imu0_v, imu1_v, fpga_v, rails_v, ssd_v, can_lb,
     hb_age) = struct.unpack_from(DBG_REPLY_FMT, payload)

    reset_causes = []
    RSR_PINRSTF  = 1 << 26
    RSR_SFTRSTF  = 1 << 24
    RSR_BORRSTF  = 1 << 21
    RSR_IWDG1    = 1 << 22
    RSR_WWDG1    = 1 << 28
    if boot_rsr & RSR_PINRSTF: reset_causes.append("PINRST(supervisor/button)")
    if boot_rsr & RSR_SFTRSTF: reset_causes.append("SW-RESET(fault)")
    if boot_rsr & RSR_BORRSTF: reset_causes.append("BOR(brownout)")
    if boot_rsr & RSR_IWDG1:   reset_causes.append("IWDG1")
    if boot_rsr & RSR_WWDG1:   reset_causes.append("WWDG1")
    rsr_str = " + ".join(reset_causes) if reset_causes else "clean"

    can_names = {0: "untested", 1: "RIGHT OK only", 2: "LEFT OK only",
                 3: "BOTH OK ✓", 0xFF: "FAIL ✗"}
    can_str = can_names.get(can_lb, f"0x{can_lb:02X}")

    hb_str = f"{hb_age}ms ago" if hb_age != 0xFFFFFFFF else "NEVER RECEIVED"

    print(f"\n  uptime       {uptime}ms  ({uptime/1000:.1f}s)")
    print(f"  last reset   {rsr_str}  (RSR=0x{boot_rsr:08X})")
    print(f"  IMU0         {'OK' if imu0_v else 'NOT VALID ← check SPI/WHO_AM_I'}")
    print(f"  IMU1         {'OK' if imu1_v else 'NOT VALID ← check SPI/WHO_AM_I'}")
    print(f"  PDU FPGA     {'OK (0x520 received)' if fpga_v else 'not received — PDU not connected?'}")
    print(f"  PDU rails    {'OK (0x521 received)' if rails_v else 'not received'}")
    print(f"  PDU Energy Meter  {'OK (0x523 received)' if ssd_v else 'not received'}")
    print(f"  PDU heartbeat {hb_str}")
    print(f"  CAN loopback  {can_str}")
    return True

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def cmd_debug(rcu_ip, subcmd, subcmd_name, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT_TELEM_OUT))
    print(f"Sending {subcmd_name} to {rcu_ip}:{PORT_CMD_IN} ...")
    pkt = build_packet(PKT_DEBUG_CMD, bytes([subcmd]))
    reply = send_and_receive(sock, rcu_ip, pkt, timeout)
    sock.close()
    print_debug_reply(reply)

def cmd_motor_cmd(rcu_ip, bus, motor_id, pos, vel, trq, kp, kd, timeout):
    pos_u16 = f_to_u16(pos, -RS04_POS_MAX, RS04_POS_MAX)
    vel_u16 = f_to_u16(vel, -RS04_VEL_MAX, RS04_VEL_MAX)
    trq_u16 = f_to_u16(trq, -RS04_TRQ_MAX, RS04_TRQ_MAX)
    kp_u8   = int(max(0, min(255, (kp / RS04_KP_MAX) * 255)))
    kd_u8   = int(max(0, min(255, (kd / RS04_KD_MAX) * 255)))

    entry = struct.pack(MOTOR_CMD_ENTRY_FMT,
                        bus, motor_id, pos_u16, vel_u16, trq_u16, kp_u8, kd_u8)
    pkt = build_packet(PKT_MOTOR_CMD, entry)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(pkt, (rcu_ip, PORT_CMD_IN))
    sock.close()

    bus_name = {0: "right", 1: "left"}.get(bus, str(bus))
    print(f"Sent motor_cmd: bus={bus_name} id={motor_id}  "
          f"pos={pos:.3f}rad vel={vel:.3f}rad/s trq={trq:.2f}Nm  "
          f"kp={kp:.1f} kd={kd:.2f}")
    print(f"  Encoded: pos_u16={pos_u16} vel_u16={vel_u16} trq_u16={trq_u16} "
          f"kp_u8={kp_u8} kd_u8={kd_u8}")
    print("  (No reply expected for motor_cmd — use rcu_monitor.py to see motor_fb)")

def cmd_motor_enable_stop(rcu_ip, bus, motor_id, enable, clear_fault):
    """
    Build a debug-cmd PING first to confirm comms, then a motor_cmd with
    the enable/stop convention documented in motor_bus.h.  Since enable/stop
    frames are sent by motor_bus_send_enable() on the board directly via CAN
    (not via eth_udp), we send them as a special motor_cmd entry with
    motor_id encoded in a reserved way.

    Actually motor_bus_send_enable is not reachable via UDP — the UDP RX path
    only handles PKT_MOTOR_CMD (Type 1 operation control) and PKT_DEBUG_CMD.
    Document this clearly rather than silently fail.
    """
    print("NOTE: motor_enable/motor_stop are not currently reachable via UDP.")
    print("      The RCU only accepts Type 1 (position/velocity control) motor commands")
    print("      over UDP.  Enable/stop must be added as a new debug sub-command if needed.")
    print("      Sending a PING instead to confirm connectivity:")
    cmd_debug(rcu_ip, DBGCMD_PING, "ping", 3.0)

_EPILOG = """
examples:
  # Check board health — uptime, IMU status, PDU CAN received, last reset cause:
  python rcu_cmd.py ping

  # Audible buzzer (200 ms) — confirm the board is alive without a monitor:
  python rcu_cmd.py buzz

  # Blink the orange LED 3x:
  python rcu_cmd.py led_blink

  # Run FDCAN1 (right bus) + FDCAN3 (left bus) loopback; result field shows 0x03 if both OK:
  python rcu_cmd.py can_loopback

  # Force the RCU to send a telemetry packet immediately (don't wait for the 100ms tick):
  python rcu_cmd.py force_telem

  # Send a position command to motor ID 1 on the right bus (bus 0):
  python rcu_cmd.py motor_cmd --bus 0 --id 1 --pos 0.0 --kp 50.0 --kd 2.0

  # Sinusoidal position sweep to ±0.5 rad with light stiffness:
  python rcu_cmd.py motor_cmd --bus 0 --id 1 --pos 0.5 --vel 0.0 --kp 20.0 --kd 1.0

  # Override RCU IP (e.g. if you changed it):
  python rcu_cmd.py --rcu-ip 192.168.100.20 ping

  # Increase timeout for slow networks:
  python rcu_cmd.py --timeout 5.0 can_loopback
"""

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="RCU command sender — send debug/motor commands and decode the reply.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_EPILOG,
    )
    ap.add_argument("--rcu-ip", default=RCU_IP_DEFAULT, help=f"RCU IP (default {RCU_IP_DEFAULT})")
    ap.add_argument("--timeout", type=float, default=3.0, help="Reply timeout seconds (default 3.0)")

    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping",         help="Request status reply from RCU")
    sub.add_parser("buzz",         help="200ms audible buzzer pulse")
    sub.add_parser("led_blink",    help="Blink orange LED 3x")
    sub.add_parser("can_loopback", help="Run FDCAN1+FDCAN3 loopback test")
    sub.add_parser("force_telem",  help="Force immediate slow-telem TX")

    mc = sub.add_parser("motor_cmd", help="Send a motor position/velocity command")
    mc.add_argument("--bus",    type=int,   required=True, choices=[0, 1],
                    help="0=right, 1=left")
    mc.add_argument("--id",     type=int,   required=True, metavar="MOTOR_ID",
                    help="Motor ID 1-8")
    mc.add_argument("--pos",    type=float, default=0.0,
                    help=f"Position rad (default 0.0, range ±{RS04_POS_MAX})")
    mc.add_argument("--vel",    type=float, default=0.0,
                    help=f"Velocity rad/s (default 0.0, range ±{RS04_VEL_MAX})")
    mc.add_argument("--trq",    type=float, default=0.0,
                    help=f"Feedforward torque Nm (default 0.0, range ±{RS04_TRQ_MAX})")
    mc.add_argument("--kp",     type=float, default=10.0,
                    help=f"Position gain (default 10.0, max {RS04_KP_MAX})")
    mc.add_argument("--kd",     type=float, default=1.0,
                    help=f"Velocity damping (default 1.0, max {RS04_KD_MAX})")

    me = sub.add_parser("motor_enable", help="[See note — not yet UDP-reachable]")
    me.add_argument("--bus", type=int, required=True, choices=[0, 1])
    me.add_argument("--id",  type=int, required=True, metavar="MOTOR_ID")

    ms = sub.add_parser("motor_stop", help="[See note — not yet UDP-reachable]")
    ms.add_argument("--bus",         type=int, required=True, choices=[0, 1])
    ms.add_argument("--id",          type=int, required=True, metavar="MOTOR_ID")
    ms.add_argument("--clear-fault", action="store_true")

    args = ap.parse_args()

    debug_map = {
        "ping":         DBGCMD_PING,
        "buzz":         DBGCMD_BUZZ,
        "led_blink":    DBGCMD_LED_BLINK,
        "can_loopback": DBGCMD_CAN_LOOPBACK,
        "force_telem":  DBGCMD_FORCE_TELEM,
    }

    if args.cmd in debug_map:
        cmd_debug(args.rcu_ip, debug_map[args.cmd], args.cmd, args.timeout)

    elif args.cmd == "motor_cmd":
        if args.id < 1 or args.id > 8:
            sys.exit("motor_id must be 1-8")
        cmd_motor_cmd(args.rcu_ip, args.bus, args.id,
                      args.pos, args.vel, args.trq, args.kp, args.kd, args.timeout)

    elif args.cmd in ("motor_enable", "motor_stop"):
        cmd_motor_enable_stop(args.rcu_ip, args.bus, args.id,
                              enable=(args.cmd == "motor_enable"),
                              clear_fault=getattr(args, "clear_fault", False))

if __name__ == "__main__":
    main()
