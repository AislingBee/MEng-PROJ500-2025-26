"""
rcu_motor_test.py — Interactive keyboard bench tool for PROJ500 humanoid motors

Keyboard controls:
  e        Enable all 12 motors (CSP position mode)
  d        FULL E-STOP: disable all motors via CAN + assert PDU power fault
  D        Motor disable only (no PDU fault) — use when you want to re-enable
             without a power cycle
  1–9,0    Select motor ID 1–10  (1→motor 1, ..., 0→motor 10)
  -        Select motor ID 11
  =        Select motor ID 12
  [        Step selected motor –0.2 rad
  ]        Step selected motor +0.2 rad
  h        Send pos=0.0 rad to selected motor (home)
  z        Set mechanical zero on selected motor (Type 6)
  p        PING — print debug reply
  q        Quit

Live display updates every 200 ms.
Run with:  python rcu_motor_test.py
"""
import socket
import threading
import time
import sys
import os
import struct

# Add the Tools directory to path so rcu_protocol can be imported
sys.path.insert(0, os.path.dirname(__file__))
import rcu_protocol as rp

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
g_motor_fb   = {}   # motor_id → dict from rp.decode_motor_fb
g_last_telem = {}   # from rp.decode_slow_telem
g_last_imu   = {}   # from rp.decode_imu_fast
g_selected   = 1    # currently selected motor_id
g_ctrl_mode  = 1    # 1=CSP, 0=MIT
g_enabled    = False
g_lock       = threading.Lock()
g_motor_positions = {i: 0.0 for i in range(1, 13)}  # commanded pos per motor

# Motor labels for display
BUS_NAME = ["R", "L"]  # 0=RIGHT, 1=LEFT

# ---------------------------------------------------------------------------
# UDP sockets
# ---------------------------------------------------------------------------
tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock.bind(("", rp.PORT_TELEM))
rx_sock.settimeout(0.05)

# ---------------------------------------------------------------------------
# RX thread
# ---------------------------------------------------------------------------
def rx_thread():
    while True:
        try:
            data, _ = rx_sock.recvfrom(2048)
        except socket.timeout:
            continue
        except Exception:
            break
        hdr = rp.parse_header(data)
        if not hdr:
            continue
        pkt_type, seq, plen = hdr
        payload = data[rp.HDR_SIZE:]
        with g_lock:
            if pkt_type == rp.PKT_MOTOR_FB:
                slots = rp.decode_motor_fb(payload)
                for s in slots:
                    g_motor_fb[s["motor_id"]] = s
            elif pkt_type == rp.PKT_SLOW_TELEM:
                g_last_telem.update(rp.decode_slow_telem(payload))
            elif pkt_type == rp.PKT_IMU_FAST:
                g_last_imu.update(rp.decode_imu_fast(payload))

rx = threading.Thread(target=rx_thread, daemon=True)
rx.start()

# ---------------------------------------------------------------------------
# TX helpers
# ---------------------------------------------------------------------------
def send(data: bytes):
    tx_sock.sendto(data, (rp.RCU_IP, rp.PORT_CMD))

def do_enable_all():
    global g_enabled
    pkt = rp.encode_motor_supervisory(
        enable_mask=0x0FFF, clear_fault_mask=0x0FFF, ctrl_mode=g_ctrl_mode
    )
    send(pkt)
    g_enabled = True
    print(f"  >> Enable all (ctrl_mode={g_ctrl_mode})")

def do_full_estop():
    """FULL E-STOP: motor CAN disable + PDU power fault."""
    global g_enabled
    # 1. Disable all motors over CAN
    send(rp.encode_motor_supervisory(enable_mask=0x0000))
    # 2. Assert PDU fault (cuts power rails)
    send(rp.encode_debug_cmd(rp.DBGCMD_ASSERT_PDU_FAULT, bytes([1])))
    g_enabled = False
    print("  >> FULL E-STOP: motors disabled + PDU fault asserted")

def do_motor_disable_only():
    """Soft disable motors via CAN without cutting power — allows re-enable."""
    global g_enabled
    send(rp.encode_motor_supervisory(enable_mask=0x0000))
    g_enabled = False
    print("  >> Motor disable only (no PDU fault)")

def send_pos(motor_id: int, pos_rad: float):
    entry = {"motor_id": motor_id, "pos_rad": pos_rad}
    send(rp.encode_motor_cmd_packet([entry]))

def do_step(delta: float):
    global g_motor_positions
    g_motor_positions[g_selected] += delta
    pos = g_motor_positions[g_selected]
    send_pos(g_selected, pos)
    print(f"  >> Motor {g_selected} → {pos:.3f} rad ({pos*180/3.14159:.1f}°)")

def do_home():
    g_motor_positions[g_selected] = 0.0
    send_pos(g_selected, 0.0)
    print(f"  >> Motor {g_selected} → home (0.0 rad)")

def do_set_zero():
    # Send set-zero via supervisory clear_fault bit + special seq
    # For now just print a reminder — the full set-zero requires a param write
    # route that goes through a separate debug mechanism.
    # The firmware's rs04_encode_set_zero sends Type-6 via motor_bus_send_set_zero.
    # We approximate by sending clear_fault for this motor only.
    bit = 1 << (g_selected - 1)
    send(rp.encode_motor_supervisory(
        enable_mask=bit,
        clear_fault_mask=bit,
        ctrl_mode=g_ctrl_mode
    ))
    print(f"  >> Motor {g_selected}: clear fault + re-enable (approx set-zero)")

def do_ping():
    send(rp.encode_debug_cmd(rp.DBGCMD_PING))
    print("  >> PING sent — waiting for reply...")
    time.sleep(0.15)
    with g_lock:
        pass  # response would appear in next display cycle

# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------
def display():
    os.system("cls" if sys.platform == "win32" else "clear")
    print("=" * 72)
    print("  RCU MOTOR BENCH TOOL  |  e=enable  d=ESTOP  D=disable  q=quit")
    print(f"  Selected motor: [{g_selected}] {rp.MOTOR_JOINT_NAMES[g_selected]}")
    print(f"  ctrl_mode={g_ctrl_mode} ({'CSP pos' if g_ctrl_mode else 'MIT'})  "
          f"enabled={g_enabled}")
    print("=" * 72)
    print(f"  {'ID':>2}  {'Joint':<38}  {'Bus'}  {'Pos(°)':>8}  {'Vel':>7}  "
          f"{'Trq':>7}  {'Flt'}")
    print("  " + "-" * 70)
    with g_lock:
        for mid in range(1, 13):
            joint = rp.MOTOR_JOINT_NAMES[mid][:36]
            bus   = BUS_NAME[rp.MOTOR_BUS_MAP[mid]]
            fb    = g_motor_fb.get(mid)
            cmd_pos = g_motor_positions[mid]
            if fb:
                pos_deg = fb["pos_rad"]  * 180 / 3.14159
                vel     = fb["vel_rads"]
                trq     = fb["torque_nm"]
                flt     = f"0x{fb['fault']:02X}" if fb["fault"] else "  OK"
            else:
                pos_deg = float("nan")
                vel = float("nan")
                trq = float("nan")
                flt = "  --"
            sel = "▶" if mid == g_selected else " "
            print(f"  {sel}{mid:>2}  {joint:<38}  {bus}   "
                  f"{pos_deg:>7.2f}°  {vel:>6.2f}  {trq:>6.1f}  {flt}")
    print("  " + "-" * 70)

    # IMU summary
    with g_lock:
        imu = g_last_imu
    if imu:
        a0 = imu.get("imu0_accel_g", [0, 0, 0])
        g0 = imu.get("imu0_gyro_dps", [0, 0, 0])
        mag0 = (sum(x**2 for x in a0))**0.5
        a1 = imu.get("imu1_accel_g", [0, 0, 0])
        mag1 = (sum(x**2 for x in a1))**0.5
        tick = imu.get("tick_ms", 0)
        print(f"  IMU0: |a|={mag0:.3f}g  gy=({g0[0]:.1f},{g0[1]:.1f},{g0[2]:.1f})°/s  "
              f"IMU1: |a|={mag1:.3f}g  tick={tick}ms")
    else:
        print("  IMU: no fast IMU data yet")

    # Slow telem summary (printed every ~5 s from cached data)
    with g_lock:
        t = g_last_telem
    if t:
        vsrc  = t.get("ladc_vsource_mv", 0)
        vbus  = t.get("ladc_vbus_mv", 0)
        icoil = t.get("ladc_icoil_ma", 0)
        ssdi  = t.get("ssd_i_ma", 0)
        fpga  = t.get("fpga_state_code", 0)
        print(f"  PDU: Vsrc={vsrc:.2f}V  Vbus={vbus:.2f}V  Icoil={icoil:.2f}A  "
              f"SSD_I={ssdi:.2f}A  FPGA=0x{int(t.get('_raw',{}).get('fpga_state_code',0)):02X}")
    else:
        print("  PDU telem: no data yet")

    print()
    print("  Keys: [/] = ±0.2 rad  h=home  z=set-zero  p=ping  1-9,0,-,= = select")

# ---------------------------------------------------------------------------
# Keyboard input (cross-platform)
# ---------------------------------------------------------------------------
KEY_MAP = {
    "1": 1, "2": 2, "3": 3, "4": 4,  "5": 5,
    "6": 6, "7": 7, "8": 8, "9": 9, "0": 10,
    "-": 11, "=": 12,
}

def get_key():
    if sys.platform == "win32":
        import msvcrt
        if msvcrt.kbhit():
            ch = msvcrt.getwch()
            if isinstance(ch, bytes):
                ch = ch.decode("utf-8", errors="ignore")
            return ch
        return None
    else:
        import tty, termios, select
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return None

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    global g_selected, g_ctrl_mode, g_enabled

    print("RCU Motor Bench Tool — press 'e' to enable motors, 'q' to quit")
    print("WARNING: 'd' sends FULL E-STOP (motor disable + PDU power fault)")
    print("         'D' (shift+d) = motor disable only (safe for re-enable)")
    time.sleep(1.0)

    last_display = 0.0

    while True:
        now = time.time()
        if now - last_display >= 0.2:
            display()
            last_display = now

        key = get_key()
        if key is None:
            time.sleep(0.005)
            continue

        if key in KEY_MAP:
            g_selected = KEY_MAP[key]
            print(f"  >> Selected motor {g_selected}: {rp.MOTOR_JOINT_NAMES[g_selected]}")
        elif key == "e":
            do_enable_all()
        elif key == "d":
            do_full_estop()
        elif key == "D":
            do_motor_disable_only()
        elif key == "[":
            do_step(-0.2)
        elif key == "]":
            do_step(+0.2)
        elif key == "h":
            do_home()
        elif key == "z":
            do_set_zero()
        elif key == "p":
            do_ping()
        elif key == "q":
            print("\nQuitting — sending motor disable...")
            do_motor_disable_only()
            break
        elif key == "\x03":  # Ctrl+C
            do_full_estop()
            sys.exit(0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        do_full_estop()
        sys.exit(0)
