#!/usr/bin/env python3
"""
rcu_monitor.py — Continuous telemetry decoder for RCU board

Binds on 0.0.0.0:7700 and decodes all incoming RCU packets:
  0x01  Slow telemetry  (10 Hz)   — PDU voltages/currents, IMU accel/gyro
  0x02  Motor feedback  (~100 Hz) — RS04 decoded position/velocity/torque
  0x21  Debug reply               — uptime, subsystem validity flags

Run:
  python rcu_monitor.py
  python rcu_monitor.py --log telem.csv      # also write CSV
  python rcu_monitor.py --quiet-fb           # suppress 100 Hz motor FB spam
"""

import argparse
import csv
import socket
import struct
import sys
import time
from datetime import datetime

# ---------------------------------------------------------------------------
# Protocol constants (must match rcu_pkt.h)
# ---------------------------------------------------------------------------
PKT_MAGIC        = 0x5243   # 'RC' LE
HDR_FMT          = "<HBBH"  # magic, type, seq, len
HDR_SIZE         = struct.calcsize(HDR_FMT)  # 6

PKT_SLOW_TELEM   = 0x01
PKT_MOTOR_FB     = 0x02
PKT_SUPERVISION  = 0x03
PKT_DEBUG_REPLY  = 0x21

# rcu_telem_payload_t  (rcu_pkt.h) — all little-endian, #pragma pack(push,1)
# PDU FPGA:      6×u8 + u16           =  8 bytes
# PDU ext ADC:   8×i16                = 16 bytes
# PDU SSD:       4×i16                =  8 bytes
# PDU local ADC: 6×i16                = 12 bytes
# IMU0:          7×i16                = 14 bytes
# IMU1:          7×i16                = 14 bytes
# Total: 72 bytes
#
# Format: <6BH 8h 4h 6h 7h 7h
#   6B = fpga_status0, fpga_fc, fpga_sc, fpga_act, fpga_inputs, fpga_version
#   H  = fpga_pchg_ms (uint16 LE)
#   8h = v_vraw..therm2
#   4h = ssd_i, ssd_v, ssd_p, ssd_t
#   6h = ladc_therm0/1/2, ladc_vsource, ladc_vbus, ladc_icoil
#   7h = imu0 (accel[3], gyro[3], temp)
#   7h = imu1
TELEM_FMT  = "<6BH8h4h6h7h7h"
TELEM_SIZE = struct.calcsize(TELEM_FMT)  # 72

# rcu_motor_fb_payload_t
# count u8 + _pad[3] + 16 × (bus u8, motor_id u8, pos u16, vel u16, cur u16, err u8, _pad u8)
FB_HEADER_FMT = "<B3x"
FB_SLOT_FMT   = "<BBHHHBx"  # 10 bytes per slot
FB_SLOTS      = 16
FB_HEADER_SIZE = struct.calcsize(FB_HEADER_FMT)  # 4
FB_SLOT_SIZE   = struct.calcsize(FB_SLOT_FMT)    # 10

# rcu_debug_reply_t
# uptime_ms u32, boot_rsr u32, imu0 u8, imu1 u8, fpga u8, rails u8, ssd u8,
# can_loopback u8, _pad[2], pdu_hb_age_ms u32
DBG_REPLY_FMT  = "<II6Bxx I"
DBG_REPLY_SIZE = struct.calcsize(DBG_REPLY_FMT)  # 20

# RS04 physical limits (must match rs04.h)
RS04_POS_MAX   = 12.57
RS04_VEL_MAX   = 15.0
RS04_TRQ_MAX   = 120.0

# IMU scale factors (imu.h)
IMU_ACCEL_G_PER_LSB  = 0.122e-3   # 0.122 mg/LSB at ±4g
IMU_GYRO_DPS_PER_LSB = 17.5e-3    # 17.5 mdps/LSB at ±500 dps
IMU_TEMP_OFFSET_C    = 25.0
IMU_TEMP_SCALE       = 1.0 / 256.0

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def u16_to_f(raw, lo, hi):
    return lo + (raw / 65535.0) * (hi - lo)

def seq_delta(prev, cur):
    """Signed delta for 0..255 wrap-around sequence numbers."""
    d = (cur - prev) & 0xFF
    return d if d <= 128 else d - 256

class SeqTracker:
    """Tracks a single global sequence counter shared across all packet types.
    The firmware increments g_seq for every packet regardless of type, so
    per-type tracking produces false gap alarms (e.g. telem jumps by 11
    because 10 motor_fb packets were sent in between)."""
    def __init__(self):
        self.last = None
        self.total = 0
        self.gaps = 0

    def update(self, seq):
        self.total += 1
        if self.last is None:
            self.last = seq
            return
        d = seq_delta(self.last, seq)
        if d < 1:
            self.gaps += 1
            print(f"  *** SEQ GAP/REORDER: {self.last} -> {seq} (delta={d}) ***")
        elif d > 1:
            self.gaps += 1
            print(f"  *** SEQ GAP: {self.last} -> {seq} (delta={d}, {d-1} lost) ***")
        self.last = seq

# ---------------------------------------------------------------------------
# Decoders
# ---------------------------------------------------------------------------
def decode_telem(payload, ts):
    if len(payload) < TELEM_SIZE:
        print(f"  [TELEM] short payload {len(payload)} < {TELEM_SIZE}")
        return None

    fields = struct.unpack_from(TELEM_FMT, payload)
    (fpga_sts0, fpga_fc, fpga_sc, fpga_act, fpga_inputs, fpga_version,
     fpga_pchg_ms,
     # ext ADC
     v_vraw_dv, v_12v, v_24v, i_vraw_sw, i_12v, i_24v, therm1_dc, therm2_dc,
     # Energy Meter (SSD)
     em_i, em_v, em_p, em_t,
     # local ADC
     lt0, lt1, lt2, lvsrc, lvbus, licoil,
     # IMU0
     a0x, a0y, a0z, g0x, g0y, g0z, t0,
     # IMU1
     a1x, a1y, a1z, g1x, g1y, g1z, t1) = fields

    return dict(
        ts=ts,
        fpga_sts0=fpga_sts0, fpga_fc=fpga_fc, fpga_sc=fpga_sc, fpga_act=fpga_act,
        fpga_inputs=fpga_inputs, fpga_version=fpga_version, fpga_pchg_ms=fpga_pchg_ms,
        # V_VRAW is in 10mV units — divide by 100 for Volts
        v_vraw_v=v_vraw_dv / 100.0,
        v_12v_mv=v_12v, v_24v_mv=v_24v,
        i_vraw_sw_ma=i_vraw_sw,
        i_12v_ma=i_12v, i_24v_ma=i_24v,
        therm1_c=therm1_dc * 0.1, therm2_c=therm2_dc * 0.1,
        em_i_ma=em_i, em_v_v=em_v / 100.0, em_p_w=em_p * 0.1, em_t_c=em_t * 0.1,
        ladc_therm0_c=lt0 * 0.1, ladc_therm1_c=lt1 * 0.1, ladc_therm2_c=lt2 * 0.1,
        ladc_vsource_v=lvsrc / 100.0, ladc_vbus_v=lvbus / 100.0,
        ladc_icoil_ma=licoil,
        imu0_ax_g=a0x * IMU_ACCEL_G_PER_LSB,
        imu0_ay_g=a0y * IMU_ACCEL_G_PER_LSB,
        imu0_az_g=a0z * IMU_ACCEL_G_PER_LSB,
        imu0_gx_dps=g0x * IMU_GYRO_DPS_PER_LSB,
        imu0_gy_dps=g0y * IMU_GYRO_DPS_PER_LSB,
        imu0_gz_dps=g0z * IMU_GYRO_DPS_PER_LSB,
        imu0_temp_c=t0 * IMU_TEMP_SCALE + IMU_TEMP_OFFSET_C,
        imu1_ax_g=a1x * IMU_ACCEL_G_PER_LSB,
        imu1_ay_g=a1y * IMU_ACCEL_G_PER_LSB,
        imu1_az_g=a1z * IMU_ACCEL_G_PER_LSB,
        imu1_gx_dps=g1x * IMU_GYRO_DPS_PER_LSB,
        imu1_gy_dps=g1y * IMU_GYRO_DPS_PER_LSB,
        imu1_gz_dps=g1z * IMU_GYRO_DPS_PER_LSB,
        imu1_temp_c=t1 * IMU_TEMP_SCALE + IMU_TEMP_OFFSET_C,
    )

# STATUS0 bit definitions (from fpga_mon.c)
_S0_BITS = [(0, "FAULT_LATCH"), (1, "PCHG_LATCH"), (2, "MOTOR_EN"),
            (3, "COMPUTE_EN"), (6, "OVUV_OK"),    (7, "ARM_PERMIT")]
# INPUTS bit definitions
_IN_BITS = [(1, "REMOTE_ARM"), (2, "ESTOP_OK"), (3, "MCU_ARM_SEEN")]
_FPGA_STATES = {0: "IDLE", 1: "PRECHARGE", 2: "ARMED", 3: "COMPUTE"}


def _decode_bits(value, bit_defs):
    active = [name for (bit, name) in bit_defs if (value >> bit) & 1]
    return " | ".join(active) if active else "none"


def print_telem(t, seq):
    print(f"\n[TELEM] seq={seq} ts={t['ts']:.3f}")

    # --- FPGA ---
    state_str = _FPGA_STATES.get(t['fpga_sc'], f"?({t['fpga_sc']})")
    sts_str   = _decode_bits(t['fpga_sts0'],  _S0_BITS)
    inp_str   = _decode_bits(t['fpga_inputs'], _IN_BITS)
    fault_str = f"0x{t['fpga_fc']:02X}" if t['fpga_fc'] else "none"
    act_str   = f"0x{t['fpga_act']:02X}" if t['fpga_act'] else "none"
    print(f"  FPGA      state={state_str}  fault={fault_str}  actions={act_str}")
    print(f"            status0=0x{t['fpga_sts0']:02X}: {sts_str}")
    print(f"            inputs=0x{t['fpga_inputs']:02X}: {inp_str}")
    print(f"            version=0x{t['fpga_version']:02X}  pchg_timer={t['fpga_pchg_ms']}ms")

    # --- Power rails ---
    print(f"  Rails     V_RAW_SW={t['v_vraw_v']:.2f}V  "
          f"12V={t['v_12v_mv']/1000:.3f}V  "
          f"24V={t['v_24v_mv']/1000:.3f}V")
    print(f"  Currents  I_RAW_SW={t['i_vraw_sw_ma']}mA  "
          f"I_12V={t['i_12v_ma']}mA  "
          f"I_24V={t['i_24v_ma']}mA")
    print(f"  Ext Therms  T1={t['therm1_c']:.1f}°C  T2={t['therm2_c']:.1f}°C")

    # --- Energy Meter ---
    print(f"  Energy Meter  {t['em_i_ma']}mA  {t['em_v_v']:.2f}V  "
          f"{t['em_p_w']:.1f}W  {t['em_t_c']:.1f}°C")

    # --- Local STM32 ADC ---
    ladc_zero = (t['ladc_vsource_v'] == 0.0 and t['ladc_vbus_v'] == 0.0
                 and t['ladc_therm0_c'] == 0.0)
    ladc_flag = "  *** all zeros \u2014 PDU needs reflash or local ADC read failing ***" if ladc_zero else ""
    print(f"  Board ADC   T0={t['ladc_therm0_c']:.1f}°C  "
          f"T1={t['ladc_therm1_c']:.1f}°C  "
          f"T2={t['ladc_therm2_c']:.1f}°C  "
          f"V_SRC={t['ladc_vsource_v']:.3f}V  "
          f"V_BUS={t['ladc_vbus_v']:.3f}V  "
          f"I_COIL={t['ladc_icoil_ma']}mA{ladc_flag}")

    # --- IMU ---
    imu0_mag = (t['imu0_ax_g']**2 + t['imu0_ay_g']**2 + t['imu0_az_g']**2) ** 0.5
    imu1_mag = (t['imu1_ax_g']**2 + t['imu1_ay_g']**2 + t['imu1_az_g']**2) ** 0.5
    imu0_flag = "  *** ALL ZEROS \u2014 IMU0 not valid ***" if imu0_mag < 0.01 else ""
    imu1_flag = "  *** ALL ZEROS \u2014 check IMU1 SPI/connection ***" if imu1_mag < 0.01 else ""
    print(f"  IMU0 accel ax={t['imu0_ax_g']:+.3f}g ay={t['imu0_ay_g']:+.3f}g "
          f"az={t['imu0_az_g']:+.3f}g  |a|={imu0_mag:.3f}g  "
          f"T={t['imu0_temp_c']:.1f}°C{imu0_flag}")
    print(f"  IMU0 gyro  gx={t['imu0_gx_dps']:+.1f} gy={t['imu0_gy_dps']:+.1f} "
          f"gz={t['imu0_gz_dps']:+.1f} dps")
    print(f"  IMU1 accel ax={t['imu1_ax_g']:+.3f}g ay={t['imu1_ay_g']:+.3f}g "
          f"az={t['imu1_az_g']:+.3f}g  |a|={imu1_mag:.3f}g  "
          f"T={t['imu1_temp_c']:.1f}°C{imu1_flag}")
    print(f"  IMU1 gyro  gx={t['imu1_gx_dps']:+.1f} gy={t['imu1_gy_dps']:+.1f} "
          f"gz={t['imu1_gz_dps']:+.1f} dps")

def decode_motor_fb(payload):
    if len(payload) < FB_HEADER_SIZE:
        return None, []
    (count,) = struct.unpack_from("<B", payload, 0)
    slots = []
    off = FB_HEADER_SIZE
    for _ in range(min(count, FB_SLOTS)):
        if off + FB_SLOT_SIZE > len(payload):
            break
        bus, motor_id, pos_u16, vel_u16, cur_u16, err = struct.unpack_from(FB_SLOT_FMT, payload, off)
        off += FB_SLOT_SIZE
        slots.append(dict(
            bus=bus,
            motor_id=motor_id,
            pos_rad=u16_to_f(pos_u16, -RS04_POS_MAX, RS04_POS_MAX),
            vel_rads=u16_to_f(vel_u16, -RS04_VEL_MAX, RS04_VEL_MAX),
            torque_nm=u16_to_f(cur_u16, -RS04_TRQ_MAX, RS04_TRQ_MAX),
            error=err,
        ))
    return count, slots

def print_motor_fb(count, slots, seq):
    bus_name = {0: "R", 1: "L"}
    print(f"[MOTOR_FB] seq={seq} count={count}")
    for s in slots:
        print(f"  bus={bus_name.get(s['bus'],s['bus'])} "
              f"id={s['motor_id']:2d}  "
              f"pos={s['pos_rad']:+7.3f}rad  "
              f"vel={s['vel_rads']:+6.2f}rad/s  "
              f"trq={s['torque_nm']:+7.2f}Nm  "
              f"err=0x{s['error']:02X}")

def decode_debug_reply(payload):
    if len(payload) < DBG_REPLY_SIZE:
        print(f"  [DEBUG_REPLY] short {len(payload)} < {DBG_REPLY_SIZE}")
        return
    (uptime, boot_rsr,
     imu0_v, imu1_v, fpga_v, rails_v, ssd_v, can_lb,
     hb_age) = struct.unpack_from(DBG_REPLY_FMT, payload)

    reset_causes = []
    if boot_rsr & (1 << 26): reset_causes.append("PINRST(supervisor/button)")
    if boot_rsr & (1 << 24): reset_causes.append("SW-RESET(fault)")
    if boot_rsr & (1 << 21): reset_causes.append("BOR(brownout)")
    if boot_rsr & (1 << 26): reset_causes.append("IWDG1")
    if boot_rsr & (1 << 28): reset_causes.append("WWDG1")
    rsr_str = ", ".join(reset_causes) if reset_causes else "clean"

    can_str = {0: "untested", 1: "RIGHT OK", 2: "LEFT OK",
               3: "BOTH OK", 0xFF: "FAIL"}.get(can_lb, f"0x{can_lb:02X}")

    hb_str = f"{hb_age}ms ago" if hb_age != 0xFFFFFFFF else "NEVER"

    print(f"\n[DEBUG_REPLY]")
    print(f"  uptime     {uptime}ms  ({uptime/1000:.1f}s)")
    print(f"  boot_rsr   0x{boot_rsr:08X}  → {rsr_str}")
    print(f"  IMU0       {'OK' if imu0_v else 'NOT VALID'}")
    print(f"  IMU1       {'OK' if imu1_v else 'NOT VALID'}")
    print(f"  PDU FPGA   {'OK' if fpga_v else 'not received'}")
    print(f"  PDU rails  {'OK' if rails_v else 'not received'}")
    print(f"  PDU Energy Meter  {'OK (0x523 received)' if ssd_v else 'not received'}")
    print(f"  PDU HB     {hb_str}")
    print(f"  CAN loop   {can_str}")

# ---------------------------------------------------------------------------
# CSV writer
# ---------------------------------------------------------------------------
CSV_TELEM_FIELDS = [
    "timestamp", "seq",
    "fpga_sts0", "fpga_fc", "fpga_sc", "fpga_act", "fpga_inputs", "fpga_version",
    "fpga_pchg_ms",
    "v_vraw_v", "v_12v_mv", "v_24v_mv", "i_vraw_sw_ma", "i_12v_ma", "i_24v_ma",
    "therm1_c", "therm2_c",
    "em_i_ma", "em_v_v", "em_p_w", "em_t_c",
    "ladc_therm0_c", "ladc_therm1_c", "ladc_therm2_c",
    "ladc_vsource_v", "ladc_vbus_v", "ladc_icoil_ma",
    "imu0_ax_g", "imu0_ay_g", "imu0_az_g",
    "imu0_gx_dps", "imu0_gy_dps", "imu0_gz_dps", "imu0_temp_c",
    "imu1_ax_g", "imu1_ay_g", "imu1_az_g",
    "imu1_gx_dps", "imu1_gy_dps", "imu1_gz_dps", "imu1_temp_c",
]

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="RCU telemetry monitor")
    ap.add_argument("--host", default="0.0.0.0", help="Bind address (default 0.0.0.0)")
    ap.add_argument("--port", type=int, default=7700, help="Listen port (default 7700)")
    ap.add_argument("--log", metavar="FILE", help="Also write telem CSV to FILE")
    ap.add_argument("--quiet-fb", action="store_true",
                    help="Suppress motor feedback packets (reduces spam at 100Hz)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(2.0)
    print(f"Listening on {args.host}:{args.port}  (Ctrl-C to stop)")
    print(f"Motor FB display: {'suppressed (use without --quiet-fb to see)' if args.quiet_fb else 'shown (tip: use --quiet-fb to reduce spam)'}")
    if args.log:
        print(f"Logging telem to: {args.log}")

    csv_file = None
    csv_writer = None
    if args.log:
        csv_file = open(args.log, "w", newline="")
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_TELEM_FIELDS, extrasaction="ignore")
        csv_writer.writeheader()

    seq_tracker = SeqTracker()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue

            if len(data) < HDR_SIZE:
                continue

            magic, pkt_type, seq, payload_len = struct.unpack_from(HDR_FMT, data, 0)
            if magic != PKT_MAGIC:
                print(f"  [WARN] bad magic 0x{magic:04X} from {addr}")
                continue

            payload = data[HDR_SIZE: HDR_SIZE + payload_len]
            if len(payload) < payload_len:
                print(f"  [WARN] truncated: got {len(payload)} expected {payload_len}")
                continue

            seq_tracker.update(seq)
            ts = time.time()

            if pkt_type == PKT_SLOW_TELEM:
                t = decode_telem(payload, ts)
                if t:
                    print_telem(t, seq)
                    if csv_writer:
                        row = dict(t)
                        row["timestamp"] = datetime.fromtimestamp(ts).isoformat()
                        row["seq"] = seq
                        csv_writer.writerow(row)
                        csv_file.flush()

            elif pkt_type == PKT_MOTOR_FB:
                count, slots = decode_motor_fb(payload)
                if not args.quiet_fb:
                    print_motor_fb(count, slots, seq)

            elif pkt_type == PKT_DEBUG_REPLY:
                decode_debug_reply(payload)

            elif pkt_type == PKT_SUPERVISION:
                print(f"[SUPERVISION] seq={seq} len={payload_len} "
                      f"data={payload.hex()}")

            else:
                print(f"[UNKNOWN] type=0x{pkt_type:02X} seq={seq} len={payload_len}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if csv_file:
            csv_file.close()
        sock.close()

    print("\n--- Sequence statistics ---")
    print(f"  total packets={seq_tracker.total}  gaps/reorders={seq_tracker.gaps}")

if __name__ == "__main__":
    main()
