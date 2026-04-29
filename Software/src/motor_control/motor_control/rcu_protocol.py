"""
rcu_protocol.py — RCU UDP binary protocol helpers
Shared by rcu_motor_test.py and Tools/ROS2/rcu_udp_bridge.py

All packets: magic=0x5243 ('RC' LE), type(u8), seq(u8), len(u16) = 6-byte header.
Little-endian throughout.  Ports: 7700 (RCU→PC), 7701 (PC→RCU).
RCU IP: 192.168.100.10   Thor/PC IP: 192.168.100.20
"""
import struct
import math

# ---------------------------------------------------------------------------
# Network constants
# ---------------------------------------------------------------------------
RCU_IP        = "192.168.100.10"
PC_IP         = "192.168.100.20"
PORT_TELEM    = 7700   # RCU → PC  (slow telem, motor FB, fast IMU)
PORT_CMD      = 7701   # PC  → RCU (motor cmd, motor supv, debug cmd)
PORT_SUPV_OUT = 7702   # RCU → PC  (supervision events)

# ---------------------------------------------------------------------------
# Packet type constants
# ---------------------------------------------------------------------------
PKT_SLOW_TELEM  = 0x01
PKT_MOTOR_FB    = 0x02
PKT_SUPERVISION = 0x03
PKT_IMU_FAST    = 0x04
PKT_MOTOR_CMD   = 0x10
PKT_MOTOR_SUPV  = 0x11
PKT_DEBUG_CMD   = 0x20
PKT_DEBUG_REPLY = 0x21

PKT_MAGIC    = 0x5243
HDR_FMT      = "<HBBh"   # magic, type, seq, len — NOTE: len is int16 but always +ve
HDR_SIZE     = 6

# Debug sub-commands
DBGCMD_PING             = 0x01
DBGCMD_BUZZ             = 0x02
DBGCMD_LED_BLINK        = 0x03
DBGCMD_CAN_LOOPBACK     = 0x04
DBGCMD_FORCE_TELEM      = 0x05
DBGCMD_ASSERT_PDU_FAULT = 0x07
DBGCMD_SOFT_RESET       = 0x08
DBGCMD_SET_TELEM_RATE   = 0x09
DBGCMD_MOTOR_BUS_CTRL   = 0x0A
DBGCMD_REQUEST_SUPV_DUMP= 0x0B

# ---------------------------------------------------------------------------
# RS04 physical limits (mirror of rs04.h)
# ---------------------------------------------------------------------------
RS04_POS_MAX_RAD   = 12.57
RS04_VEL_MAX_RADS  = 15.0
RS04_TRQ_MAX_NM    = 120.0
RS04_KP_MAX        = 5000.0
RS04_KD_MAX        = 100.0

# ---------------------------------------------------------------------------
# Motor joint names (index = motor_id 1–12; index 0 = unused)
# ---------------------------------------------------------------------------
MOTOR_JOINT_NAMES = [
    "unused",                           # [0]
    "pelvis_link_l_yaw_joint",          # [1]  LEFT bus
    "pelvis_link_r_yaw_joint",          # [2]  RIGHT bus
    "l_hip_yaw_link_l_pitch_joint",     # [3]  LEFT
    "r_hip_yaw_link_r_pitch_joint",     # [4]  RIGHT
    "l_hip_pitch_link_l_roll_joint",    # [5]  LEFT
    "r_hip_pitch_link_r_roll_joint",    # [6]  RIGHT
    "l_thigh_link_l_knee_joint",        # [7]  LEFT
    "r_thigh_link_r_knee_joint",        # [8]  RIGHT
    "l_shank_link_l_ankle_joint",       # [9]  LEFT
    "r_shank_link_r_ankle_joint",       # [10] RIGHT
    "l_ankle_link_l_foot_joint",        # [11] LEFT
    "r_ankle_link_r_foot_joint",        # [12] RIGHT
]

# motor_id → bus (0=RIGHT, 1=LEFT); matches firmware MOTOR_BUS_MAP
MOTOR_BUS_MAP = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0]  # index=motor_id

# joint name → motor_id lookup
JOINT_TO_MOTOR_ID = {name: idx for idx, name in enumerate(MOTOR_JOINT_NAMES) if idx > 0}

# ---------------------------------------------------------------------------
# Header helpers
# ---------------------------------------------------------------------------
_seq = 0

def _next_seq():
    global _seq
    s = _seq & 0xFF
    _seq = (_seq + 1) & 0xFF
    return s

def build_packet(pkt_type: int, payload: bytes) -> bytes:
    """Build a complete RCU UDP packet (header + payload)."""
    hdr = struct.pack("<HBBh", PKT_MAGIC, pkt_type, _next_seq(), len(payload))
    return hdr + payload

def parse_header(data: bytes):
    """Parse 6-byte header.  Returns (type, seq, payload_len) or None if invalid."""
    if len(data) < HDR_SIZE:
        return None
    magic, pkt_type, seq, plen = struct.unpack_from("<HBBh", data, 0)
    if magic != PKT_MAGIC:
        return None
    return pkt_type, seq, plen

# ---------------------------------------------------------------------------
# Encode helpers
# ---------------------------------------------------------------------------
def _f_to_u16(v, vmin, vmax):
    v = max(vmin, min(vmax, v))
    return int((v - vmin) / (vmax - vmin) * 65535 + 0.5) & 0xFFFF

def _u16_to_f(raw, vmin, vmax):
    return vmin + (raw / 65535.0) * (vmax - vmin)

def encode_motor_cmd_entry(motor_id: int, bus: int, pos_rad: float,
                           vel_rads: float = 0.0, torque_nm: float = 0.0,
                           kp: float = 0.0, kd: float = 0.0) -> bytes:
    """Encode one rcu_motor_cmd_entry_t (10 bytes, little-endian)."""
    pos = _f_to_u16(pos_rad,   -RS04_POS_MAX_RAD,  RS04_POS_MAX_RAD)
    vel = _f_to_u16(vel_rads,  -RS04_VEL_MAX_RADS, RS04_VEL_MAX_RADS)
    trq = _f_to_u16(torque_nm, -RS04_TRQ_MAX_NM,   RS04_TRQ_MAX_NM)
    kp8 = int(kp / RS04_KP_MAX * 255) & 0xFF
    kd8 = int(kd / RS04_KD_MAX * 255) & 0xFF
    return struct.pack("<BBHHHBB", bus, motor_id, pos, vel, trq, kp8, kd8)

def encode_motor_supervisory(enable_mask: int = 0, clear_fault_mask: int = 0,
                             ctrl_mode: int = 0) -> bytes:
    """Build a Type 0x11 motor supervisory packet (header + 8-byte payload)."""
    payload = struct.pack("<HHBxxx", enable_mask & 0xFFFF,
                         clear_fault_mask & 0xFFFF, ctrl_mode & 0xFF)
    return build_packet(PKT_MOTOR_SUPV, payload)

def encode_debug_cmd(subcmd: int, extra: bytes = b"") -> bytes:
    """Build a Type 0x20 debug command packet."""
    return build_packet(PKT_DEBUG_CMD, bytes([subcmd]) + extra)

def encode_motor_cmd_packet(entries: list) -> bytes:
    """
    Build a Type 0x10 motor command packet.
    entries: list of dicts with keys: motor_id, pos_rad, vel_rads, torque_nm, kp, kd.
    Bus is derived from MOTOR_BUS_MAP.
    """
    payload = b""
    for e in entries:
        mid = e["motor_id"]
        bus = MOTOR_BUS_MAP[mid] if 1 <= mid <= 12 else 0
        payload += encode_motor_cmd_entry(
            mid, bus,
            e.get("pos_rad", 0.0),
            e.get("vel_rads", 0.0),
            e.get("torque_nm", 0.0),
            e.get("kp", 0.0),
            e.get("kd", 0.0),
        )
    return build_packet(PKT_MOTOR_CMD, payload)

# ---------------------------------------------------------------------------
# Decode helpers
# ---------------------------------------------------------------------------

# Slow telem format (72 bytes payload after header)
# "<6BH8h4h6h7h7h" — see rcu_pkt.h rcu_telem_payload_t
TELEM_FMT   = "<6BH8h4h6h7h7h"
TELEM_NAMES = [
    "fpga_status0", "fpga_fault_code", "fpga_state_code",
    "fpga_actions", "fpga_inputs", "fpga_version",
    "fpga_pchg_ms",
    "v_vraw_dv", "v_12v_mv", "v_24v_mv",
    "i_vraw_sw_ma", "i_12v_ma", "i_24v_ma",
    "therm1_dc", "therm2_dc",
    "ssd_i_ma", "ssd_v_dv", "ssd_p_dw", "ssd_t_dc",
    "ladc_therm0_dc", "ladc_therm1_dc", "ladc_therm2_dc",
    "ladc_vsource_mv", "ladc_vbus_mv", "ladc_icoil_ma",
    "imu0_accel_x", "imu0_accel_y", "imu0_accel_z",
    "imu0_gyro_x",  "imu0_gyro_y",  "imu0_gyro_z", "imu0_temp",
    "imu1_accel_x", "imu1_accel_y", "imu1_accel_z",
    "imu1_gyro_x",  "imu1_gyro_y",  "imu1_gyro_z", "imu1_temp",
]
# Scale factors to SI units
TELEM_SCALE = {
    "v_vraw_dv":     0.01,   # → V
    "v_12v_mv":      0.001,  # → V
    "v_24v_mv":      0.001,  # → V
    "i_vraw_sw_ma":  0.001,  # → A
    "i_12v_ma":      0.001,  # → A
    "i_24v_ma":      0.001,  # → A
    "therm1_dc":     0.1,    # → °C
    "therm2_dc":     0.1,    # → °C
    "ssd_i_ma":      0.001,  # → A
    "ssd_v_dv":      0.01,   # → V
    "ssd_p_dw":      0.1,    # → W
    "ssd_t_dc":      0.1,    # → °C
    "ladc_therm0_dc":0.1,    "ladc_therm1_dc":0.1, "ladc_therm2_dc":0.1,
    "ladc_vsource_mv":0.001, "ladc_vbus_mv":0.001, "ladc_icoil_ma":0.001,
    "imu0_accel_x":  0.000122, "imu0_accel_y": 0.000122, "imu0_accel_z": 0.000122,
    "imu0_gyro_x":   0.0175,   "imu0_gyro_y":  0.0175,   "imu0_gyro_z":  0.0175,
    "imu0_temp":     1/256,
    "imu1_accel_x":  0.000122, "imu1_accel_y": 0.000122, "imu1_accel_z": 0.000122,
    "imu1_gyro_x":   0.0175,   "imu1_gyro_y":  0.0175,   "imu1_gyro_z":  0.0175,
    "imu1_temp":     1/256,
}

def decode_slow_telem(payload: bytes) -> dict:
    """Decode Type 0x01 payload → dict of all 30 named fields (raw and scaled)."""
    if len(payload) < struct.calcsize(TELEM_FMT):
        return {}
    values = struct.unpack_from(TELEM_FMT, payload)
    raw = dict(zip(TELEM_NAMES, values))
    scaled = {}
    for k, v in raw.items():
        scaled[k] = v * TELEM_SCALE.get(k, 1)
    scaled["_raw"] = raw
    return scaled


# Motor feedback slot: bus(u8), motor_id(u8), pos(u16), vel(u16), cur(u16), err(u8), _pad(u8)
FB_SLOT_FMT  = "<BBHHHBx"
FB_SLOT_SIZE = struct.calcsize(FB_SLOT_FMT)   # 10 bytes

def decode_motor_fb(payload: bytes) -> list:
    """Decode Type 0x02 payload → list of dicts per motor slot."""
    if len(payload) < 4:
        return []
    count = payload[0]
    slots = []
    offset = 4  # skip count + 3 pad bytes
    for _ in range(count):
        if offset + FB_SLOT_SIZE > len(payload):
            break
        bus, mid, p16, v16, c16, err, *_ = struct.unpack_from(FB_SLOT_FMT, payload, offset)
        slots.append({
            "bus":       bus,
            "motor_id":  mid,
            "joint":     MOTOR_JOINT_NAMES[mid] if 1 <= mid <= 12 else "unknown",
            "pos_rad":   _u16_to_f(p16, -RS04_POS_MAX_RAD,  RS04_POS_MAX_RAD),
            "vel_rads":  _u16_to_f(v16, -RS04_VEL_MAX_RADS, RS04_VEL_MAX_RADS),
            "torque_nm": _u16_to_f(c16, -RS04_TRQ_MAX_NM,   RS04_TRQ_MAX_NM),
            "fault":     err,
        })
        offset += FB_SLOT_SIZE
    return slots


# Fast IMU packet: imu0_accel[3], imu0_gyro[3], imu1_accel[3], imu1_gyro[3], tick_ms (28 B)
IMU_FAST_FMT  = "<6h6hI"   # 6 int16 + 6 int16 + uint32 = 28 bytes
IMU_ACCEL_SCALE = 0.122e-3  # g/LSB  → multiply by 9.81 for m/s²
IMU_GYRO_SCALE  = 17.5e-3   # dps/LSB → multiply by π/180 for rad/s

def decode_imu_fast(payload: bytes) -> dict:
    """Decode Type 0x04 fast IMU payload → dict."""
    if len(payload) < struct.calcsize(IMU_FAST_FMT):
        return {}
    v = struct.unpack_from(IMU_FAST_FMT, payload)
    # v[0:3]  = imu0_accel, v[3:6] = imu0_gyro
    # v[6:9]  = imu1_accel, v[9:12]= imu1_gyro
    # v[12]   = tick_ms
    return {
        "imu0_accel_g":    [v[i]  * IMU_ACCEL_SCALE for i in range(3)],
        "imu0_gyro_dps":   [v[i]  * IMU_GYRO_SCALE  for i in range(3, 6)],
        "imu1_accel_g":    [v[i]  * IMU_ACCEL_SCALE for i in range(6, 9)],
        "imu1_gyro_dps":   [v[i]  * IMU_GYRO_SCALE  for i in range(9, 12)],
        "tick_ms":          v[12],
    }


def decode_debug_reply(payload: bytes) -> dict:
    """Decode Type 0x21 debug reply payload."""
    FMT = "<II4BH2xI"   # uptime_ms, boot_rsr, 4×uint8, pad×2, pdu_hb_age_ms
    # Actual struct: uptime_ms(4) boot_rsr(4) imu0(1) imu1(1) fpga(1) rails(1) ssd(1) can_lb(1) pad(2) hb_age(4)
    FMT2 = "<II6BxxI"
    if len(payload) < struct.calcsize(FMT2):
        return {}
    up, rsr, i0, i1, fpga, rails, ssd, can_lb, age = struct.unpack_from(FMT2, payload)
    return {
        "uptime_ms":       up,
        "boot_rsr":        rsr,
        "imu0_valid":      bool(i0),
        "imu1_valid":      bool(i1),
        "pdu_fpga_valid":  bool(fpga),
        "pdu_rails_valid": bool(rails),
        "pdu_ssd_valid":   bool(ssd),
        "can_loopback":    can_lb,
        "pdu_hb_age_ms":   age,
    }
