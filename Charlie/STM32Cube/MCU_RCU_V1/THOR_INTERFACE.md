# RCU UDP Interface — Thor Integration Notes

## Network Configuration

| Parameter | Value |
|---|---|
| RCU static IP | `192.168.100.10` |
| Subnet mask | `255.255.255.0` |
| Default gateway | `192.168.100.1` |
| **Thor IP (required)** | `192.168.100.20` ← update `THOR_IP_D` in `eth_udp.c` once confirmed |
| Telem TX port (RCU→Thor) | **7700** |
| Cmd RX port (Thor→RCU) | **7701** |
| Supervision TX port (RCU→Thor) | **7702** |

---

## Packet Format

All packets share a 6-byte header, followed by a fixed payload. All
multi-byte fields are **little-endian** unless noted.

```
Offset  Size  Field
0       2     Magic = 0x5243  ('RC')
2       1     Type  (see below)
3       1     Seq   (rolling 0–255)
4       2     Len   (payload bytes, LE uint16)
6       N     Payload
```

---

## Type 0x01 — Slow Telemetry (RCU → Thor, 10 Hz)

Payload: `rcu_telem_payload_t` (see `rcu_pkt.h`)

| Field | Type | Unit | Notes |
|---|---|---|---|
| `fpga_status0` | uint8 | — | FPGA STATUS0 register |
| `fpga_fault_code` | uint8 | — | FPGA fault register |
| `fpga_state_code` | uint8 | — | 0=IDLE 1=PCHG 2=ARMED 3=COMPUTE |
| `fpga_actions` | uint8 | — | FPGA action register |
| `v_vraw_mv` | int16 | mV | Raw battery voltage |
| `v_12v_mv` | int16 | mV | 12 V rail |
| `v_24v_mv` | int16 | mV | 24 V rail |
| `i_vraw_ma` | int16 | mA | Battery current |
| `i_12v_ma` | int16 | mA | 12 V current |
| `i_24v_ma` | int16 | mA | 24 V current |
| `therm1_dc` | int16 | 0.1 °C | PDU thermistor 1 |
| `therm2_dc` | int16 | 0.1 °C | PDU thermistor 2 |
| `ssd_i_ma` | int16 | mA | SSD energy meter current |
| `ssd_v_mv` | int16 | mV | SSD energy meter voltage |
| `ssd_t_dc` | int16 | 0.1 °C | SSD energy meter temperature |
| `imu0_accel[3]` | int16×3 | LSB | 0.122 mg/LSB at ±4g |
| `imu0_gyro[3]` | int16×3 | LSB | 17.5 mdps/LSB at ±500 dps |
| `imu0_temp` | int16 | — | Raw (°C = raw/256 + 25) |
| `imu1_accel[3]` | int16×3 | LSB | Same scaling as IMU0 |
| `imu1_gyro[3]` | int16×3 | LSB | Same scaling as IMU0 |
| `imu1_temp` | int16 | — | Same scaling as IMU0 |

---

## Type 0x02 — Motor Feedback (RCU → Thor, ~100 Hz)

Payload: `rcu_motor_fb_payload_t` (see `rcu_pkt.h`)

```
uint8   count        — number of valid slots (0..16)
uint8   _pad[3]
slot[0..count-1]:
  uint8   bus        — 0=right, 1=left
  uint8   motor_id   — 1..8
  uint16  pos_u16    — encoded: 0..65535 → -12.57..+12.57 rad
  uint16  vel_u16    — encoded: 0..65535 → -15..+15 rad/s
  uint16  cur_u16    — encoded: 0..65535 → -120..+120 N·m
  uint8   error_code — RS04 fault_bits from feedback frame
  uint8   _pad
```

**Decode formula** for pos/vel/torque:
```
value = min + (raw / 65535.0) * (max - min)
```

---

## Type 0x10 — Motor Command (Thor → RCU)

Payload: array of `rcu_motor_cmd_entry_t` entries.
`count = payload_len / sizeof(rcu_motor_cmd_entry_t)` (10 bytes each).

```
uint8   bus        — 0=right, 1=left
uint8   motor_id   — 1..8
uint16  pos_u16    — encoded position  (-12.57..+12.57 rad)
uint16  vel_u16    — encoded velocity  (-15..+15 rad/s)
uint16  trq_u16    — encoded torque ff (-120..+120 N·m)
uint8   kp_u8      — encoded Kp: (raw/255) * 5000
uint8   kd_u8      — encoded Kd: (raw/255) * 100
```

Motors default to **operation control mode (Type 1, RS04 private protocol)**.
Before sending motion commands, the motor must be enabled via a separate
mechanism (currently: `motor_bus_send_enable(bus, id, true, false)` called
from RCU startup or a future supervisory packet).

---

## Motor Coordinate / Scale Reference (RS04 private protocol)

| Quantity | Range | 16-bit span |
|---|---|---|
| Position | ±12.57 rad (±4π) | 0..65535 |
| Velocity | ±15 rad/s | 0..65535 |
| Torque | ±120 N·m | 0..65535 |
| Kp | 0..5000 | in command only |
| Kd | 0..100 | in command only |

---

## Bench Bring-Up Without Thor (Windows 11 PC)

Set your PC NIC to static IP `192.168.100.20/24`.  Use **Wireshark** or a
simple Python script to receive UDP on port 7700.

Minimal Python listener:
```python
import socket, struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 7700))

while True:
    data, addr = sock.recvfrom(2048)
    magic, ptype, seq, length = struct.unpack_from("<HBBh", data, 0)
    if magic != 0x5243:
        continue
    print(f"type=0x{ptype:02X} seq={seq} len={length} from {addr}")
    # For slow telem (0x01), first int16 LE at offset 6 = fpga_status0 + fault packed
    # Use struct.unpack to parse rcu_telem_payload_t fields
```

To send a motor command from the PC:
```python
import socket, struct

HDR = struct.pack("<HBBh", 0x5243, 0x10, 0, 10)  # 1 entry = 10 bytes
# bus=0 right, motor_id=1, pos=0, vel=0, trq=0 (mid-scale=32767), kp=0, kd=0
ENTRY = struct.pack("<BBHHHBBxx", 0, 1, 32767, 32767, 32767, 0, 0)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(HDR + ENTRY, ("192.168.100.10", 7701))
```

> **Note:** mid-scale (32767) on pos/vel = 0 rad / 0 rad/s.
> mid-scale on torque = 0 N·m feedforward.

---

## Interface Status

All integration items below are resolved as of handover.

- **Thor static IP** confirmed as `192.168.100.20`. `THOR_IP_D` in `eth_udp.c` is set accordingly.
- **Motor enable/disable supervisory command** is implemented as `DBGCMD_MOTOR_ENABLE` (0x0C). See §Debug Commands above.
- **Motor feedback strategy** settled as command-response: the RCU caches the last RS04 Type-2 reply per motor and re-transmits at 200 Hz via the fast telemetry loop. Active RS04 Type-24 reporting is not required.
