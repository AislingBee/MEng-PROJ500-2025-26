# PROJ500 Humanoid — RCU Motor Control Handover Instructions

**Document version:** 1.1  
**Date:** 2026-05-02  
**Author:** MCU team (Charlie)  
**Intended audience:** Thor ROS2 team  
**Purpose:** Complete, self-contained handover for integrating the RCU motor
control interface into the Thor ROS2 stack. Feed this document into an AI
assistant to generate missing code, or follow it step-by-step.

---

## 1. System Overview

The **RCU** (Robot Control Unit) is an STM32H723-based microcontroller that:
- Drives **12 RS04 motors** over two 1 Mbps CAN buses (FDCAN1=right, FDCAN3=left)
- Receives **UDP binary packets** from Thor on port 7701 (motor commands + supervisory)
- Transmits **UDP binary packets** to Thor on port 7700 (motor feedback, fast IMU, slow telem)
- Monitors the **PDU** (Power Distribution Unit) over a separate CAN bus (FDCAN2)

The **old** `ethernet_can_bridge.py` + `nucleo_can_bridge` serial bridge is **replaced** by
`rcu_udp_bridge.py` which speaks the RCU binary protocol directly.

---

## 2. Network Setup

| Device | Interface | Static IP | Notes |
|---|---|---|---|
| RCU (STM32H723) | Ethernet (RMII) | `192.168.100.10` | configured in CubeMX lwIP |
| Thor (Jetson/PC) | eth0 or enp... | `192.168.100.20` | must be on same subnet |

**Thor static IP setup (Ubuntu/ROS2):**
```bash
sudo ip addr add 192.168.100.20/24 dev eth0
sudo ip link set eth0 up
# Or persist via /etc/netplan or nmcli
```

**Ports:**
| Port | Direction | Content |
|---|---|---|
| 7700 | RCU → Thor | Slow telem (0x01), Motor FB (0x02), Fast IMU (0x04) |
| 7701 | Thor → RCU | Motor cmd (0x10), Motor supervisory (0x11), Debug cmd (0x20) |
| 7702 | RCU → Thor | Supervision events (0x03) |

**Firewall (if UFW is active):**
```bash
sudo ufw allow 7700/udp
sudo ufw allow 7701/udp
sudo ufw allow 7702/udp
```

---

## 3. Full Binary Protocol Reference

### 3.1 Packet Header (6 bytes, all packets)

```
Offset  Size  Field   Value
0       2     magic   0x5243 ('RC', little-endian)
2       1     type    packet type (see below)
3       1     seq     rolling counter 0–255
4       2     len     payload length in bytes (little-endian uint16)
```
Total frame size = 6 + len bytes. All multi-byte fields are **little-endian**.

### 3.2 Type 0x01 — Slow Telemetry (RCU → Thor, 10 Hz)

Payload: **72 bytes**. Format string: `"<6BH8h4h6h7h7h"`

| Offset | Type | Field | Units / Notes |
|---|---|---|---|
| 0 | u8 | fpga_status0 | FPGA status register 0 |
| 1 | u8 | fpga_fault_code | FPGA fault code |
| 2 | u8 | fpga_state_code | FPGA state machine code |
| 3 | u8 | fpga_actions | FPGA output actions |
| 4 | u8 | fpga_inputs | FPGA register 0x04 (e-stop/arm flags) |
| 5 | u8 | fpga_version | FPGA version register 0x7F |
| 6 | u16 | fpga_pchg_ms | Precharge timer [ms] |
| 8 | i16 | v_vraw_dv | VRAW voltage [10mV] → ÷100 for V |
| 10 | i16 | v_12v_mv | 12V rail [mV] → ÷1000 for V |
| 12 | i16 | v_24v_mv | 24V rail [mV] → ÷1000 for V |
| 14 | i16 | i_vraw_sw_ma | Switched VRAW current [mA] |
| 16 | i16 | i_12v_ma | 12V rail current [mA] |
| 18 | i16 | i_24v_ma | 24V rail current [mA] |
| 20 | i16 | therm1_dc | External thermistor 1 [0.1°C] |
| 22 | i16 | therm2_dc | External thermistor 2 [0.1°C] |
| 24 | i16 | ssd_i_ma | SSD energy meter current [mA] |
| 26 | i16 | ssd_v_dv | SSD voltage [10mV] |
| 28 | i16 | ssd_p_dw | SSD power [0.1W] |
| 30 | i16 | ssd_t_dc | SSD temperature [0.1°C] |
| 32 | i16 | ladc_therm0_dc | Board thermistor 0 [0.1°C] |
| 34 | i16 | ladc_therm1_dc | Board thermistor 1 [0.1°C] |
| 36 | i16 | ladc_therm2_dc | Board thermistor 2 [0.1°C] |
| 38 | i16 | ladc_vsource_mv | V_SOURCE [mV] |
| 40 | i16 | ladc_vbus_mv | V_BUS [mV] |
| 42 | i16 | ladc_icoil_ma | I_COIL [mA] |
| 44 | i16×3 | imu0_accel[xyz] | IMU0 accel raw — ×0.000122 → g |
| 50 | i16×3 | imu0_gyro[xyz] | IMU0 gyro raw — ×0.0175 → °/s |
| 56 | i16 | imu0_temp | IMU0 temp — ÷256 + 25 → °C |
| 58 | i16×3 | imu1_accel[xyz] | IMU1 accel raw |
| 64 | i16×3 | imu1_gyro[xyz] | IMU1 gyro raw |
| 70 | i16 | imu1_temp | IMU1 temp |

**Note:** The slow telem is for monitoring/logging only. Use Type 0x04 for the
RL control loop IMU (see §3.4).

### 3.3 Type 0x02 — Motor Feedback (RCU → Thor, 200 Hz)

Payload: **4 + n×10 bytes** (n ≤ 16 slots).

Header (4 bytes): `count(u8), pad[3]`

Per slot (10 bytes): `bus(u8), motor_id(u8), pos_u16, vel_u16, cur_u16, error(u8), mode_status(u8)`

`mode_status` is bits [23:22] of the RS04 Type-2 CAN ID: **0=idle/reset, 1=calibrating, 2=MIT running**.
A motor that has been enabled and is accepting MIT commands will show `mode_status=2`.

Decoding pos/vel/torque from uint16:
```python
pos_rad   = -12.57 + (pos_u16 / 65535.0) * (12.57 - (-12.57))
vel_rads  = -15.0  + (vel_u16 / 65535.0) * 30.0
torque_nm = -120.0 + (cur_u16 / 65535.0) * 240.0
```

The `/motor_can_feedback` ROS2 topic uses **8 bytes/motor** (float32 pos + float32 vel, LE),
ordered motor_id 1→12, totalling 96 bytes. This is consumed by `robot_observation_bridge.py`.

### 3.4 Type 0x04 — Fast IMU Packet (RCU → Thor, 200 Hz)

Payload: **28 bytes**. Format: `"<6h6hI"` (6×int16 + 6×int16 + uint32)

| Offset | Type | Field | Scale |
|---|---|---|---|
| 0 | i16×3 | imu0_accel[xyz] | ×0.000122 → g; ×(0.000122×9.81) → m/s² |
| 6 | i16×3 | imu0_gyro[xyz] | ×0.0175 → °/s; ×(0.0175×π/180) → rad/s |
| 12 | i16×3 | imu1_accel[xyz] | same as imu0 |
| 18 | i16×3 | imu1_gyro[xyz] | same as imu0 |
| 24 | u32 | tick_ms | HAL_GetTick() at RCU at time of packing |

Published to `/imu0` and `/imu1` as `sensor_msgs/Imu`.  
**This is the primary IMU source for the RL observation — not the slow telem.**

### 3.5 Type 0x10 — Motor Command (Thor → RCU)

Payload: **variable** (n × 12 bytes, one entry per motor).

Per entry (12 bytes):
```
bus(u8), motor_id(u8), pos_u16, vel_u16, trq_u16, kp_u16, kd_u16
```

> **Breaking change (2026-05-02):** `kp` and `kd` are now **uint16** (2 bytes each),
> replacing the former `kp_u8, kd_u8` + 2 padding bytes. Entry size is **unchanged
> at 12 bytes**; the two padding bytes are repurposed. If you have existing bridge code
> using `struct.pack("<BBHHHBBxx", ...)` you must change it to
> `struct.pack("<BBHHHHH", ...)` (see §9).

In **CSP mode (Phase 1)**, only `pos_u16` is used. Encode with:
```python
pos_u16 = int((pos_rad - (-12.57)) / (25.14) * 65535)
```

In **MIT mode (Phase 2)**, all fields are used — see §9.

The `bus` field should be set from `MOTOR_BUS_MAP[motor_id]` (0=RIGHT, 1=LEFT).

### 3.6 Type 0x11 — Motor Supervisory (Thor → RCU)

Payload: **8 bytes**.

```
enable_mask    u16  bit N = enable motor_id N+1
clear_fault_mask u16  bit N = clear fault for motor_id N+1
ctrl_mode      u8   0=MIT Type1, 1=CSP param-write
_pad           u8×3
```

**Examples:**
```python
# Enable all 12 motors in CSP mode (Phase 1)
payload = struct.pack("<HHBxxx", 0x0FFF, 0x0FFF, 1)

# FULL E-STOP (disable all motors)
payload = struct.pack("<HHBxxx", 0x0000, 0x0000, 1)
```

### 3.7 Type 0x20 — Debug Command (Thor/PC → RCU)

Payload: `subcmd(u8) [+ extra bytes]`

| Sub-cmd | Value | Extra | Action |
|---|---|---|---|
| PING | 0x01 | — | Request immediate debug reply |
| BUZZ | 0x02 | — | 200ms buzzer pulse + PDU buzzer |
| LED_BLINK | 0x03 | — | Blink orange LED 3× |
| CAN_LOOPBACK | 0x04 | — | Run FDCAN1+FDCAN3 loopback, result in reply |
| FORCE_TELEM | 0x05 | — | Force immediate slow-telem TX |
| ASSERT_PDU_FAULT | 0x07 | `1=assert, 0=clear` | Assert/clear PDU power fault |
| SOFT_RESET | 0x08 | `0=RCU only` | NVIC_SystemReset() |
| SET_TELEM_RATE | 0x09 | Hz (5/10/20) | Change slow telem rate |
| MOTOR_BUS_CTRL | 0x0A | bitmask: bit0=L_STB, bit1=R_STB | CAN transceiver standby control |
| REQUEST_SUPV_DUMP | 0x0B | — | Force supervisory state dump to UART |
| MOTOR_ENABLE | 0x0C | `bus(u8), motor_id(u8), enable(u8), clr_fault(u8)` | Enable/disable one motor directly, bypasses MOTOR_BUS_MAP — use for bench testing |
| MOTOR_SET_ZERO | 0x0D | `bus(u8), motor_id(u8)` | Set the motor's current angle as its position zero reference (RS04 Type 6). Always call this before starting position control after a power cycle to establish a safe reference angle. |

**Full e-stop procedure uses TWO commands:**
1. Type 0x11 with `enable_mask=0` (stops motors via CAN Type 4)
2. Type 0x20 sub-cmd 0x07 with extra byte `0x01` (cuts PDU power rails)

Both must be sent. The RCU firmware handles them independently.

### 3.8 Type 0x21 — Debug Reply (RCU → Thor)

Payload: 20 bytes. Sent in response to any debug command.

---

## 4. Motor Joint Mapping

| motor_id | Joint Name | CAN Bus | Bus Num |
|---|---|---|---|
| 1 | pelvis_link_l_yaw_joint | FDCAN3 (left) | 1 |
| 2 | pelvis_link_r_yaw_joint | FDCAN1 (right) | 0 |
| 3 | l_hip_yaw_link_l_pitch_joint | FDCAN3 | 1 |
| 4 | r_hip_yaw_link_r_pitch_joint | FDCAN1 | 0 |
| 5 | l_hip_pitch_link_l_roll_joint | FDCAN3 | 1 |
| 6 | r_hip_pitch_link_r_roll_joint | FDCAN1 | 0 |
| 7 | l_thigh_link_l_knee_joint | FDCAN3 | 1 |
| 8 | r_thigh_link_r_knee_joint | FDCAN1 | 0 |
| 9 | l_shank_link_l_ankle_joint | FDCAN3 | 1 |
| 10 | r_shank_link_r_ankle_joint | FDCAN1 | 0 |
| 11 | l_ankle_link_l_foot_joint | FDCAN3 | 1 |
| 12 | r_ankle_link_r_foot_joint | FDCAN1 | 0 |

Bus mapping is hardcoded in `MOTOR_BUS_MAP[13]` in `rcu_protocol.py` and the
firmware's `motor_bus.h`. If motor IDs are reassigned, update BOTH.

---

## 5. E-Stop Procedure

### 5.1 Full E-Stop (recommended — use this in production)

```python
import rcu_protocol as rp
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Step 1: Disable all motors via CAN (RS04 Type 4 to each motor)
sock.sendto(rp.encode_motor_supervisory(enable_mask=0x0000),
            (rp.RCU_IP, rp.PORT_CMD))

# Step 2: Assert PDU fault (cuts 12V/24V/VRAW power rails entirely)
sock.sendto(rp.encode_debug_cmd(rp.DBGCMD_ASSERT_PDU_FAULT, bytes([1])),
            (rp.RCU_IP, rp.PORT_CMD))
```

Both steps are sent immediately. Step 2 is the physical power cut.

### 5.2 Software-Only Motor Stop (for bench testing / mode switching)

```python
sock.sendto(rp.encode_motor_supervisory(enable_mask=0x0000),
            (rp.RCU_IP, rp.PORT_CMD))
# PDU power remains on — motors are free-wheeling
```

### 5.3 PDU Fault Recovery

```python
# Clear fault (restore power)
sock.sendto(rp.encode_debug_cmd(rp.DBGCMD_ASSERT_PDU_FAULT, bytes([0])),
            (rp.RCU_IP, rp.PORT_CMD))
# Then re-enable motors (ctrl_mode=0 = MIT, the firmware default)
sock.sendto(rp.encode_motor_supervisory(enable_mask=0x0FFF,
                                         clear_fault_mask=0x0FFF,
                                         ctrl_mode=0),
            (rp.RCU_IP, rp.PORT_CMD))
```

---

## 6. Motor Enable Sequence

The RCU firmware handles the enable sequence automatically when Type 0x11 is received
with `enable_mask != 0`. For reference, the CAN frames sent per motor are:

**CSP mode (`ctrl_mode=1`):**
1. **RS04 Type 18** param write: `run_mode = 5` (CSP position mode)
2. **RS04 Type 3** COMM_ENABLE
3. **RS04 Type 18** param write: `limit_spd = 15.0` rad/s

**MIT mode (`ctrl_mode=0`, default):**
1. **RS04 Type 18** param write: `run_mode = 0` (MIT / operation control mode)
2. **RS04 Type 3** COMM_ENABLE

The `run_mode` write must precede COMM_ENABLE in both cases — the motor ignores mode
changes while already enabled. The firmware enforces this order.

**Firmware default is MIT (`ctrl_mode=0`).** If you want CSP, you must send Type 0x11
with `ctrl_mode=1` explicitly.

---

## 7. File Placement in ROS2 Package

Assuming your ROS2 package is named `proj500_control` inside `src/`:

```
src/
  proj500_control/
    proj500_control/
      rcu_udp_bridge.py       ← copy from Tools/ROS2/
      rcu_protocol.py         ← copy from Tools/
      robot_command_bridge.py  ← your existing file
      robot_observation_bridge.py ← your existing file
    launch/
      rcu_launch.py           ← copy from Tools/ROS2/
    setup.py
    package.xml
    CMakeLists.txt
```

### 7.1 setup.py Changes

Add to `entry_points['console_scripts']`:
```python
"rcu_udp_bridge = proj500_control.rcu_udp_bridge:main",
```

### 7.2 package.xml Changes

Add inside `<package>`:
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>std_srvs</exec_depend>
```

### 7.3 CMakeLists.txt (if using ament_cmake_python)

```cmake
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  proj500_control/rcu_udp_bridge.py
  proj500_control/rcu_protocol.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```

### 7.4 Update launch file

In `rcu_launch.py`, replace `<your_ros2_package>` with `proj500_control` (or your actual
package name). Update `executable` names to match your `setup.py` entry points.

---

## 8. Testing Sequence

### Step 1: Network connectivity
```bash
ping 192.168.100.10          # RCU must respond
```

### Step 2: Slow telem check
```bash
python Tools/rcu_monitor.py  # existing tool — should show PDU data at 10 Hz
```

### Step 3: Bench tool (no motors)
```bash
python Tools/rcu_motor_test.py
# Press 'p' → should see debug reply with uptime_ms > 0
# Note: motor_fb table will show '--' until motors are connected
```

### Step 4: Single motor test (one RS04 connected)
```
In rcu_motor_test.py:
1. Press '1' to select motor 1
2. Press 'e' to enable (observe: motor LED changes, motor holds position)
3. Press ']' three times (motor should move ~0.6 rad total)
4. Press 'h' (motor should return to 0 rad)
5. Press 'd' for full e-stop (motor de-energises, PDU power cuts)
6. Verify PDU fault cleared before re-enabling: press 'd' then PDU fault clears, then 'e'
```

### Step 5: Fast IMU verification
```bash
python -c "
import socket, rcu_protocol as rp, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', rp.PORT_TELEM))
s.settimeout(1.0)
count = 0
t0 = time.time()
while time.time() - t0 < 2.0:
    try:
        d, _ = s.recvfrom(256)
        h = rp.parse_header(d)
        if h and h[0] == rp.PKT_IMU_FAST:
            count += 1
    except: pass
print(f'IMU fast packets in 2s: {count} (expected ~400)')
"
```

### Step 6: ROS2 integration
```bash
# Terminal 1
ros2 launch proj500_control rcu_launch.py

# Terminal 2 — verify fast IMU at ~200 Hz
ros2 topic hz /imu0

# Terminal 3 — verify motor feedback
ros2 topic echo /motor_can_feedback --no-arr

# Terminal 4 — test enable via service
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: true}"

# Terminal 4 — test e-stop
ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: false}"
```

---

## 9. Phase 2 — MIT Impedance Control Migration

Phase 1 uses CSP position control (parameter write, proven simple and robust).
Phase 2 uses MIT impedance control (Type 1 CAN frame with full pos/vel/kp/kd/torque)
for the RL policy output. **No firmware changes are needed** — only the supervisory
packet `ctrl_mode` field changes.

> **Note:** The firmware now defaults to MIT (`g_ctrl_mode=0`). Phase 1 CSP requires
> explicitly sending `ctrl_mode=1` in every Type 0x11 packet.

### Activating Phase 2

```python
# Send Type 0x11 with ctrl_mode=0 (MIT) — this is already the firmware default
sock.sendto(rp.encode_motor_supervisory(
    enable_mask=0x0FFF,
    clear_fault_mask=0x0FFF,
    ctrl_mode=0     # MIT impedance control
), (rp.RCU_IP, rp.PORT_CMD))
```

The firmware writes `run_mode=0` before COMM_ENABLE, then accepts Type 1 CAN frames.

### Type 0x10 command entries in MIT mode

All fields are now used. **kp and kd are encoded as uint16** (0–65535 mapping to
0–RS04_KP_MAX and 0–RS04_KD_MAX respectively):
```python
# Wire format changed: "<BBHHHHH" (was "<BBHHHBBxx")
entry = rcu_protocol.encode_motor_cmd_entry(
    motor_id=1,
    bus=1,           # from MOTOR_BUS_MAP
    pos_rad=theta,   # RL target position
    vel_rads=dtheta, # RL target velocity
    torque_nm=tau,   # feedforward torque — set to 0.0 for pure position control
    kp=30.0,         # position stiffness (0–5000 Nm/rad); see gain guidance below
    kd=5.0,          # velocity damping (0–100 Nm·s/rad)
)
```

The MIT control law on the motor is:
`torque = Kp*(pos_target - pos_actual) + Kd*(vel_target - vel_actual) + torque_ff`

> **Important:** Setting `torque_ff` to any non-zero value while in position control
> will cause the motor to spin continuously at `vel_steady = torque_ff / Kd`.
> **Always set `torque_nm=0.0` unless you specifically need feedforward torque.**

### `/robot_command` message requirements for MIT mode

The bridge reads the following fields from `custom_msgs/RobotCommand` (all optional,
default 0.0 if absent):

| Field | Type | Required for MIT | Notes |
|---|---|---|---|
| `joint_names` | string[] | yes | matched to MOTOR_JOINT_NAMES |
| `positions` | float64[] | yes | rad |
| `velocities` | float64[] | **recommended** | rad/s — feedforward velocity |
| `efforts` | float64[] | optional | Nm — feedforward torque |
| `kp_gains` | float64[] | **required** | position stiffness (0–5000 Nm/rad); **0 = no movement** |
| `kd_gains` | float64[] | **required** | velocity damping (0–100 Nm·s/rad); **0 = no movement** |

> **Critical:** If `kp_gains` and `kd_gains` are zero or absent, the motor receives
> zero PD impedance and will not track position targets regardless of the enable state.
>
> **Gain guidance:**
> - If the RL policy outputs per-joint kp/kd as part of its action, use those directly.
> - **Recommended bring-up starting point: `kp=30 Nm/rad`, `kd=5 Nm·s/rad`.**
>   This gives an overdamped response on typical limb inertias (ζ > 1 for J < 0.6 kg·m²).
>   Increase Kp gradually (30 → 50 → 100) once steady-state position hold is confirmed.
> - For RL policy deployment, `kp=20–80 Nm/rad`, `kd=1–5 Nm·s/rad` is typical.
>   The RS04 saturates at 120 Nm, so `kp=500` with a 1 rad error would hit the torque
>   limit immediately — reserve high gains only for quasi-static commissioning tests.
> - **kp/kd resolution:** both are now uint16-encoded (0–65535 → full range), giving
>   ~0.076 Nm/rad steps for Kp and ~0.0015 Nm·s/rad steps for Kd. Kp values below ~0.076
>   will be sent as zero (motor holds no position).

If `custom_msgs/RobotCommand` does not yet have `kp_gains`/`kd_gains` fields, add
them to the message definition and rebuild the package. Until then, the bridge will
silently send zeros and the motors will not move.

### Set-Zero Before Position Control

The RS04 position counter is relative to the motor's angle at power-on, not a
mechanical hard-stop. Before issuing any position commands, send `MOTOR_SET_ZERO
(0x0D)` to define the current joint angle as `pos=0.0`:

```python
# Set zero reference for motor 1 on bus 0 before enabling
sock.sendto(rp.encode_debug_cmd(0x0D, bytes([0, 1])),
            (rp.RCU_IP, rp.PORT_CMD))
# Then enable and send pos=0.0 commands — motor will hold current angle
```

This must be done after every power cycle. Without it, commanding `pos=0.0` will
drive the motor toward an arbitrary power-on reference angle, which can cause large
unexpected movements.

### rcu_launch.py

`ctrl_mode` now defaults to `"0"` (MIT) in both `rcu_launch.py` and `rcu_udp_bridge.py`.
No manual override is needed for Phase 2. To run Phase 1 CSP explicitly:
```bash
ros2 launch proj500_control rcu_launch.py ctrl_mode:=1
```

---

## 10. Known Caveats and Notes

1. **motor_id=0 is invalid** — always rejected by firmware. Motor IDs start at 1.

2. **Mode change requires disabled state** — the RCU firmware handles this: it always
   writes `run_mode` BEFORE sending the Type 3 enable frame. Do not send mode changes
   to an already-enabled motor. Use the supervisory packet to disable first.

3. **CAN bus loading at 200 Hz** — 12 motors × `loc_ref` param writes ≈ 2400 CAN
   frames/sec across two 1 Mbps buses (≈10% utilisation per bus). Monitor FDCAN
   TX FIFO overflow counters via the debug reply `can_loopback` field if latency
   is observed.

4. **IMU ODR must be ≥200 Hz** — verify the ICM IMU on the RCU is configured for
   at least 200 Hz output data rate. The RCU firmware polls `imu_get_sample()` at
   200 Hz from the fast loop. If the ODR is lower, the fast IMU packets will contain
   duplicate samples.

5. **Ethernet link must be 1000BASE-T or 100BASE-TX** — the RCU uses RMII. A
   10BASE-T switch between RCU and Thor will add ~1 ms latency per packet.
   Use a direct cable or a GbE switch.

6. **PDU fault recovery** — after asserting PDU fault (full e-stop), the 12V/24V/VRAW
   rails are cut. Motors lose power. To recover: (a) send clear-fault debug cmd 0x07
   with byte=0, (b) wait ~500 ms for rails to ramp up, (c) re-enable motors via
   Type 0x11.

7. **RS04 CAN_TIMEOUT** — the RS04 motor has a configurable CAN watchdog
   (`CAN_TIMEOUT` parameter, function code `0x200B`). Default is 0 (disabled). If
   you enable it (e.g. 200000 = 10 s), the motor will reset itself if no CAN frame
   is received for that duration. Keep this in mind during debug pauses.

8. **Set zero before position control** — the RS04 position counter is relative to
   power-on angle. Always send `DBGCMD_MOTOR_SET_ZERO (0x0D)` before issuing position
   commands to avoid large unexpected movements on first command.

9. **Torque feedforward causes steady-state velocity** — in MIT mode,
   `vel_steady = torque_ff / Kd`. A non-zero `torque_nm` while trying to hold position
   will cause the motor to spin. This is not a firmware bug.

---

## 11. Changelog

### 2026-05-02 — kp/kd precision upgrade, Set Zero command

**Summary:** Three changes that affect the binary wire protocol and bring-up procedure.
RCU firmware must be reflashed before these take effect.

#### Wire protocol change — `rcu_motor_cmd_entry_t` (Type 0x10)

| Field | Old | New |
|---|---|---|
| `kp` encoding | `kp_u8` (1 byte, 0–255 → 0–5000 Nm/rad) | `kp_u16` (2 bytes, 0–65535 → 0–5000 Nm/rad) |
| `kd` encoding | `kd_u8` (1 byte, 0–255 → 0–100 Nm·s/rad) | `kd_u16` (2 bytes, 0–65535 → 0–100 Nm·s/rad) |
| Padding | 2 bytes `_pad` at end | Repurposed as kd_u16 high byte — no longer present |
| Entry size | 12 bytes (10 payload + 2 pad) | 12 bytes (all payload) |
| Python format | `"<BBHHHBBxx"` | `"<BBHHHHH"` |

**Why:** At 8-bit resolution, Kp steps were ~19.6 Nm/rad. `kp=10` encoded as
`kp_u8 = int(10/5000×255) = 0` — zero stiffness, causing the motor to not hold
position at all. The u16 encoding gives ~0.076 Nm/rad steps.

**Bridge code change required:**
```python
# Old (DO NOT USE)
kp_u8  = int(max(0, min(255, (kp / RS04_KP_MAX) * 255)))
entry  = struct.pack("<BBHHHBBxx", bus, motor_id, pos_u16, vel_u16, trq_u16, kp_u8, kd_u8)

# New
kp_u16 = int(max(0, min(65535, (kp / RS04_KP_MAX) * 65535)))
kd_u16 = int(max(0, min(65535, (kd / RS04_KD_MAX) * 65535)))
entry  = struct.pack("<BBHHHHH", bus, motor_id, pos_u16, vel_u16, trq_u16, kp_u16, kd_u16)
```

#### New debug command — `MOTOR_SET_ZERO (0x0D)`

Added to firmware (`eth_udp.c`) and bench GUI. Sends RS04 Type-6 set-zero to the
target motor, defining its current angle as `pos=0.0`. See §3.7 and §9.

#### Updated bring-up defaults

The bench GUI (`plymouth_humanoid_bench_monitor.py`) default gains changed:
- Kp: 10 → **30 Nm/rad** (previous default silently encoded as kp_u8=0 = zero stiffness)
- Kd: 1.0 → **5.0 Nm·s/rad** (previous default was underdamped for typical limb inertia)

#### Files changed

| File | Change |
|---|---|
| `MCU_RCU_V1/Core/Inc/rcu_pkt.h` | `kp_u8/kd_u8` → `kp_u16/kd_u16` in `rcu_motor_cmd_entry_t`; added `RCU_DBGCMD_MOTOR_SET_ZERO 0x0D` |
| `MCU_RCU_V1/Core/Src/telem_pack.c` | Kp/Kd decode: `/ 255.0f` → `/ 65535.0f`, `kp_u8/kd_u8` → `kp_u16/kd_u16` |
| `MCU_RCU_V1/Core/Src/eth_udp.c` | Added `case RCU_DBGCMD_MOTOR_SET_ZERO` handler calling `motor_bus_send_set_zero()` |
| `Tools/plymouth_humanoid_bench_monitor.py` | `send_motor_cmd` uses `"<BBHHHHH"` + u16 encoding; added `send_motor_set_zero()`; added "Set Zero" button; Kp/Kd defaults updated |
