# PROJ500 Humanoid Robot — Charlie's Handover

**Author:** Charlie  
**Date:** May 2026  
**Project:** MEng PROJ500 — Plymouth Humanoid Robot  

This folder contains all hardware and firmware deliverables produced during the 2025–26 academic year. It is structured into three top-level areas:

```
Charlie/
├── KiCad/                  — PCB schematics and layouts (5 boards)
├── PROJ500_PDU_FPGA/       — Lattice MachXO2 FPGA project (PDU sequencer)
└── STM32Cube/              — STM32 firmware (PDU + RCU) and PC tools
```

---

## System Overview

The robot's electronic architecture is split across two custom boards:

| Board | MCU | Role |
|---|---|---|
| **PDU** — Power Distribution Unit | STM32G474 | Battery management, power sequencing, relay/contactor control via FPGA |
| **RCU** — Real-Time Control Unit | STM32H723 | Motor CAN buses, Ethernet UDP to Thor, dual IMU, PDU telemetry forwarding |

The PDU and RCU communicate over a dedicated **management CAN bus (250 kbps)**. The RCU talks to the Thor AI computer (Jetson/PC) over **UDP/Ethernet**. Motors are driven by the RCU over two **1 Mbps CAN buses** using the RobStride RS04 private protocol.

```
[ Battery ] → [ PDU ] ──CAN(250k)──→ [ RCU ] ──Ethernet(UDP)──→ [ Thor / ROS2 ]
                 ↑FPGA                    ↓ ↑
               sequencer              FDCAN1/3 (1Mbps)
                                    [ 12× RS04 Motors ]
```

---

## 1. KiCad — PCB Designs

All boards were designed in **KiCad**. Each folder contains the full KiCad project (`.kicad_sch`, `.kicad_pcb`, `.kicad_pro`), a rendered **PDF schematic**, exported **STEP** 3D model, **BOM** (bill of materials in `bom/`), and **Gerber/drill files** for fabrication in `production/`.

### 1.1 Power Distribution Board (PDU)

**Folder:** `KiCad/PROJ500 - Power Distribution Board/`

The most complex board. Manages all power rails from the battery pack and implements hardware safety logic.

**Key schematics:**
| Sub-sheet | File | Description |
|---|---|---|
| Top level | `PROJ500 - Power Distribution Board.kicad_sch` | Hierarchy root |
| MCU | `MCU.kicad_sch` | STM32G474 with FDCAN1 (management CAN), I2C4 (FPGA), UART3/4 (RS485, debug), ADC, SWD debug header |
| FPGA | `FPGA.kicad_sch` | Lattice MachXO2-256HC-5SG48C QFN48 — relay sequencer + I2C slave |
| Safety logic | `safety_logic.kicad_sch` | Hardware e-stop and overvoltage/undervoltage comparators feeding FPGA inputs |
| Comparators | `Comparators.kicad_sch` | OV/UV sense circuits |
| Coil driver | `Coil_Driver.kicad_sch` | Relay coil drive circuit (K_EN / K_SEL) |
| Aux outputs | `Aux_Outputs.kicad_sch` | Auxiliary 12 V/24 V switched outputs, buzzer, status LEDs |

**Power rails** — refer to `PROJ500 - Power Distribution Board.pdf` for the confirmed rail architecture and voltage levels.

**ADC channels (TLA2528, I2C, address 0x17):**
| CH | Signal | Scaling |
|---|---|---|
| 0 | V_VRAW | ÷31 |
| 1 | I_VRAW_SW | 9.47× shunt amp |
| 2 | V_12V_SW | ÷7.8 |
| 3 | V_24V_SW | ÷14 |
| 4 | I_12V_SW | 4× shunt amp |
| 5 | I_24V_SW | 4× shunt amp |
| 6 | THERM1 | NTC β-model |
| 7 | THERM2 | NTC β-model |

**Local STM32G474 ADC pins:**
| Pin | Signal |
|---|---|
| PA6 ADC2_CH3 | Board thermistor 0 |
| PA7 ADC2_CH4 | Board thermistor 1 |
| PC4 ADC2_CH5 | Board thermistor 2 |
| PB0 ADC1_CH15 | V_SOURCE (36.71× divider) |
| PB1 ADC1_CH12 | V_BUS (36.71× divider) |
| PB2 ADC2_CH12 | I_COIL (2× shunt amp, 10 mΩ / ×50) |

---

### 1.2 Real-Time Control Unit (RCU)

**Folder:** `KiCad/PROJ500 - Real-Time Control Unit/`

**Key schematics:**
| Sub-sheet | File | Description |
|---|---|---|
| MCU | `STM32_MCU.kicad_sch` | STM32H723VGT6, FDCAN1/2/3, SPI3/4 (IMUs), Ethernet RMII |
| Ethernet | `ETHERNET.kicad_sch` | RJ45 + magnetics |
| ETH PHY | `ETH_PHY.kicad_sch` | PHY chip (RMII mode) |
| IMU | `IMU.kicad_sch` | Dual LSM6DSOX IMUs on SPI3/4 |
| ESP32 | `ESP32_MCU.kicad_sch` | Co-processor / future wireless |

A **production-ready Gerber zip** (`PROJ500 - Real-Time Control Unit.zip`) is included for direct upload to a PCB manufacturer.

---

### 1.3 Power Supply Board

**Folder:** `KiCad/PROJ500 - Power Supply Board/`

Standalone DC-DC converter board providing regulated rails from a high-voltage input.

**Converters:**
| File | Part | Output |
|---|---|---|
| `AP62200 - 5V 2A.kicad_sch` | AP62200 | 5 V / 2 A |
| `MIC28517 - 12V 4A.kicad_sch` | MIC28517 | 12 V / 4 A |
| `MIC28517 - 24V 8A.kicad_sch` | MIC28517 | 24 V / 8 A |
| `SiC437 - 12V 8A.kicad_sch` | SiC437 | 12 V / 8 A (high efficiency) |
| `Hot Swap Switch.kicad_sch` | TPS249x | Hot-swap / inrush limiting |
| `Shunt Amplifiers.kicad_sch` | — | Current sense amps |
| `TLA2528 ADC.kicad_sch` | TLA2528 | 8-ch ADC for rail monitoring |

A **TPS249x design spreadsheet** (`TPS249x_8x_Design_Calculator_REV_B.xlsx`) is included for recalculating hot-swap component values.

---

### 1.4 Auxiliary IMU Board

**Folder:** `KiCad/PROJ500 - Auxillary IMU Board/`

Compact breakout board for a single IMU (e.g. LSM6DSOX). Intended for body segments that cannot fit the main RCU.

---

### 1.5 Braking Chopper Controller

**Folder:** `KiCad/PROJ500 - Braking Chopper Controller/`

Handles regenerative braking energy from motors. Dissipates or redirects back-EMF during deceleration to protect the bus capacitors.

---

## 2. PROJ500_PDU_FPGA — Lattice MachXO2 FPGA

**Tool required:** Lattice Diamond (tested with 3.13)  
**Device:** MachXO2-256HC-5SG48C (SG48C QFN48 package)  
**Clock:** Internal OSCH @ 2.08 MHz  

### 2.1 Source Files

| File | Description |
|---|---|
| `pdu_glue_mxo2.sv` | **Main production image** — full relay sequencing FSM + EFB I2C slave |
| `pdu_selftest_mxo2.sv` | **Board self-test image** — drives all outputs for PCB bring-up continuity checks |
| `PDU_EFB_I2C.v` / `.edn` | Lattice EFB I2C slave instantiation (auto-generated from Diamond IP) |
| `PROJ500_PDU_FPGA.lpf` | Pin Location File — maps all signal names to physical QFN48 pins |
| `PROJ500_PDU_FPGA.ldf` | Diamond project file |
| `pdu_glue_synth.sdc` | Timing constraints (2.08 MHz clock constraint) |
| `tb_pdu_glue.sv` | Simulation testbench for the main FSM |
| `tb_pdu_selftest_mxo2.sv` | Simulation testbench for the self-test image |
| `sim_do.do` | ModelSim run script |

### 2.2 FPGA State Machine

The FPGA implements the main contactor and precharge relay sequencer:

| State | STATE_CODE | Description |
|---|---|---|
| ST_IDLE | 0 | All outputs de-asserted, waiting for SW_COMPUTE |
| ST_COMPUTE | 3 | SW_COMPUTE=1, awaiting arm permission |
| ST_PRECHARGE | 1 | Relay closed on NC (precharge) path; charging bus |
| ST_PRECHARGE_ABORT | 1 | Relay held closed while bus discharges safely |
| ST_ARMED | 2 | Main contactor closed, motors active |
| ST_FAULT | 0 | Latched fault — requires manual SW_RST_FAULT |

**Relay control note:** `K_SEL=0` selects the **precharge (NC) path**; `K_SEL=1` selects the **dump path**. Always verify polarity on the bench before first energisation.

### 2.3 FPGA I2C Register Map

The FPGA exposes a read-only I2C slave at **address 0x55** (7-bit):

| Register | Address | Description |
|---|---|---|
| STATUS0 | 0x00 | Fault/status flags bitmask |
| FAULT_CODE | 0x01 | Fault reason code |
| STATE_CODE | 0x02 | FSM state (0=IDLE, 1=PCHG, 2=ARMED, 3=COMPUTE) |
| ACTIONS | 0x03 | Current output action register |
| INPUTS | 0x04 | Latched input states (e-stop, arm flags) |
| PCHG_HI | 0x05 | Precharge timer high byte |
| PCHG_LO | 0x06 | Precharge timer low byte |
| VERSION | 0x7F | FPGA firmware version |

### 2.4 Built Implementations

Two ready-to-program bitfiles are included:

| Implementation | Folder | Bitfile | JEDEC |
|---|---|---|---|
| Production (V1) | `V1/` | `PROJ500_PDU_FPGA_V1.bit` | `PROJ500_PDU_FPGA_V1.jed` |
| Self-test | `SELFTEST/` | `PROJ500_PDU_FPGA_SELFTEST.bit` | `PROJ500_PDU_FPGA_SELFTEST.jed` |

### 2.5 FPGA JTAG Programming (FT2232H)

Connect a FT2232H development board to the FPGA JTAG header (02×05 IDC):

| IDC Pin | Signal | FT2232H (ADBUS) |
|---|---|---|
| 1 | TCK | ADBUS0 |
| 2 | GND | GND |
| 3 | TMS | ADBUS3 |
| 4 | GND | GND |
| 5 | TDI | ADBUS1 |
| 6 | 3V3 | 3V3 |
| 7 | TDO | ADBUS2 |
| 8 | GND | GND |

Use Diamond Programmer or `openFPGALoader` with the `.jed` or `.bit` file.

---

## 3. STM32Cube — Firmware and Tools

### 3.1 MCU_PDU_V1 — PDU Firmware

**Target:** STM32G474RET6 (LQFP64)  
**IDE:** STM32CubeIDE  
**Project file:** `STM32Cube/MCU_PDU_V1/MCU_PDU_V1.ioc`  

The PDU runs a **bare-metal superloop** (no RTOS). All modules expose an `_init()` / `_tick()` API called from `pdu_app.c`.

#### Source Modules

| File | Description |
|---|---|
| `main.c` | CubeMX-generated HAL init; calls `PDU_App_Init()` then loops `PDU_App_Task()` |
| `pdu_app.c` | Superloop coordinator — sequences all subsystem ticks |
| `fpga_mon.c` | Polls FPGA I2C registers every ~50 ms; caches in `fpga_snapshot_t` |
| `pdu_adc.c` | Samples TLA2528 (I2C4, ext ADC) and local STM32 ADC channels |
| `ssd_energy.c` | Queries SSD RS485 energy meter over USART3 at 19200 baud (~5 Hz) |
| `pdu_mcan_app.c` | Management CAN TX/RX on FDCAN1 (250 kbps); sends telemetry to RCU |
| `pdu_selftest_cli_v3.c` | UART self-test CLI (build-mode selectable) |

#### Management CAN Frame IDs (PDU → RCU)

| CAN ID | Rate | Content |
|---|---|---|
| 0x511 | 2 Hz | PDU heartbeat |
| 0x520 | 10 Hz | FPGA status (status0, fault, state, actions, inputs, version, pchg) |
| 0x521 | 10 Hz | Power rail voltages (V_RAW, 12 V, 24 V) |
| 0x522 | 10 Hz | Rail currents + thermistors |
| 0x523 | 5 Hz | SSD energy meter (I, V, P, T) |
| 0x524 | 10 Hz | Local ADC A (board therms 0–2, V_SOURCE) |
| 0x525 | 10 Hz | Local ADC B (V_BUS, I_COIL) |

**RCU → PDU:**

| CAN ID | Content |
|---|---|
| 0x530 | AUX switch command (bits[2:0]=CH3/2/1, bit3=buzz, bit4=LED blink) |
| 0x531 | CMD_FAULT request (bit0=assert, bit1=clear) |

#### STLink Debug Header (02×05 IDC)

| IDC Pin | Signal | Wire colour |
|---|---|---|
| 1 | 3.3 V | Red |
| 2 | SWDIO | Yellow |
| 3 | GND | Black |
| 4 | SWCLK | Green |
| 5 | GND | Black |
| 6 | SWO | Blue |
| 7 | UART_RX | Brown |
| 8 | UART_TX | Grey |
| 9 | 5 V | Purple |
| 10 | NRST | White |

---

### 3.2 MCU_RCU_V1 — RCU Firmware

**Target:** STM32H723VGT6 (LQFP100)  
**IDE:** STM32CubeIDE  
**Project file:** `STM32Cube/MCU_RCU_V1/MCU_RCU_V1.ioc`  

Also a **bare-metal superloop**. All modules expose `_init()` / `_tick()` called from `rcu_app.c`.

#### Source Modules

| File | Description |
|---|---|
| `main.c` | HAL init; calls `RCU_App_Init()` then loops `RCU_App_Task()` |
| `rcu_app.c` | Superloop coordinator; scheduling buckets at 10 Hz and 200 Hz |
| `motor_bus.c` | FDCAN1 (right) + FDCAN3 (left) motor CAN; drains RX FIFOs, caches feedback |
| `rs04.c` | RobStride RS04 CAN codec — pure, no HAL dependency |
| `imu.c` | Dual LSM6DSOX on SPI3/4 at 416 Hz ODR; DRDY interrupt driven |
| `eth_udp.c` | lwIP raw-API UDP sockets; TX/RX on ports 7700/7701/7702 |
| `mcan_pdu.c` | Receives PDU telemetry over FDCAN2 (management CAN); caches in `pdu_telem_t` |
| `telem_pack.c` | Assembles UDP packets from live subsystem data; dispatches inbound motor commands |
| `rcu_pkt.h` | All UDP packet type definitions and structs (source of truth for the protocol) |
| `rcu_selftest_cli_v1.c` | UART self-test CLI |

#### Motor Bus Mapping (12-DOF Lower Body)

| Motor ID | Joint | Bus |
|---|---|---|
| 1 | pelvis_link_l_yaw_joint | LEFT (FDCAN3) |
| 2 | pelvis_link_r_yaw_joint | RIGHT (FDCAN1) |
| 3 | l_hip_yaw_link_l_pitch_joint | LEFT |
| 4 | r_hip_yaw_link_r_pitch_joint | RIGHT |
| 5 | l_hip_pitch_link_l_roll_joint | LEFT |
| 6 | r_hip_pitch_link_r_roll_joint | RIGHT |
| 7 | l_thigh_link_l_knee_joint | LEFT |
| 8 | r_thigh_link_r_knee_joint | RIGHT |
| 9 | l_shank_link_l_ankle_joint | LEFT |
| 10 | r_shank_link_r_ankle_joint | RIGHT |
| 11 | l_ankle_link_l_foot_joint | LEFT |
| 12 | r_ankle_link_r_foot_joint | RIGHT |

#### IMU Configuration

| | IMU0 | IMU1 |
|---|---|---|
| SPI bus | SPI4 (PE2/5/6) | SPI3 (PC10/11/12) |
| Chip Select | PC13 | PA15 |
| DRDY / INT1 | PE3 (EXTI3) | PA9 (EXTI9_5) |
| ODR | 416 Hz | 416 Hz |
| Accel range | ±4 g (0.122 mg/LSB) | same |
| Gyro range | ±500 dps (17.5 mdps/LSB) | same |
| Temp | raw/256 + 25 → °C | same |

#### Network Configuration

| Parameter | Value |
|---|---|
| RCU static IP | `192.168.100.10` |
| Thor IP | `192.168.100.20` |
| Subnet mask | `255.255.255.0` |
| Telem TX port (RCU→Thor) | 7700 |
| Cmd RX port (Thor→RCU) | 7701 |
| Supervision TX port (RCU→Thor) | 7702 |

---

### 3.3 UDP Binary Protocol Reference

Full detail is in `STM32Cube/MCU_RCU_V1/THOR_INTERFACE.md` and `STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md`. Summary below.

**Packet header (6 bytes, all packets):**
```
[0:1]  magic  = 0x5243  ('RC' little-endian)
[2]    type   — see below
[3]    seq    — rolling 0–255
[4:5]  len    — payload bytes (LE uint16)
```

**Packet types:**

| Type | Direction | Rate | Description |
|---|---|---|---|
| 0x01 | RCU → Thor | 10 Hz | Slow telemetry (PDU power, therms, SSD, IMU) — 72 bytes |
| 0x02 | RCU → Thor | 200 Hz | Motor feedback (pos, vel, torque, error) — 4 + n×10 bytes |
| 0x03 | RCU → Thor | on-change | Supervision events |
| 0x04 | RCU → Thor | 200 Hz | Fast IMU packet (28 bytes) — **primary RL loop IMU source** |
| 0x10 | Thor → RCU | 200 Hz | Motor command (n × 10 bytes) |
| 0x11 | Thor → RCU | on-demand | Motor supervisory (enable mask, ctrl_mode, fault clear) |
| 0x20 | Thor → RCU | on-demand | Debug command (ping, buzz, LED, CAN loopback, soft reset, etc.) |
| 0x21 | RCU → Thor | on-demand | Debug reply |

**Motor command encoding (uint16 on wire):**
```
value_raw = int((value - min) / (max - min) * 65535)
```
| Quantity | Min | Max |
|---|---|---|
| Position | −12.57 rad | +12.57 rad |
| Velocity | −15 rad/s | +15 rad/s |
| Torque | −120 N·m | +120 N·m |

Mid-scale (32767) = zero for position, velocity, and torque.

**Motor supervisory packet (Type 0x11) — enable all motors in CSP mode:**
```python
import struct
payload = struct.pack("<HHBxxx", 0x0FFF, 0x0FFF, 1)  # enable + CSP
```

**Full e-stop:**
```python
payload = struct.pack("<HHBxxx", 0x0000, 0x0000, 1)
```

---

### 3.4 Shared Library

**Folder:** `STM32Cube/Shared/`

| File | Description |
|---|---|
| `Inc/st_mcan.h` | Shared FDCAN helper types and wrappers |
| `Src/st_mcan.c` | Implementation |
| `Inc/st_common.h` | Common defines shared between PDU and RCU firmware |

Both CubeIDE projects reference these files so changes propagate to both builds automatically.

---

### 3.5 Tools — Python Bench Utilities

**Folder:** `STM32Cube/Tools/`

| File | Description |
|---|---|
| `plymouth_humanoid_bench_monitor.py` | **Bench monitoring GUI** — connects to RCU UDP port 7700, plots live power rails, IMU data, thermal readings with configurable alerts |
| `bench_config.json` | GUI configuration — IP, display settings, alert thresholds, recorded signal list |
| `ROS2/rcu_udp_bridge.py` | **ROS2 bridge node** — replaces the old `ethernet_can_bridge.py` / `nucleo_can_bridge` serial bridge; subscribes `/robot_command`, publishes `/motor_can_feedback`, `/imu0`, `/imu1`, `/rcu_pdu_telem`; exposes `/rcu_motor_estop` and `/rcu_pdu_fault` services |
| `ROS2/rcu_launch.py` | ROS2 launch file for the full motor control stack |
| `ROS2/HANDOVER_INSTRUCTIONS.md` | **Complete Thor integration guide** — read this first when integrating with ROS2 |

#### Running the Bench Monitor (Windows/Linux)

```bash
pip install pyqt5 pyqtgraph
python plymouth_humanoid_bench_monitor.py
```

Set your PC NIC to static IP `192.168.100.20/24`. The GUI reads `bench_config.json` from the same directory.

#### Running the ROS2 Bridge

```bash
# Basic launch
ros2 launch <your_package> rcu_launch.py

# With arguments
ros2 launch <your_package> rcu_launch.py rcu_ip:=192.168.100.10 ctrl_mode:=1
```

**ctrl_mode values:**
- `1` = CSP position mode (Phase 1 — simpler, recommended first)
- `0` = MIT impedance mode (Phase 2 — full torque/velocity/gain control)

> **Package name:** Update `<your_ros2_package>` in `rcu_launch.py` to match your ROS2 workspace package before running.

---

## 4. Key Integration Notes

### 4.1 First Power-On Sequence

1. Program the FPGA with the **SELFTEST** bitfile (`SELFTEST/PROJ500_PDU_FPGA_SELFTEST.jed`) first to verify PCB continuity.
2. Verify all output pins toggle correctly with a scope or logic analyser.
3. Once hardware is confirmed, re-program with the **V1 production** bitfile (`V1/PROJ500_PDU_FPGA_V1.jed`).
4. Flash the PDU MCU firmware (`MCU_PDU_V1`) and check UART self-test output via the STLink header.
5. Flash the RCU MCU firmware (`MCU_RCU_V1`). Run `RCU_SelfTest` via UART to verify CAN loopback, IMU WHO_AM_I, and Ethernet link.

### 4.2 Relay Polarity Warning

> **CRITICAL:** Verify `K_SEL` polarity on the bench with a multimeter **before** enabling `K_EN` for the first time. A wiring error will connect the dump resistor during precharge (inadequate inrush protection) or the precharge resistor during dump (insufficient energy dissipation). The K_SEL blanking time (30 ms) does not protect against a polarity error.

### 4.3 Motor Enable Procedure

Motors must be explicitly enabled before motion commands are accepted. Use the RCU debug command or supervisory packet:

```python
# Via UDP debug command (bench testing without ROS2)
import socket, struct
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Enable motor_id=1 on its mapped bus, clear fault
payload = bytes([0x0C, 0, 1, 1, 1])  # MOTOR_ENABLE: bus=0, id=1, enable=1, clr_fault=1
hdr = struct.pack("<HBBh", 0x5243, 0x20, 0, len(payload))
sock.sendto(hdr + payload, ("192.168.100.10", 7701))
```

### 4.4 RS04 Motor Protocol Notes

- All RS04 CAN frames use **29-bit extended IDs** at 1 Mbps.
- The host (RCU) CAN ID is `0xFD`.
- `mode_status` in feedback (bits [23:22] of CAN ID): `0`=reset, `1`=calibrating, `2`=MIT running. A motor ready to accept commands will show `mode_status=2`.
- CSP position mode is configured via RS04 Type-18 parameter write (`RS04_PARAM_RUN_MODE = 0x7005`, value `5.0f`). The codec helper is in `rs04.c`.

### 4.5 Old Bridge — Superseded

The old `ethernet_can_bridge.py` + `nucleo_can_bridge` (serial CAN bridge via STM32 Nucleo) is **fully superseded** by `rcu_udp_bridge.py`. Do not attempt to run both simultaneously.

---

## 5. Software Requirements

### Firmware (STM32CubeIDE)
- **STM32CubeIDE** ≥ 1.14 (includes GCC ARM toolchain)
- HAL drivers are included in the project (`Drivers/` folders) — no external downloads needed

### FPGA (Lattice Diamond)
- **Lattice Diamond** ≥ 3.12 for synthesis and programming
- `openFPGALoader` can be used as an alternative programmer with the `.jed` files

### Python Tools
```
Python ≥ 3.9
pyqt5         (bench monitor GUI)
pyqtgraph     (bench monitor plots)
rclpy         (ROS2 bridge — ROS2 Humble or later)
```

### ROS2 Topics Published by `rcu_udp_bridge`

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/motor_can_feedback` | `UInt8MultiArray` | 200 Hz | 8 bytes/motor (float32 pos + float32 vel), motors 1→12, 96 bytes total |
| `/imu0` | `sensor_msgs/Imu` | 200 Hz | IMU0 in ROS2 body frame (X=fwd, Y=left, Z=up) |
| `/imu1` | `sensor_msgs/Imu` | 200 Hz | IMU1 in ROS2 body frame |
| `/rcu_pdu_telem` | `std_msgs/String` | 10 Hz | JSON-encoded PDU telemetry snapshot |

---

## 6. File Quick-Reference

| What you need | Where to find it |
|---|---|
| PDU schematic PDF | `KiCad/PROJ500 - Power Distribution Board/PROJ500 - Power Distribution Board.pdf` |
| RCU schematic PDF | `KiCad/PROJ500 - Real-Time Control Unit/PROJ500 - Real-Time Control Unit.pdf` |
| Power Supply schematic PDF | `KiCad/PROJ500 - Power Supply Board/PROJ500 - Power Supply Board.pdf` |
| PDU Gerbers | `KiCad/PROJ500 - Power Distribution Board/production/` |
| RCU Gerbers (zip) | `KiCad/PROJ500 - Real-Time Control Unit/PROJ500 - Real-Time Control Unit.zip` |
| FPGA production bitfile | `PROJ500_PDU_FPGA/V1/PROJ500_PDU_FPGA_V1.jed` |
| FPGA self-test bitfile | `PROJ500_PDU_FPGA/SELFTEST/PROJ500_PDU_FPGA_SELFTEST.jed` |
| FPGA pinout | `PROJ500_PDU_FPGA/PROJ500_PDU_FPGA.lpf` |
| UDP protocol spec | `STM32Cube/MCU_RCU_V1/THOR_INTERFACE.md` |
| ROS2 integration guide | `STM32Cube/Tools/ROS2/HANDOVER_INSTRUCTIONS.md` |
| STLink wiring | `STM32Cube/STLINK - 02x05 IDC HEADER.txt` |
| FPGA JTAG wiring | `PROJ500_PDU_FPGA/FPGA JTAG - 02x05 IDC HEADER.txt` |
| Bench monitor config | `STM32Cube/Tools/bench_config.json` |
