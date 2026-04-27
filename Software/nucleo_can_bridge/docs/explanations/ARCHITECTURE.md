# System Architecture Diagram

## Hardware Layout

```
┌──────────────────────────────────────────────────────────────────┐
│ PC (Windows)                                                     │
│ ┌────────────────────────────────────────────────────────────┐  │
│ │ pc_control_2motors.py                                     │  │
│ │  - Keyboard input handler                                 │  │
│ │  - CAN protocol encoder                                   │  │
│ │  - Telemetry parser & display                            │  │
│ └────────────────────────────────────────────────────────────┘  │
│                             │                                    │
│        USB Serial (COM6 @ 921600 baud)                          │
│        Frame format: [0x41,0x54,CAN_ID(4),DLC,DATA(8),CR,LF]   │
│                             │                                    │
└─────────────────────────────┼────────────────────────────────────┘
                              │
                    ┌─────────▼──────────┐
                    │  Nucleo F429ZI    │
                    │  Motor Controller  │
                    │                   │
                    │ USART3 (PD8/PD9)  │
                    │ ◄──────────────►  │
                    │ (Serial bridge)    │
                    │                   │
                    │ CAN1 (PD0/PD1)    │
                    │ ───────────────►  │
                    │ (Motor commands)   │
                    └─────────┬──────────┘
                              │
                    ┌─────────▼────────────────┐
                    │   CAN Bus (1 Mbps)       │
                    │   Extended ID protocol   │
                    └──────┬──────────┬────────┘
                           │          │
            ┌──────────────▼─┐   ┌────▼──────────────┐
            │  Motor ID=127  │   │  Motor ID=1      │
            │  (Controller)  │   │  (Controller)    │
            │                │   │                  │
            │ SPI/CAN        │   │ SPI/CAN          │
            │ Encoder Input  │   │ Encoder Input    │
            │ PWM Output     │   │ PWM Output       │
            └────────────────┘   └──────────────────┘
```

## Command Flow

```
1. PC Sends Command (Serial)
   ↓
   Frame: [0x41,0x54,CAN_ID_bytes,DLC,DATA,0x0D,0x0A]
   Example: Enable motor 127
            [0x41,0x54] + CAN_ID(0x03FD7F) + DLC(8) + [0,0,0,0,0,0,0,0] + [0x0D,0x0A]
   
2. Nucleo Receives Serial
   ↓
   UART ISR → serial_buf (ring buffer)
   
3. Parse Frame
   ↓
   ProcessSerialData() extracts:
   - CAN Extended ID
   - DLC (data length)
   - 8 bytes of CAN data
   
4. Process Command
   ↓
   MotorController_ProcessCommand():
   - Decode comm_type, sender_id, motor_id from CAN ID
   - Switch on command type:
     * COMM_ENABLE (3)    → motor->enabled = 1
     * COMM_DISABLE (4)   → motor->enabled = 0
     * COMM_SET_ZERO (6)  → motor->position = 0
     * PARAM_WRITE        → set control parameters
   
5. Motor Control Loop (1 kHz)
   ↓
   Timer 2 fires every 1ms:
   - Read current motor state (position, velocity)
   - Compute control law:
     * If MODE_POSITION: velocity = Kp × (target - actual)
     * If MODE_VELOCITY: velocity = target (direct command)
   - Update position: position += velocity × dt
   - Limit/clamp values
   
6. Send Telemetry (Every 50ms)
   ↓
   MotorController_SendTelemetry():
   - Encode motor state as uint16 values
   - Build CAN frame with COMM_OPERATION_STATUS (2) type
   - SendCanFrame() → CAN peripheral
   
7. CAN to Serial
   ↓
   ForwardCanToSerial():
   - Receive CAN frame from motor (loopback or real response)
   - Encode in serial frame format
   - Send back to PC via UART3
   
8. PC Receives Telemetry
   ↓
   PC script parses frame and updates display:
   Motor 127 | Pos:+45.23 deg | Vel:+2.15 r/s | Torq:+12.5 Nm | Temp:28.3 C
```

## CAN ID Structure

```
Extended CAN ID (29-bit):
┌─────────────────────────────────────────────────────────────┐
│ Bits 24-28 │ Bits 8-23          │ Bits 0-7                │
│ comm_type  │ sender_id (8 bits) │ motor_id (8 bits)       │
└─────────────────────────────────────────────────────────────┘

Examples:
  Motor 127 ENABLE:     (0x03 << 24) | (0xFD << 8) | 0x7F = 0x03FD7F
  Motor 1 STATUS:       (0x02 << 24) | (0xFD << 8) | 0x01 = 0x02FD01
  Parameter write:      (0x10 << 24) | (0xFD << 8) | 0x7F = 0x10FD7F
```

## Data Frame Formats

### Command Frame (PC → Nucleo)

```
Serial Format:
[0x41, 0x54] + [ID_byte3, ID_byte2, ID_byte1, ID_byte0] + [DLC] + [data0-7] + [0x0D, 0x0A]

Example - Motor 127 Enable:
[0x41, 0x54, 0x03, 0xFD, 0x00, 0x7F, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x0A]
 ^    ^     ^-----------^     ^-----------^    ^       ^------ Data -----^                 ^     ^
AT   +     CAN ID              Length      Payload (8 bytes)                             CR    LF
```

### Telemetry Frame (Motor → PC)

```
CAN Format:
ID: 0x02FD7F (Motor 127 Status)
Data (8 bytes):
  [0:1] - Position (big-endian uint16)
  [2:3] - Velocity (big-endian uint16)
  [4:5] - Torque (big-endian uint16)
  [6:7] - Temperature (big-endian uint16)

Encoding Example:
  Position = 45.23° = 0.789 rad
  Normalized: (0.789 / 12.566 + 1.0) × 32767 = 34521 (0x8709)
  Big-endian bytes: [0x87, 0x09]

Full telemetry example:
  Position: 34521 → [0x87, 0x09]
  Velocity: 33500 → [0x82, 0x7C]
  Torque:   33000 → [0x80, 0xD8]
  Temperature: 283 → [0x01, 0x1B]
  
  Full data: [0x87, 0x09, 0x82, 0x7C, 0x80, 0xD8, 0x01, 0x1B]
```

### Parameter Command Frame

```
For setting MODE, POSITION_TARGET, etc:

Data bytes (8):
  [0:1] - Parameter ID (little-endian uint16)
           0x05 0x70 = PARAM_MODE (0x7005)
           0x16 0x70 = PARAM_POSITION_TARGET (0x7016)
           0x24 0x70 = PARAM_PP_SPEED_LIMIT (0x7024)
           0x25 0x70 = PARAM_PP_ACCEL (0x7025)
  
  [2:5] - Value (4 bytes, interpreted as float for most params)
           Little-endian IEEE 754 float
           E.g., 1.5708 rad = 0x3FC90FDB in hex
           Bytes: [0xDB, 0x0F, 0xC9, 0x3F]
  
  [6:7] - Reserved (0x00, 0x00)

Example - Set position target to 90° (1.5708 rad):
  ID: 0x10FD7F (parameter write to motor 127)
  Data: [0x16, 0x70, 0xDB, 0x0F, 0xC9, 0x3F, 0x00, 0x00]
         └─ param ID    └─ value (float 1.5708)
```

## State Machine

```
Motor State Transitions:

              POWER-ON
                │
                ▼
           [DISABLED]
         mode = 0x00
                │
                │ ENABLE command
                │
                ▼
        [POSITION_CONTROL]
         mode = 0x01
         ├── Receives POSITION_TARGET
         ├── Proportional control active
         ├── Velocity = Kp × (target - actual)
         │
         ├─ JOGA command (velocity=-1) ──► [VELOCITY_JOG]
         │                                  mode = 0x07
         │  Continuous velocity control     ├─ velocity_target = -JOG_SPEED
         │                                  ├─ Direct velocity command
         │  ◄──── STOP command ─────────────┤
         │       Sets velocity_target = 0   │
         └────────────────────────────────┬─┘
                 │                        │
                 ├─ DISABLE cmd ──► [DISABLED]
                 │                  mode = 0x00
                 │                  enabled = 0
                 │                  velocity = 0
                 │
                 └─ ZERO cmd ─── Sets position = 0
                                  (state unchanged)
```

## Interrupt Priorities

```
Priority 0 (Highest):
  ├─ TIM2_IRQHandler       ← Control loop (1kHz)
  └─ Keeps motor control responsive

Priority 1:
  ├─ USART3_IRQHandler     ← Serial input (commands from PC)
  └─ Queued in ring buffer

Default (Lower):
  ├─ CAN1 RX FIFO0         ← CAN messages (telemetry RX)
  └─ Processed in main loop

Rationale: Control loop gets highest priority to maintain
1kHz timing. Serial/CAN handled in main loop.
```

## Memory Usage

```
Stack:
  ├─ HAL structures        ~2 KB
  ├─ Motor state (2×200B)  ~400 B
  ├─ Serial buffer         512 B
  └─ Local variables       ~1 KB
  └─ TOTAL: ~4 KB (plenty, Nucleo has 192 KB)

Flash:
  ├─ HAL+drivers          ~100 KB
  ├─ Motor controller      ~8 KB
  ├─ Main loop            ~2 KB
  └─ TOTAL: ~110 KB (Nucleo has 512 KB)
```

---

This architecture allows the Nucleo to function as a **standalone motor controller**
while maintaining compatibility with the existing PC control application.
