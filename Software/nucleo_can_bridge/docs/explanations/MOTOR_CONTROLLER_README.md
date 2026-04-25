# STM32 Motor Controller Firmware

## Overview

This is a complete motor controller implementation for RobStride motors running on the STM32F429ZI Nucleo board. It receives control commands via CAN and manages two motors with:

- **Enable/Disable control**
- **Position control** (point-to-point motion with acceleration limiting)
- **Velocity control** (jogging)  
- **Zero calibration**
- **Real-time telemetry feedback** (position, velocity, torque, temperature at 50ms intervals)

## Architecture

### Command Protocol

Commands are transmitted via CAN with 29-bit extended IDs structured as:

```
Extended ID = (comm_type << 24) | (sender_id << 8) | motor_id
```

Where:
- `comm_type`: Command type (2=status, 3=enable, 4=disable, 6=zero)
- `sender_id`: Sender ID (0xFD for host)
- `motor_id`: Target motor (127 or 1)

### Command Types

#### COMM_ENABLE (3)
Enables motor and sets it to position control mode.
```
CAN ID: 0x03FD7F (motor 127) or 0x03FD01 (motor 1)
Data: 8 bytes (any value, ignored)
```

#### COMM_DISABLE (4)
Disables motor immediately, stops all motion.
```
CAN ID: 0x04FD7F or 0x04FD01
Data: 8 bytes (any value)
```

#### COMM_SET_ZERO (6)
Sets current position as zero reference.
```
CAN ID: 0x06FD7F or 0x06FD01
Data[0]: 0x01 (enable), rest ignored
```

#### Parameter Write (0x10 or 0x00)
Sets motor control parameters.
```
Data[0:1]:  Parameter ID (little-endian uint16)
Data[2:5]:  Value (4 bytes, interpreted as float)
Data[6:7]:  Reserved
```

#### Telemetry (COMM_OPERATION_STATUS = 2)
Motor automatically broadcasts telemetry every 50ms.
```
CAN ID: 0x02FD7F (from motor 127) or 0x02FD01 (from motor 1)
Data[0:1]: Position      (uint16_t, big-endian)
Data[2:3]: Velocity      (uint16_t, big-endian)
Data[4:5]: Torque        (uint16_t, big-endian)
Data[6:7]: Temperature   (uint16_t, big-endian)

Encoding:
  float_value = (uint16_t - 32767) / 32767 * RANGE
  Where RANGE depends on parameter (see motor_controller.c)
```

### Parameter IDs

| ID     | Name                  | Units    | Function |
|--------|----------------------|----------|----------|
| 0x7005 | PARAM_MODE           | -        | Motor mode (0=disabled, 1=position, 7=velocity) |
| 0x7016 | PARAM_POSITION_TARGET| rad      | Target position for point-to-point motion |
| 0x7024 | PARAM_PP_SPEED_LIMIT | rad/s    | Maximum speed during point-to-point motion |
| 0x7025 | PARAM_PP_ACCEL       | rad/s²   | Acceleration limit for point-to-point motion |

## Motor States

### Position Control (MODE = 1)
- Motor moves toward `PARAM_POSITION_TARGET` using proportional control
- Velocity limited by `PARAM_PP_SPEED_LIMIT`
- Position range: ±2π radians

### Velocity Jog (MODE = 7)
- Motor spins at constant velocity set by `velocity_target`
- Used by PC control script with jog commands
- Velocity range: ±15 rad/s

### Disabled (MODE = 0)
- Motor holds (velocity = 0)
- Position frozen
- Can be re-enabled at any time

## Control Loop

The firmware runs a control loop at **1 kHz** via Timer 2 interrupt:

1. **Position Control**: P-controller regulates velocity toward position target
   - `desired_velocity = (target - current) * Kp`
   - Kp ≈ 2.0 × speed_limit (tunable)
   
2. **Velocity Jog**: Direct velocity command

3. **Telemetry**: Sent every 50ms via CAN

4. **Position Integration**: `position += velocity × dt`

## Building & Uploading

### Prerequisites
- PlatformIO (VSCode extension)
- STM32CubeMX packages (auto-installed by PlatformIO)
- Nucleo F429ZI board connected via USB

### Build
```bash
platformio run -e nucleo_f429zi
```

### Upload
```bash
platformio run -e nucleo_f429zi --target upload
```

**Note**: The Nucleo board appears as a USB mass-storage device. PlatformIO will auto-detect and upload the binary.

### Monitor Serial
```bash
platformio device monitor -e nucleo_f429zi --baud 921600
```

Or manually: Open COM6 at 921600 baud in terminal.

## Motor Configuration

Edit the following in `motor_controller.c` to match your setup:

```c
#define MOTOR_1_ID            127        /* ID of first motor */
#define MOTOR_2_ID            1          /* ID of second motor */
#define POS_RANGE             (4.0f * M_PI)  /* ±2π radians */
#define VEL_RANGE             15.0f      /* ±15 rad/s */
#define TORQUE_RANGE          120.0f     /* ±120 Nm */
#define TELEMETRY_PERIOD_MS   50         /* Feedback frequency */
```

## Pin Configuration

| Pin    | Function  | Notes |
|--------|-----------|-------|
| PD0    | CAN1_RX   | CAN bus receive |
| PD1    | CAN1_TX   | CAN bus transmit |
| PD8    | USART3_TX | Serial output (debug) |
| PD9    | USART3_RX | Serial input (commands) |
| PB0    | LED1      | Heartbeat (toggles every 500ms) |
| PB7    | LED2      | CAN activity indicator |
| PB14   | LED3      | CAN error indicator |

## PC Control Integration

Use the existing `pc_control_2motors.py` to control this firmware:

```python
from Software.nucleo_can_bridge.pc_control_2motors import DualMotorController

ctrl = DualMotorController(port="COM6", baud=921600, motor_ids=[127, 1])
if ctrl.connect():
    ctrl.enable_all()
    ctrl.goto_targets(90, [127])      # Move motor 127 to 90°
    ctrl.jog_targets(1, [1])          # Jog motor 1 clockwise
    time.sleep(2)
    ctrl.stop_all()
    ctrl.disconnect()
```

## Debugging

### Serial Output
The firmware prints status messages on USART3 (COM6, 921600 baud):
- `CMD: ENABLE motor 127` - Motor enabled
- `PARAM: POS_TARGET=180 motor 127` - Position command received
- `CAN ERR TEC=5 REC=0` - CAN bus errors (if any)

### LED Indicators
- **LED1 (PB0)**: Blinks every 500ms = firmware running
- **LED2 (PB7)**: Pulses when processing CAN frames
- **LED3 (PB14)**: On if CAN bus has errors

### Motor Simulation
The firmware internally simulates motor dynamics:
- Position integrates velocity over time: `pos += vel × 1ms`
- Velocity follows proportional control toward target
- Torque/temperature are synthetic but follow motor behavior

For real motor integration, replace the simulation with:
```c
/* In MotorController_ControlLoop() */
HAL_ADC_Start(&hadc1);  // Read actual position from encoder
motor->position = read_encoder(motor);
PWM_SetCommand(motor, motor->velocity);  // Drive motor
```

## Extending the Controller

### Adding PWM Output
```c
static void PWM_Init(void) {
    // Configure TIM3 on PA6/PA7 for PWM
    // Set up encoder input on TIM4
}

void PWM_SetCommand(MotorState_t *m, float velocity) {
    uint32_t pulse = (velocity / VEL_RANGE + 1.0f) * 50000;  // 0-100000
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}
```

### Adding Encoder Feedback
```c
float read_encoder(MotorState_t *m) {
    uint16_t count = TIM4->CNT;
    return (float)count * CPR_TO_RAD;  // Counts to radians
}
```

### Real Motor Control Loop
Replace the simulation in `MotorController_ControlLoop()`:
```c
if (m->enabled) {
    m->position = read_encoder(m);           // Actual position
    m->velocity = read_velocity_observer(m); // Actual velocity
    float cmd = compute_control_law(m);      // Compute command
    PWM_SetCommand(m, cmd);                   // Drive motor
}
```

## Troubleshooting

### "CAN init failed" message
- Check PD0/PD1 connections to CAN transceiver
- Verify 120Ω termination resistor on CAN bus
- Try different CAN baud rates in `CAN1_Init()`

### No telemetry received
- Verify CAN ID is correct (check LED2 pulses when sending commands)
- Check that motor is enabled before telemetry expected
- Monitor serial for "PARAM" messages to confirm commands received

### Motor doesn't move
- Confirm motor ID matches (127 or 1)
- Check if motor is enabled: `ctrl.enable_all()`
- Verify position target is within range: ±2π radians

## Performance Notes

- **Control loop**: 1 kHz (1ms resolution)
- **Telemetry**: 50 ms period (20 Hz bandwidth)
- **CAN bitrate**: 1 Mbps (configured in hardware)
- **Serial bitrate**: 921600 baud (commands/debug)

## References

- [STM32F429 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-417-427-429-437-439-and-stm32f469-479-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
- [PlatformIO Docs](https://docs.platformio.org/)
- PC control reference: [pc_control_2motors.py](./pc_control_2motors.py)

