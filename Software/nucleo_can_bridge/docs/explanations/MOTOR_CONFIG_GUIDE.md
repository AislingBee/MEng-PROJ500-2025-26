# Motor Configuration Guide

## Overview

The motor controller now uses **configuration files** instead of hardcoded values. This makes it easy to:
- ✅ Add/remove motors without modifying C code
- ✅ Change motor parameters (speed limits, acceleration, etc.)
- ✅ Scale from 2 motors to many motors
- ✅ Use different motor types with different parameters

## How It Works

### Configuration Architecture

```
motor_config.h
├── MotorConfig_t structure (defines parameters for ONE motor)
├── MOTOR1_CONFIG (predefined config for motor ID 127)
├── MOTOR2_CONFIG (predefined config for motor ID 1)
└── MOTOR_CONFIGS[] array (all motor configs in one place)

motor_controller.c
├── MotorController_Init() - Reads MOTOR_CONFIGS array
├── MotorState_t motors[] - Runtime state (sized to NUM_MOTOR_CONFIGS)
└── All motor parameters come from their config
```

### Adding a New Motor (3 Steps)

#### Step 1: Create a Configuration

In `include/motor_config.h`, add a new configuration struct:

```c
/**
 * Configuration for Motor 3 (ID = 2)
 * Custom settings for this motor
 */
static const MotorConfig_t MOTOR3_CONFIG = {
    .motor_id = 2,
    .pp_speed_limit = 8.0f,           /* Slower than default */
    .pp_accel = 5.0f,                 /* Less aggressive acceleration */
    .position_range = 2.0f * M_PI,    /* Standard ±2π */
    .velocity_range = 10.0f,          /* Lower max velocity */
    .torque_range = 100.0f,           /* Lower torque */
    .enabled_on_startup = 0,          /* Start disabled */
    .initial_position = 0.0f,
    .initial_velocity = 0.0f,
};
```

#### Step 2: Add to Configuration Array

In `include/motor_config.h`, add your config to the `MOTOR_CONFIGS` array:

```c
static const MotorConfig_t MOTOR_CONFIGS[] = {
    MOTOR1_CONFIG,
    MOTOR2_CONFIG,
    MOTOR3_CONFIG,  // ← ADD YOUR NEW MOTOR HERE
};
```

#### Step 3: Done!

That's it! The firmware automatically:
-  Allocates motor state for the new motor
-  Initializes it with your parameters
-  Starts accepting commands for it
-  Sends telemetry for it

---

## Configuration Parameters

### MotorConfig_t Structure

```c
typedef struct {
    uint8_t motor_id;                  /* CAN ID of motor (0-255) */
    
    float pp_speed_limit;              /* Max speed during point-to-point (rad/s) */
    float pp_accel;                    /* Acceleration limit (rad/s²) */
    float position_range;              /* Valid motion range: ±position_range (rad) */
    float velocity_range;              /* Max velocity (rad/s) */
    float torque_range;                /* Max torque for telemetry scaling (Nm) */
    
    uint8_t enabled_on_startup;        /* 1 = auto-enable, 0 = start disabled */
    float initial_position;            /* Starting position (rad) */
    float initial_velocity;            /* Starting velocity (rad/s) */
} MotorConfig_t;
```

### Parameter Explanations

| Parameter | Purpose | Example |
|-----------|---------|---------|
| `motor_id` | CAN identifier (must be unique) | 127, 1, 2, 3... |
| `pp_speed_limit` | Max speed for point-to-point motion | 10.0 rad/s ≈ 95 rpm |
| `pp_accel` | Acceleration limit | 10.0 rad/s² (not a hard limit, P-control based) |
| `position_range` | Clamp position to ±this value | 2π rad (±360°) or π rad (±180°) |
| `velocity_range` | Used for telemetry scaling | 15.0 rad/s (matching motor capability) |
| `torque_range` | Used for telemetry scaling | 120.0 Nm (matching motor rating) |
| `enabled_on_startup` | Auto-enable on power-up | 0 (normally start disabled) |
| `initial_position` | Where motor starts | 0.0 rad (at zero) |
| `initial_velocity` | Initial velocity | 0.0 rad/s (stationary) |

---

## Common Scenarios

### Scenario 1: Standard Motors (Like You Have)

```c
static const MotorConfig_t MOTOR1_CONFIG = {
    .motor_id = 127,
    .pp_speed_limit = 10.0f,
    .pp_accel = 10.0f,
    .position_range = 2.0f * M_PI,    /* ±360° */
    .velocity_range = 15.0f,
    .torque_range = 120.0f,
    .enabled_on_startup = 0,
    .initial_position = 0.0f,
    .initial_velocity = 0.0f,
};
```

### Scenario 2: Slow, Precise Motor (Low Speed, High Accuracy)

```c
static const MotorConfig_t MOTOR_SLOW_CONFIG = {
    .motor_id = 10,
    .pp_speed_limit = 2.0f,           /* Very slow */
    .pp_accel = 1.0f,                 /* Gentle acceleration */
    .position_range = 1.0f * M_PI,    /* Only ±180° */
    .velocity_range = 3.0f,           /* Low max velocity */
    .torque_range = 50.0f,            /* Lower torque */
    .enabled_on_startup = 0,
    .initial_position = 0.0f,
    .initial_velocity = 0.0f,
};
```

### Scenario 3: Fast Motor (High Speed, Less Precision)

```c
static const MotorConfig_t MOTOR_FAST_CONFIG = {
    .motor_id = 20,
    .pp_speed_limit = 30.0f,          /* Very fast */
    .pp_accel = 50.0f,                /* Aggressive acceleration */
    .position_range = 4.0f * M_PI,    /* Full ±720° */
    .velocity_range = 50.0f,          /* High max velocity */
    .torque_range = 200.0f,           /* High torque */
    .enabled_on_startup = 0,
    .initial_position = 0.0f,
    .initial_velocity = 0.0f,
};
```

### Scenario 4: Many Motors (Scalable System)

```c
/* Define configs for 8 motors */
static const MotorConfig_t MOTOR1_CONFIG  = { .motor_id = 1, ... };
static const MotorConfig_t MOTOR2_CONFIG  = { .motor_id = 2, ... };
static const MotorConfig_t MOTOR3_CONFIG  = { .motor_id = 3, ... };
static const MotorConfig_t MOTOR4_CONFIG  = { .motor_id = 4, ... };
static const MotorConfig_t MOTOR5_CONFIG  = { .motor_id = 5, ... };
static const MotorConfig_t MOTOR6_CONFIG  = { .motor_id = 6, ... };
static const MotorConfig_t MOTOR7_CONFIG  = { .motor_id = 7, ... };
static const MotorConfig_t MOTOR8_CONFIG  = { .motor_id = 8, ... };

static const MotorConfig_t MOTOR_CONFIGS[] = {
    MOTOR1_CONFIG,
    MOTOR2_CONFIG,
    MOTOR3_CONFIG,
    MOTOR4_CONFIG,
    MOTOR5_CONFIG,
    MOTOR6_CONFIG,
    MOTOR7_CONFIG,
    MOTOR8_CONFIG,
};

/* Firmware automatically:
   - Allocates 8 motor states
   - Initializes 8 CAN IDs
   - Manages 8 telemetry streams
   - No code changes needed!
*/
```

---

## Tuning Parameters

### Proportional Gain (P)

The control loop uses: `velocity = Kp × (target - actual)`

Where: `Kp = pp_speed_limit × 2.0`

This is tuned automatically based on `pp_speed_limit`:
- Higher `pp_speed_limit` → Faster response (but may oscillate)
- Lower `pp_speed_limit` → Slower response (but more stable)

**To tune:** Adjust `pp_speed_limit` and rebuild

```c
.pp_speed_limit = 10.0f,  /* Kp = 20.0 */
/* vs */
.pp_speed_limit = 5.0f,   /* Kp = 10.0 (half the gain) */
```

### Velocity Limiting

The `pp_accel` parameter is not a hard acceleration limit but a control parameter:

```c
/* In control loop */
float cmd_vel = error * kp;  /* Computed velocity command */
if (cmd_vel > pp_speed_limit) cmd_vel = pp_speed_limit;  /* Hard limit */
```

**To get smooth acceleration:** Lower `pp_speed_limit` (proportional control handles it)

---

## Runtime Behavior

### Automatic Motor Discovery

On startup, the firmware:
1. Reads `NUM_MOTOR_CONFIGS` from `MOTOR_CONFIGS` array size
2. Allocates `motors[]` array with that many entries
3. Initializes each motor from its config
4. Prints: `Motors: 127, 1, 2` (all configured motor IDs)

```
=== RobStride Motor Controller v1 ===
Nucleo F429ZI
Motors: 127, 1, 2, 5, 10        ← Shows all configured motors
```

### Per-Motor Isolation

Each motor maintains its own state and parameters:

```c
MotorState_t *motor = MotorController_GetMotor(motor_id);
if (motor) {
    float target = motor->position_target;
    float limit = motor->pp_speed_limit;
    /* Use motor-specific config values */
}
```

## PC Control Integration

The PC control script works **automatically** with the new configuration:

```python
from Software.nucleo_can_bridge.pc_control_2motors import DualMotorController

# Works with any motors in MOTOR_CONFIGS array
ctrl = DualMotorController(port="COM6")
ctrl.connect()

# Send commands to any motor ID
ctrl.enable_targets([1, 2, 5, 10])      # All configured motors
ctrl.goto_targets(90, [2])              # Motor 2 only
ctrl.jog_targets(1, [5])                # Motor 5 only
```

---

## Example: Adding Motor 3

**Before (just 2 motors):**
```c
/* motor_config.h */
static const MotorConfig_t MOTOR1_CONFIG = { .motor_id = 127, ... };
static const MotorConfig_t MOTOR2_CONFIG = { .motor_id = 1, ... };

static const MotorConfig_t MOTOR_CONFIGS[] = {
    MOTOR1_CONFIG,
    MOTOR2_CONFIG,
};
```

**After (3 motors):**
```c
/* motor_config.h */
static const MotorConfig_t MOTOR1_CONFIG = { .motor_id = 127, ... };
static const MotorConfig_t MOTOR2_CONFIG = { .motor_id = 1, ... };
static const MotorConfig_t MOTOR3_CONFIG = { .motor_id = 3, ... };  // NEW

static const MotorConfig_t MOTOR_CONFIGS[] = {
    MOTOR1_CONFIG,
    MOTOR2_CONFIG,
    MOTOR3_CONFIG,  // NEW
};
```
---

## Troubleshooting

### Motor doesn't respond
- Check `motor_id` in config matches CAN commands
- Verify motor ID is in `MOTOR_CONFIGS` array
- Check serial output: `Motors: ...` should list your ID

### Wrong parameters applied
- Verify config values in `motor_config.h`
- Check which config is actually used (wrong array index?)
- Rebuild firmware after editing `motor_config.h`

### Firmware won't compile
- Check for syntax errors in new config struct
- Verify all fields are initialized (no missing commas)
- Make sure motor IDs are unique

### Motor parameters changing unexpectedly
- Confirm you edited the correct config struct
- Make sure you added to `MOTOR_CONFIGS` array
- Check that firmware was rebuilt and uploaded

---

## What's Next?

1. **Test it:** Build and upload with existing 2 motors
2. **Add a motor:** Follow the 3-step process above
3. **Tune parameters:** Adjust `pp_speed_limit` for your motors
4. **Scale up:** Add as many motors as needed


