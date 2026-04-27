# Header File Organization

## Overview

The motor controller firmware is now organized with proper separation of concerns across header files:

```
Include Files Structure:
├── motor_types.h           (NEW) Core types & command definitions
├── motor_config.h          (NEW) Motor parameter configurations  
├── motor_controller.h      (UPDATED) Public API
└── stm32f4xx_hal.h         (STM32 HAL)

Source Files:
└── motor_controller.c      Core implementation (simplified)
```

---

## File Breakdown

### **motor_types.h** (NEW)
**Purpose**: Core data structures and command/parameter constants

**Contains**:
- `MotorState_t` structure - Complete motor runtime state
- `COMM_*` defines - CAN command type codes (ENABLE, DISABLE, SET_ZERO)
- `PARAM_*` defines - Motor parameter IDs (MODE, POSITION_TARGET, etc.)
- `MODE_*` defines - Motor control modes (DISABLED, POSITION_CONTROL, VELOCITY_JOG)
- `TEMP_SCALE` constant

**Who includes it**:
- `motor_controller.h` → `motor_types.h`
- `motor_controller.c` → `motor_types.h`
- Any code that needs `MotorState_t` or command constants


### **motor_config.h** (EXISTING)
**Purpose**: Motor parameter configurations

**Contains**:
- `MotorConfig_t` structure - Per-motor tuning parameters
- `MOTOR1_CONFIG`, `MOTOR2_CONFIG` - Predefined configs
- `MOTOR_CONFIGS[]` array - All configured motors
- `NUM_MOTOR_CONFIGS` - Auto-calculated count
- Helper functions: `MotorConfig_Get()`, `MotorConfig_GetCount()`
- `HOST_ID`, `TELEMETRY_PERIOD_MS`, `CONTROL_LOOP_FREQ_HZ` globals

**Who includes it**:
- `motor_types.h` → `motor_config.h`
- `motor_controller.c` → (via motor_types.h)

### **motor_controller.h** (UPDATED)
**Purpose**: Public API for motor controller

**Contains**:
- `#include "motor_types.h"` - Pulls in all types
- Public function declarations:
  - `MotorController_Init()`
  - `MotorController_GetMotor()`
  - `MotorController_ProcessCommand()`
  - `MotorController_ControlLoop()`

**Who includes it**:
- `motor_controller.c` - Implementation
- External code that uses motor controller (like tests)

### **motor_controller.c** (SIMPLIFIED)
**Purpose**: Core implementation

**Contains**:
- `#include "motor_controller.h"` + `#include "motor_types.h"` (now pulled in)
- Static HAL handles (CAN, UART, Timer)
- Static motor array initialized from configs
- Implementation of all public functions
- Internal helper functions (SendCanFrame, ProcessSerialData, etc.)
- ISR handlers

---

## Include Dependency Graph

```
motor_controller.c
    ↓
motor_controller.h
    ↓
motor_types.h
    ↓
motor_config.h
    ↓
motor_config.h specific types (MotorConfig_t)
```

## Adding New Motors

To add Motor 3, you only edit **motor_config.h**:

```c
static const MotorConfig_t MOTOR3_CONFIG = {
    .motor_id = 3,
    .pp_speed_limit = 10.0f,
    .pp_accel = 10.0f,
    .position_range = 2.0f * M_PI,
    .velocity_range = 15.0f,
    .torque_range = 120.0f,
    .enabled_on_startup = 0,
    .initial_position = 0.0f,
    .initial_velocity = 0.0f,
};

static const MotorConfig_t MOTOR_CONFIGS[] = {
    MOTOR1_CONFIG,
    MOTOR2_CONFIG,
    MOTOR3_CONFIG,  // NEW
};
```

## Modifying Command Types

To add a new CAN command (hypothetically), you edit **motor_types.h**:

```c
#define COMM_NEW_COMMAND   7  // New command type

#define PARAM_NEW_PARAM    0x7050  // New parameter
```

Then use in `motor_controller.c`:

```c
else if (comm_type == COMM_NEW_COMMAND) {
    // Handle new command
}
```

## Best Practices

### DO:
- Include `motor_controller.h` when using the public API
- Edit `motor_config.h` to add motors
- Edit `motor_types.h` to change command types
- Keep internal functions in `motor_controller.c`

### DON'T:
- Include `motor_types.h` directly if you can use `motor_controller.h`
- Modify `motor_controller.c` to add motors
- Add motor configs in `motor_controller.c`
- Change public API without updating `.h` files

## Testing

The firmware should compile with no changes:

```bash
cd Software/nucleo_can_bridge
python -m platformio run -e nucleo_f429zi
```
