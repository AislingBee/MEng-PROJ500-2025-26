# ROS → STM32 Motor Controller Integration

## Overview

The ROS code now interfaces with the STM32 motor controller firmware using **PD control** with dynamic gain tuning from ROS.

## Connection Flow

```
ROS2 Application
    │
    ├─ Publishes to /motor_can_tx topic
    │  Format: UInt8MultiArray with 16-byte chunks [q, kp, kd, tau]
    │
    ▼
SerialCanBridge ROS Node
    │
    ├─ Converts position command (q) to STM32 protocol
    ├─ Sends Kp (proportional gain) parameter
    ├─ Sends Kd (damping gain) parameter
    ├─ Sends tau (feedforward torque) parameter
    ├─ Sends position target (radians)
    ├─ Sends AT-frame via serial @ 921600 baud
    │
    ▼
STM32 Motor Controller
    │
    ├─ Receives AT-frame over USART3
    ├─ Parses position target (radians)
    ├─ Updates motor state via 1kHz control loop
    ├─ Sends telemetry every 50ms
    │
    ▼
SerialCanBridge ROS Node
    │
    ├─ Receives telemetry AT-frame
    ├─ Decodes position (q) and velocity (q_dot)
    ├─ Publishes to /motor_can_feedback topic
    │  Format: UInt8MultiArray with 2 floats [q, q_dot]
    │
    ▼
ROS2 Application
    └─ Subscribes to /motor_can_feedback
       Reads position and velocity feedback
```

---

## Command Protocol

### 1. First Motor Enable

**ROS sends:**
```python
command = [q, kp, kd, tau]  # 4 floats = 16 bytes
publisher.publish(UInt8MultiArray(data=command))
```

**ROS bridge does:**
1. Sends ENABLE frame (COMM_ENABLE = 0x03)
2. Marks motor as enabled

**STM32 firmware receives:**
```
CAN ID: 0x03FD{motor_id}
Data: 8 bytes (any value, ignored)
Action: Sets motor->enabled = 1, mode = POSITION_CONTROL
```

### 2. Position Target Command

**ROS sends:**
```python
command = [90.0 * pi/180, 50.0, 0.5, 0.0]  # q, kp, kd, tau
publisher.publish(UInt8MultiArray(data=command))
```

**ROS bridge converts:**
- Sends PARAM_KP (0x7014) = 50.0
- Sends PARAM_KD (0x7015) = 0.5
- Sends PARAM_FEEDFORWARD_TORQUE (0x7026) = 0.0
- Sends PARAM_POSITION_TARGET (0x7016) = 1.5708 rad

**STM32 firmware receives:**
```
Parameter ID: 0x7014  Value: 50.0 (Kp)
Parameter ID: 0x7015  Value: 0.5  (Kd)
Parameter ID: 0x7026  Value: 0.0  (tau_feedforward)
Parameter ID: 0x7016  Value: 1.5708 (position_target)

Action: Control loop computes:
  error = position_target - current_position
  velocity_error = -current_velocity
  command_velocity = kp * error + kd * velocity_error
  torque = kp * error * 0.1 + feedforward_torque
```

### 3. Telemetry Feedback

**STM32 firmware sends (every 50ms):**
```
CAN ID: 0x02FD{motor_id}  (COMM_OPERATION_STATUS)
Data (8 bytes):
  [0:1] Position (big-endian uint16)
  [2:3] Velocity (big-endian uint16)
  [4:5] Torque (big-endian uint16)
  [6:7] Temperature (big-endian uint16)
```

**ROS bridge receives:**
1. Parses AT-frame
2. Decodes 4x uint16 values
3. Converts to float: `value = (uint16/32767 - 1) * range`
4. Publishes [q, q_dot] as 2 floats

**ROS application receives:**
```python
def feedback_callback(msg):
    q, q_dot = struct.unpack('<ff', bytes(msg.data))
    print(f"Position: {q:.4f} rad, Velocity: {q_dot:.4f} rad/s")
```

---

## Parameter Mapping

| ROS Command | STM32 Parameter | Effect |
|-------------|-----------------|--------|
| **q** (position) | PARAM_POSITION_TARGET (0x7016) | Motor moves to this angle |
| **kp** (gain) | PARAM_KP (0x7014) | Proportional control gain [0, 500] |
| **kd** (damping) | PARAM_KD (0x7015) | Damping gain [0, 10] |
| **tau** (torque) | PARAM_FEEDFORWARD_TORQUE (0x7026) | Feedforward torque [Nm] |

---

## Configuration

In ROS launch file, set motor parameters:

```yaml
serial_can_bridge:
  ros__parameters:
    serial_port: /dev/ttyACM0        # or COM6 on Windows
    baud_rate: 921600
    command_topic: motor_can_tx       # ROS publishes here
    feedback_topic: motor_can_feedback # ROS subscribes here
    can_id_per_joint: true            # Use different IDs per motor
    can_id_base: 127                  # First motor ID
```

---

## Changes Made to ROS Code

### 1. **Protocol Constants**
```python
# OLD (RS04 MIT)
_COMM_MOTION_CONTROL = 0x01
_COMM_MOTOR_FEEDBACK = 0x02
_P_MIN, _P_MAX = -12.5, 12.5

# NEW (STM32 with PD Control)
_COMM_ENABLE = 3
_COMM_OPERATION_STATUS = 2
_PARAM_KP = 0x7014
_PARAM_KD = 0x7015
_PARAM_POSITION_TARGET = 0x7016
_PARAM_FEEDFORWARD_TORQUE = 0x7026
_POS_RANGE = 4.0 * math.pi  # ±2π radians
```

### 2. **Command Functions**
```python
# NEW frame builders
def _kp_frame(motor_id, kp)
def _kd_frame(motor_id, kd)
def _feedforward_torque_frame(motor_id, tau)
def _position_target_frame(motor_id, position_rad)
```

### 3. **Enable Sequence**
```python
# OLD
mode_frame, enable_frame = _enable_frames(motor_id)  # 2 frames

# NEW
enable_frame = _enable_frame(motor_id)  # 1 frame
```

### 4. **Command Callback**
```python
# OLD
self._serial_write(_motion_frame(motor_id, q, kp, kd, tau))

# NEW
if kp > 0:
    self._serial_write(_kp_frame(motor_id, kp))
if kd > 0:
    self._serial_write(_kd_frame(motor_id, kd))
if tau != 0:
    self._serial_write(_feedforward_torque_frame(motor_id, tau))
self._serial_write(_position_target_frame(motor_id, q))
```

### 5. **Feedback Parsing**
```python
# Same as before
if comm_type == _COMM_OPERATION_STATUS:
    q = _u16_to_float(pos_u16, -_POS_RANGE/2, _POS_RANGE/2)
```

---

## Testing the Integration

### 1. **Start STM32 Firmware**
```bash
cd Software/nucleo_can_bridge
python -m platformio run -e nucleo_f429zi --target upload
```

### 2. **Start ROS Bridge**
```bash
ros2 launch motor_test serial_can_bridge.launch.yaml
```

You should see:
```
SerialCanBridge ready: /dev/ttyACM0@921600 cmd="motor_can_tx" fbk="motor_can_feedback"
```

### 3. **Send Motor Command from ROS**
```bash
# In another terminal
ros2 topic pub /motor_can_tx std_msgs/msg/UInt8MultiArray \
  "data: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0]"
# That's 1.0 rad as little-endian float
```

### 4. **Monitor Feedback**
```bash
ros2 topic echo /motor_can_feedback
```

---

## Limitations (STM32 vs RS04)

| Feature | STM32 | RS04 | Notes |
|---------|-------|------|-------|
| Position control | ✅ | ✅ | Both support it |
| Position gain (Kp) | ✅ Tunable | ✅ Tunable | Both allow dynamic Kp |
| Damping (Kd) | ✅ Yes | ✅ Yes | STM32 now supports PD control |
| Feed-forward torque | ✅ Yes | ✅ Yes | Both support feedforward |
| Velocity range | ±15 rad/s | ±44 rad/s | STM32 has lower max velocity |
| Position range | ±2π rad | ±12.5 rad | STM32 has narrower range |
| Motors connected | 🚫 None (simulation) | ✅ Real motors | Need to add motor driver code |

---

## Next Steps

### To Use with Real Motors

1. Add encoder input to STM32 (read actual position)
2. Add PWM output to STM32 (drive motor)
3. Replace motor simulation in `MotorController_ControlLoop()`
4. Fine-tune proportional gain

See `Software/nucleo_can_bridge/docs/explanations/MOTOR_CONTROLLER_README.md` → "Extending to Real Motors" for code examples.

### To Tune Position Control

Edit in `motor_config.h`:
```c
.pp_speed_limit = 10.0f,  // Higher = faster response
.pp_accel = 10.0f,        // Not a hard limit, just parameter
```

The control gain is: `Kp = pp_speed_limit × 2.0`

---

## Backward Compatibility

The ROS bridge code maintains the same topics and message format:
- ✅ **Command topic**: Still expects 16-byte chunks [q, kp, kd, tau]
- ✅ **Feedback topic**: Still publishes 2 floats [q, q_dot]
- ✅ **Behavior change**: kp/kd/tau now used in firmware (not ignored!)

Your higher-level ROS code doesn't need changes! All parameters are now effective. 🎉

---

## File Changes

| File | Change | Reason |
|------|--------|--------|
| `serial_can_bridge.py` | Protocol constants updated | Use STM32 command types |
| `serial_can_bridge.py` | `_position_target_frame()` | New function for position commands |
| `serial_can_bridge.py` | `_enable_frame()` simplified | Only one enable frame needed |
| `serial_can_bridge.py` | `command_callback()` updated | Send position instead of MIT motion |
| `serial_can_bridge.py` | `_handle_can_frame()` updated | Parse STM32 telemetry format |
| `serial_can_bridge.py` | Removed `_f2u()`, `_u2f()` | Use new `_u16_to_float()` |

---

## Summary

✅ ROS code now compatible with STM32 motor controller  
✅ **Full dynamic tuning**: Kp, Kd, and tau all supported and adjustable from ROS  
✅ PD control with damping for smoother motion  
✅ Feedforward torque for gravity compensation or force control  
✅ Same message topics and format for your application  
✅ Ready to test with motor simulation  
✅ Ready to extend to real motors with encoder/PWM  

**Your ROS application can remain unchanged!** All parameters are now effective. 🎉

