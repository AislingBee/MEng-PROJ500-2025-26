# Terminal Commands Reference

All commands assume the working directory is the repo root (`c:\git\MEng-PROJ500-2025-26`) unless noted.

---

## Table of Contents

1. [STM32 Firmware (PlatformIO)](#1-stm32-firmware-platformio)
2. [ROS 2 — Build](#2-ros-2--build)
3. [ROS 2 — Launch Files](#3-ros-2--launch-files)
4. [ROS 2 — Inspect Topics at Runtime](#4-ros-2--inspect-topics-at-runtime)
5. [Manual Python Tests](#5-manual-python-tests)
6. [Serial Port Utilities](#6-serial-port-utilities)


---

## 1. STM32 Firmware (PlatformIO)

Run from `Software/nucleo_can_bridge/`.

```bash
# Navigate to firmware directory
cd Software/nucleo_can_bridge

# Build only (no upload)
pio run

# Build and flash via mass-storage (drag-drop / mbed protocol)
pio run -t upload

# Open serial monitor (921600 baud, COM6)
pio device monitor

# Open serial monitor on a specific port
pio device monitor --port COM8 --baud 921600

# List connected serial devices
pio device list

# Clean build artefacts
pio run -t clean
```

---

## 2. ROS 2 — Build

Run from a ROS 2 workspace root that contains the `Software/src` tree (or from wherever `src/` lives).

```bash
# Source ROS 2 base installation (do once per shell)
source /opt/ros/humble/setup.bash          # Linux
# (Windows: call C:\dev\ros2_humble\setup.bat)

# Build the motor_test package only
colcon build --packages-select motor_test

# Build with symlinks (avoids re-building when only Python files change)
colcon build --packages-select motor_test --symlink-install

# Source the local overlay after every build
source install/setup.bash                  # Linux / WSL
# install\setup.bat                        # Windows CMD
# . install/setup.ps1                      # Windows PowerShell

# Build everything
colcon build

# Run tests
colcon test --packages-select motor_test
colcon test-result --verbose
```

---

## 3. ROS 2 — Launch Files

All launch files live in `Software/src/motor_test/launch/`.  
**Remember to build and source before launching.**

### Single-motor test

```bash
ros2 launch motor_test single_motor_launch.py
```

Defaults: `serial_port=/dev/ttyACM0`, `motor_id=1`.

### Multi-motor test

```bash
ros2 launch motor_test multi_motor_launch.py
```

### Multi-state test (recommended for sequenced joint commands)

```bash
# Default (reads multi_state_test_config.json bundled with the package)
ros2 launch motor_test multi_state_launch.py

# Override serial port and motor count
ros2 launch motor_test multi_state_launch.py serial_port:=COM6 motor_count:=2

# Override with a custom config file
ros2 launch motor_test multi_state_launch.py \
    config_file:=/absolute/path/to/my_config.json

# Full parameter override example
ros2 launch motor_test multi_state_launch.py \
    serial_port:=/dev/ttyACM0 \
    baud_rate:=921600 \
    motor_count:=2 \
    kp:=50.0 \
    kd:=2.0 \
    tau_ff:=0.0 \
    state_duration:=3.0 \
    rate_hz:=50.0
```

### RL-to-hardware pipeline

```bash
ros2 launch motor_test rl_robot_launch.py
```

Edit joint names and serial port at the top of the file before running.

---

## 4. ROS 2 — Inspect Topics at Runtime

```bash
# List all active topics
ros2 topic list

# Echo robot commands being published
ros2 topic echo /robot_command

# Echo motor feedback
ros2 topic echo /motor_feedback

# Check publish rate of a topic
ros2 topic hz /motor_feedback

# Show topic type
ros2 topic info /robot_command

# List running nodes
ros2 node list

# Show node parameters
ros2 param list /multi_state_motor_test
ros2 param get  /multi_state_motor_test kp
ros2 param set  /multi_state_motor_test kp 60.0
```

---

## 5. Manual Python Tests

Run from `Software/nucleo_can_bridge/`.  
Requires `pyserial`: `pip install pyserial`.

### Bridge ID scanner

```bash
python tests/manual/test_bridge.py

# With explicit options
python tests/manual/test_bridge.py --port COM6 --baud 921600 --max-id 127 --verbose

# Linux
python tests/manual/test_bridge.py --port /dev/ttyACM0 --verbose
```

Arguments:

| Flag | Default | Description |
|------|---------|-------------|
| `--port` | `COM6` | Serial port of Nucleo |
| `--baud` | `921600` | Baud rate |
| `--max-id` | `127` | Highest CAN ID to probe |
| `--verbose` | off | Print raw frame bytes |

### Boot diagnostics

```bash
# Edit COM_PORT at top of file if needed (default COM8), then:
python tests/manual/test_boot.py
```

### Quick motor jog

```bash
# Edit COM_PORT / MOTOR_ID at top of file if needed (default COM8 / ID 127), then:
python tests/manual/test_motor.py
```

---

## 6. Serial Port Utilities

### Windows — find Nucleo COM port

```powershell
# List all COM ports
Get-WmiObject Win32_SerialPort | Select-Object Name, DeviceID, Description

# Or via mode command
mode
```

### Linux — find Nucleo device

```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*

# Detailed USB device info
dmesg | grep -i tty | tail -20

# Grant permission (if access denied)
sudo usermod -aG dialout $USER   # log out and back in
# or one-shot:
sudo chmod 666 /dev/ttyACM0
```

### Quick serial read (Python one-liner)

```bash
python -c "import serial, time; s=serial.Serial('COM6',921600,timeout=2); time.sleep(0.5); print(s.read_all())"
```

---