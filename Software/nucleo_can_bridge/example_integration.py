"""
Integration Examples - Using PC Control with STM32 Motor Controller
=====================================================================

The new motor_controller.c firmware on the Nucleo F429ZI is fully compatible
with the existing pc_control_2motors.py script. No changes needed to the PC side!

Below are practical examples showing the complete workflow.
"""

# ═══════════════════════════════════════════════════════════════════════════
# EXAMPLE 1: Basic Motor Control
# ═══════════════════════════════════════════════════════════════════════════

from Software.nucleo_can_bridge.pc_control_2motors import DualMotorController
import time

def basic_control_example():
    """Demonstrates basic motor enable/disable and movement."""
    
    # Connect to Nucleo board running motor_controller.c firmware
    ctrl = DualMotorController(port="COM6", baud=921600, motor_ids=[127, 1])
    
    print("Connecting...")
    if not ctrl.connect():
        print("Failed to connect!")
        return
    
    print("Enabling motors...")
    ctrl.enable_all()
    time.sleep(0.5)
    
    # Move motor 1 (ID=127) to 90 degrees
    print("Motor 1 -> 90°")
    ctrl.goto_targets(90, [127])
    time.sleep(2)
    
    # Move motor 2 (ID=1) to -90 degrees  
    print("Motor 2 -> -90°")
    ctrl.goto_targets(-90, [1])
    time.sleep(2)
    
    # Both motors back to zero
    print("Both motors -> 0°")
    ctrl.goto_targets(0, [127, 1])
    time.sleep(2)
    
    print("Stopping...")
    ctrl.stop_all()
    ctrl.disconnect()


# ═══════════════════════════════════════════════════════════════════════════
# EXAMPLE 2: Jogging (Velocity Control)
# ═══════════════════════════════════════════════════════════════════════════

def jogging_example():
    """Demonstrates continuous jogging (velocity control)."""
    
    ctrl = DualMotorController(port="COM6")
    if not ctrl.connect():
        return
    
    ctrl.enable_all()
    time.sleep(0.5)
    
    # Jog motor 1 forward for 2 seconds
    print("Jog motor 1 forward...")
    ctrl.jog_targets(1, [127])  # direction=1 (positive/right)
    time.sleep(2)
    
    # Stop jog
    print("Stop jog...")
    ctrl.stop_targets([127])
    time.sleep(0.5)
    
    # Jog motor 1 backward
    print("Jog motor 1 backward...")
    ctrl.jog_targets(-1, [127])  # direction=-1 (negative/left)
    time.sleep(2)
    
    print("Stop...")
    ctrl.stop_all()
    ctrl.disconnect()


# ═══════════════════════════════════════════════════════════════════════════
# EXAMPLE 3: Zero Calibration
# ═══════════════════════════════════════════════════════════════════════════

def zero_calibration_example():
    """Demonstrates zeroing (setting current position as reference)."""
    
    ctrl = DualMotorController(port="COM6")
    if not ctrl.connect():
        return
    
    ctrl.enable_all()
    
    # Move to some position
    print("Move to 180°...")
    ctrl.goto_targets(180, [127, 1])
    time.sleep(2)
    
    # Set THAT as the new zero point
    print("Zero at 180°...")
    ctrl.zero_targets([127, 1])
    time.sleep(0.5)
    
    # Now 90° relative motion means go to -90° absolute
    print("Go to relative 90°...")
    ctrl.goto_targets(90, [127, 1])
    time.sleep(2)
    
    ctrl.disconnect()


# ═══════════════════════════════════════════════════════════════════════════
# EXAMPLE 4: Individual Motor Control (Motor Isolation)
# ═══════════════════════════════════════════════════════════════════════════

def individual_motor_example():
    """Shows how to control motors independently."""
    
    ctrl = DualMotorController(port="COM6")
    if not ctrl.connect():
        return
    
    # Enable both
    ctrl.enable_all()
    time.sleep(0.5)
    
    # Move motor 1 only
    print("Motor 1 -> 90°, Motor 2 stays still")
    ctrl.goto_targets(90, [127])
    time.sleep(2)
    
    # Move motor 2 only
    print("Motor 1 stays at 90°, Motor 2 -> -90°")
    ctrl.goto_targets(-90, [1])
    time.sleep(2)
    
    # Move both together
    print("Both motors to 0°")
    ctrl.goto_targets(0, [127, 1])
    time.sleep(2)
    
    ctrl.disconnect()


# ═══════════════════════════════════════════════════════════════════════════
# EXAMPLE 5: Monitoring Telemetry (Position, Velocity, Torque)
# ═══════════════════════════════════════════════════════════════════════════

def telemetry_monitoring_example():
    """Shows how to read real-time motor telemetry."""
    
    ctrl = DualMotorController(port="COM6")
    if not ctrl.connect():
        return
    
    ctrl.enable_all()
    time.sleep(0.5)
    
    # Start moving
    print("Moving motor 1...")
    ctrl.goto_targets(180, [127])
    
    # Monitor telemetry while moving
    for i in range(20):
        time.sleep(0.1)
        
        # Access telemetry dictionary
        m1_state = ctrl.telemetry[127]
        m2_state = ctrl.telemetry[1]
        
        print(f"M127: pos={m1_state['pos_deg']:7.2f}° vel={m1_state['vel']:6.2f}r/s "
              f"torq={m1_state['torq']:6.2f}Nm temp={m1_state['temp']:5.1f}°C")
        print(f"M001: pos={m2_state['pos_deg']:7.2f}° vel={m2_state['vel']:6.2f}r/s "
              f"torq={m2_state['torq']:6.2f}Nm temp={m2_state['temp']:5.1f}°C")
        print()
    
    ctrl.disconnect()


# ═══════════════════════════════════════════════════════════════════════════
# EXAMPLE 6: Sequence of Movements (Choreography)
# ═══════════════════════════════════════════════════════════════════════════

def choreography_example():
    """Example of programming a movement sequence."""
    
    ctrl = DualMotorController(port="COM6")
    if not ctrl.connect():
        return
    
    ctrl.enable_all()
    time.sleep(0.5)
    
    # Define a choreography sequence
    moves = [
        (0,   [127, 1],    "Both to center"),
        (90,  [127],       "M1 -> +90"),
        (-90, [1],         "M2 -> -90"),
        (0,   [127],       "M1 -> center"),
        (0,   [1],         "M2 -> center"),
        (180, [127, 1],    "Both to 180"),
        (-180, [127, 1],   "Both to -180"),
        (0,   [127, 1],    "Return to center"),
    ]
    
    for target_deg, motor_list, description in moves:
        print(f"{description}...")
        ctrl.goto_targets(target_deg, motor_list)
        time.sleep(1.5)  # Wait for motion to complete
    
    ctrl.disconnect()


# ═══════════════════════════════════════════════════════════════════════════
# EXAMPLE 7: Custom Control - Sine Wave Test
# ═══════════════════════════════════════════════════════════════════════════

import math

def sine_wave_test():
    """Moves motor in a smooth sine wave pattern."""
    
    ctrl = DualMotorController(port="COM6")
    if not ctrl.connect():
        return
    
    ctrl.enable_all()
    time.sleep(0.5)
    
    print("Sine wave test (2 cycles, 20 steps per cycle)...")
    for cycle in range(2):
        for step in range(40):
            # Generate sine wave: -90 to +90 degrees
            angle = 90.0 * math.sin(2 * math.pi * step / 40)
            
            ctrl.goto_targets(angle, [127])
            time.sleep(0.05)
            
            # Print position every 10 steps
            if step % 10 == 0:
                state = ctrl.telemetry[127]
                print(f"  Step {step:2d}: pos={state['pos_deg']:7.2f}° "
                      f"vel={state['vel']:6.2f}r/s")
    
    ctrl.goto_targets(0, [127])
    time.sleep(0.5)
    ctrl.disconnect()


# ═══════════════════════════════════════════════════════════════════════════
# HARDWARE SETUP
# ═══════════════════════════════════════════════════════════════════════════

"""
Before running these examples, ensure:

1. FIRMWARE UPLOAD:
   cd Software/nucleo_can_bridge/
   platformio run -e nucleo_f429zi --target upload

2. CONNECTIONS:
   - Nucleo board connected via USB to PC (COM6 by default)
   - CAN1_RX (PD0) and CAN1_TX (PD1) connected to motors or CAN bus
   - Optional: Serial monitor on COM6 at 921600 baud for debug messages

3. MOTOR IDS:
   - Motor 1: ID = 127
   - Motor 2: ID = 1
   (Change in pc_control_2motors.py MOTOR_IDS if different)

4. RUN EXAMPLES:
   python3 example_integration.py
   # Then call desired function:
   basic_control_example()
   jogging_example()
   etc.
"""


if __name__ == "__main__":
    # Uncomment the example you want to run:
    
    basic_control_example()
    # jogging_example()
    # zero_calibration_example()
    # individual_motor_example()
    # telemetry_monitoring_example()
    # choreography_example()
    # sine_wave_test()
