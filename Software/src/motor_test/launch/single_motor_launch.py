# Single-motor test launch file.
#
# Build: colcon build --packages-select motor_test && source install/setup.bash
# Run:   ros2 launch motor_test single_motor_launch.py
#
# Pipeline:
#   single_motor_test → /robot_command → robot_command_bridge
#       → /motor_can_tx → serial_can_bridge (AT-frame binary)
#       → STM32 USART3 → CAN bus → RS04 motor feedback
#       → motor_feedback_listener → /motor_feedback → single_motor_test
#
# Adjust the parameters below before running.

from launch import LaunchDescription
from launch_ros.actions import Node

# ── Hardware settings ──────────────────────────────────────────────────────────

# Serial port of the ST-Link VCP (Nucleo).
# Linux: /dev/ttyACM0  or  /dev/ttyUSB0
# Windows: COM6  (match platformio.ini monitor_port)
_USE_WINDOWS_SERIAL = False  # False -> /dev/ttyACM0, True -> COM6
_SERIAL_PORT = 'COM6' if _USE_WINDOWS_SERIAL else '/dev/ttyACM0'
_BAUD_RATE   = 921600

# RS04 motor CAN ID (default factory ID = 1).
_CAN_ID = 1

# Joint name — used only for logging
_JOINT_NAME = 'test_motor'

# ── Motion settings ────────────────────────────────────────────────────────────

_TARGET_Q      = 0.4    # amplitude of step test [rad]
_KP            = 20.0   # position gain  [Nm/rad]
_KD            = 1.0    # damping gain   [Nm·s/rad]
_TAU_FF        = 0.0    # feedforward torque [Nm]
_STEP_DURATION = 4.0    # seconds per half-step (motor holds, then reverses)
_RATE_HZ       = 50.0   # command publish rate

# ── Node definitions ───────────────────────────────────────────────────────────


def generate_launch_description():
    # 1. single_motor_test: publishes a ±target_q step command and logs feedback
    single_motor_test_node = Node(
        package='motor_test',
        executable='single_motor_test.py',
        name='single_motor_test',
        output='screen',
        parameters=[{
            'joint_name':     _JOINT_NAME,
            'target_q':       _TARGET_Q,
            'kp':             _KP,
            'kd':             _KD,
            'tau_ff':         _TAU_FF,
            'step_duration':  _STEP_DURATION,
            'rate_hz':        _RATE_HZ,
            'command_topic':  'robot_command',
            'feedback_topic': 'motor_feedback',
        }],
    )

    # 2. robot_command_bridge: /robot_command → /motor_can_tx (packed 16-byte payload)
    robot_command_bridge_node = Node(
        package='motor_test',
        executable='robot_command_bridge.py',
        name='robot_command_bridge',
        output='screen',
        parameters=[{
            'command_topic':    'robot_command',
            'can_tx_topic':     'motor_can_tx',
            'all_logging_info': False,
        }],
    )

    # 3. serial_can_bridge: /motor_can_tx → STM32 over USART3 (AT-frame binary)
    #    can_id_per_joint=False: all chunks use the same CAN ID (single motor mode)
    serial_can_bridge_node = Node(
        package='motor_test',
        executable='serial_can_bridge.py',
        name='serial_can_bridge',
        output='screen',
        parameters=[{
            'serial_port':      _SERIAL_PORT,
            'baud_rate':        _BAUD_RATE,
            'command_topic':    'motor_can_tx',
            'feedback_topic':   'motor_can_feedback',
            'can_id_per_joint': False,
            'can_id':           _CAN_ID,
            'all_logging_info': False,
        }],
    )

    # 4. motor_feedback_listener: /motor_can_feedback → /motor_feedback
    motor_feedback_node = Node(
        package='motor_test',
        executable='motor_feedback_listener.py',
        name='motor_feedback_listener',
        output='screen',
        parameters=[{
            'input':            'motor_can_feedback',
            'motor_count':      1,
            'names_file':       'motor_names.json',
            'all_logging_info': False,
        }],
    )

    return LaunchDescription([
        single_motor_test_node,
        robot_command_bridge_node,
        serial_can_bridge_node,
        motor_feedback_node,
    ])
