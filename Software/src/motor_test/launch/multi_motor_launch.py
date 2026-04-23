# Multi-motor step-test launch file — 2 to 12 motors over serial (nucleo bridge).
#
# Build: colcon build --packages-select motor_test && source install/setup.bash
# Run:   ros2 launch motor_test multi_motor_launch.py
#
# Pipeline (same as single_motor but N motors):
#   multi_motor_test → /robot_command → robot_command_bridge
#       → /motor_can_tx → serial_can_bridge (AT-frame binary)
#       → STM32 USART3 → CAN bus → RS04 motor feedback
#       → serial_can_bridge → /motor_can_feedback → motor_feedback_listener
#       → /motor_feedback → multi_motor_test (logs position)
#
# ── Configure these values before running ──────────────────────────────────────

# Serial port of the ST-Link VCP (Nucleo).
# Linux: /dev/ttyACM0  or  /dev/ttyUSB0
# Windows: COM6  (match platformio.ini monitor_port)
_SERIAL_PORT = '/dev/ttyACM0'
_BAUD_RATE   = 921600

# ── Motor configuration ────────────────────────────────────────────────────────
# RS04 motor CAN IDs. Default factory IDs are 1..N.
# Edit _MOTOR_IDS to match the IDs programmed on your motors.
# The list length determines how many motors are tested.
_MOTOR_IDS   = [1, 2]            # change to e.g. [1, 2, 3] for 3 motors
_JOINT_NAMES = [f'motor_{i}' for i in _MOTOR_IDS]  # or set custom names

# ── Motion settings ────────────────────────────────────────────────────────────
_TARGET_Q      = 0.4    # step amplitude [rad]
_KP            = 20.0   # position gain  [Nm/rad]
_KD            = 1.0    # damping gain   [Nm·s/rad]
_TAU_FF        = 0.0    # feedforward torque [Nm]
_STEP_DURATION = 4.0    # seconds per half-step
_RATE_HZ       = 50.0   # command publish rate [Hz]

# ── Node definitions ───────────────────────────────────────────────────────────

from launch import LaunchDescription
from launch_ros.actions import Node

_MOTOR_COUNT = len(_MOTOR_IDS)
_BASE_ID     = _MOTOR_IDS[0]   # first motor CAN ID; bridge assigns base, base+1, …


def generate_launch_description():
    # 1. multi_motor_test: publishes ±target_q step commands for all N joints
    multi_motor_test_node = Node(
        package='motor_test',
        executable='multi_motor_test.py',
        name='multi_motor_test',
        output='screen',
        parameters=[{
            'motor_count':    _MOTOR_COUNT,
            'joint_names':    ','.join(_JOINT_NAMES),
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

    # 2. robot_command_bridge: /robot_command → /motor_can_tx  (16 bytes × N joints)
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

    # 3. serial_can_bridge: /motor_can_tx → STM32 via USART3 (AT-frame binary)
    #    can_id_per_joint=True: chunk 0 → motor _BASE_ID, chunk 1 → _BASE_ID+1, …
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
            'can_id_per_joint': True,
            'can_id_base':      _BASE_ID,
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
            'motor_count':      _MOTOR_COUNT,
            'names_file':       'motor_names.json',
            'all_logging_info': False,
        }],
    )

    return LaunchDescription([
        multi_motor_test_node,
        robot_command_bridge_node,
        serial_can_bridge_node,
        motor_feedback_node,
    ])
