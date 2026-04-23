# Single-motor test launch file.
#
# Build: colcon build --packages-select motor_test && source install/setup.bash
# Run:   ros2 launch motor_test single_motor_launch.py
#
# Pipeline:
#   single_motor_test → /robot_command → robot_command_bridge
#       → /motor_can_tx → ethernet_can_bridge ↔ STM32 (UDP)
#       → /motor_can_feedback → motor_feedback_listener
#       → /motor_feedback → single_motor_test (logs position)
#
# Adjust the parameters below before running.

from launch import LaunchDescription
from launch_ros.actions import Node

# ── Hardware settings ──────────────────────────────────────────────────────────

# STM32 Ethernet settings
_STM32_IP   = '192.168.1.100'
_STM32_PORT = 7777

# CAN ID assigned to this motor by the STM32 (default first motor = 0x201)
_CAN_ID = 0x201

# Joint name — can be anything; used only for logging
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

    # 3. ethernet_can_bridge: /motor_can_tx → STM32 over UDP → /motor_can_feedback
    #    can_id_per_joint=False: all chunks use the same CAN ID (single motor mode)
    ethernet_can_bridge_node = Node(
        package='motor_test',
        executable='ethernet_can_bridge.py',
        name='ethernet_can_bridge',
        output='screen',
        parameters=[{
            'stm32_ip':         _STM32_IP,
            'stm32_port':       _STM32_PORT,
            'listen_port':      _STM32_PORT,
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
        ethernet_can_bridge_node,
        motor_feedback_node,
    ])
