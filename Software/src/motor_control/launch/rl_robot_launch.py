# Build: colcon build --packages-select motor_control && source install/setup.bash
#
# Full RL-to-hardware pipeline:
#   robot_command_bridge → ethernet_can_bridge ↔ STM32
#   motor_feedback_listener + STM32 IMU → robot_observation_bridge → RL policy
#   (IMU path is one-way STM32 -> ROS; no ROS IMU feedback to STM32)
#
# Adjust parameters below to match your hardware setup.

from launch import LaunchDescription
from launch_ros.actions import Node

# Ordered list of joint names – must match simulation/isaac/configuration/standing_pose.py
_JOINT_NAMES = [
    'pelvis_link_l_yaw_joint',
    'pelvis_link_r_yaw_joint',
    'l_hip_yaw_link_l_pitch_joint',
    'r_hip_yaw_link_r_pitch_joint',
    'l_hip_pitch_link_l_roll_joint',
    'r_hip_pitch_link_r_roll_joint',
    'l_thigh_link_l_knee_joint',
    'r_thigh_link_r_knee_joint',
    'l_shank_link_l_ankle_joint',
    'r_shank_link_r_ankle_joint',
    'l_ankle_link_l_foot_joint',
    'r_ankle_link_r_foot_joint',
]

_N_JOINTS = len(_JOINT_NAMES)

# CAN IDs follow joint-name order: first joint -> ID 1, second -> ID 2, ...
_CAN_ID_BASE = 1


def generate_launch_description():
    # 1. robot_command_bridge: /robot_command → /motor_can_tx
    robot_command_bridge_node = Node(
        package='motor_control',
        executable='robot_command_bridge.py',
        name='robot_command_bridge',
        output='screen',
        parameters=[{
            'command_topic': 'robot_command',
            'can_tx_topic':  'motor_can_tx',
            'enforce_joint_order': True,
            'strict_joint_names': True,
            'names_file': 'joint_limits_config.json',
            'all_logging_info': False,
        }],
    )

    # 2. ethernet_can_bridge: /motor_can_tx → STM32 (UDP) → /motor_can_feedback
    ethernet_can_bridge_node = Node(
        package='motor_control',
        executable='ethernet_can_bridge.py',
        name='ethernet_can_bridge',
        output='screen',
        parameters=[{
            'stm32_ip':         '192.168.1.100',
            'stm32_port':       7777,
            'listen_port':      7777,
            'command_topic':    'motor_can_tx',
            'feedback_topic':   'motor_can_feedback',
            'imu_topic':        'imu',
            'imu_frame_id':     'imu_link',
            'publish_stm32_imu': True,
            'can_id_per_joint': True,
            'can_id_base':      _CAN_ID_BASE,
            'all_logging_info': False,
        }],
    )

    # 3. motor_feedback_listener: /motor_can_feedback → /motor_feedback
    motor_feedback_node = Node(
        package='motor_control',
        executable='motor_feedback_listener.py',
        name='motor_feedback_listener',
        output='screen',
        parameters=[{
            'input':           'motor_can_feedback',
            'motor_count':     _N_JOINTS,
            'names_file':      'joint_limits_config.json',
            'all_logging_info': False,
        }],
    )

    # 4. robot_observation_bridge: /motor_feedback + /imu → /robot_observation
    robot_observation_bridge_node = Node(
        package='motor_control',
        executable='robot_observation_bridge.py',
        name='robot_observation_bridge',
        output='screen',
        parameters=[{
            'feedback_topic':    'motor_feedback',
            'imu_topic':         'imu',
            'observation_topic': 'robot_observation',
            'all_logging_info':  False,
        }],
    )

    return LaunchDescription([
        robot_command_bridge_node,
        ethernet_can_bridge_node,
        motor_feedback_node,
        robot_observation_bridge_node,
    ])
