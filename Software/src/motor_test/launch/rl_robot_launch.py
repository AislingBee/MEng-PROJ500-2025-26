# Build and source before launching:
#   colcon build --packages-select motor_test
#   source install/setup.bash
#
# This launch file starts the full RL-to-hardware pipeline:
#
#   Ros2RobotBridge (simulation side)
#       │  /robot_command  ──►  robot_command_bridge
#       │                              │  /motor_can_tx
#       │                              ▼
#       │                      ethernet_can_bridge  ◄──►  STM32
#       │                              │  /motor_can_feedback
#       │                              ▼
#       │                      motor_feedback_listener ──► /motor_feedback
#       │                                                        │  (+/imu)
#       │  /robot_observation  ◄──  robot_observation_bridge ◄──┘
#       ▼
#   RL policy reads ObservationPacket
#
# Adjust the parameters below to match your hardware setup.

from launch import LaunchDescription
from launch_ros.actions import Node

# Ordered list of joint names – must match simulation/isaac/configuration/standing_pose.py
_JOINT_NAMES = [
    'robot_pelvis_link_l_yaw_joint',
    'robot_pelvis_link_r_yaw_joint',
    'robot_l_hip_yaw_link_l_pitch_joint',
    'robot_r_hip_yaw_link_r_pitch_joint',
    'robot_l_hip_pitch_link_l_roll_joint',
    'robot_r_hip_pitch_link_r_roll_joint',
    'robot_l_thigh_link_l_knee_joint',
    'robot_r_thigh_link_r_knee_joint',
    'robot_l_shank_link_l_ankle_joint',
    'robot_r_shank_link_r_ankle_joint',
    'robot_l_ankle_link_l_foot_joint',
    'robot_r_ankle_link_r_foot_joint',
]

_N_JOINTS = len(_JOINT_NAMES)

# STM32 assigns CAN IDs 0x201 … 0x201+N-1 per joint (one ID per joint).
_CAN_ID_BASE = 0x201


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1. robot_command_bridge
    #    Converts /robot_command (RobotCommand) → /motor_can_tx (UInt8MultiArray)
    # ------------------------------------------------------------------
    robot_command_bridge_node = Node(
        package='motor_test',
        executable='robot_command_bridge.py',
        name='robot_command_bridge',
        output='screen',
        parameters=[{
            'command_topic': 'robot_command',
            'can_tx_topic':  'motor_can_tx',
            'all_logging_info': False,
        }],
    )

    # ------------------------------------------------------------------
    # 2. ethernet_can_bridge
    #    Forwards /motor_can_tx bytes to STM32 over UDP; receives FBK lines
    #    and publishes raw CAN feedback on /motor_can_feedback.
    #
    #    can_id_per_joint=True: joint 0 → 0x201, joint 1 → 0x202, …
    # ------------------------------------------------------------------
    ethernet_can_bridge_node = Node(
        package='motor_test',
        executable='ethernet_can_bridge.py',
        name='ethernet_can_bridge',
        output='screen',
        parameters=[{
            'stm32_ip':         '192.168.1.100',
            'stm32_port':       7777,
            'listen_port':      7777,
            'command_topic':    'motor_can_tx',
            'feedback_topic':   'motor_can_feedback',
            'can_id_per_joint': True,
            'can_id_base':      _CAN_ID_BASE,
            'all_logging_info': False,
        }],
    )

    # ------------------------------------------------------------------
    # 3. motor_feedback_listener
    #    Decodes /motor_can_feedback bytes → /motor_feedback (MotorFeedback)
    # ------------------------------------------------------------------
    motor_feedback_node = Node(
        package='motor_test',
        executable='motor_feedback_listener.py',
        name='motor_feedback_listener',
        output='screen',
        parameters=[{
            'input':           'motor_can_feedback',
            'motor_count':     _N_JOINTS,
            'names_file':      'motor_names.json',
            'all_logging_info': False,
        }],
    )

    # ------------------------------------------------------------------
    # 4. robot_observation_bridge
    #    Aggregates /motor_feedback + /imu → /robot_observation (RobotObservation)
    #
    #    If no IMU is available the gravity vector defaults to [0,0,-1].
    #    Remap the imu_topic parameter if your IMU publishes on a different topic.
    # ------------------------------------------------------------------
    robot_observation_bridge_node = Node(
        package='motor_test',
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
