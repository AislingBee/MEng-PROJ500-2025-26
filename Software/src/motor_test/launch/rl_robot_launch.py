# Build: colcon build --packages-select motor_test && source install/setup.bash
#
# Full RL-to-hardware pipeline (RCU binary protocol):
#   thor_policy_runner.py (Ros2RobotBridge) → /robot_command → rcu_udp_bridge ↔ RCU
#   RCU → rcu_udp_bridge → /motor_can_feedback → motor_feedback_listener
#                        → /imu0 → robot_observation_bridge → /robot_observation
#                        → thor_policy_runner.py (Ros2RobotBridge)
#
# Adjust parameters below to match your hardware setup.

from launch import LaunchDescription
from launch_ros.actions import Node

_N_JOINTS = 12


def generate_launch_description():
    # 1. rcu_udp_bridge: /robot_command → RCU (192.168.100.10) → /motor_can_feedback, /imu0, /imu1
    #    Replaces ethernet_can_bridge + nucleo_can_bridge + imu_publisher
    rcu_udp_bridge_node = Node(
        package='motor_test',
        executable='rcu_udp_bridge.py',
        name='rcu_udp_bridge',
        output='screen',
        parameters=[{
            'rcu_ip':       '192.168.100.10',
            'rcu_cmd_port': 7701,
            'telem_port':   7700,
            'ctrl_mode':    1,        # 1=CSP Phase 1 | 0=MIT Phase 2
            'auto_enable':  False,
            'loop_rate_hz': 200.0,
        }],
    )

    # 2. motor_feedback_listener: /motor_can_feedback → /motor_feedback
    motor_feedback_node = Node(
        package='motor_test',
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

    # 3. robot_observation_bridge: /motor_feedback + /imu0 → /robot_observation
    #    Uses /imu0 from rcu_udp_bridge (RCU fast IMU at 200 Hz)
    robot_observation_bridge_node = Node(
        package='motor_test',
        executable='robot_observation_bridge.py',
        name='robot_observation_bridge',
        output='screen',
        parameters=[{
            'feedback_topic':    'motor_feedback',
            'imu_topic':         'imu0',
            'observation_topic': 'robot_observation',
            'all_logging_info':  False,
        }],
    )

    return LaunchDescription([
        rcu_udp_bridge_node,
        rcu_udp_bridge_node,
        motor_feedback_node,
        robot_observation_bridge_node,
    ])
