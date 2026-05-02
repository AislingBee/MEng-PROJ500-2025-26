"""
Diagnosis launch: enable motors 9/10, check online CAN IDs, and stream feedback.

This launch is configured for the 9/10 bench pair:
        - enables configured active motors at startup,
    - scans and logs online CAN IDs,
    - publishes motor feedback and IMU topics.

Usage:
  ros2 launch motor_control rcu_diagnosis_launch.py

Note:
    Keep list-like values as strings (e.g. "[9,10]") for ROS2 Jazzy parameter compatibility.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rcu_ip = LaunchConfiguration("rcu_ip", default="192.168.100.10")
    ctrl_mode = LaunchConfiguration("ctrl_mode", default="0")
    log_dir = LaunchConfiguration("log_dir", default="~/rcu_logs")
    names_file = LaunchConfiguration("names_file", default="joint_limits_config.json")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rcu_ip",
            default_value="192.168.100.10",
            description="RCU static IP address",
        ),
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="0",
            description="Motor control mode used for enable command path",
        ),
        DeclareLaunchArgument(
            "log_dir",
            default_value="~/rcu_logs",
            description="Directory for telemetry CSV logs",
        ),
        DeclareLaunchArgument(
            "names_file",
            default_value="joint_limits_config.json",
            description="Joint name config used by motor_feedback_listener",
        ),

        # RCU bridge in diagnosis mode.
        Node(
            package="motor_control",
            executable="rcu_udp_bridge.py",
            name="rcu_udp_bridge",
            output="screen",
            parameters=[{
                "rcu_ip": rcu_ip,
                "ctrl_mode": ctrl_mode,
                "log_dir": log_dir,
                "auto_enable": True,
                "active_motor_ids": "[9,10]",
                "left_bus_motor_ids": "[9]",
                "right_bus_motor_ids": "[10]",
                "scan_motor_can_ids": True,
                "can_id_online_timeout_s": 1.0,
                "can_id_scan_log_period_s": 5.0,
                "wait_for_expected_online_ids": True,
                "expected_online_motor_ids": "[9,10]",
                "loop_rate_hz": 200.0,
            }],
        ),

        # Decode /motor_can_feedback to named joints for easier diagnosis.
        Node(
            package="motor_control",
            executable="motor_feedback_listener.py",
            name="motor_feedback_listener",
            output="screen",
            parameters=[{
                "input": "motor_can_feedback",
                "motor_count": 12,
                "names_file": names_file,
                "all_logging_info": False,
            }],
        ),

        # Build RobotObservation from /motor_feedback + /imu0 and print ROS logs.
        Node(
            package="motor_control",
            executable="robot_observation_bridge.py",
            name="robot_observation_bridge",
            output="screen",
            parameters=[{
                "feedback_topic": "motor_feedback",
                "imu_topic": "imu0",
                "observation_topic": "robot_observation",
                "all_logging_info": True,
            }],
        ),
    ])
