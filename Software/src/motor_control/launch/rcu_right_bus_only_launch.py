"""
Right-bus-only diagnostic launch for motor 10.

Purpose:
  Isolate right CAN bus behavior with minimal launch argument complexity.

Usage:
  ros2 launch motor_control rcu_right_bus_only_launch.py

Notes:
  - left bus is explicitly empty.
  - startup gate is disabled so command TX is not blocked by missing feedback.
  - auto_enable defaults to False for safer bring-up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("motor_control")
    rcu_launch_path = os.path.join(pkg_share, "launch", "rcu_launch.py")

    hold_rate_hz = LaunchConfiguration("hold_rate_hz")
    hold_kp = LaunchConfiguration("hold_kp")
    hold_kd = LaunchConfiguration("hold_kd")
    auto_enable = LaunchConfiguration("auto_enable")
    ctrl_mode = LaunchConfiguration("ctrl_mode")

    rcu_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rcu_launch_path),
        launch_arguments={
            "active_motor_ids": "[10]",
            "left_bus_motor_ids": "[]",
            "right_bus_motor_ids": "[10]",
            "auto_enable": auto_enable,
            "ctrl_mode": ctrl_mode,
            "scan_motor_can_ids": "True",
            "can_id_scan_log_period_s": "1.0",
            "wait_for_expected_online_ids": "False",
            "feedback_all_logging_info": "True",
            "observation_all_logging_info": "False",
        }.items(),
    )

    hold_position_test = Node(
        package="motor_control",
        executable="hold_position_test.py",
        name="hold_position_test",
        output="screen",
        parameters=[{
            "motor_ids": "10",
            "rate_hz": hold_rate_hz,
            "kp": hold_kp,
            "kd": hold_kd,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "hold_rate_hz",
            default_value="200.0",
            description="Hold-command publish rate in Hz",
        ),
        DeclareLaunchArgument(
            "hold_kp",
            default_value="10.0",
            description="Hold-position proportional gain",
        ),
        DeclareLaunchArgument(
            "hold_kd",
            default_value="0.5",
            description="Hold-position derivative gain",
        ),
        DeclareLaunchArgument(
            "auto_enable",
            default_value="False",
            description="Automatically enable motor at startup",
        ),
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="0",
            description="Motor control mode: 0=MIT, 1=CSP",
        ),
        rcu_stack,
        hold_position_test,
    ])
