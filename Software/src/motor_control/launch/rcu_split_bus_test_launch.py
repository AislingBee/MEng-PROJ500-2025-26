"""
Split-bus command TX diagnostic launch.

Purpose:
  Reproduce/test the case where motor commands are transmitted as two packets
  per control tick instead of one mixed packet:
    1) left-bus entries
    2) right-bus entries

Usage:
  ros2 launch motor_control rcu_split_bus_test_launch.py

Notes:
  - This launch keeps startup gating enabled and defaults to motor IDs 9/10.
  - auto_enable defaults to False for safer bring-up. Enable manually once
    command streaming looks correct:
      ros2 service call /rcu_motor_estop std_srvs/srv/SetBool "{data: true}"
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

    active_motor_ids = LaunchConfiguration("active_motor_ids")
    left_bus_motor_ids = LaunchConfiguration("left_bus_motor_ids")
    right_bus_motor_ids = LaunchConfiguration("right_bus_motor_ids")
    expected_online_motor_ids = LaunchConfiguration("expected_online_motor_ids")
    hold_motor_ids = LaunchConfiguration("hold_motor_ids")
    hold_rate_hz = LaunchConfiguration("hold_rate_hz")
    hold_kp = LaunchConfiguration("hold_kp")
    hold_kd = LaunchConfiguration("hold_kd")
    scan_log_period_s = LaunchConfiguration("scan_log_period_s")
    feedback_all_logging_info = LaunchConfiguration("feedback_all_logging_info")
    observation_all_logging_info = LaunchConfiguration("observation_all_logging_info")
    auto_enable = LaunchConfiguration("auto_enable")
    ctrl_mode = LaunchConfiguration("ctrl_mode")

    rcu_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rcu_launch_path),
        launch_arguments={
            "active_motor_ids": active_motor_ids,
            "left_bus_motor_ids": left_bus_motor_ids,
            "right_bus_motor_ids": right_bus_motor_ids,
            "auto_enable": auto_enable,
            "ctrl_mode": ctrl_mode,
            "split_tx_by_bus": "True",
            "scan_motor_can_ids": "True",
            "can_id_scan_log_period_s": scan_log_period_s,
            "feedback_all_logging_info": feedback_all_logging_info,
            "observation_all_logging_info": observation_all_logging_info,
            "wait_for_expected_online_ids": "True",
            "expected_online_motor_ids": expected_online_motor_ids,
        }.items(),
    )

    hold_position_test = Node(
        package="motor_control",
        executable="hold_position_test.py",
        name="hold_position_test",
        output="screen",
        parameters=[{
            "motor_ids": hold_motor_ids,
            "rate_hz": hold_rate_hz,
            "kp": hold_kp,
            "kd": hold_kd,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "active_motor_ids",
            default_value="[9,10]",
            description="Motor IDs to enable and command during split-bus test",
        ),
        DeclareLaunchArgument(
            "left_bus_motor_ids",
            default_value="[9]",
            description="Subset of active motor IDs forced to left bus",
        ),
        DeclareLaunchArgument(
            "right_bus_motor_ids",
            default_value="[10]",
            description="Subset of active motor IDs forced to right bus",
        ),
        DeclareLaunchArgument(
            "expected_online_motor_ids",
            default_value="[9,10]",
            description="Motor IDs required online before startup gate releases",
        ),
        DeclareLaunchArgument(
            "hold_motor_ids",
            default_value="9,10",
            description="Comma-separated motor IDs for hold_position_test",
        ),
        DeclareLaunchArgument(
            "hold_rate_hz",
            default_value="200.0",
            description="Hold-command publish rate in Hz",
        ),
        DeclareLaunchArgument(
            "hold_kp",
            default_value="20.0",
            description="Hold-position proportional gain",
        ),
        DeclareLaunchArgument(
            "hold_kd",
            default_value="1.0",
            description="Hold-position derivative gain",
        ),
        DeclareLaunchArgument(
            "auto_enable",
            default_value="False",
            description="Automatically enable motors at startup (False recommended)",
        ),
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="1",
            description="Motor control mode for test: 1=CSP, 0=MIT",
        ),
        DeclareLaunchArgument(
            "scan_log_period_s",
            default_value="1.0",
            description="Seconds between CAN online-ID log lines from rcu_udp_bridge",
        ),
        DeclareLaunchArgument(
            "feedback_all_logging_info",
            default_value="True",
            description="Enable verbose motor_feedback_listener logs",
        ),
        DeclareLaunchArgument(
            "observation_all_logging_info",
            default_value="True",
            description="Enable verbose robot_observation_bridge logs",
        ),
        rcu_stack,
        hold_position_test,
    ])
