"""
Start the RCU stack and a simple two-motor command publisher for bench tests.

Usage:
  ros2 launch motor_control rcu_bench_test_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("motor_control")
    rcu_launch_path = os.path.join(pkg_share, "launch", "rcu_launch.py")

    rcu_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rcu_launch_path),
        launch_arguments={
            "active_motor_ids": "[9,10]",
            "left_bus_motor_ids": "[9]",
            "auto_enable": "True",
            "ctrl_mode": "0",
            "scan_motor_can_ids": "True",
        }.items(),
    )

    bench_cmd_test = Node(
        package="motor_control",
        executable="rcu_bench_command_test.py",
        name="rcu_bench_command_test",
        output="screen",
        parameters=[{
            "command_topic": "/robot_command",
            "joint_names": "motor_9,motor_10",
            "rate_hz": 10.0,
            "match_plymouth_bench_mode": True,
            "target_q_rad": 0.0,
            "step_duration_s": 2.5,
            "out_of_phase": True,
            "kp": 10.0,
            "kd": 1.0,
            "tau_ff": 1.0,
            "max_q_slew_rad_s": 1.2,
            "send_velocity_commands": True,
            "use_fixed_velocity_command": True,
            "velocity_command_rad_s": 1.0,
            "command_log_hz": 2.0,
        }],
    )

    return LaunchDescription([
        rcu_stack,
        bench_cmd_test,
    ])
