"""
rcu_launch.py — ROS2 launch file for PROJ500 RCU motor control stack

Launches:
  1. rcu_udp_bridge      — UDP ↔ ROS2 bridge (this repo, Tools/ROS2/)
  2. robot_command_bridge — your existing joint-command → /robot_command publisher
  3. robot_observation_bridge — your existing /motor_can_feedback consumer

Usage:
  ros2 launch <package> rcu_launch.py
  ros2 launch <package> rcu_launch.py rcu_ip:=192.168.100.10 ctrl_mode:=1

Parameters (all optional):
  rcu_ip       default "192.168.100.10"
  ctrl_mode    default 1   (1=CSP Phase 1, 0=MIT Phase 2)
  log_dir      default "~/rcu_logs"
  auto_enable  default "False"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rcu_ip      = LaunchConfiguration("rcu_ip",      default="192.168.100.10")
    ctrl_mode   = LaunchConfiguration("ctrl_mode",   default="1")
    log_dir     = LaunchConfiguration("log_dir",     default="~/rcu_logs")
    auto_enable = LaunchConfiguration("auto_enable", default="False")

    return LaunchDescription([
        DeclareLaunchArgument("rcu_ip",
            default_value="192.168.100.10",
            description="RCU static IP address"),
        DeclareLaunchArgument("ctrl_mode",
            default_value="1",
            description="Motor control mode: 1=CSP (Phase 1), 0=MIT (Phase 2)"),
        DeclareLaunchArgument("log_dir",
            default_value="~/rcu_logs",
            description="Directory for PDU telemetry CSV logs"),
        DeclareLaunchArgument("auto_enable",
            default_value="False",
            description="Automatically enable all motors at startup (unsafe)"),

        # ----------------------------------------------------------------
        # RCU UDP bridge — replaces ethernet_can_bridge + nucleo_can_bridge
        # ----------------------------------------------------------------
        Node(
            package="<your_ros2_package>",   # update with actual package name
            executable="rcu_udp_bridge",
            name="rcu_udp_bridge",
            output="screen",
            parameters=[{
                "rcu_ip":      rcu_ip,
                "ctrl_mode":   ctrl_mode,
                "log_dir":     log_dir,
                "auto_enable": auto_enable,
                "loop_rate_hz": 200.0,
            }],
        ),

        # ----------------------------------------------------------------
        # Robot command bridge — your existing node that publishes
        # /robot_command from the RL policy output.
        # Update package/executable to match your setup.
        # ----------------------------------------------------------------
        Node(
            package="<your_ros2_package>",
            executable="robot_command_bridge",
            name="robot_command_bridge",
            output="screen",
        ),

        # ----------------------------------------------------------------
        # Robot observation bridge — your existing node that consumes
        # /motor_can_feedback and /imu0 /imu1 for the RL observation.
        # Update package/executable to match your setup.
        # ----------------------------------------------------------------
        Node(
            package="<your_ros2_package>",
            executable="robot_observation_bridge",
            name="robot_observation_bridge",
            output="screen",
        ),
    ])
