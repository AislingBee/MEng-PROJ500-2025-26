"""
rcu_launch.py — ROS2 launch file for PROJ500 RCU motor control stack

Launches:
  1. rcu_udp_bridge           — RCU UDP ↔ ROS2 bridge (replaces ethernet_can_bridge + nucleo)
    2. motor_feedback_listener  — /motor_can_feedback → /motor_feedback
    3. robot_observation_bridge — /motor_feedback → /robot_observation

Usage:
    ros2 launch motor_control rcu_launch.py
    ros2 launch motor_control rcu_launch.py rcu_ip:=192.168.100.10 ctrl_mode:=0

Parameters (all optional):
  rcu_ip       default "192.168.100.10"
  ctrl_mode    default 0   (0=MIT Phase 2, 1=CSP Phase 1)
  log_dir      default "~/rcu_logs"
  auto_enable  default "False"
    active_motor_ids default "[1,2]"  (bench mode)
    left_bus_motor_ids default "[1,2]" (bench mode)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rcu_ip      = LaunchConfiguration("rcu_ip",      default="192.168.100.10")
    ctrl_mode   = LaunchConfiguration("ctrl_mode",   default="0")
    log_dir     = LaunchConfiguration("log_dir",     default="~/rcu_logs")
    auto_enable = LaunchConfiguration("auto_enable", default="False")
    active_motor_ids = LaunchConfiguration("active_motor_ids", default="[1,2]")
    left_bus_motor_ids = LaunchConfiguration("left_bus_motor_ids", default="[1,2]")
    scan_motor_can_ids = LaunchConfiguration("scan_motor_can_ids", default="False")

    return LaunchDescription([

        DeclareLaunchArgument("rcu_ip",
            default_value="192.168.100.10",
            description="RCU static IP address"),
        DeclareLaunchArgument("ctrl_mode",
            default_value="0",
            description="Motor control mode: 1=CSP (Phase 1), 0=MIT (Phase 2)"),
        DeclareLaunchArgument("log_dir",
            default_value="~/rcu_logs",
            description="Directory for PDU telemetry CSV logs"),
        DeclareLaunchArgument("auto_enable",
            default_value="False",
            description="Automatically enable all motors at startup (unsafe)"),
        DeclareLaunchArgument("active_motor_ids",
            default_value="[1,2]",
            description="Motor IDs included in outgoing Type 0x10 command packets"),
        DeclareLaunchArgument("left_bus_motor_ids",
            default_value="[1,2]",
            description="Motor IDs forced onto left bus for bench wiring"),
        DeclareLaunchArgument("scan_motor_can_ids",
            default_value="False",
            description="Log online CAN motor IDs seen in feedback"),

        # ----------------------------------------------------------------
        # RCU UDP bridge — replaces ethernet_can_bridge + nucleo_can_bridge
        # Publishes:  /motor_can_feedback (UInt8MultiArray), /rcu_pdu_telem (String)
        # Subscribes: /robot_command (RobotCommand)
        # Services:   /rcu_motor_estop, /rcu_pdu_fault (SetBool)
        # ----------------------------------------------------------------
        Node(
            package="motor_control",
            executable="rcu_udp_bridge.py",
            name="rcu_udp_bridge",
            output="screen",
            parameters=[{
                "rcu_ip":       rcu_ip,
                "ctrl_mode":    ctrl_mode,
                "log_dir":      log_dir,
                "auto_enable":  auto_enable,
                "active_motor_ids": active_motor_ids,
                "left_bus_motor_ids": left_bus_motor_ids,
                "scan_motor_can_ids": scan_motor_can_ids,
                "loop_rate_hz": 200.0,
            }],
        ),

        # ----------------------------------------------------------------
        # Motor feedback listener — converts /motor_can_feedback → /motor_feedback
        # ----------------------------------------------------------------
        Node(
            package="motor_control",
            executable="motor_feedback_listener.py",
            name="motor_feedback_listener",
            output="screen",
            parameters=[{
                "input":            "motor_can_feedback",
                "motor_count":      12,
                "names_file":       "joint_limits_config.json",
                "all_logging_info": False,
            }],
        ),

        # ----------------------------------------------------------------
        # Robot observation bridge — /motor_feedback → /robot_observation
        # ----------------------------------------------------------------
        Node(
            package="motor_control",
            executable="robot_observation_bridge.py",
            name="robot_observation_bridge",
            output="screen",
            parameters=[{
                "feedback_topic":    "motor_feedback",
                "observation_topic": "robot_observation",
                "all_logging_info":  False,
            }],
        ),
    ])
