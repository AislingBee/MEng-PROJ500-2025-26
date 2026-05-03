"""
rl_robot_launch.py — full RL-to-hardware pipeline over RCU UDP binary protocol.

Pipeline:
  thor_policy_runner.py (Ros2RobotBridge) -> /robot_command -> rcu_udp_bridge <-> RCU
  RCU -> rcu_udp_bridge -> /motor_can_feedback + /imu0
      -> motor_feedback_listener + robot_observation_bridge -> /robot_observation
      -> thor_policy_runner.py

Usage:
  ros2 launch motor_control rl_robot_launch.py
  ros2 launch motor_control rl_robot_launch.py rcu_ip:=192.168.100.10 ctrl_mode:=0

Note:
    Keep list-like values as strings (e.g. "[9,10]") for ROS2 Jazzy parameter compatibility.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rcu_ip = LaunchConfiguration("rcu_ip", default="192.168.100.10")
    rcu_cmd_port = LaunchConfiguration("rcu_cmd_port", default="7701")
    telem_port = LaunchConfiguration("telem_port", default="7700")
    ctrl_mode = LaunchConfiguration("ctrl_mode", default="0")
    auto_enable = LaunchConfiguration("auto_enable", default="False")
    log_dir = LaunchConfiguration("log_dir", default="~/rcu_logs")
    loop_rate_hz = LaunchConfiguration("loop_rate_hz", default="200.0")
    names_file = LaunchConfiguration("names_file", default="joint_limits_config.json")

    # Keep strict checks opt-in so single/bench workflows do not break.
    active_motor_ids = LaunchConfiguration("active_motor_ids", default="[1,2,3,4,5,6,7,8,9,10,11,12]")
    left_bus_motor_ids = LaunchConfiguration("left_bus_motor_ids", default="[1,3,5,7,9,11]")
    right_bus_motor_ids = LaunchConfiguration("right_bus_motor_ids", default="[2,4,6,8,10,12]")
    scan_motor_can_ids = LaunchConfiguration("scan_motor_can_ids", default="False")
    wait_for_expected_online_ids = LaunchConfiguration("wait_for_expected_online_ids", default="False")
    expected_online_motor_ids = LaunchConfiguration("expected_online_motor_ids", default="[]")

    return LaunchDescription([
        DeclareLaunchArgument("rcu_ip", default_value="192.168.100.10", description="RCU static IP address"),
        DeclareLaunchArgument("rcu_cmd_port", default_value="7701", description="RCU command UDP port"),
        DeclareLaunchArgument("telem_port", default_value="7700", description="RCU telemetry UDP port"),
        DeclareLaunchArgument("ctrl_mode", default_value="0", description="Motor control mode: 0=MIT, 1=CSP"),
        DeclareLaunchArgument("auto_enable", default_value="False", description="Automatically enable active motors at startup"),
        DeclareLaunchArgument("log_dir", default_value="~/rcu_logs", description="Directory for telemetry CSV logs"),
        DeclareLaunchArgument("loop_rate_hz", default_value="200.0", description="Command TX loop rate in Hz"),
        DeclareLaunchArgument("names_file", default_value="joint_limits_config.json", description="Joint name config file for motor_feedback_listener"),
        DeclareLaunchArgument("active_motor_ids", default_value="[1,2,3,4,5,6,7,8,9,10,11,12]", description="Motor IDs included in outgoing Type 0x10 command and enable scope"),
        DeclareLaunchArgument("left_bus_motor_ids", default_value="[1,3,5,7,9,11]", description="Motor IDs forced onto left bus"),
        DeclareLaunchArgument("right_bus_motor_ids", default_value="[2,4,6,8,10,12]", description="Motor IDs forced onto right bus"),
        DeclareLaunchArgument("scan_motor_can_ids", default_value="False", description="Log online CAN motor IDs seen in feedback"),
        DeclareLaunchArgument("wait_for_expected_online_ids", default_value="False", description="Hold command TX until expected_online_motor_ids are online"),
        DeclareLaunchArgument("expected_online_motor_ids", default_value="[]", description="Motor IDs required online before TX when startup gate is enabled"),

        Node(
            package="motor_control",
            executable="rcu_udp_bridge.py",
            name="rcu_udp_bridge",
            output="screen",
            parameters=[{
                "rcu_ip": rcu_ip,
                "rcu_cmd_port": rcu_cmd_port,
                "telem_port": telem_port,
                "ctrl_mode": ctrl_mode,
                "auto_enable": auto_enable,
                "log_dir": log_dir,
                "loop_rate_hz": loop_rate_hz,
                "active_motor_ids": active_motor_ids,
                "left_bus_motor_ids": left_bus_motor_ids,
                "right_bus_motor_ids": right_bus_motor_ids,
                "scan_motor_can_ids": scan_motor_can_ids,
                "wait_for_expected_online_ids": wait_for_expected_online_ids,
                "expected_online_motor_ids": expected_online_motor_ids,
            }],
        ),

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

        Node(
            package="motor_control",
            executable="robot_observation_bridge.py",
            name="robot_observation_bridge",
            output="screen",
            parameters=[{
                "feedback_topic": "motor_feedback",
                "imu_topic": "imu0",
                "observation_topic": "robot_observation",
                "all_logging_info": False,
            }],
        ),
    ])
