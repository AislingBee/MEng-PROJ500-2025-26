"""
rcu_launch.py — ROS2 launch file for PROJ500 RCU motor control stack

Launches:
  1. rcu_udp_bridge           — RCU UDP ↔ ROS2 bridge (replaces ethernet_can_bridge + nucleo)
    2. motor_feedback_listener  — /motor_can_feedback → /motor_feedback
    3. robot_observation_bridge — /motor_feedback → /robot_observation

Usage:
    ros2 launch motor_control rcu_launch.py
    ros2 launch motor_control rcu_launch.py rcu_ip:=192.168.100.10 ctrl_mode:=0
    ros2 launch motor_control rcu_launch.py active_motor_ids:="[9,10]" left_bus_motor_ids:="[9]" right_bus_motor_ids:="[10]"

Parameters (all optional):
    rcu_ip                      default "192.168.100.10"
    rcu_cmd_port                default "7701"
    telem_port                  default "7700"
    ctrl_mode                   default "0"    (0=MIT Phase 2, 1=CSP Phase 1)
    auto_enable                 default "False"
    log_dir                     default "~/rcu_logs"
    loop_rate_hz                default "200.0"
    active_motor_ids            default "[1,2,3,4,5,6,7,8,9,10,11,12]"
    left_bus_motor_ids          default "[1,3,5,7,9,11]"
    right_bus_motor_ids         default "[2,4,6,8,10,12]"
    scan_motor_can_ids          default "False"
    can_id_online_timeout_s     default "1.0"
    can_id_scan_log_period_s    default "1.0"
    wait_for_expected_online_ids default "False"
    expected_online_motor_ids   default "[]"
    startup_gate_error_after_s  default "5.0"
    feedback_all_logging_info    default "False"
    imu_topic                    default "imu0"
    observation_topic            default "robot_observation"
    observation_all_logging_info default "False"
    observation_log_hz           default "2.0"

Note:
    Keep list-like values as strings (e.g. "[9,10]") for ROS2 Jazzy parameter compatibility.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rcu_ip      = LaunchConfiguration("rcu_ip",      default="192.168.100.10")
    rcu_cmd_port = LaunchConfiguration("rcu_cmd_port", default="7701")
    telem_port   = LaunchConfiguration("telem_port", default="7700")
    ctrl_mode   = LaunchConfiguration("ctrl_mode",   default="0")
    log_dir     = LaunchConfiguration("log_dir",     default="~/rcu_logs")
    auto_enable = LaunchConfiguration("auto_enable", default="False")
    loop_rate_hz = LaunchConfiguration("loop_rate_hz", default="200.0")
    active_motor_ids = LaunchConfiguration("active_motor_ids", default="[1,2,3,4,5,6,7,8,9,10,11,12]")
    left_bus_motor_ids = LaunchConfiguration("left_bus_motor_ids", default="[1,3,5,7,9,11]")
    right_bus_motor_ids = LaunchConfiguration("right_bus_motor_ids", default="[2,4,6,8,10,12]")
    scan_motor_can_ids = LaunchConfiguration("scan_motor_can_ids", default="False")
    can_id_online_timeout_s = LaunchConfiguration("can_id_online_timeout_s", default="1.0")
    can_id_scan_log_period_s = LaunchConfiguration("can_id_scan_log_period_s", default="1.0")
    names_file = LaunchConfiguration("names_file", default="joint_limits_config.json")
    feedback_all_logging_info = LaunchConfiguration("feedback_all_logging_info", default="False")
    imu_topic = LaunchConfiguration("imu_topic", default="imu0")
    observation_topic = LaunchConfiguration("observation_topic", default="robot_observation")
    observation_all_logging_info = LaunchConfiguration("observation_all_logging_info", default="False")
    observation_log_hz = LaunchConfiguration("observation_log_hz", default="2.0")
    wait_for_expected_online_ids = LaunchConfiguration("wait_for_expected_online_ids", default="False")
    expected_online_motor_ids = LaunchConfiguration("expected_online_motor_ids", default="[]")
    startup_gate_error_after_s = LaunchConfiguration("startup_gate_error_after_s", default="5.0")

    return LaunchDescription([

        DeclareLaunchArgument("rcu_ip",
            default_value="192.168.100.10",
            description="RCU static IP address"),
        DeclareLaunchArgument("rcu_cmd_port",
            default_value="7701",
            description="RCU command UDP port"),
        DeclareLaunchArgument("telem_port",
            default_value="7700",
            description="RCU telemetry UDP port"),
        DeclareLaunchArgument("ctrl_mode",
            default_value="0",
            description="Motor control mode: 1=CSP (Phase 1), 0=MIT (Phase 2)"),
        DeclareLaunchArgument("log_dir",
            default_value="~/rcu_logs",
            description="Directory for PDU telemetry CSV logs"),
        DeclareLaunchArgument("auto_enable",
            default_value="False",
            description="Automatically enable active motors at startup"),
        DeclareLaunchArgument("loop_rate_hz",
            default_value="200.0",
            description="Command TX loop rate in Hz"),
        DeclareLaunchArgument("active_motor_ids",
            default_value="[1,2,3,4,5,6,7,8,9,10,11,12]",
            description="Motor IDs included in outgoing Type 0x10 command and enable scope"),
        DeclareLaunchArgument("left_bus_motor_ids",
            default_value="[1,3,5,7,9,11]",
            description="Motor IDs forced onto left bus"),
        DeclareLaunchArgument("right_bus_motor_ids",
            default_value="[2,4,6,8,10,12]",
            description="Motor IDs forced onto right bus"),
        DeclareLaunchArgument("scan_motor_can_ids",
            default_value="False",
            description="Log online CAN motor IDs seen in feedback"),
        DeclareLaunchArgument("can_id_online_timeout_s",
            default_value="1.0",
            description="Timeout window in seconds used to mark CAN IDs online"),
        DeclareLaunchArgument("can_id_scan_log_period_s",
            default_value="1.0",
            description="Period in seconds for CAN online-ID scan logging"),
        DeclareLaunchArgument("names_file",
            default_value="joint_limits_config.json",
            description="Joint name config file for motor_feedback_listener"),
        DeclareLaunchArgument("feedback_all_logging_info",
            default_value="False",
            description="Enable verbose motor_feedback_listener ROS logs"),
        DeclareLaunchArgument("imu_topic",
            default_value="imu0",
            description="IMU topic consumed by robot_observation_bridge"),
        DeclareLaunchArgument("observation_topic",
            default_value="robot_observation",
            description="Output topic for RobotObservation messages"),
        DeclareLaunchArgument("observation_all_logging_info",
            default_value="False",
            description="Enable verbose robot_observation_bridge ROS logs"),
        DeclareLaunchArgument("observation_log_hz",
            default_value="2.0",
            description="Robot observation terminal log rate in Hz when observation logging is enabled"),
        DeclareLaunchArgument("wait_for_expected_online_ids",
            default_value="False",
            description="Hold command TX until expected_online_motor_ids are online"),
        DeclareLaunchArgument("expected_online_motor_ids",
            default_value="[]",
            description="Motor IDs required online before TX when startup gate is enabled"),
        DeclareLaunchArgument("startup_gate_error_after_s",
            default_value="5.0",
            description="Time in seconds after which startup-gate blocking is escalated as error"),

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
                "rcu_cmd_port": rcu_cmd_port,
                "telem_port":   telem_port,
                "ctrl_mode":    ctrl_mode,
                "log_dir":      log_dir,
                "auto_enable":  auto_enable,
                "loop_rate_hz": loop_rate_hz,
                "active_motor_ids": ParameterValue(active_motor_ids, value_type=str),
                "left_bus_motor_ids": ParameterValue(left_bus_motor_ids, value_type=str),
                "right_bus_motor_ids": ParameterValue(right_bus_motor_ids, value_type=str),
                "scan_motor_can_ids": scan_motor_can_ids,
                "can_id_online_timeout_s": can_id_online_timeout_s,
                "can_id_scan_log_period_s": can_id_scan_log_period_s,
                "wait_for_expected_online_ids": wait_for_expected_online_ids,
                "expected_online_motor_ids": ParameterValue(expected_online_motor_ids, value_type=str),
                "startup_gate_error_after_s": startup_gate_error_after_s,
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
                "names_file":       names_file,
                "all_logging_info": feedback_all_logging_info,
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
                "imu_topic":         imu_topic,
                "observation_topic": observation_topic,
                "all_logging_info":  observation_all_logging_info,
                "log_hz":            observation_log_hz,
            }],
        ),
    ])
