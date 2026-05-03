"""
Basic full-hardware launch for Thor standing policy with all 12 motors.

References:
    - Working ROS bench baseline:
            Software/src/motor_control/launch/rcu_bench_test_launch.py
        This launch keeps the same known-good RCU stack pattern
        (IncludeLaunchDescription -> rcu_launch.py with auto_enable, ctrl_mode=0,
        active motor IDs and CAN-ID scan).

    - Plymouth humanoid bench protocol baseline:
            Charlie/STM32Cube/Tools/plymouth_humanoid_bench_monitor.py
        Port and packet conventions aligned with Plymouth monitor:
            RCU->PC telem on UDP 7700, PC->RCU commands on UDP 7701,
            motor feedback Type 0x02, motor command Type 0x10,
            debug command Type 0x20 with per-motor enable subcommand 0x0C.

Pipeline:
  thor_policy_runner.py -> /robot_command -> rcu_udp_bridge -> RCU
  RCU -> /motor_can_feedback + /imu0 -> robot_observation_bridge -> /robot_observation

Named motor mapping (CAN ID -> joint name) follows:
    simulation/isaac/configuration/joint_limits_config.json
which matches motor_control/rcu_protocol.py MOTOR_JOINT_NAMES for IDs 1..12.

Usage:
  ros2 launch motor_control thor_12_motor_pipeline_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_share = get_package_share_directory("motor_control")
    rcu_launch_path = os.path.join(pkg_share, "launch", "rcu_launch.py")

    workspace_root_default = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "..")
    )

    workspace_root = LaunchConfiguration("workspace_root")
    rcu_ip = LaunchConfiguration("rcu_ip")
    ctrl_mode = LaunchConfiguration("ctrl_mode")
    auto_enable = LaunchConfiguration("auto_enable")
    scan_motor_can_ids = LaunchConfiguration("scan_motor_can_ids")
    names_file = LaunchConfiguration("names_file")
    wait_for_expected_online_ids = LaunchConfiguration("wait_for_expected_online_ids")
    expected_online_motor_ids = LaunchConfiguration("expected_online_motor_ids")
    thor_runner_script = LaunchConfiguration("thor_runner_script")

    rcu_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rcu_launch_path),
        launch_arguments={
            "rcu_ip": rcu_ip,
            "ctrl_mode": ctrl_mode,
            "auto_enable": auto_enable,
            "active_motor_ids": "[1,2,3,4,5,6,7,8,9,10,11,12]",
            "left_bus_motor_ids": "[1,3,5,7,9,11]",
            "scan_motor_can_ids": scan_motor_can_ids,
            "names_file": names_file,
            "wait_for_expected_online_ids": wait_for_expected_online_ids,
            "expected_online_motor_ids": expected_online_motor_ids,
        }.items(),
    )

    chmod_runner = ExecuteProcess(
        cmd=["chmod", "+x", thor_runner_script],
        output="screen",
    )

    _existing_pythonpath = os.environ.get("PYTHONPATH", "")
    _runner_pythonpath = (
        workspace_root_default + ":" + _existing_pythonpath
        if _existing_pythonpath
        else workspace_root_default
    )

    thor_policy_runner = ExecuteProcess(
        cmd=["python3", thor_runner_script],
        cwd=workspace_root,
        additional_env={"PYTHONPATH": _runner_pythonpath},
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "workspace_root",
            default_value=workspace_root_default,
            description="Workspace root used for Thor policy runner working directory",
        ),
        DeclareLaunchArgument(
            "thor_runner_script",
            default_value=PathJoinSubstitution([workspace_root, "hardware", "thor", "thor_policy_runner.py"]),
            description="Absolute path to thor_policy_runner.py",
        ),
        DeclareLaunchArgument(
            "rcu_ip",
            default_value="192.168.100.10",
            description="RCU static IP address",
        ),
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="0",
            description="Motor control mode: 0=MIT, 1=CSP",
        ),
        DeclareLaunchArgument(
            "auto_enable",
            default_value="True",
            description="Automatically enable all motors on startup",
        ),
        DeclareLaunchArgument(
            "scan_motor_can_ids",
            default_value="True",
            description="Log online CAN motor IDs seen in feedback",
        ),
        DeclareLaunchArgument(
            "wait_for_expected_online_ids",
            default_value="True",
            description="Hold command TX until all expected motor IDs are online",
        ),
        DeclareLaunchArgument(
            "expected_online_motor_ids",
            default_value="[1,2,3,4,5,6,7,8,9,10,11,12]",
            description="Motor IDs that must be online before startup gate releases",
        ),
        DeclareLaunchArgument(
            "names_file",
            default_value="joint_limits_config.json",
            description="Joint name config used to label motor IDs 1..12",
        ),
        rcu_stack,
        chmod_runner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=chmod_runner,
                on_exit=[thor_policy_runner],
            )
        ),
    ])
