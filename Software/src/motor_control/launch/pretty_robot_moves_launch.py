"""
ROS2 launch file for Thor Pretty Robot Moves choreography runner.

Pipeline (identical RCU stack to thor_12_motor_pipeline_launch.py):
  pretty_robot_moves.py -> /robot_command -> rcu_udp_bridge -> RCU
  RCU -> /motor_can_feedback + /imu0 -> imu_publisher -> /imu0_remapped
  /motor_can_feedback + /imu0_remapped -> robot_observation_bridge -> /robot_observation
  <- pretty_robot_moves.py reads /robot_observation for position feedback

pretty_robot_moves.py smoothly interpolates between named joint-space poses
using cosine ease-in/ease-out PD control.  No RL policy is loaded.

Keyboard controls while running (type in the terminal, press Enter):
    q   – safe stop (ramps back to standing then exits)
    p   – pause / resume
    n   – skip to the next move immediately
    l   – toggle looping
    +   – increase speed by 10 %
    -   – decrease speed by 10 %

Usage:
    ros2 launch motor_control pretty_robot_moves_launch.py
    ros2 launch motor_control pretty_robot_moves_launch.py speed:=0.7
    ros2 launch motor_control pretty_robot_moves_launch.py no_loop:=True rcu_ip:=192.168.100.10
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("motor_control")
    rcu_launch_path = os.path.join(pkg_share, "launch", "rcu_launch.py")

    workspace_root_default = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "..")
    )
    pretty_moves_script_default = os.path.join(
        workspace_root_default,
        "Software", "src", "motor_control", "motor_control", "pretty_robot_moves.py",
    )

    # ---------------------------------------------------------------------------
    # LaunchConfiguration handles (RCU stack only – script args resolved later)
    # ---------------------------------------------------------------------------
    rcu_ip                       = LaunchConfiguration("rcu_ip")
    ctrl_mode                    = LaunchConfiguration("ctrl_mode")
    auto_enable                  = LaunchConfiguration("auto_enable")
    scan_motor_can_ids           = LaunchConfiguration("scan_motor_can_ids")
    names_file                   = LaunchConfiguration("names_file")
    wait_for_expected_online_ids = LaunchConfiguration("wait_for_expected_online_ids")
    expected_online_motor_ids    = LaunchConfiguration("expected_online_motor_ids")
    right_bus_motor_ids          = LaunchConfiguration("right_bus_motor_ids")

    # ---------------------------------------------------------------------------
    # RCU stack – identical to thor_12_motor_pipeline_launch.py
    # ---------------------------------------------------------------------------
    rcu_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rcu_launch_path),
        launch_arguments={
            "rcu_ip":                       rcu_ip,
            "ctrl_mode":                    ctrl_mode,
            "auto_enable":                  auto_enable,
            "active_motor_ids":             "[1,2,3,4,5,6,7,8,9,10,11,12]",
            "left_bus_motor_ids":           "[1,3,5,7,9,11]",
            "right_bus_motor_ids":          right_bus_motor_ids,
            "scan_motor_can_ids":           scan_motor_can_ids,
            "names_file":                   names_file,
            "wait_for_expected_online_ids": wait_for_expected_online_ids,
            "expected_online_motor_ids":    expected_online_motor_ids,
            "imu_print_hz":                 "0.0",
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # chmod the script, then launch it via OpaqueFunction so we can
    # conditionally append --no-loop (a store_true flag).
    # ---------------------------------------------------------------------------
    _existing_pythonpath = os.environ.get("PYTHONPATH", "")
    _runner_pythonpath = (
        workspace_root_default + ":" + _existing_pythonpath
        if _existing_pythonpath
        else workspace_root_default
    )

    chmod_script = ExecuteProcess(
        cmd=["chmod", "+x", pretty_moves_script_default],
        output="screen",
    )

    def _make_runner(context, *args, **kwargs):
        """Resolved at launch time; reads all LaunchConfigurations."""
        script      = context.launch_configurations.get("pretty_moves_script", pretty_moves_script_default)
        speed       = context.launch_configurations.get("speed",                 "0.3")
        kp_scale    = context.launch_configurations.get("kp_scale",              "0.20")
        kd_scale    = context.launch_configurations.get("kd_scale",              "1.00")
        loop_hz     = context.launch_configurations.get("loop_hz",               "66.67")
        max_err     = context.launch_configurations.get("max_position_error_rad","1.50")
        debug_every = context.launch_configurations.get("debug_every",           "100")
        device      = context.launch_configurations.get("device",                "cpu")
        no_loop     = context.launch_configurations.get("no_loop",               "False")
        sequence    = context.launch_configurations.get("sequence",              "taps")

        cmd = [
            "python3", script,
            "--speed",                  speed,
            "--kp-scale",               kp_scale,
            "--kd-scale",               kd_scale,
            "--loop-hz",                loop_hz,
            "--max-position-error-rad", max_err,
            "--debug-every",            debug_every,
            "--device",                 device,
            "--sequence",               sequence,
        ]
        if no_loop.strip().lower() in ("true", "1", "yes"):
            cmd.append("--no-loop")

        runner = ExecuteProcess(
            cmd=cmd,
            cwd=workspace_root_default,
            additional_env={"PYTHONPATH": _runner_pythonpath},
            output="screen",
        )
        return [
            RegisterEventHandler(
                OnProcessExit(target_action=chmod_script, on_exit=[runner])
            )
        ]

    # ---------------------------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------------------------
    return LaunchDescription([

        # -- workspace / script paths ------------------------------------------
        DeclareLaunchArgument(
            "pretty_moves_script",
            default_value=pretty_moves_script_default,
            description="Absolute path to pretty_robot_moves.py",
        ),

        # -- RCU network -------------------------------------------------------
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
            description="Automatically enable all 12 motors on startup",
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
        DeclareLaunchArgument(
            "right_bus_motor_ids",
            default_value="[2,4,6,8,10,12]",
            description="Motor IDs forced onto right CAN bus",
        ),

        # -- pretty_robot_moves tuning ----------------------------------------
        DeclareLaunchArgument(
            "sequence",
            default_value="taps",
            description="Which movement sequence to run: kicks, taps, shapes",
        ),
        DeclareLaunchArgument(
            "speed",
            default_value="0.3",
            description="Global speed multiplier (>1 faster, <1 slower)",
        ),
        DeclareLaunchArgument(
            "no_loop",
            default_value="False",
            description="Set True to play the sequence once then exit instead of looping",
        ),
        DeclareLaunchArgument(
            "kp_scale",
            default_value="0.20",
            description="Scale applied to joint stiffness gains (keep ≤0.25 for safety)",
        ),
        DeclareLaunchArgument(
            "kd_scale",
            default_value="1.00",
            description="Scale applied to joint damping gains",
        ),
        DeclareLaunchArgument(
            "loop_hz",
            default_value="66.67",
            description="Control loop frequency in Hz (default matches CONTRACT.policy_loop_hz)",
        ),
        DeclareLaunchArgument(
            "max_position_error_rad",
            default_value="1.50",
            description="Abort if any joint tracking error exceeds this value (rad)",
        ),
        DeclareLaunchArgument(
            "debug_every",
            default_value="100",
            description="Print debug line every N control steps",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="Torch device for tensor operations",
        ),

        # -- execution chain ---------------------------------------------------
        rcu_stack,
        chmod_script,
        OpaqueFunction(function=_make_runner),
    ])
