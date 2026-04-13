#!/usr/bin/env python3
"""
joint_torque_saturation_validation.py

PROJ500 - Isaac Lab actuator torque saturation validation

Purpose
-------
Validate that commanded joint motion is being limited by the configured actuator
effort limit, and log the observed response for each joint.

Test method
-----------
For each joint:
    neutral
    lower target
    neutral
    upper target
    neutral

During each target hold, the script:
    - commands a position target
    - steps the simulation
    - records final reached joint position
    - attempts to record maximum observed joint effort
    - reports whether effort appears clipped to the configured effort limit

Notes
-----
1. This script follows the working PROJ500 Isaac Lab scripting structure.
2. Gravity is disabled by default to isolate actuator behaviour.
3. Torque readback attribute names can differ slightly between Isaac Lab versions.
   The helper `get_joint_efforts()` tries several likely fields. If none are found,
   the script still runs and logs position/error data, but torque columns will be blank.
"""

from simulation.isaac.configuration.walking_actuator_config import (
    ACTUATOR_SETTINGS,
    build_per_joint_limits_and_gains,
)


import argparse
import csv
import math
import os
from typing import Optional, Tuple

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="PROJ500 joint torque saturation validation")
parser.add_argument("--usd", type=str, required=True, help="Path to robot USD")
parser.add_argument("--output_csv", type=str, default="joint_torque_saturation_results.csv", help="CSV output path")

parser.add_argument("--dt", type=float, default=1.0 / 120.0, help="Simulation timestep")
parser.add_argument("--gravity", type=float, nargs=3, default=(0.0, 0.0, 0.0), help="Gravity vector")
parser.add_argument("--effort_limit", type=float, default=70.0, help="Actuator effort limit in Nm")
parser.add_argument("--velocity_limit", type=float, default=10.0, help="Actuator velocity limit in rad/s")
parser.add_argument("--stiffness", type=float, default=100.0, help="Implicit actuator stiffness")
parser.add_argument("--damping", type=float, default=3.0, help="Implicit actuator damping")

parser.add_argument("--limit_fraction", type=float, default=0.90, help="Fraction of joint limit used for target")
parser.add_argument("--settle_steps", type=int, default=120, help="Steps to settle at neutral")
parser.add_argument("--hold_steps", type=int, default=180, help="Steps to hold each test target")
parser.add_argument("--tolerance", type=float, default=0.03, help="Position error tolerance in rad")
parser.add_argument("--saturation_fraction", type=float, default=0.95, help="Fraction of effort limit that counts as saturation")
parser.add_argument("--camera_eye", type=float, nargs=3, default=(3.0, 3.0, 2.0), help="Camera eye")
parser.add_argument("--camera_target", type=float, nargs=3, default=(0.0, 0.0, 1.0), help="Camera target")

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# Disable global actuator overrides
args.effort_limit = None
args.velocity_limit = None
args.stiffness = None
args.damping = None

# -----------------------------------------------------------------------------
# Launch app before Isaac imports
# -----------------------------------------------------------------------------

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# Isaac imports must happen after app launch
import torch
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def get_joint_names(robot: Articulation):
    if hasattr(robot, "joint_names"):
        return list(robot.joint_names)
    if hasattr(robot.data, "joint_names"):
        return list(robot.data.joint_names)
    raise RuntimeError("Could not find joint names on articulation.")


def get_joint_limits(robot: Articulation) -> Tuple[torch.Tensor, torch.Tensor]:
    """
    Returns lower, upper joint limits for env 0.
    Tries a few likely Isaac Lab data fields.
    """
    candidates = [
        "joint_pos_limits",
        "soft_joint_pos_limits",
        "default_joint_pos_limits",
    ]

    for name in candidates:
        if hasattr(robot.data, name):
            limits = getattr(robot.data, name)
            if limits is None:
                continue

            # Expected shape usually [num_envs, num_joints, 2]
            if isinstance(limits, torch.Tensor):
                if limits.ndim == 3 and limits.shape[-1] == 2:
                    lower = limits[0, :, 0].clone()
                    upper = limits[0, :, 1].clone()
                    return lower, upper
                if limits.ndim == 2 and limits.shape[-1] == 2:
                    lower = limits[:, 0].clone()
                    upper = limits[:, 1].clone()
                    return lower, upper

    raise RuntimeError(
        "Could not find joint limits on robot.data. "
        "Check available fields on your Isaac Lab build."
    )


def get_joint_positions(robot: Articulation) -> torch.Tensor:
    if hasattr(robot.data, "joint_pos"):
        return robot.data.joint_pos[0].clone()
    raise RuntimeError("Could not find robot.data.joint_pos")


def get_joint_efforts(robot: Articulation) -> Optional[torch.Tensor]:
    """
    Attempt to retrieve current joint efforts for env 0.

    Because Isaac Lab field names can vary between versions, this tries several
    likely attributes. If none are found, returns None.
    """
    candidates = [
        "applied_torque",
        "computed_torque",
        "joint_applied_torque",
        "joint_effort",
        "joint_efforts",
    ]

    for name in candidates:
        if hasattr(robot.data, name):
            value = getattr(robot.data, name)
            if isinstance(value, torch.Tensor):
                if value.ndim == 2:
                    return value[0].clone()
                if value.ndim == 1:
                    return value.clone()

    return None


def step_with_target(
    sim: SimulationContext,
    robot: Articulation,
    q_target: torch.Tensor,
    dt: float,
    steps: int,
) -> Tuple[torch.Tensor, Optional[torch.Tensor]]:
    """
    Step the sim while commanding a constant joint target.
    Returns:
        final joint position vector
        max absolute observed effort vector (or None if unavailable)
    """
    max_effort = None

    for _ in range(steps):
        robot.set_joint_position_target(q_target.unsqueeze(0))
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

        efforts = get_joint_efforts(robot)
        if efforts is not None:
            efforts_abs = efforts.abs()
            if max_effort is None:
                max_effort = efforts_abs.clone()
            else:
                max_effort = torch.maximum(max_effort, efforts_abs)

    final_pos = get_joint_positions(robot)
    return final_pos, max_effort


def move_to_neutral(
    sim: SimulationContext,
    robot: Articulation,
    q_neutral: torch.Tensor,
    dt: float,
    settle_steps: int,
):
    step_with_target(sim, robot, q_neutral, dt, settle_steps)


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    if not os.path.isfile(args.usd):
        raise FileNotFoundError(f"USD not found: {args.usd}")

    # Simulation config
    sim_cfg = sim_utils.SimulationCfg(
        dt=args.dt,
        device=args.device,
        gravity=tuple(args.gravity),
    )
    sim = SimulationContext(sim_cfg)

    # Scene
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)

    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0)
    light_cfg.func("/World/light", light_cfg)

    actuators = {}

    for group_name, cfg in ACTUATOR_SETTINGS.items():
        actuators[group_name] = ImplicitActuatorCfg(
            joint_names_expr=cfg["joint_names"],
            effort_limit_sim=cfg["effort_limit"],
            velocity_limit_sim=cfg["velocity_limit"],
            stiffness=cfg["stiffness"],
            damping=cfg["damping"],
        )

    # Robot
    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=args.usd),

        # FIX THE BASE FOR THIS TEST
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),
        ),
        actuators=actuators
    )

    robot = Articulation(robot_cfg)

    # Reset
    sim.reset()
    robot.reset()
    sim.set_camera_view(list(args.camera_eye), list(args.camera_target))

    # Let buffers populate
    robot.update(args.dt)

    joint_names = robot.joint_names

    (   
        effort_limits_per_joint, 
        velocity_limits_per_joint, 
        stiffness_per_joint, 
        damping_per_joint,
    ) = build_per_joint_limits_and_gains(joint_names)


    print("\n[RUNTIME JOINT GAINS]")
    for i, name in enumerate(joint_names):
        k = float(robot.data.joint_stiffness[0, i].item())
        d = float(robot.data.joint_damping[0, i].item())
        print(f"{i:2d}  {name:40s}  Kp={k:8.3f}  Kd={d:8.3f}")

    print("\n[RUNTIME LIMITS]")
    for i, name in enumerate(joint_names):
        v = float(robot.data.joint_vel_limits[0, i].item())
        print(f"{i:2d}  {name:40s}  vel_limit={v:8.3f}")

    joint_names = get_joint_names(robot)
    lower_limits, upper_limits = get_joint_limits(robot)
    q_neutral = get_joint_positions(robot)

    num_joints = len(joint_names)

    print(f"[INFO] Loaded robot: {args.usd}")
    print(f"[INFO] Number of joints: {num_joints}")
    print("[INFO] Per-joint actuator settings active from ACTUATOR_SETTINGS")
    print(f"[INFO] CSV output: {args.output_csv}")

    sample_effort = get_joint_efforts(robot)
    torque_readback_available = sample_effort is not None
    if torque_readback_available:
        print("[INFO] Joint effort readback detected.")
    else:
        print("[WARN] No joint effort field detected on robot.data.")
        print("[WARN] CSV will still be produced, but torque fields will be blank.")
        print("[WARN] If needed, update get_joint_efforts() with the correct field for your Isaac Lab build.")

    results = []

    # Initial settle at neutral
    move_to_neutral(sim, robot, q_neutral, args.dt, args.settle_steps)

    for joint_i, joint_name in enumerate(joint_names):
        print(f"\n[TEST] Joint {joint_i}: {joint_name}")

        j_lower = float(lower_limits[joint_i].item())
        j_upper = float(upper_limits[joint_i].item())

        # Skip effectively fixed joints
        if abs(j_upper - j_lower) < 1e-6:
            print("  [SKIP] Joint appears fixed.")
            results.append({
                "joint_index": joint_i,
                "joint_name": joint_name,
                "direction": "fixed",
                "lower_limit_rad": j_lower,
                "upper_limit_rad": j_upper,
                "target_rad": "",
                "reached_rad": "",
                "position_error_rad": "",
                "max_abs_effort_nm": "",
                "effort_limit_nm": effort_limits_per_joint[joint_i],
                "saturation_reached": "",
                "torque_clip_ok": "",
                "overall_pass": "",
                "notes": "fixed joint / zero motion range",
            })
            continue

        test_targets = [
            ("lower", j_lower * args.limit_fraction),
            ("upper", j_upper * args.limit_fraction),
        ]

        for direction, target_value in test_targets:
            # Return to neutral before each test
            move_to_neutral(sim, robot, q_neutral, args.dt, args.settle_steps)

            q_target = q_neutral.clone()
            q_target[joint_i] = target_value

            reached_pos, max_effort = step_with_target(
                sim=sim,
                robot=robot,
                q_target=q_target,
                dt=args.dt,
                steps=args.hold_steps,
            )

            reached_value = float(reached_pos[joint_i].item())
            pos_error = reached_value - float(target_value)

            joint_effort_limit = float(effort_limits_per_joint[joint_i])

            if max_effort is not None:
                max_abs_effort = float(max_effort[joint_i].item())
                saturation_reached = max_abs_effort >= (joint_effort_limit * args.saturation_fraction)
                torque_clip_ok = max_abs_effort <= (joint_effort_limit * 1.05)
            else:
                max_abs_effort = None
                saturation_reached = None
                torque_clip_ok = None

            # Overall pass logic:
            # - If torque readback exists: effort should not exceed limit significantly.
            # - If no torque readback: use position-error-only check as a fallback.
            if torque_clip_ok is not None:
                overall_pass = bool(torque_clip_ok)
            else:
                overall_pass = abs(pos_error) <= args.tolerance

            note_parts = []
            if max_effort is None:
                note_parts.append("torque readback unavailable")
            if abs(pos_error) > args.tolerance:
                note_parts.append("position error exceeds tolerance")
            if torque_clip_ok is False:
                note_parts.append("observed effort exceeded configured limit")
            if saturation_reached is True:
                note_parts.append("effort reached saturation band")

            notes = "; ".join(note_parts)

            print(
                f"  [{direction.upper()}] "
                f"target={target_value:+.4f} rad, "
                f"reached={reached_value:+.4f} rad, "
                f"error={pos_error:+.4f} rad, "
                f"max_effort={max_abs_effort if max_abs_effort is not None else 'N/A'}"
            )

            results.append({
                "joint_index": joint_i,
                "joint_name": joint_name,
                "direction": direction,
                "lower_limit_rad": j_lower,
                "upper_limit_rad": j_upper,
                "target_rad": float(target_value),
                "reached_rad": reached_value,
                "position_error_rad": pos_error,
                "max_abs_effort_nm": max_abs_effort if max_abs_effort is not None else "",
                "effort_limit_nm": float(effort_limits_per_joint[joint_i]),
                "saturation_reached": "" if saturation_reached is None else saturation_reached,
                "torque_clip_ok": "" if torque_clip_ok is None else torque_clip_ok,
                "overall_pass": overall_pass,
                "notes": notes,
            })

            # Return to neutral after each test
            move_to_neutral(sim, robot, q_neutral, args.dt, args.settle_steps)

    # Write CSV
    os.makedirs(os.path.dirname(os.path.abspath(args.output_csv)), exist_ok=True)

    fieldnames = [
        "joint_index",
        "joint_name",
        "direction",
        "lower_limit_rad",
        "upper_limit_rad",
        "target_rad",
        "reached_rad",
        "position_error_rad",
        "max_abs_effort_nm",
        "effort_limit_nm",
        "saturation_reached",
        "torque_clip_ok",
        "overall_pass",
        "notes",
    ]

    with open(args.output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    total_tests = sum(1 for r in results if r["direction"] in ("lower", "upper"))
    total_pass = sum(1 for r in results if r["direction"] in ("lower", "upper") and r["overall_pass"] is True)

    print("\n[SUMMARY]")
    print(f"  Total tests: {total_tests}")
    print(f"  Passed:      {total_pass}")
    print(f"  Failed:      {total_tests - total_pass}")
    print(f"  Results CSV: {args.output_csv}")

    # Keep the app alive only while running interactively
    # then close cleanly.
    simulation_app.close()


if __name__ == "__main__":
    main()