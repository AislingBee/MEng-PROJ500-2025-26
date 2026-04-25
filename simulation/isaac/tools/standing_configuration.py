#!/usr/bin/env python3
"""
standing_configuration.py

PROJ500 - Isaac Lab standing configuration validation

Purpose
-------
Spawn the humanoid into a nominal standing pose and hold it using
joint position targets with the current implicit actuator setup.

This is intended to define and validate a stable reset posture
for future RL work.
"""

from simulation.isaac.configuration.walking_actuator_config import (
    WALKING_ACTUATOR_SETTINGS,
)

import argparse
import math
import os
import time

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="PROJ500 standing configuration validation")
parser.add_argument("--usd", type=str, required=True, help="Path to robot USD")

parser.add_argument("--dt", type=float, default=1.0 / 120.0, help="Simulation timestep")
parser.add_argument("--gravity", type=float, nargs=3, default=(0.0, 0.0, -9.81), help="Gravity vector")

parser.add_argument("--base_height", type=float, default=0.83, help="Robot base height")
parser.add_argument("--settle_steps", type=int, default=240, help="Steps to hold standing pose")
parser.add_argument("--camera_eye", type=float, nargs=3, default=(3.0, 3.0, 2.0), help="Camera eye")
parser.add_argument("--camera_target", type=float, nargs=3, default=(0.0, 0.0, 1.0), help="Camera target")

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

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

from simulation.isaac.configuration.standing_pose import STANDING_TARGETS_DEG


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def get_joint_positions(robot: Articulation) -> torch.Tensor:
    if hasattr(robot.data, "joint_pos"):
        return robot.data.joint_pos[0].clone()
    raise RuntimeError("Could not find robot.data.joint_pos")


def build_standing_target_tensor(robot: Articulation, device: str) -> torch.Tensor:
    joint_names = list(robot.joint_names)
    q_target = torch.zeros(len(joint_names), dtype=torch.float32, device=device)

    for i, name in enumerate(joint_names):
        if name in STANDING_TARGETS_DEG:
            q_target[i] = math.radians(STANDING_TARGETS_DEG[name])

    return q_target


def step_with_target(
    sim: SimulationContext,
    robot: Articulation,
    q_target: torch.Tensor,
    dt: float,
    steps: int,
):
    for _ in range(steps):
        robot.set_joint_position_target(q_target.unsqueeze(0))
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)


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

    # Per-joint actuator setup
    actuators = {}
    for group_name, cfg in WALKING_ACTUATOR_SETTINGS.items():
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
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, args.base_height),
        ),
        actuators=actuators,
    )

    robot = Articulation(robot_cfg)

    # Reset
    sim.reset()
    robot.reset()
    sim.set_camera_view(list(args.camera_eye), list(args.camera_target))

    # Let buffers populate
    robot.update(args.dt)

    # print("[INFO] Waiting for viewer to initialise...")
    # time.sleep(5.0)   # Small delay to allow pc to catch
    # print("[INFO] Starting standing pose...")

    print("[INFO] Robot spawned. Holding viewer before physics starts...")

    pause_seconds = 15.0
    t0 = time.time()

    while simulation_app.is_running() and (time.time() - t0) < pause_seconds:
        sim.render()

    print("[INFO] Starting standing pose...")


    joint_names = list(robot.joint_names)
    q_target = build_standing_target_tensor(robot, args.device)

    print(f"[INFO] Loaded robot: {args.usd}")
    print(f"[INFO] Number of joints: {len(joint_names)}")
    print("[INFO] Per-joint actuator settings active from WALKING_ACTUATOR_SETTINGS")
    print(f"[INFO] Base height: {args.base_height:.3f} m")
    print("[INFO] Standing targets:")

    for i, name in enumerate(joint_names):
        target_deg = math.degrees(float(q_target[i].item()))
        print(f"  {i:2d}  {name:40s}  target={target_deg:+7.2f} deg")

    print("\n[INFO] Applying standing pose...")
    step_with_target(sim, robot, q_target, args.dt, args.settle_steps)

    q_final = get_joint_positions(robot)
    print("\n[RESULT] Final joint positions:")
    for i, name in enumerate(joint_names):
        final_deg = math.degrees(float(q_final[i].item()))
        target_deg = math.degrees(float(q_target[i].item()))
        err_deg = final_deg - target_deg
        print(
            f"  {i:2d}  {name:40s}  "
            f"target={target_deg:+7.2f} deg  "
            f"final={final_deg:+7.2f} deg  "
            f"err={err_deg:+7.2f} deg"
        )

    print("\n[INFO] Holding pose. Close app window to exit.")

    while simulation_app.is_running():
        robot.set_joint_position_target(q_target.unsqueeze(0))
        robot.write_data_to_sim()
        sim.step()
        robot.update(args.dt)

    simulation_app.close()


if __name__ == "__main__":
    main()