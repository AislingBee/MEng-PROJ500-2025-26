import argparse
import json
import math
import os

from isaaclab.app import AppLauncher


JOINT_NAMES = [
    "pelvis_link_l_yaw_joint",
    "l_hip_yaw_link_l_pitch_joint",
    "l_hip_pitch_link_l_roll_joint",
    "l_thigh_link_l_knee_joint",
    "l_shank_link_l_ankle_joint",
    "l_ankle_link_l_foot_joint",
    "pelvis_link_r_yaw_joint",
    "r_hip_yaw_link_r_pitch_joint",
    "r_hip_pitch_link_r_roll_joint",
    "r_thigh_link_r_knee_joint",
    "r_shank_link_r_ankle_joint",
    "r_ankle_link_r_foot_joint",
]


def load_joint_pose_config(config_path: str) -> dict[str, float]:
    with open(config_path, "r", encoding="utf-8") as file:
        joint_pose_config = json.load(file)

    if not isinstance(joint_pose_config, dict):
        raise ValueError("Joint pose config must be a JSON object mapping joint_name to angle_rad.")

    invalid_joint_names = [name for name in joint_pose_config if name not in JOINT_NAMES]
    if invalid_joint_names:
        raise ValueError(
            "Joint pose config contains unsupported joints: "
            + ", ".join(sorted(invalid_joint_names))
        )

    parsed_joint_pose_config: dict[str, float] = {}
    for joint_name, angle_rad in joint_pose_config.items():
        if not isinstance(angle_rad, (int, float)):
            raise ValueError(f"Joint '{joint_name}' must map to a numeric angle in radians.")
        parsed_joint_pose_config[joint_name] = float(angle_rad)

    return parsed_joint_pose_config


def print_target_table(joint_pose_config: dict[str, float]) -> None:
    header = f"{'joint name':<34} | {'target rad':>12} | {'target deg':>12}"
    print(header)
    print("-" * len(header))
    for joint_name in JOINT_NAMES:
        if joint_name not in joint_pose_config:
            continue
        angle_rad = joint_pose_config[joint_name]
        angle_deg = math.degrees(angle_rad)
        print(f"{joint_name:<34} | {angle_rad:>12.6f} | {angle_deg:>12.3f}")


def main():
    # ---------------- CLI ----------------
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)
    parser.add_argument("--usd", type=str, required=True, help="Path to robot USD file.")
    parser.add_argument(
        "--joint_pose_config",
        type=str,
        required=True,
        help="Path to JSON file mapping joint names to target angles in radians.",
    )
    args = parser.parse_args()

    usd_path = os.path.abspath(args.usd)
    if not os.path.isfile(usd_path):
        raise FileNotFoundError(f"USD not found: {usd_path}")

    joint_pose_config_path = os.path.abspath(args.joint_pose_config)
    if not os.path.isfile(joint_pose_config_path):
        raise FileNotFoundError(f"Joint pose config not found: {joint_pose_config_path}")

    joint_pose_config = load_joint_pose_config(joint_pose_config_path)

    # ---------------- Launch Isaac Sim via Isaac Lab ----------------
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    # Isaac Lab imports (inside, after app launch)
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.sim import SimulationContext

    # ---------------- Simulation Config (NO GRAVITY) ----------------
    sim_cfg = sim_utils.SimulationCfg(
        dt=1.0 / 120.0,
        device=args.device,
        gravity=(0.0, 0.0, 0.0),
        physx=sim_utils.PhysxCfg(
            enable_external_forces_every_iteration=True,
        ),
    )
    sim = SimulationContext(sim_cfg)

    # ---------------- Scene ----------------
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)

    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0)
    light_cfg.func("/World/light", light_cfg)

    # ---------------- Robot ----------------
    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_depenetration_velocity=5.0,
            ),
        ),
        actuators={
            "all_joints": ImplicitActuatorCfg(
                joint_names_expr=".*",
                effort_limit_sim=200.0,
                velocity_limit_sim=100.0,
                stiffness=500.0,
                damping=50.0,
            )
        },
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    robot = Articulation(robot_cfg)

    # ---------------- Reset ----------------
    sim.reset()
    robot.reset()

    sim.set_camera_view([3.0, 3.0, 2.0], [0.0, 0.0, 1.0])

    joint_name_to_index = {name: robot.joint_names.index(name) for name in JOINT_NAMES}

    robot.update(sim_cfg.dt)
    q_target = robot.data.joint_pos[0].clone()

    for joint_name, angle_rad in joint_pose_config.items():
        joint_index = joint_name_to_index[joint_name]
        q_target[joint_index] = angle_rad

    print_target_table(joint_pose_config)
    print("\nCommanding target pose continuously. Close the simulation window to exit.")

    # ---------------- Main Loop ----------------
    while simulation_app.is_running():
        robot.set_joint_position_target(q_target.unsqueeze(0))
        robot.write_data_to_sim()

        sim.step()
        robot.update(sim_cfg.dt)

    simulation_app.close()


if __name__ == "__main__":
    main()
