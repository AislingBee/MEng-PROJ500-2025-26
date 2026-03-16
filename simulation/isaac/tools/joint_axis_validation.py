import argparse
import os

from isaaclab.app import AppLauncher


def main():
    # ---------------- CLI ----------------
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)
    parser.add_argument("--usd", type=str, required=True, help="Path to robot USD file.")
    # You can drive tests by joint index for now (simple + robust)
    parser.add_argument("--joint_index", type=int, default=0, help="Joint index to test.")
    parser.add_argument("--step_rad", type=float, default=0.5, help="Step size in radians (+/-).")
    parser.add_argument("--hold_s", type=float, default=1.0, help="How long to hold each step (seconds).")
    args = parser.parse_args()

    usd_path = os.path.abspath(args.usd)
    if not os.path.isfile(usd_path):
        raise FileNotFoundError(f"USD not found: {usd_path}")

    # ---------------- Launch Isaac Sim via Isaac Lab ----------------
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    # Isaac Lab imports (inside, after app launch)
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.sim import SimulationContext
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.actuators import ImplicitActuatorCfg

    # ---------------- Simulation Config (NO GRAVITY) ----------------
    sim_cfg = sim_utils.SimulationCfg(
        dt=1.0 / 120.0,
        device=args.device,
        gravity=(0.0, 0.0, 0.0),  # <-- key change for kinematic validation
        physx=sim_utils.PhysxCfg(
            enable_external_forces_every_iteration=True,
        ),
    )
    sim = SimulationContext(sim_cfg)

    # ---------------- Scene ----------------
    # Ground is optional with gravity disabled, but keeping it is fine.
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
            # OPTIONAL (recommended later):
            # joint_pos={ "hip_pitch_l": 0.1, "knee_l": -0.2, ... }
        ),
    )

    robot = Articulation(robot_cfg)

    # ---------------- Reset ----------------
    sim.reset()
    robot.reset()

    sim.set_camera_view([3.0, 3.0, 2.0], [0.0, 0.0, 1.0])

# ----------------Define Joint Names---------------------------

    JOINT_NAMES = [
        "robot_pelvis_link_l_yaw_joint",
        "robot_l_hip_yaw_link_l_pitch_joint",
        "robot_l_hip_pitch_link_l_roll_joint",
        "robot_l_thigh_link_l_knee_joint",
        "robot_l_shank_link_l_ankle_joint",
        "robot_l_ankle_link_l_foot_joint",
        "robot_pelvis_link_r_yaw_joint",
        "robot_r_hip_yaw_link_r_pitch_joint",
        "robot_r_hip_pitch_link_r_roll_joint",
        "robot_r_thigh_link_r_knee_joint",
        "robot_r_shank_link_r_ankle_joint",
        "robot_r_ankle_link_r_foot_joint",
    ]

    joint_name_to_index = {
        name: robot.joint_names.index(name) for name in JOINT_NAMES
    }

    # TEMP: dump all joint names so we can see the exact strings
    # print("\n--- robot.joint_names (Isaac articulation) ---")
    # for i, n in enumerate(list(robot.joint_names)):
    #     print(f"{i:02d} : {n}")
    # print("--------------------------------------------\n")

    # # Stop here so it doesn't crash later
    # raise SystemExit("Printed joint names. Update JOINT_NAMES to match exactly.")


    # ---------------- Define Neutral "Zero" Pose ----------------
    # This is NOT changing URDF zero; it's capturing a reference pose.
    # You can replace this with your own "standing neutral" init_state joint_pos later.
    robot.update(sim_cfg.dt)  # ensure buffers are populated
    q_zero = robot.data.joint_pos[0].clone()  # tensor [num_joints]
    # If you want: print(q_zero) for sanity, but it’s usually noisy.

    # ---------------- Test Setup ----------------
    device = robot.device
    num_joints = robot.num_joints

    joint_i = int(args.joint_index)
    if joint_i < 0 or joint_i >= num_joints:
        raise ValueError(f"joint_index out of range. Must be 0..{num_joints-1}")

    step_rad = float(args.step_rad)
    hold_steps = max(1, int(args.hold_s / sim_cfg.dt))

    # Target buffer (starts at neutral)
    q_target = q_zero.clone()

    # Simple 3-state cycle: neutral -> +step -> -step -> neutral ...
    states = [0.0, +step_rad, -step_rad, 0.0]
    state_k = 0
    state_counter = 0

    print(f"Testing joint index {joint_i}: {robot.joint_names[joint_i]}")
    print(f"Hold {args.hold_s:.2f}s each at dt={sim_cfg.dt:.5f} ({hold_steps} steps)")
    print("Sequence: neutral -> +step -> -step -> neutral -> repeat\n")

    # ---------------- Main Loop ----------------
    while simulation_app.is_running():
        # Update state timing
        state_counter += 1
        if state_counter >= hold_steps:
            state_counter = 0
            state_k = (state_k + 1) % len(states)

        # Command only one joint away from neutral
        q_target[:] = q_zero
        q_target[joint_i] = q_zero[joint_i] + states[state_k]

        # Send targets to robot
        robot.set_joint_position_target(q_target.unsqueeze(0))
        robot.write_data_to_sim()

        # Step sim
        sim.step()
        robot.update(sim_cfg.dt)

        # print(q_target)
        # print the command being sent. 
        print("state:", states[state_k]) 

    simulation_app.close()


if __name__ == "__main__":
    main()