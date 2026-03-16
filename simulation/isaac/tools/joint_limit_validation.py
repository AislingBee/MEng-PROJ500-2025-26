import argparse
import os
import csv


from isaaclab.app import AppLauncher


def main():
    # ---------------- CLI ----------------
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)

    parser.add_argument("--usd", type=str, required=True, help="Path to robot USD file.")
    parser.add_argument(
        "--joint_index",
        type=int,
        default=-1,
        help="Joint index to test. Use -1 to test all joints in sequence.",
    )
    parser.add_argument(
        "--hold_s",
        type=float,
        default=1.0,
        help="Hold time at each commanded target (seconds).",
    )
    parser.add_argument(
        "--settle_tol_rad",
        type=float,
        default=0.03,
        help="Allowed steady-state error tolerance in radians.",
    )
    parser.add_argument(
        "--limit_fraction",
        type=float,
        default=0.90,
        help="Fraction of lower/upper limit to command during validation.",
    )
    parser.add_argument(
        "--csv_out",
        type=str,
        default="joint_limit_results.csv",
        help="CSV file to save results.",
    )
    parser.add_argument(
        "--enable_self_collisions",
        action="store_true",
        help="Enable self-collisions in articulation props.",
    )
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
                enabled_self_collisions=args.enable_self_collisions,
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

    # ---------------- Buffer update ----------------
    robot.update(sim_cfg.dt)

    q_zero = robot.data.joint_pos[0].clone()
    num_joints = robot.num_joints
    hold_steps = max(1, int(args.hold_s / sim_cfg.dt))
    settle_tol = float(args.settle_tol_rad)
    limit_fraction = float(args.limit_fraction)

    # ---------------- Get joint limits robustly ----------------
    joint_limits = None
    source_name = None

    if hasattr(robot.data, "joint_pos_limits") and robot.data.joint_pos_limits is not None:
        joint_limits = robot.data.joint_pos_limits[0].clone()
        source_name = "robot.data.joint_pos_limits"
    elif hasattr(robot.data, "soft_joint_pos_limits") and robot.data.soft_joint_pos_limits is not None:
        joint_limits = robot.data.soft_joint_pos_limits[0].clone()
        source_name = "robot.data.soft_joint_pos_limits"
    elif hasattr(robot.data, "default_joint_pos_limits") and robot.data.default_joint_pos_limits is not None:
        joint_limits = robot.data.default_joint_pos_limits[0].clone()
        source_name = "robot.data.default_joint_pos_limits"
    else:
        raise RuntimeError("Could not find joint position limits on robot.data")

    # ---------------- Print joint limit table ----------------
    print("\n=== JOINT LIMIT TABLE ===")
    print(f"Limit source: {source_name}")
    print(f"{'idx':>3}  {'joint_name':<50} {'lower(rad)':>12} {'upper(rad)':>12} {'lower(deg)':>12} {'upper(deg)':>12}")
    for i, name in enumerate(robot.joint_names):
        lower = float(joint_limits[i, 0].item())
        upper = float(joint_limits[i, 1].item())
        lower_deg = lower * 180.0 / 3.141592653589793
        upper_deg = upper * 180.0 / 3.141592653589793
        print(f"{i:>3}  {name:<50} {lower:>12.4f} {upper:>12.4f} {lower_deg:>12.2f} {upper_deg:>12.2f}")
    print("")

    # ---------------- Choose joints to test ----------------
    if args.joint_index == -1:
        joints_to_test = list(range(num_joints))
    else:
        if args.joint_index < 0 or args.joint_index >= num_joints:
            raise ValueError(f"joint_index out of range. Must be -1 or 0..{num_joints - 1}")
        joints_to_test = [args.joint_index]

    device = robot.device
    q_target = q_zero.clone()

    results = []

    def step_and_hold(target_q, steps):
        """Command joint position targets and step simulation."""
        for _ in range(steps):
            robot.set_joint_position_target(target_q.unsqueeze(0))
            robot.write_data_to_sim()
            sim.step()
            robot.update(sim_cfg.dt)

    def evaluate_target(joint_i, label, target_value):
        """Move to target, measure reached position, compute error."""
        q_target[:] = q_zero
        q_target[joint_i] = target_value

        print(f"  -> {label:<12} target = {target_value:+.4f} rad")
        step_and_hold(q_target, hold_steps)

        reached = float(robot.data.joint_pos[0, joint_i].item())
        error = reached - target_value
        passed = abs(error) <= settle_tol

        print(
            f"     reached = {reached:+.4f} rad | "
            f"error = {error:+.4f} rad | "
            f"{'PASS' if passed else 'FAIL'}"
        )

        return reached, error, passed

    # ---------------- Main validation loop ----------------
    print("=== STARTING JOINT LIMIT VALIDATION ===")
    print(f"Testing {len(joints_to_test)} joint(s)")
    print(f"Hold time per state: {args.hold_s:.2f} s ({hold_steps} sim steps)")
    print(f"Limit fraction: {limit_fraction:.2f}")
    print(f"Settle tolerance: {settle_tol:.4f} rad\n")

    # First settle robot at neutral
    step_and_hold(q_zero, hold_steps)

    for joint_i in joints_to_test:
        name = robot.joint_names[joint_i]
        lower = float(joint_limits[joint_i, 0].item())
        upper = float(joint_limits[joint_i, 1].item())
        neutral = float(q_zero[joint_i].item())

        target_lower = lower * limit_fraction
        target_upper = upper * limit_fraction

        print(f"\n--- Joint {joint_i}: {name} ---")
        print(f"    lower = {lower:+.4f} rad")
        print(f"    upper = {upper:+.4f} rad")
        print(f"    neutral(reference) = {neutral:+.4f} rad")

        # neutral
        reached_neutral_1, error_neutral_1, pass_neutral_1 = evaluate_target(joint_i, "neutral_1", neutral)

        # lower
        reached_lower, error_lower, pass_lower = evaluate_target(joint_i, "lower_test", target_lower)

        # neutral again
        reached_neutral_2, error_neutral_2, pass_neutral_2 = evaluate_target(joint_i, "neutral_2", neutral)

        # upper
        reached_upper, error_upper, pass_upper = evaluate_target(joint_i, "upper_test", target_upper)

        # final neutral
        reached_neutral_3, error_neutral_3, pass_neutral_3 = evaluate_target(joint_i, "neutral_3", neutral)

        overall_pass = all([
            pass_neutral_1,
            pass_lower,
            pass_neutral_2,
            pass_upper,
            pass_neutral_3,
        ])

        results.append({
            "joint_index": joint_i,
            "joint_name": name,
            "lower_limit_rad": lower,
            "upper_limit_rad": upper,
            "neutral_ref_rad": neutral,
            "target_lower_rad": target_lower,
            "reached_lower_rad": reached_lower,
            "lower_error_rad": error_lower,
            "target_upper_rad": target_upper,
            "reached_upper_rad": reached_upper,
            "upper_error_rad": error_upper,
            "pass_neutral_1": pass_neutral_1,
            "pass_lower": pass_lower,
            "pass_neutral_2": pass_neutral_2,
            "pass_upper": pass_upper,
            "pass_neutral_3": pass_neutral_3,
            "overall_pass": overall_pass,
        })

        print(f"    RESULT: {'PASS' if overall_pass else 'FAIL'}")

    # ---------------- Return to neutral and hold ----------------
    print("\nReturning to neutral pose...")
    step_and_hold(q_zero, hold_steps)

    # ---------------- Save CSV ----------------
    csv_path = os.path.abspath(args.csv_out)
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()) if results else [
            "joint_index",
            "joint_name",
            "lower_limit_rad",
            "upper_limit_rad",
            "neutral_ref_rad",
            "target_lower_rad",
            "reached_lower_rad",
            "lower_error_rad",
            "target_upper_rad",
            "reached_upper_rad",
            "upper_error_rad",
            "pass_neutral_1",
            "pass_lower",
            "pass_neutral_2",
            "pass_upper",
            "pass_neutral_3",
            "overall_pass",
        ])
        writer.writeheader()
        for row in results:
            writer.writerow(row)

    print(f"\nResults written to: {csv_path}")


    # ---------------- Keep app open until closed ----------------
    print("\nValidation complete. Close the Isaac Sim window to exit.")
    while simulation_app.is_running():
        sim.step()
        robot.update(sim_cfg.dt)

    simulation_app.close()


if __name__ == "__main__":
    main()