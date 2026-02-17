import argparse
import os

from isaaclab.app import AppLauncher


def main():
    parser = argparse.ArgumentParser()

    # ADD THIS FIRST
    AppLauncher.add_app_launcher_args(parser)

    # Only add YOUR custom args
    parser.add_argument("--usd", type=str, required=True, help="Path to robot USD file.")

    args = parser.parse_args()

    usd_path = os.path.abspath(args.usd)
    if not os.path.isfile(usd_path):
        raise FileNotFoundError(f"USD not found: {usd_path}")

    # Use Isaac Lab launcher args directly
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import isaaclab.sim as sim_utils
    from isaaclab.sim import SimulationContext
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.actuators import ImplicitActuatorCfg


    # Notice: device now comes from args.device automatically
    sim_cfg = sim_utils.SimulationCfg(
        dt=1.0 / 120.0,
        device=args.device
    )
    sim = SimulationContext(sim_cfg)

    # Scene
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)

    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0)
    light_cfg.func("/World/light", light_cfg)

    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            # THIS is the important part:
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=1,
            ),
            # (optional but recommended)
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_depenetration_velocity=5.0,
            ),
        ),
        actuators={
            "all_joints": ImplicitActuatorCfg(
                joint_names_expr=".*",
                effort_limit=200.0,
                velocity_limit=100.0,
                stiffness=0.0,
                damping=0.0,
            )
        },
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )



    robot = Articulation(robot_cfg)

    sim.reset()
    robot.reset()

    sim.set_camera_view([3.0, 3.0, 2.0], [0.0, 0.0, 1.0])

    while simulation_app.is_running():
        sim.step()
        robot.update(sim_cfg.dt)

    simulation_app.close()


if __name__ == "__main__":
    main()
