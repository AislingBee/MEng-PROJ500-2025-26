import argparse
import os

from isaaclab.app import AppLauncher

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from isaaclab.assets import Articulation, ArticulationCfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=str, required=True, help="Path to robot USD file.")
    parser.add_argument("--device", type=str, default="cuda:0", help="cpu or cuda:0")
    parser.add_argument("--headless", action="store_true", help="Run without UI")
    args = parser.parse_args()

    usd_path = os.path.abspath(args.usd)
    if not os.path.isfile(usd_path):
        raise FileNotFoundError(f"USD not found: {usd_path}")

    # launch Isaac Sim (via Isaac Lab)
    app_launcher = AppLauncher(headless=args.headless)
    simulation_app = app_launcher.app

    # sim config
    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 120.0, device=args.device)
    sim = SimulationContext(sim_cfg)

    # basic scene
    sim_utils.GroundPlaneCfg(prim_path="/World/ground").func("/World/ground", sim_cfg)
    sim_utils.DomeLightCfg(prim_path="/World/light", intensity=2000.0).func("/World/light", sim_cfg)

    # robot articulation config (spawn from USD)
    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),  # start 1m above ground so you can see it drop
            rot=(1.0, 0.0, 0.0, 0.0),  # quaternion (w, x, y, z)
        ),
    )

    robot = Articulation(robot_cfg)

    # reset once (spawns everything)
    sim.reset()
    robot.reset()

    # run loop
    while simulation_app.is_running():
        sim.step()
        robot.update(sim_cfg.dt)

    simulation_app.close()


if __name__ == "__main__":
    main()
