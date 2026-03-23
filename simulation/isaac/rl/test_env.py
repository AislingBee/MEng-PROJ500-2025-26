#!/usr/bin/env python3

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="PROJ500 humanoid stand env test")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch

from simulation.isaac.rl.envs.humanoid_stand_env import HumanoidStandEnv, HumanoidStandEnvCfg


def main():
    cfg = HumanoidStandEnvCfg()
    env = HumanoidStandEnv(cfg)

    obs = env.reset()
    print("Reset OK, obs shape:", obs["policy"].shape)

    actions = torch.zeros((env.num_envs, cfg.action_space), device=env.device)

    for i in range(10):
        obs, reward, terminated, truncated, info = env.step(actions)
        print(f"Step {i} OK")

    print("Done")
    simulation_app.close()


if __name__ == "__main__":
    main()