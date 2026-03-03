import torch
from isaaclab.app import AppLauncher
import argparse

from isaaclab.envs import DirectRLEnv
from .proj500_humanoid_cfg import Proj500HumanoidCfg
from .proj500_humanoid_env import Proj500HumanoidEnv


def main():
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    cfg = Proj500HumanoidCfg()
    env: DirectRLEnv = Proj500HumanoidEnv(cfg)

    obs, _ = env.reset()

    for i in range(2000):
        actions = torch.randn((env.num_envs, env.action_space.shape[0]), device=env.device).clamp(-1, 1)
        obs, rew, terminated, truncated, info = env.step(actions)

        if i % 200 == 0:
            print(i, rew.mean().item())

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()