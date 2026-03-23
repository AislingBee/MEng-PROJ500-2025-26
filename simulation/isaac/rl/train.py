#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from datetime import datetime

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Train PROJ500 humanoid standing policy with PPO.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--num_envs", type=int, default=256, help="Number of parallel environments.")
parser.add_argument("--max_iterations", type=int, default=1000, help="Number of PPO iterations.")
parser.add_argument("--seed", type=int, default=42, help="Random seed.")
args = parser.parse_args()

# launch Isaac Sim first
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# -----------------------------------------------------------------------------
# imports after app launch
# -----------------------------------------------------------------------------
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

from simulation.isaac.rl.envs.humanoid_stand_env import HumanoidStandEnv, HumanoidStandEnvCfg


def build_train_cfg(device: str, max_iterations: int, seed: int) -> dict:
    return {
        "seed": seed,
        "device": device,
        "num_steps_per_env": 24,
        "max_iterations": max_iterations,
        "empirical_normalization": False,
        "save_interval": 50,
        "experiment_name": "humanoid_stand",
        "run_name": "",
        "logger": "tensorboard",
        "policy": {
            "class_name": "ActorCritic",
            "init_noise_std": 1.0,
            "actor_hidden_dims": [256, 256, 128],
            "critic_hidden_dims": [256, 256, 128],
            "activation": "elu",
        },
        "algorithm": {
            "class_name": "PPO",
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "entropy_coef": 0.01,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "learning_rate": 1.0e-3,
            "schedule": "adaptive",
            "gamma": 0.99,
            "lam": 0.95,
            "desired_kl": 0.01,
            "max_grad_norm": 1.0,
        },
    }


def main():
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True

    env_cfg = HumanoidStandEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = args.seed

    env = HumanoidStandEnv(env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=1.0)

    train_cfg = build_train_cfg(
        device=env.unwrapped.device,
        max_iterations=args.max_iterations,
        seed=args.seed,
    )

    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", train_cfg["experiment_name"]))
    os.makedirs(log_root, exist_ok=True)

    run_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(log_root, run_name)
    os.makedirs(log_dir, exist_ok=True)

    runner = OnPolicyRunner(env, train_cfg, log_dir=log_dir, device=train_cfg["device"])
    runner.learn(
        num_learning_iterations=train_cfg["max_iterations"],
        init_at_random_ep_len=True,
    )

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()