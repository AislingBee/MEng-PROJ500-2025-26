#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from datetime import datetime

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Train PROJ500 humanoid stand with PPO.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--max_iterations", type=int, default=1000)
parser.add_argument("--task", type=str, default="Humanoid-Stand-v0")
args = parser.parse_args()

# launch Isaac Sim
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# -----------------------------------------------------------------------------
# imports AFTER launch
# -----------------------------------------------------------------------------
import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

import importlib.util
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent

# load task registration file directly
task_file = THIS_DIR / "tasks" / "humanoid_stand.py"
spec = importlib.util.spec_from_file_location("humanoid_stand_task", task_file)
task_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task_module)

# load PPO config file directly
ppo_cfg_file = THIS_DIR / "config" / "humanoid_stand_ppo_cfg.py"
spec = importlib.util.spec_from_file_location("humanoid_stand_ppo_cfg", ppo_cfg_file)
ppo_cfg_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ppo_cfg_module)
get_humanoid_stand_ppo_cfg = ppo_cfg_module.get_humanoid_stand_ppo_cfg

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


def main():
    # build env
    env = gym.make(args.task, render_mode=None)

    # override num envs
    env.unwrapped.cfg.scene.num_envs = args.num_envs

    # wrap for RSL-RL
    env = RslRlVecEnvWrapper(env, clip_actions=1.0)
    print("Env ready.")

    # get PPO config
    agent_cfg = get_humanoid_stand_ppo_cfg()
    agent_cfg.max_iterations = args.max_iterations

    # logging
    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    os.makedirs(log_root, exist_ok=True)

    run_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(log_root, run_name)
    os.makedirs(log_dir, exist_ok=True)

    # runner
    print("Creating PPO runner...")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

    print("Starting training...")
    runner.learn(
        num_learning_iterations=agent_cfg.max_iterations,
        init_at_random_ep_len=True,
    )

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()