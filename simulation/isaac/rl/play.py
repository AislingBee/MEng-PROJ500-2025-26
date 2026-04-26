#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import importlib.util
from pathlib import Path
from importlib.metadata import PackageNotFoundError, version

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Play back PROJ500 humanoid walk PPO checkpoint.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--task", type=str, default="Humanoid-Stand-s2r-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--checkpoint", type=str, required=True, help="Path to .pt checkpoint file")
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
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg

THIS_DIR = Path(__file__).resolve().parent
ISAAC_DIR = THIS_DIR.parent  # simulation/isaac

# -----------------------------------------------------------------------------
# Task registration
# -----------------------------------------------------------------------------
#
# # Walking Task #################################################################
#
# # task registration
# task_file = ISAAC_DIR / "tasks" / "humanoid_walk_task.py"
# spec = importlib.util.spec_from_file_location("humanoid_walk_task", task_file)
# task_module = importlib.util.module_from_spec(spec)
# spec.loader.exec_module(task_module)
#
# # PPO config
# ppo_cfg_file = ISAAC_DIR / "configuration" / "humanoid_walk_ppo_cfg.py"
# spec = importlib.util.spec_from_file_location("humanoid_walk_ppo_cfg", ppo_cfg_file)
# ppo_cfg_module = importlib.util.module_from_spec(spec)
# spec.loader.exec_module(ppo_cfg_module)
# get_humanoid_walk_ppo_cfg = ppo_cfg_module.get_humanoid_walk_ppo_cfg

# Standing S2R Task #################################################################

# task registration
task_file = ISAAC_DIR / "tasks" / "humanoid_stand_s2r_task.py"
spec = importlib.util.spec_from_file_location("humanoid_stand_s2r_task", task_file)
task_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task_module)

# PPO config
ppo_cfg_file = ISAAC_DIR / "configuration" / "humanoid_stand_ppo_cfg.py"
spec = importlib.util.spec_from_file_location("humanoid_stand_ppo_cfg", ppo_cfg_file)
ppo_cfg_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ppo_cfg_module)
get_humanoid_stand_ppo_cfg = ppo_cfg_module.get_humanoid_stand_ppo_cfg

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


def main():
    from simulation.isaac.rl.envs.humanoid_stand_s2r_env import HumanoidStandEnvS2rCfg

    checkpoint_path = Path(args.checkpoint).resolve()
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    env_cfg = HumanoidStandEnvS2rCfg()
    env_cfg.scene.num_envs = args.num_envs

    # GUI visual inspection, so no video wrapper and no rgb_array needed
    env = gym.make(args.task, cfg=env_cfg, render_mode=None)

    # wrap for RSL-RL
    env = RslRlVecEnvWrapper(env, clip_actions=1.0)
    print("Env ready.")

    # PPO config
    agent_cfg = get_humanoid_stand_ppo_cfg()

    # convert deprecated Isaac Lab / rsl_rl config fields
    try:
        rsl_rl_version = version("rsl-rl-lib")
    except PackageNotFoundError:
        rsl_rl_version = version("rsl-rl")

    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, rsl_rl_version)
    cfg_dict = agent_cfg.to_dict()

    # strip legacy fields if still present after conversion
    if "actor" in cfg_dict:
        cfg_dict["actor"].pop("stochastic", None)
        cfg_dict["actor"].pop("init_noise_std", None)
        cfg_dict["actor"].pop("noise_std_type", None)
        cfg_dict["actor"].pop("state_dependent_std", None)

    if "critic" in cfg_dict:
        cfg_dict["critic"].pop("stochastic", None)
        cfg_dict["critic"].pop("init_noise_std", None)
        cfg_dict["critic"].pop("noise_std_type", None)
        cfg_dict["critic"].pop("state_dependent_std", None)

    # dummy log dir for runner construction
    log_dir = os.path.abspath(os.path.join("logs", "rsl_rl", "playback"))
    os.makedirs(log_dir, exist_ok=True)

    print("Creating PPO runner...")
    runner = OnPolicyRunner(env, cfg_dict, log_dir=log_dir, device=agent_cfg.device)

    print(f"Loading checkpoint: {checkpoint_path}")
    runner.load(str(checkpoint_path))

    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.reset()
    print("Starting playback...")

    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)

            # optional reset handling if needed
            if torch.any(dones):
                obs, _ = env.reset()

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
