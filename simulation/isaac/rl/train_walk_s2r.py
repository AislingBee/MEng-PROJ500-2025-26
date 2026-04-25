#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from datetime import datetime
import importlib.util
from pathlib import Path
from importlib.metadata import PackageNotFoundError, version

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Train PROJ500 humanoid WALK Sim to Real with PPO.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--num_envs", type=int, default=4) #
parser.add_argument("--max_iterations", type=int, default=200)
parser.add_argument("--task", type=str, default="Humanoid-Walk-s2r-v0")
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

# Training Task Selection ###########################################################
#####################################################################################
# Update this section when changing the training type.
# Walking Sim 2 Real Task #####################################################################

# task registration
task_file = ISAAC_DIR / "tasks" / "humanoid_walk_s2r_task.py"
spec = importlib.util.spec_from_file_location("humanoid_walk_s2r_task", task_file)
task_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task_module)

# PPO config
ppo_cfg_file = ISAAC_DIR / "configuration" / "humanoid_walk_ppo_cfg.py"
spec = importlib.util.spec_from_file_location("humanoid_walk_ppo_cfg", ppo_cfg_file)
ppo_cfg_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ppo_cfg_module)
get_humanoid_walk_ppo_cfg = ppo_cfg_module.get_humanoid_walk_ppo_cfg


def export_deployable_policy(runner, env_cfg, export_dir: str) -> None:
    os.makedirs(export_dir, exist_ok=True)

    actor = runner.alg.actor
    actor.eval()

    device = next(actor.parameters()).device
    example_obs = {
        "policy": torch.zeros(1, env_cfg.observation_space, device=device)
    }

    scripted_actor = torch.jit.trace(actor, (example_obs,))
    scripted_actor.save(os.path.join(export_dir, "policy_jit.pt"))

    torch.save(actor.state_dict(), os.path.join(export_dir, "actor_state_dict.pt"))

    print(f"Deployable TorchScript actor saved to: {os.path.join(export_dir, 'policy_jit.pt')}")


def main():
    from simulation.isaac.rl.envs.humanoid_walk_s2r_env import HumanoidWalkEnvS2rCfg

    env_cfg = HumanoidWalkEnvS2rCfg()

    env_cfg.scene.num_envs = args.num_envs

    env = gym.make(args.task, cfg=env_cfg, render_mode=None)

    # wrap for RSL-RL
    env = RslRlVecEnvWrapper(env, clip_actions=1.0)
    print("Env ready.")

    # get PPO config
    agent_cfg = get_humanoid_walk_ppo_cfg()

    agent_cfg.max_iterations = args.max_iterations

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

    # logging
    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    os.makedirs(log_root, exist_ok=True)

    run_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(log_root, run_name)
    os.makedirs(log_dir, exist_ok=True)

    # runner
    print("Creating PPO runner...")
    runner = OnPolicyRunner(env, cfg_dict, log_dir=log_dir, device=agent_cfg.device)

    print("Starting training...")
    runner.learn(
        num_learning_iterations=agent_cfg.max_iterations,
        init_at_random_ep_len=True,
    )

    export_deployable_policy(runner, env_cfg, os.path.join(log_dir, "exported"))

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
