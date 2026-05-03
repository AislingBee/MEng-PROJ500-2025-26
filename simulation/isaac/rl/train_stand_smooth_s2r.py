#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import os
from datetime import datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Train smooth deployable humanoid standing S2R PPO.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--num_envs", type=int, default=24000)
parser.add_argument("--max_iterations", type=int, default=200)
parser.add_argument("--task", type=str, default="Humanoid-Stand-Smooth-S2R-v0")
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
from rsl_rl.runners import OnPolicyRunner

from simulation.isaac.configuration.humanoid_stand_smooth_ppo_cfg import (
    SMOOTH_STAND_DEPLOYMENT_CFG,
)
from simulation.isaac.rl.envs.humanoid_stand_smooth_s2r_env import OBS_DIM


THIS_DIR = Path(__file__).resolve().parent
ISAAC_DIR = THIS_DIR.parent

task_file = ISAAC_DIR / "tasks" / "humanoid_stand_smooth_s2r_task.py"
spec = importlib.util.spec_from_file_location("humanoid_stand_smooth_s2r_task", task_file)
task_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task_module)

ppo_cfg_file = ISAAC_DIR / "configuration" / "humanoid_stand_smooth_ppo_cfg.py"
spec = importlib.util.spec_from_file_location("humanoid_stand_smooth_ppo_cfg", ppo_cfg_file)
ppo_cfg_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ppo_cfg_module)
get_humanoid_stand_smooth_ppo_cfg = ppo_cfg_module.get_humanoid_stand_smooth_ppo_cfg


class DeployableActor(torch.nn.Module):
    def __init__(self, actor: torch.nn.Module):
        super().__init__()
        self.actor = actor

    def forward(self, obs_dict: dict[str, torch.Tensor]) -> torch.Tensor:
        return self.actor(obs_dict)


def _find_obs_normalizer(runner) -> torch.nn.Module | None:
    for owner in (runner, getattr(runner, "alg", None)):
        if owner is None:
            continue
        for name in ("obs_normalizer", "empirical_normalizer", "normalizer", "obs_norm"):
            normalizer = getattr(owner, name, None)
            if isinstance(normalizer, torch.nn.Module):
                normalizer.eval()
                return normalizer
    return None


def export_deployable_policy(runner, export_dir: str) -> None:
    os.makedirs(export_dir, exist_ok=True)

    actor = runner.alg.actor
    actor.eval()
    device = next(actor.parameters()).device
    normalizer = _find_obs_normalizer(runner)

    example_obs = {"policy": torch.zeros(1, OBS_DIM, device=device)}
    scripted_actor = torch.jit.trace(DeployableActor(actor).to(device).eval(), (example_obs,))
    scripted_actor.save(os.path.join(export_dir, "policy_jit.pt"))
    torch.save(actor.state_dict(), os.path.join(export_dir, "actor_state_dict.pt"))

    normalizer_path = os.path.join(export_dir, SMOOTH_STAND_DEPLOYMENT_CFG.obs_normalizer_artifact_name)
    if SMOOTH_STAND_DEPLOYMENT_CFG.use_obs_normalization:
        if normalizer is None:
            if SMOOTH_STAND_DEPLOYMENT_CFG.obs_normalizer_required:
                raise RuntimeError(
                    "Smooth standing deploy export requires an observation normalizer, "
                    "but none was found on the trained runner."
                )
            print("Observation normalizer export skipped: no runner normalizer found.")
        else:
            example_obs_tensor = torch.zeros(1, OBS_DIM, device=device)
            scripted_normalizer = torch.jit.trace(normalizer.to(device).eval(), (example_obs_tensor,))
            scripted_normalizer.save(normalizer_path)
            print(f"Observation normalizer saved to: {normalizer_path}")

    print(f"Deployable TorchScript actor saved to: {os.path.join(export_dir, 'policy_jit.pt')}")


def main():
    from simulation.isaac.rl.envs.humanoid_stand_smooth_s2r_env import HumanoidStandSmoothS2REnvCfg

    env_cfg = HumanoidStandSmoothS2REnvCfg()
    if env_cfg.observation_space != OBS_DIM:
        raise RuntimeError(f"Smooth standing observation dim mismatch: cfg={env_cfg.observation_space}, layout={OBS_DIM}")

    env_cfg.scene.num_envs = args.num_envs
    env = gym.make(args.task, cfg=env_cfg, render_mode=None)

    # Use a wide wrapper action clip so raw policy saturation reaches the env
    # diagnostics and reward; the env still applies the deployable [-1, 1] clamp.
    env = RslRlVecEnvWrapper(env, clip_actions=100.0)
    print("Smooth standing env ready.")

    agent_cfg = get_humanoid_stand_smooth_ppo_cfg()
    agent_cfg.max_iterations = args.max_iterations

    try:
        rsl_rl_version = version("rsl-rl-lib")
    except PackageNotFoundError:
        rsl_rl_version = version("rsl-rl")

    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, rsl_rl_version)
    cfg_dict = agent_cfg.to_dict()

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

    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    os.makedirs(log_root, exist_ok=True)
    run_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(log_root, run_name)
    os.makedirs(log_dir, exist_ok=True)

    print("Creating smooth standing PPO runner...")
    runner = OnPolicyRunner(env, cfg_dict, log_dir=log_dir, device=agent_cfg.device)

    print("Starting smooth standing training...")
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
    export_deployable_policy(runner, os.path.join(log_dir, "exported"))
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
