#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import os
from datetime import datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Train smooth deployable humanoid walking S2R PPO.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--num_envs", type=int, default=24000)
parser.add_argument("--max_iterations", type=int, default=2000)
parser.add_argument("--task", type=str, default="Humanoid-Walk-Smooth-S2R-v0")
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--run_name", type=str, default="")
parser.add_argument(
    "--checkpoint",
    type=str,
    default=None,
    help="Optional checkpoint file or run directory to resume from.",
)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
from rsl_rl.runners import OnPolicyRunner

from simulation.isaac.configuration.humanoid_walk_smooth_policy_contract import CONTRACT


THIS_DIR = Path(__file__).resolve().parent
ISAAC_DIR = THIS_DIR.parent

task_file = ISAAC_DIR / "tasks" / "humanoid_walk_smooth_task.py"
spec = importlib.util.spec_from_file_location("humanoid_walk_smooth_task", task_file)
task_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task_module)

ppo_cfg_file = ISAAC_DIR / "configuration" / "humanoid_walk_smooth_ppo_cfg.py"
spec = importlib.util.spec_from_file_location("humanoid_walk_smooth_ppo_cfg", ppo_cfg_file)
ppo_cfg_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ppo_cfg_module)
get_humanoid_walk_smooth_ppo_cfg = ppo_cfg_module.get_humanoid_walk_smooth_ppo_cfg


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


def _latest_model_in_dir(run_dir: Path) -> Path | None:
    def _checkpoint_sort_key(path: Path) -> tuple[int, float, str]:
        try:
            iteration = int(path.stem.split("_")[-1])
        except ValueError:
            iteration = -1
        return iteration, path.stat().st_mtime, path.name

    checkpoints = sorted(run_dir.glob("model_*.pt"), key=_checkpoint_sort_key)
    if checkpoints:
        return checkpoints[-1]
    return None


def resolve_checkpoint_path(raw_checkpoint: str | None) -> Path | None:
    if raw_checkpoint is None:
        return None

    candidate = Path(raw_checkpoint).expanduser().resolve()
    if candidate.is_file():
        return candidate
    if candidate.is_dir():
        latest = _latest_model_in_dir(candidate)
        if latest is not None:
            return latest.resolve()
        raise FileNotFoundError(f"No model_*.pt checkpoint found in run directory: {candidate}")
    raise FileNotFoundError(f"Checkpoint path does not exist: {candidate}")


def export_deployable_policy(runner, export_dir: str) -> None:
    os.makedirs(export_dir, exist_ok=True)

    actor = runner.alg.actor
    actor.eval()
    device = next(actor.parameters()).device
    example_obs = {"policy": torch.zeros(1, CONTRACT.obs_dim, device=device)}

    scripted_actor = torch.jit.trace(DeployableActor(actor).to(device).eval(), (example_obs,))
    scripted_actor.save(os.path.join(export_dir, "policy_jit.pt"))
    torch.save(actor.state_dict(), os.path.join(export_dir, "actor_state_dict.pt"))

    if CONTRACT.use_obs_normalization:
        normalizer = _find_obs_normalizer(runner)
        if normalizer is None:
            if CONTRACT.obs_normalizer_required:
                raise RuntimeError(
                    "Smooth walking deploy export requires an observation normalizer, "
                    "but none was found on the trained runner."
                )
        else:
            normalizer_path = os.path.join(export_dir, CONTRACT.obs_normalizer_artifact_name)
            example_obs_tensor = torch.zeros(1, CONTRACT.obs_dim, device=device)
            scripted_normalizer = torch.jit.trace(normalizer.to(device).eval(), (example_obs_tensor,))
            scripted_normalizer.save(normalizer_path)
            print(f"Observation normalizer saved to: {normalizer_path}")

    print(f"Deployable TorchScript actor saved to: {os.path.join(export_dir, 'policy_jit.pt')}")


def main():
    from simulation.isaac.rl.envs.humanoid_walk_smooth_env import HumanoidWalkSmoothEnvCfg

    env_cfg = HumanoidWalkSmoothEnvCfg()
    if env_cfg.action_space != CONTRACT.action_dim:
        raise RuntimeError(
            f"Smooth walking contract expects action dim {CONTRACT.action_dim}, but env config uses {env_cfg.action_space}."
        )
    if tuple(env_cfg.action_scale) != CONTRACT.action_scale:
        raise RuntimeError("Smooth walking action_scale no longer matches the deployable policy contract.")
    if env_cfg.decimation != CONTRACT.decimation:
        raise RuntimeError(
            f"Smooth walking contract expects decimation {CONTRACT.decimation}, but env config uses {env_cfg.decimation}."
        )
    if env_cfg.sim.dt != CONTRACT.sim_dt_s:
        raise RuntimeError(
            f"Smooth walking contract expects sim dt {CONTRACT.sim_dt_s}, but env config uses {env_cfg.sim.dt}."
        )
    if env_cfg.observation_space != CONTRACT.obs_dim:
        raise RuntimeError(
            f"Smooth walking observation dim mismatch: cfg={env_cfg.observation_space}, contract={CONTRACT.obs_dim}"
        )

    env_cfg.scene.num_envs = args.num_envs
    env_cfg.command_value = CONTRACT.default_command_value
    env = gym.make(args.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=100.0)

    agent_cfg = get_humanoid_walk_smooth_ppo_cfg()
    agent_cfg.max_iterations = args.max_iterations
    agent_cfg.seed = args.seed
    if args.run_name:
        agent_cfg.run_name = args.run_name
    if hasattr(args, "device"):
        agent_cfg.device = args.device

    try:
        rsl_rl_version = version("rsl-rl-lib")
    except PackageNotFoundError:
        rsl_rl_version = version("rsl-rl")

    print(
        "smooth walk train | "
        f"task={args.task} | "
        f"num_envs={args.num_envs} | "
        f"max_iterations={args.max_iterations} | "
        f"device={agent_cfg.device} | "
        f"policy_loop_hz={CONTRACT.policy_loop_hz:.3f} | "
        f"action_delay_steps={env_cfg.action_delay_steps} | "
        f"obs_normalization={CONTRACT.use_obs_normalization} | "
        f"command_value={CONTRACT.default_command_value:.2f} | "
        f"gait_frequency={CONTRACT.default_gait_frequency_hz:.3f}"
    )

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
    run_name = args.run_name or datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(log_root, run_name)
    os.makedirs(log_dir, exist_ok=True)

    print("Creating smooth walking PPO runner...")
    runner = OnPolicyRunner(env, cfg_dict, log_dir=log_dir, device=agent_cfg.device)

    checkpoint_path = resolve_checkpoint_path(args.checkpoint)
    if checkpoint_path is not None:
        print(f"Loading checkpoint: {checkpoint_path}")
        runner.load(str(checkpoint_path))

    print("Starting smooth walking training...")
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
    export_deployable_policy(runner, os.path.join(log_dir, "exported"))
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
