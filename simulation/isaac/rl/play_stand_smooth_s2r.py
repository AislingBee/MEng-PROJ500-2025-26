#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import os
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Play back smooth deployable humanoid standing S2R PPO checkpoint.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--task", type=str, default="Humanoid-Stand-Smooth-S2R-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument(
    "--checkpoint",
    type=str,
    default=None,
    help="Path to a checkpoint file or run directory. Defaults to the latest smooth standing checkpoint.",
)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
from rsl_rl.runners import OnPolicyRunner

from simulation.isaac.configuration.humanoid_stand_smooth_policy_contract import CONTRACT


THIS_DIR = Path(__file__).resolve().parent
ISAAC_DIR = THIS_DIR.parent
REPO_ROOT = THIS_DIR.parents[2]
LOG_ROOT = REPO_ROOT / "logs" / "rsl_rl" / "humanoid_stand_smooth_s2r"

task_file = ISAAC_DIR / "tasks" / "humanoid_stand_smooth_s2r_task.py"
spec = importlib.util.spec_from_file_location("humanoid_stand_smooth_s2r_task", task_file)
task_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task_module)

ppo_cfg_file = ISAAC_DIR / "configuration" / "humanoid_stand_smooth_ppo_cfg.py"
spec = importlib.util.spec_from_file_location("humanoid_stand_smooth_ppo_cfg", ppo_cfg_file)
ppo_cfg_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ppo_cfg_module)
get_humanoid_stand_smooth_ppo_cfg = ppo_cfg_module.get_humanoid_stand_smooth_ppo_cfg

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


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


def resolve_checkpoint_path(raw_checkpoint: str | None) -> Path:
    if raw_checkpoint is not None:
        candidate = Path(raw_checkpoint).expanduser().resolve()
        if candidate.is_file():
            return candidate
        if candidate.is_dir():
            latest = _latest_model_in_dir(candidate)
            if latest is not None:
                return latest.resolve()
            raise FileNotFoundError(f"No model_*.pt checkpoint found in run directory: {candidate}")
        raise FileNotFoundError(f"Checkpoint path does not exist: {candidate}")

    if not LOG_ROOT.exists():
        raise FileNotFoundError(
            f"No smooth standing log directory found at {LOG_ROOT.resolve()}. "
            "Pass --checkpoint explicitly to choose a checkpoint."
        )

    run_dirs = sorted(
        (path for path in LOG_ROOT.iterdir() if path.is_dir()),
        key=lambda path: (path.stat().st_mtime, path.name),
    )
    for run_dir in reversed(run_dirs):
        latest = _latest_model_in_dir(run_dir)
        if latest is not None:
            return latest.resolve()

    raise FileNotFoundError(
        f"No smooth standing checkpoints found under {LOG_ROOT.resolve()}. "
        "Pass --checkpoint explicitly to choose a checkpoint."
    )


def main():
    from simulation.isaac.rl.envs.humanoid_stand_smooth_s2r_env import HumanoidStandSmoothS2REnvCfg

    checkpoint_path = resolve_checkpoint_path(args.checkpoint)

    env_cfg = HumanoidStandSmoothS2REnvCfg()
    if env_cfg.action_space != CONTRACT.action_dim:
        raise RuntimeError(
            f"Smooth standing contract expects action dim {CONTRACT.action_dim}, but env config uses {env_cfg.action_space}."
        )
    if tuple(env_cfg.action_scale) != CONTRACT.action_scale:
        raise RuntimeError("Smooth standing action_scale no longer matches the smooth standing policy contract.")
    if env_cfg.decimation != CONTRACT.decimation:
        raise RuntimeError(
            f"Smooth standing contract expects decimation {CONTRACT.decimation}, but env config uses {env_cfg.decimation}."
        )
    if env_cfg.sim.dt != CONTRACT.sim_dt_s:
        raise RuntimeError(
            f"Smooth standing contract expects sim dt {CONTRACT.sim_dt_s}, but env config uses {env_cfg.sim.dt}."
        )
    if env_cfg.observation_space != CONTRACT.obs_dim:
        raise RuntimeError(
            f"Smooth standing observation dim mismatch: cfg={env_cfg.observation_space}, contract={CONTRACT.obs_dim}"
        )

    env_cfg.scene.num_envs = args.num_envs
    env_cfg.default_command_value = CONTRACT.default_command_value
    env = gym.make(args.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=100.0)

    agent_cfg = get_humanoid_stand_smooth_ppo_cfg()
    if hasattr(args, "device"):
        agent_cfg.device = args.device

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

    log_dir = os.path.abspath(os.path.join("logs", "rsl_rl", "playback"))
    os.makedirs(log_dir, exist_ok=True)

    print("Creating smooth standing PPO runner...")
    runner = OnPolicyRunner(env, cfg_dict, log_dir=log_dir, device=agent_cfg.device)
    print(f"Loading checkpoint: {checkpoint_path}")
    runner.load(str(checkpoint_path))

    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.reset()
    step_count = 0
    print("Starting smooth standing playback...")

    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, reward, dones, _ = env.step(actions)
            step_count += 1

            if step_count % 100 == 0:
                metrics = env.unwrapped.get_runtime_debug_metrics()
                reward_mean = reward.mean().item() if hasattr(reward, "mean") else float(reward)
                print(
                    "smooth stand play | "
                    f"reward={reward_mean:.4f} | "
                    f"action_saturation_pct={metrics['action_saturation_pct']:.2f}% | "
                    f"raw_abs_mean={metrics['raw_action_abs_mean']:.3f} | "
                    f"q_des_min/max={metrics['q_des_min']:+.3f}/{metrics['q_des_max']:+.3f} | "
                    f"command={metrics['command_mean']:.3f} | "
                    f"tilt_metric={metrics['tilt_metric_mean']:.4f}"
                )

            if torch.any(dones):
                obs, _ = env.reset()

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
