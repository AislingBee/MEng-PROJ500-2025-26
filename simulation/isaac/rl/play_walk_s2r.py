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
parser = argparse.ArgumentParser(description="Play back PROJ500 humanoid WALK Sim to Real PPO checkpoint.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--task", type=str, default="Humanoid-Walk-s2r-v0")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument(
    "--checkpoint",
    type=str,
    default=None,
    help="Path to a checkpoint file or run directory. Defaults to the latest walk checkpoint.",
)
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
from simulation.isaac.configuration.standing_s2r_policy_contract import CONTRACT

THIS_DIR = Path(__file__).resolve().parent
ISAAC_DIR = THIS_DIR.parent  # simulation/isaac
REPO_ROOT = THIS_DIR.parents[2]
LOG_ROOT = REPO_ROOT / "logs" / "rsl_rl" / "humanoid_walk"

# -----------------------------------------------------------------------------
# Task registration
# -----------------------------------------------------------------------------
task_file = ISAAC_DIR / "tasks" / "humanoid_walk_s2r_task.py"
spec = importlib.util.spec_from_file_location("humanoid_walk_s2r_task", task_file)
task_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task_module)

ppo_cfg_file = ISAAC_DIR / "configuration" / "humanoid_walk_ppo_cfg.py"
spec = importlib.util.spec_from_file_location("humanoid_walk_ppo_cfg", ppo_cfg_file)
ppo_cfg_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ppo_cfg_module)
get_humanoid_walk_ppo_cfg = ppo_cfg_module.get_humanoid_walk_ppo_cfg

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


def _latest_model_in_dir(run_dir: Path) -> Path | None:
    def _checkpoint_sort_key(path: Path) -> tuple[int, float, str]:
        try:
            iteration = int(path.stem.split("_")[-1])
        except ValueError:
            iteration = -1
        return iteration, path.stat().st_mtime, path.name

    checkpoints = sorted(
        run_dir.glob("model_*.pt"),
        key=_checkpoint_sort_key,
    )
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
            f"No walk log directory found at {LOG_ROOT.resolve()}. "
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
        f"No walk checkpoints found under {LOG_ROOT.resolve()}. "
        "Pass --checkpoint explicitly to choose a checkpoint."
    )


def main():
    from simulation.isaac.rl.envs.humanoid_walk_s2r_env import HumanoidWalkEnvS2rCfg

    checkpoint_path = resolve_checkpoint_path(args.checkpoint)

    env_cfg = HumanoidWalkEnvS2rCfg()
    if env_cfg.action_space != CONTRACT.action_dim:
        raise RuntimeError(
            f"Shared S2R contract expects action dim {CONTRACT.action_dim}, "
            f"but env config uses {env_cfg.action_space}."
        )
    if tuple(env_cfg.action_scale) != CONTRACT.action_scale:
        raise RuntimeError("Walking action_scale no longer matches the shared S2R policy contract.")
    if env_cfg.decimation != CONTRACT.decimation:
        raise RuntimeError(
            f"Shared S2R contract expects decimation {CONTRACT.decimation}, "
            f"but env config uses {env_cfg.decimation}."
        )
    if env_cfg.sim.dt != CONTRACT.sim_dt_s:
        raise RuntimeError(
            f"Shared S2R contract expects sim dt {CONTRACT.sim_dt_s}, "
            f"but env config uses {env_cfg.sim.dt}."
        )

    env_cfg.scene.num_envs = args.num_envs

    env = gym.make(args.task, cfg=env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=1.0)
    print("Env ready.")

    agent_cfg = get_humanoid_walk_ppo_cfg()

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

    print("Creating PPO runner...")
    runner = OnPolicyRunner(env, cfg_dict, log_dir=log_dir, device=agent_cfg.device)

    print(f"Loading checkpoint: {checkpoint_path}")
    runner.load(str(checkpoint_path))

    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.reset()
    print("Starting walking playback...")

    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            if torch.any(dones):
                obs, _ = env.reset()

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
