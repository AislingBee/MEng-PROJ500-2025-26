#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Play a trained PROJ500 humanoid standing policy.")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--checkpoint", type=str, required=True, help="Path to model_*.pt checkpoint file.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
args = parser.parse_args()

# launch Isaac Sim
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# -----------------------------------------------------------------------------
# imports AFTER launch
# -----------------------------------------------------------------------------
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

from simulation.isaac.rl.envs.humanoid_stand_env import HumanoidStandEnv, HumanoidStandEnvCfg


def _load_ppo_cfg():
    this_file = Path(__file__).resolve()
    isaac_dir = this_file.parent.parent  # simulation/isaac
    ppo_cfg_file = isaac_dir / "configuration" / "humanoid_stand_ppo_cfg.py"

    spec = importlib.util.spec_from_file_location("humanoid_stand_ppo_cfg", ppo_cfg_file)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.get_humanoid_stand_ppo_cfg


def main():
    checkpoint = os.path.abspath(args.checkpoint)
    if not os.path.isfile(checkpoint):
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint}")

    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True

    # build env directly
    env_cfg = HumanoidStandEnvCfg()
    env_cfg.scene.num_envs = args.num_envs

    env = HumanoidStandEnv(env_cfg, render_mode=None)
    env = RslRlVecEnvWrapper(env, clip_actions=1.0)
    print(f"[INFO] Env ready with {env.num_envs} env(s).")

    # load PPO config
    get_humanoid_stand_ppo_cfg = _load_ppo_cfg()
    agent_cfg = get_humanoid_stand_ppo_cfg()

    # create runner and load checkpoint
    print(f"[INFO] Loading checkpoint: {checkpoint}")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=env.unwrapped.device)
    runner.load(checkpoint, load_optimizer=False)

    # inference policy
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    print("[INFO] Policy loaded. Starting play loop...")

    obs, _ = env.get_observations()

    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()