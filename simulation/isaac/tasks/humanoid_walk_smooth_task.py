import gymnasium as gym

from simulation.isaac.rl.envs.humanoid_walk_smooth_env import HumanoidWalkSmoothEnvCfg


gym.register(
    id="Humanoid-Walk-Smooth-S2R-v0",
    entry_point="simulation.isaac.rl.envs.humanoid_walk_smooth_env:HumanoidWalkSmoothEnv",
    disable_env_checker=True,
    kwargs={
        "cfg": HumanoidWalkSmoothEnvCfg(),
    },
)

gym.spec("Humanoid-Walk-Smooth-S2R-v0").kwargs["env_cfg_entry_point"] = (
    "simulation.isaac.rl.envs.humanoid_walk_smooth_env:HumanoidWalkSmoothEnvCfg"
)
gym.spec("Humanoid-Walk-Smooth-S2R-v0").kwargs["rsl_rl_cfg_entry_point"] = (
    "simulation.isaac.configuration.humanoid_walk_smooth_ppo_cfg:get_humanoid_walk_smooth_ppo_cfg"
)
