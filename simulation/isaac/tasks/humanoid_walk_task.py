import gymnasium as gym

from simulation.isaac.rl.envs.humanoid_walk_env import HumanoidWalkEnvCfg


gym.register(
    id="Humanoid-Walk-v0",
    entry_point="simulation.isaac.rl.envs.humanoid_walk_env:HumanoidWalkEnv",
    disable_env_checker=True,
    kwargs={
        "cfg": HumanoidWalkEnvCfg(),
    },
)

gym.spec("Humanoid-Walk-v0").kwargs["env_cfg_entry_point"] = (
    "simulation.isaac.rl.envs.humanoid_walk_env:HumanoidWalkEnvCfg"
)
gym.spec("Humanoid-Walk-v0").kwargs["rsl_rl_cfg_entry_point"] = (
    "simulation.isaac.configuration.humanoid_walk_ppo_cfg:get_humanoid_walk_ppo_cfg"
)