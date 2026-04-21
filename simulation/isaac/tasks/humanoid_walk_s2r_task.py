import gymnasium as gym

from simulation.isaac.rl.envs.humanoid_walk_s2r_env import HumanoidWalkEnvS2rCfg


gym.register(
    id="Humanoid-Walk-s2r-v0",
    entry_point="simulation.isaac.rl.envs.humanoid_walk_s2r_env:HumanoidWalkEnvS2r",
    disable_env_checker=True,
    kwargs={
        "cfg": HumanoidWalkEnvS2rCfg(),
    },
)

gym.spec("Humanoid-Walk-s2r-v0").kwargs["env_cfg_entry_point"] = (
    "simulation.isaac.rl.envs.humanoid_walk_s2r_env:HumanoidWalkEnvS2rCfg"
)
gym.spec("Humanoid-Walk-s2r-v0").kwargs["rsl_rl_cfg_entry_point"] = (
    "simulation.isaac.configuration.humanoid_walk_ppo_cfg:get_humanoid_walk_ppo_cfg"
)
