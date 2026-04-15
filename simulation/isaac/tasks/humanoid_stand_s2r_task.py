import gymnasium as gym

from simulation.isaac.rl.envs.humanoid_stand_s2r_env import HumanoidStandEnvS2rCfg

gym.register(
    id="Humanoid-Stand-s2r-v0",
    entry_point="simulation.isaac.rl.envs.humanoid_stand_s2r_env:HumanoidStandEnvS2r",
    disable_env_checker=True,
    kwargs={
        "cfg": HumanoidStandEnvS2rCfg(),
    },
)

gym.spec("Humanoid-Stand-s2r-v0").kwargs["env_cfg_entry_point"] = (
    "simulation.isaac.rl.envs.humanoid_stand_s2r_env:HumanoidStandEnvS2rCfg"
)
gym.spec("Humanoid-Stand-s2r-v0").kwargs["rsl_rl_cfg_entry_point"] = (
    "simulation.isaac.configuration.humanoid_stand_ppo_cfg:get_humanoid_stand_ppo_cfg"
)