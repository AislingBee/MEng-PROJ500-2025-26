import gymnasium as gym

from simulation.isaac.rl.envs.humanoid_stand_smooth_s2r_env import (
    HumanoidStandSmoothS2REnvCfg,
)


gym.register(
    id="Humanoid-Stand-Smooth-S2R-v0",
    entry_point="simulation.isaac.rl.envs.humanoid_stand_smooth_s2r_env:HumanoidStandSmoothS2REnv",
    disable_env_checker=True,
    kwargs={
        "cfg": HumanoidStandSmoothS2REnvCfg(),
    },
)

gym.spec("Humanoid-Stand-Smooth-S2R-v0").kwargs["env_cfg_entry_point"] = (
    "simulation.isaac.rl.envs.humanoid_stand_smooth_s2r_env:HumanoidStandSmoothS2REnvCfg"
)
gym.spec("Humanoid-Stand-Smooth-S2R-v0").kwargs["rsl_rl_cfg_entry_point"] = (
    "simulation.isaac.configuration.humanoid_stand_smooth_ppo_cfg:get_humanoid_stand_smooth_ppo_cfg"
)
