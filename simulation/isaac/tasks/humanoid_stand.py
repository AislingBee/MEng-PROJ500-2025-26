import gym

from simulation.isaac.rl.envs.humanoid_stand_env import (
    HumanoidStandEnv,
    HumanoidStandEnvCfg,
)


gym.register(
    id="Humanoid-Stand-v0",
    entry_point="simulation.isaac.rl.envs.humanoid_stand_env:HumanoidStandEnv",
    disable_env_checker=True,
    kwargs={
        "cfg": HumanoidStandEnvCfg(),
    },
)

gym.spec("Humanoid-Stand-v0").kwargs["env_cfg_entry_point"] = (
    "simulation.isaac.rl.envs.humanoid_stand_env:HumanoidStandEnvCfg"
)
gym.spec("Humanoid-Stand-v0").kwargs["rsl_rl_cfg_entry_point"] = (
    "simulation.isaac.rl.config.humanoid_stand_ppo_cfg:get_humanoid_stand_ppo_cfg"
)