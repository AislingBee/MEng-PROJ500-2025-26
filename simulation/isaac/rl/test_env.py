from simulation.isaac.rl.envs.humanoid_stand_env import HumanoidStandEnv, HumanoidStandEnvCfg

cfg = HumanoidStandEnvCfg()
env = HumanoidStandEnv(cfg)

# reset
obs = env.reset()
print("Reset OK, obs shape:", obs["policy"].shape)

# step a few times with zero action
import torch
actions = torch.zeros((env.num_envs, cfg.action_space), device=env.device)

for i in range(10):
    obs, reward, terminated, truncated, info = env.step(actions)
    print(f"Step {i} OK")

print("Done")