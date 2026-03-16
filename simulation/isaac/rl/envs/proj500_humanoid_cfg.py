from dataclasses import dataclass
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass


@configclass
class Proj500HumanoidCfg(DirectRLEnvCfg):
    # ---- sim ----
    sim: SimulationCfg = SimulationCfg(dt=1.0 / 120.0, render_interval=2)

    # ---- scene ----
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=256, env_spacing=3.0)

    # ---- RL sizes (filled in env at runtime) ----
    observation_space: int = 0
    action_space: int = 0

    # ---- episode ----
    episode_length_s: float = 10.0

    # ---- robot ----
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=ArticulationCfg.SpawnCfg(
            usd_path=r"C:\Users\jandr\Git\MEng-PROJ500-2025-26\simulation\isaac\assets\usd_generated\skeleton\skeleton_fixed.usd",
        ),
        actuators={
            # Position/impedance style control via implicit actuator.
            # We keep it simple now and tune later.
            "legs_and_torso": ImplicitActuatorCfg(
                joint_names_expr=[".*"],   # we’ll select the exact joints in env.py via name list
                stiffness=60.0,
                damping=2.0,
                velocity_limit=10.0,
                effort_limit=120.0,         # clamp (roughly your motor class) – refine later
            )
        },
    )

    # ---- task params ----
    # stand first, then change to walking reward
    target_base_height_m: float = 0.80
    alive_reward: float = 1.0
    fall_penalty: float = -50.0