from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_rotate_inverse

from ...configuration.actuator_config import ACTUATOR_SETTINGS
from simulation.isaac.configuration.standing_pose import STANDING_TARGETS_DEG


USD_PATH = Path(__file__).resolve().parents[2] / "assets" / "usd_generated" / "skeleton" / "skeleton_fixed.usd"


@configclass
class HumanoidStandEnvCfg(DirectRLEnvCfg):
    decimation: int = 2
    episode_length_s: float = 10.0

    sim: SimulationCfg = SimulationCfg(
        dt=1.0 / 120.0,
        render_interval=decimation,
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=256,
        env_spacing=3.0,
        replicate_physics=True,
    )

    action_space: int = 12
    observation_space: int = 42

    action_scale: tuple[float, ...] = (
        0.10, 0.08, 0.15, 0.20, 0.12, 0.08,
        0.10, 0.08, 0.15, 0.20, 0.12, 0.08,
    )

    state_space: int = 0

    usd_path: str = str(USD_PATH)
    base_height: float = 0.05
    alive_reward: float = 1.0
    fall_height_threshold: float = 0.20
    tilt_gravity_z_threshold: float = -0.60

    # Reward Variables
    target_base_height: float = 0.05
    upright_k: float = 8.0
    pose_k: float = 4.0
    height_k: float = 40.0
    reward_scales = {
        "upright": 2.0,
        "pose": 1.5,
        "height": 0.5,
        "ang_vel": 0.05,
        "joint_vel": 0.02,
        "action_rate": 0.05,
    }




class HumanoidStandEnv(DirectRLEnv):
    cfg: HumanoidStandEnvCfg

    def __init__(self, cfg: HumanoidStandEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode=render_mode, **kwargs)

        self._actions = torch.zeros((self.num_envs, self.cfg.action_space), device=self.device)
        self._last_actions = torch.zeros((self.num_envs, self.cfg.action_space), device=self.device)

        self._action_scale = torch.tensor(self.cfg.action_scale, device=self.device).unsqueeze(0)

        self.robot: Articulation = self.scene.articulations["robot"]
        self.num_dofs = self.robot.num_joints
        self.joint_ids = torch.arange(self.num_dofs, device=self.device, dtype=torch.long)

        if self.num_dofs != self.cfg.action_space:
            raise RuntimeError(
                f"Expected {self.cfg.action_space} joints, got {self.num_dofs}. "
                f"Check USD / standing pose / actuator config alignment."
            )

        self._actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._gravity_vec_w = torch.tensor([0.0, 0.0, -1.0], device=self.device).repeat(self.num_envs, 1)

        self._standing_q = self._build_standing_pose_tensor()
        self._joint_lower = self.robot.data.soft_joint_pos_limits[..., 0].clone()
        self._joint_upper = self.robot.data.soft_joint_pos_limits[..., 1].clone()

    def _setup_scene(self):
        ground_cfg = sim_utils.GroundPlaneCfg()
        ground_cfg.func("/World/ground", ground_cfg)

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0)
        light_cfg.func("/World/light", light_cfg)

        actuators = {}
        for group_name, cfg in ACTUATOR_SETTINGS.items():
            actuators[group_name] = ImplicitActuatorCfg(
                joint_names_expr=cfg["joint_names"],
                effort_limit_sim=cfg["effort_limit"],
                velocity_limit_sim=cfg["velocity_limit"],
                stiffness=cfg["stiffness"],
                damping=cfg["damping"],
            )

        robot_cfg = ArticulationCfg(
            prim_path="/World/envs/env_.*/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=self.cfg.usd_path),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, self.cfg.base_height),
                rot=(0.0, 0.0, 0.0, 0.0),
            ),
            actuators=actuators,
        )

        self.scene.articulations["robot"] = Articulation(robot_cfg)
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])

    def _build_standing_pose_tensor(self) -> torch.Tensor:
        q = torch.zeros(self.num_dofs, dtype=torch.float32, device=self.device)
        for i, joint_name in enumerate(self.robot.joint_names):
            q[i] = math.radians(STANDING_TARGETS_DEG.get(joint_name, 0.0))
        return q

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._last_actions[:] = self._actions
        self._actions = torch.clamp(actions, -1.0, 1.0)

    def _apply_action(self) -> None:
        targets = self._standing_q.unsqueeze(0) + self._action_scale * self._actions
        targets = torch.max(torch.min(targets, self._joint_upper), self._joint_lower)
        self.robot.set_joint_position_target(targets, joint_ids=self.joint_ids)

    def _get_observations(self) -> dict:
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        q_rel = q - self._standing_q.unsqueeze(0)

        root_quat_w = self.robot.data.root_quat_w
        root_ang_vel_b = self.robot.data.root_ang_vel_b
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)

        obs = torch.cat(
            (
                q_rel,  # 12
                qd,  # 12
                projected_gravity_b,  # 3
                root_ang_vel_b,  # 3
                self._last_actions,  # 12
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        base_z = self.robot.data.root_pos_w[:, 2]
        root_quat_w = self.robot.data.root_quat_w
        root_ang_vel_b = self.robot.data.root_ang_vel_b
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)

        q_err = q - self._standing_q.unsqueeze(0)
        action_rate = self._actions - self._last_actions

        r_upright = torch.exp(
            -self.cfg.upright_k * torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)
        )
        r_pose = torch.exp(
            -self.cfg.pose_k * torch.mean(q_err ** 2, dim=1)
        )
        r_height = torch.exp(
            -self.cfg.height_k * (base_z - self.cfg.target_base_height) ** 2
        )

        p_ang_vel = torch.mean(root_ang_vel_b ** 2, dim=1)
        p_joint_vel = torch.mean(qd ** 2, dim=1)
        p_action_rate = torch.mean(action_rate ** 2, dim=1)

        reward = (
                self.cfg.reward_scales["upright"] * r_upright
                + self.cfg.reward_scales["pose"] * r_pose
                + self.cfg.reward_scales["height"] * r_height
                - self.cfg.reward_scales["ang_vel"] * p_ang_vel
                - self.cfg.reward_scales["joint_vel"] * p_joint_vel
                - self.cfg.reward_scales["action_rate"] * p_action_rate
        )
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        base_height = self.robot.data.root_pos_w[:, 2]
        root_quat_w = self.robot.data.root_quat_w
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)

        fallen = base_height < self.cfg.fall_height_threshold
        over_tilted = projected_gravity_b[:, 2] > self.cfg.tilt_gravity_z_threshold

        terminated = fallen | over_tilted
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        else:
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        super()._reset_idx(env_ids)

        default_root_state = self.robot.data.default_root_state[env_ids].clone()
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        joint_pos = self._standing_q.unsqueeze(0).repeat(len(env_ids), 1)
        joint_vel = torch.zeros((len(env_ids), self.num_dofs), device=self.device)
        joint_pos = torch.max(torch.min(joint_pos, self._joint_upper[env_ids]), self._joint_lower[env_ids])

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)
