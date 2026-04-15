from __future__ import annotations

from asyncio import wait_for
from collections.abc import Sequence
from pathlib import Path
import math
import torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_rotate_inverse

from ...configuration.walking_actuator_config import (
    WALKING_ACTUATOR_SETTINGS,
    build_per_joint_walking_actuator_cfg,
)
from simulation.isaac.configuration.standing_pose import STANDING_TARGETS_DEG


USD_PATH = Path(__file__).resolve().parents[2] / "assets" / "usd_generated" / "skeleton" / "skeleton.usd"


@configclass
class HumanoidStandEnvS2rCfg(DirectRLEnvCfg):
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
    base_height: float = 0

    # Reward Variables
    upright_k: float = 8.0
    pose_k: float = 4.0
    reward_scales = {
        "upright": 2.0,
        "pose": 1.5,
        "ang_vel": 0.05,
        "joint_vel": 0.02,
        "action_rate": 0.05,
    }

    # Termination
    tilt_limit: float = 0.25  # projected gravity xy squared magnitude threshold

    forbidden_body_names: tuple[str, ...] = (
        "robot_l_hip_yaw_link",
        "robot_r_hip_yaw_link",
        "robot_l_hip_pitch_link",
        "robot_r_hip_pitch_link",
        "robot_l_thigh_link",
        "robot_r_thigh_link",
        "robot_l_shank_link",
        "robot_r_shank_link",
        "robot_l_ankle_link",
        "robot_r_ankle_link",
    )
    forbidden_body_height_limit: float = 0.075


class HumanoidStandEnvS2r(DirectRLEnv):
    cfg: HumanoidStandEnvS2rCfg

    def __init__(self, cfg: HumanoidStandEnvS2rCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode=render_mode, **kwargs)

        # Define the robot articulation
        self.robot: Articulation = self.scene.articulations["robot"]
        self.num_dofs = self.robot.num_joints
        self.joint_ids = torch.arange(self.num_dofs, device=self.device, dtype=torch.long)

        if self.num_dofs != self.cfg.action_space:
            raise RuntimeError(
                f"Expected {self.cfg.action_space} joints, got {self.num_dofs}. "
                f"Check USD / standing pose / actuator config alignment."
            )

        # Buffers
        self._actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._last_actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._joint_pos_targets = torch.zeros((self.num_envs, self.num_dofs), device=self.device)

        # Action scale buffers
        self._action_scale = torch.tensor(
            self.cfg.action_scale, dtype=torch.float32, device=self.device
        ).unsqueeze(0)

        # IMU Privileged Date, needs updating for IMU actual data
        self._gravity_vec_w = torch.tensor([0.0, 0.0, -1.0], device=self.device).repeat(self.num_envs, 1)

        # Standing pose and limits data
        self._standing_q = self._build_standing_pose_tensor()
        self._joint_lower = self.robot.data.soft_joint_pos_limits[..., 0].clone()
        self._joint_upper = self.robot.data.soft_joint_pos_limits[..., 1].clone()

        # Build per-joint actuator values in the articulation joint order.
        self._per_joint_actuator_cfg = build_per_joint_walking_actuator_cfg(self.robot.joint_names)
        self._kp_fixed = torch.tensor(
            self._per_joint_actuator_cfg["stiffness"],
            dtype=torch.float32,
            device=self.device,
        ).unsqueeze(0).repeat(self.num_envs, 1)
        self._kd_fixed = torch.tensor(
            self._per_joint_actuator_cfg["damping"],
            dtype=torch.float32,
            device=self.device,
        ).unsqueeze(0).repeat(self.num_envs, 1)
        self._tau_ff = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float32, device=self.device)
        self._q_des = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float32, device=self.device)

        # Forbidden body contact with ground
        name_to_body_idx = {name: i for i, name in enumerate(self.robot.body_names)}
        missing_bodies = [name for name in self.cfg.forbidden_body_names if name not in name_to_body_idx]
        if missing_bodies:
            raise RuntimeError(f"Forbidden body names not found in robot.body_names: {missing_bodies}")
        self._forbidden_body_ids = torch.tensor(
            [name_to_body_idx[name] for name in self.cfg.forbidden_body_names],
            device=self.device,
            dtype=torch.long,
        )

        # Foot Kinematics Setup
        foot_name_candidates = {
            "left": ["robot_l_foot_link"],
            "right": ["robot_r_foot_link"],
        }
        self._foot_body_ids = {}
        for side, names in foot_name_candidates.items():
            found = None
            for name in names:
                if name in name_to_body_idx:
                    found = name_to_body_idx[name]
                    break
            if found is None:
                raise RuntimeError(f"Could not find {side} foot body in robot.body_names. Tried {names}")
            self._foot_body_ids[side] = found

    def _setup_scene(self):
        ground_cfg = sim_utils.GroundPlaneCfg()
        ground_cfg.func("/World/ground", ground_cfg)

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0)
        light_cfg.func("/World/light", light_cfg)

        actuators = {}
        for group_name, cfg in WALKING_ACTUATOR_SETTINGS.items():
            actuators[group_name] = IdealPDActuatorCfg(
                joint_names_expr=cfg["joint_names"],
                effort_limit=cfg["effort_limit"],
                effort_limit_sim=cfg.get("effort_limit_sim", cfg["effort_limit"]),
                velocity_limit=cfg["velocity_limit"],
                velocity_limit_sim=cfg.get("velocity_limit_sim", cfg["velocity_limit"]),
                stiffness=cfg["stiffness"],
                damping=cfg["damping"],
                armature=cfg.get("armature", None),
                friction=cfg.get("friction", None),
                dynamic_friction=cfg.get("dynamic_friction", None),
                viscous_friction=cfg.get("viscous_friction", None),
            )

        robot_cfg = ArticulationCfg(
            prim_path="/World/envs/env_.*/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path=self.cfg.usd_path,
                activate_contact_sensors=True,
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 0.0),
                rot=(1.0, 0.0, 0.0, 0.0),
            ),
            actuators=actuators,
        )

        # print("ROBOT USD:", robot_cfg.spawn.usd_path)
        # print("spawn pos", robot_cfg.init_state.pos)
        # print("spawn rot", robot_cfg.init_state.rot)

        self.scene.articulations["robot"] = Articulation(robot_cfg)
        self.robot = self.scene.articulations["robot"]

        print("\nConfigured actuator groups:")
        for group_name, cfg in WALKING_ACTUATOR_SETTINGS.items():
            print(f"\nActuator group: {group_name}")
            print(f"  joint_names   : {cfg['joint_names']}")
            print(f"  stiffness     : {cfg['stiffness']}")
            print(f"  damping       : {cfg['damping']}")
            print(f"  effort_limit  : {cfg['effort_limit']}")
            print(f"  velocity_limit: {cfg['velocity_limit']}")

        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])


    def _build_standing_pose_tensor(self) -> torch.Tensor:
        q = torch.zeros(self.num_dofs, dtype=torch.float32, device=self.device)
        for i, joint_name in enumerate(self.robot.joint_names):
            q[i] = math.radians(STANDING_TARGETS_DEG.get(joint_name, 0.0))
        return q

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if torch.isnan(actions).any():
            raise RuntimeError("NaN detected in actions")

        self._last_actions[:] = self._actions
        self._actions = torch.clamp(actions, -1.0, 1.0)

    def _apply_action(self) -> None:
        q_des = self._standing_q.unsqueeze(0) + self._action_scale * self._actions
        q_des = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)

        self._q_des[:] = q_des
        self._joint_pos_targets[:] = q_des
        self._tau_ff.zero_()

        # MIT-style packet fields in sim:
        #   q_des  -> joint position target
        #   Kp/Kd  -> fixed per-joint gains from the actuator config
        #   tau_ff -> zero for now, but still surfaced as part of the packet interface
        self.robot.set_joint_position_target(self._q_des, joint_ids=self.joint_ids)
        self.robot.set_joint_effort_target(self._tau_ff, joint_ids=self.joint_ids)

    def get_mit_style_control_packets(self, env_ids: Sequence[int] | None = None) -> dict:
        """Return MIT-style control packets for one or more environments."""
        if env_ids is None:
            env_ids_t = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        else:
            env_ids_t = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        return {
            "joint_names": list(self.robot.joint_names),
            "q_des": self._q_des[env_ids_t].clone(),
            "Kp": self._kp_fixed[env_ids_t].clone(),
            "Kd": self._kd_fixed[env_ids_t].clone(),
            "tau_ff": self._tau_ff[env_ids_t].clone(),
        }

    def _get_joint_effort_obs(self) -> torch.Tensor:
        applied_effort = getattr(self.robot.data, "applied_effort", None)
        if applied_effort is not None:
            return applied_effort
        computed_effort = getattr(self.robot.data, "computed_effort", None)
        if computed_effort is not None:
            return computed_effort

        # Fallbacks for older naming.
        applied_torque = getattr(self.robot.data, "applied_torque", None)
        if applied_torque is not None:
            return applied_torque
        computed_torque = getattr(self.robot.data, "computed_torque", None)
        if computed_torque is not None:
            return computed_torque

        return torch.zeros_like(self.robot.data.joint_pos)

    def _get_foot_kinematics(self):
        root_pos_w = self.robot.data.root_pos_w
        root_quat_w = self.robot.data.root_quat_w
        root_lin_vel_w = self.robot.data.root_lin_vel_w

        body_pos_w = self.robot.data.body_pos_w
        body_quat_w = self.robot.data.body_quat_w
        body_lin_vel_w = self.robot.data.body_lin_vel_w

        left_pos_w = body_pos_w[:, self._foot_body_ids["left"], :]
        right_pos_w = body_pos_w[:, self._foot_body_ids["right"], :]
        left_vel_w = body_lin_vel_w[:, self._foot_body_ids["left"], :]
        right_vel_w = body_lin_vel_w[:, self._foot_body_ids["right"], :]
        left_quat_w = body_quat_w[:, self._foot_body_ids["left"], :]
        right_quat_w = body_quat_w[:, self._foot_body_ids["right"], :]

        left_pos_b = quat_rotate_inverse(root_quat_w, left_pos_w - root_pos_w)
        right_pos_b = quat_rotate_inverse(root_quat_w, right_pos_w - root_pos_w)
        left_vel_b = quat_rotate_inverse(root_quat_w, left_vel_w - root_lin_vel_w)
        right_vel_b = quat_rotate_inverse(root_quat_w, right_vel_w - root_lin_vel_w)

        return {
            "left_pos_w": left_pos_w,
            "right_pos_w": right_pos_w,
            "left_vel_w": left_vel_w,
            "right_vel_w": right_vel_w,
            "left_pos_b": left_pos_b,
            "right_pos_b": right_pos_b,
            "left_vel_b": left_vel_b,
            "right_vel_b": right_vel_b,
            "left_quat_w": left_quat_w,
            "right_quat_w": right_quat_w,
        }

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

        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observations")

        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        root_quat_w = self.robot.data.root_quat_w
        root_ang_vel_b = self.robot.data.root_ang_vel_b
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)

        q_err = q - self._standing_q.unsqueeze(0)
        action_rate = self._actions - self._last_actions

        tilt_metric = torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)

        r_upright = torch.exp(-self.cfg.upright_k * tilt_metric)
        r_pose = torch.exp(-self.cfg.pose_k * torch.mean(q_err ** 2, dim=1))

        p_ang_vel = torch.mean(root_ang_vel_b ** 2, dim=1)
        p_joint_vel = torch.mean(qd ** 2, dim=1)
        p_action_rate = torch.mean(action_rate ** 2, dim=1)
        survival_reward = 0.2


        reward = (
                survival_reward
                + self.cfg.reward_scales["upright"] * r_upright
                + self.cfg.reward_scales["pose"] * r_pose
                - self.cfg.reward_scales["ang_vel"] * p_ang_vel
                - self.cfg.reward_scales["joint_vel"] * p_joint_vel
                - self.cfg.reward_scales["action_rate"] * p_action_rate
        )

        if torch.isnan(reward).any():
            raise RuntimeError("NaN detected in rewards")

        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        root_quat_w = self.robot.data.root_quat_w
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)
        tilt_metric = torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)
        over_tilted = tilt_metric > self.cfg.tilt_limit

        bad_state = (
                torch.isnan(q).any(dim=1)
                | torch.isnan(qd).any(dim=1)
                | torch.isnan(projected_gravity_b).any(dim=1)
        )

        forbidden_body_heights = self.robot.data.body_pos_w[:, self._forbidden_body_ids, 2]
        # print("forbidden_body_heights", forbidden_body_heights)
        body_hit_ground = torch.any(
            forbidden_body_heights < self.cfg.forbidden_body_height_limit,
            dim=1,
        )

        terminated = over_tilted | bad_state | body_hit_ground

        time_out = self.episode_length_buf >= self.max_episode_length - 1

        # if torch.any(over_tilted):
        #     print("Terminated Reason: Over Tilted")
        # if torch.any(bad_state):
        #     print("Terminated Reason: Bad State")
        # if torch.any(body_hit_ground):
        #     print("Terminated Reason: Body Hit Ground")
        # if time_out.any():
        #     print("Terminated Reason: Time Out")

        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):

        # print("###################################|RESET|#######################################")

        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        else:
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        super()._reset_idx(env_ids)

        default_root_state = self.robot.data.default_root_state[env_ids].clone()
        default_root_state[:, :3] = self.scene.env_origins[env_ids]
        default_root_state[:, 2] += self.cfg.base_height

        joint_pos = self._standing_q.unsqueeze(0).repeat(len(env_ids), 1)
        joint_pos += 0.02 * torch.randn_like(joint_pos) # Randomisation

        joint_vel = 0.05 * torch.randn(
            (len(env_ids), self.num_dofs),
            device=self.device
        )

        joint_pos = torch.max(
            torch.min(joint_pos,
            self._joint_upper[env_ids]),
            self._joint_lower[env_ids]
        )

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)
        self._actions[env_ids] = 0.0
        self._last_actions[env_ids] = 0.0

        # if len(env_ids) > 0 and int(env_ids[0]) == 0:
        #     root_height = self.robot.data.root_pos_w[env_ids, 2]
        #     print("Reset root height sample:", root_height[:5])

