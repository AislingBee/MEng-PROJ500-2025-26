from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from simulation.isaac.configuration.standing_s2r_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
)
from simulation.isaac.configuration.walking_actuator_config import WALKING_ACTUATOR_SETTINGS
from simulation.isaac.rl.interface.hardware_interface import ControlPacket
from simulation.isaac.rl.interface.isaac_hardware_interface import IsaacHardwareInterface


USD_PATH = Path(__file__).resolve().parents[3] / "assets" / "usd_generated" / "robot" / "robot.usd"

OBS_LAYOUT: dict[str, int] = {
    "q_rel": 12,
    "qd": 12,
    "q_target_err": 12,
    "joint_effort": 12,
    "projected_gravity_b": 3,
    "imu_gyro_b": 3,
    "last_actions": 12,
}
OBS_DIM = sum(OBS_LAYOUT.values())


@configclass
class HumanoidStandSmoothS2REnvCfg(DirectRLEnvCfg):
    """Smooth deployable standing task.

    Observation order is fixed by OBS_LAYOUT:
      q_rel, qd, q_target_err, joint_effort, projected_gravity_b,
      imu_gyro_b, last_actions.

    Joint order, standing_q, limits, action scale, dt, decimation, and MIT-style
    gains all come from the shared standing S2R policy contract.
    """

    decimation: int = CONTRACT.decimation
    episode_length_s: float = 10.0
    action_delay_steps: int = 2

    sim: SimulationCfg = SimulationCfg(dt=CONTRACT.sim_dt_s, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=256,
        env_spacing=3.0,
        replicate_physics=True,
    )

    action_space: int = CONTRACT.action_dim
    observation_space: int = OBS_DIM
    state_space: int = 0

    action_scale: tuple[float, ...] = tuple(0.65 * x for x in CONTRACT.action_scale)
    usd_path: str = str(USD_PATH)
    base_height: float = 0.0

    upright_k: float = 10.0
    pose_k: float = 8.0
    survival_reward: float = 0.2
    reward_scales = {
        "upright": 2.5,
        "pose": 2.0,
        "joint_vel": 0.035,
        "gyro": 0.08,
        "raw_action": 0.18,
        "action_rate": 0.22,
    }

    tilt_limit: float = 0.20
    forbidden_body_names: tuple[str, ...] = (
        "l_hip_yaw_link",
        "r_hip_yaw_link",
        "l_hip_pitch_link",
        "r_hip_pitch_link",
        "l_thigh_link",
        "r_thigh_link",
        "l_shank_link",
        "r_shank_link",
        "l_ankle_link",
        "r_ankle_link",
    )
    forbidden_body_height_limit: float = 0.02

    reset_joint_pos_noise: float = 0.003
    reset_joint_vel_noise: float = 0.008
    reward_debug_interval: int = 100


class HumanoidStandSmoothS2REnv(DirectRLEnv):
    cfg: HumanoidStandSmoothS2REnvCfg

    def __init__(self, cfg: HumanoidStandSmoothS2REnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode=render_mode, **kwargs)

        self.robot: Articulation = self.scene.articulations["robot"]
        self.num_dofs = self.robot.num_joints
        self.joint_ids = torch.arange(self.num_dofs, device=self.device, dtype=torch.long)

        if self.num_dofs != CONTRACT.action_dim:
            raise RuntimeError(f"Expected {CONTRACT.action_dim} joints, got {self.num_dofs}")
        if tuple(self.robot.joint_names) != CONTRACT.joint_names:
            raise RuntimeError("Robot joint_names do not match CONTRACT.joint_names")
        if self.cfg.observation_space != OBS_DIM:
            raise RuntimeError(f"Observation contract mismatch: cfg={self.cfg.observation_space}, layout={OBS_DIM}")

        self._step_dt = float(self.cfg.sim.dt * self.cfg.decimation)
        self._common_step_counter = 0
        self._reward_debug_counter = 0

        self._raw_actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._clamped_actions = torch.zeros_like(self._raw_actions)
        self._actions = torch.zeros_like(self._raw_actions)
        self._last_actions = torch.zeros_like(self._raw_actions)
        self._action_rate = torch.zeros_like(self._raw_actions)
        self._action_delay_steps = int(self.cfg.action_delay_steps)
        self._action_buffer = torch.zeros(
            (self.num_envs, self._action_delay_steps + 1, self.num_dofs),
            device=self.device,
        )

        self._standing_q = build_standing_q(device=self.device)
        self._action_scale = torch.tensor(self.cfg.action_scale, dtype=torch.float32, device=self.device).unsqueeze(0)
        self._joint_lower = torch.tensor(
            CONTRACT.joint_lower_limits_rad,
            dtype=torch.float32,
            device=self.device,
        ).unsqueeze(0).repeat(self.num_envs, 1)
        self._joint_upper = torch.tensor(
            CONTRACT.joint_upper_limits_rad,
            dtype=torch.float32,
            device=self.device,
        ).unsqueeze(0).repeat(self.num_envs, 1)

        soft_joint_lower = self.robot.data.soft_joint_pos_limits[..., 0]
        soft_joint_upper = self.robot.data.soft_joint_pos_limits[..., 1]
        if not torch.allclose(soft_joint_lower[0], self._joint_lower[0], atol=1e-5, rtol=0.0):
            raise RuntimeError("Robot soft lower limits do not match the standing S2R contract")
        if not torch.allclose(soft_joint_upper[0], self._joint_upper[0], atol=1e-5, rtol=0.0):
            raise RuntimeError("Robot soft upper limits do not match the standing S2R contract")

        kp_fixed, kd_fixed = build_fixed_gains(device=self.device)
        self._kp_fixed = kp_fixed.unsqueeze(0).repeat(self.num_envs, 1)
        self._kd_fixed = kd_fixed.unsqueeze(0).repeat(self.num_envs, 1)
        self._kp_gains = self._kp_fixed.clone()
        self._kd_gains = self._kd_fixed.clone()
        self._tau_ff = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float32, device=self.device)
        self._q_des = self._standing_q.unsqueeze(0).repeat(self.num_envs, 1)
        self._joint_pos_targets = self._q_des.clone()

        name_to_body_idx = {name: i for i, name in enumerate(self.robot.body_names)}
        missing_bodies = [name for name in self.cfg.forbidden_body_names if name not in name_to_body_idx]
        if missing_bodies:
            raise RuntimeError(f"Forbidden body names not found in robot.body_names: {missing_bodies}")
        self._forbidden_body_ids = torch.tensor(
            [name_to_body_idx[name] for name in self.cfg.forbidden_body_names],
            device=self.device,
            dtype=torch.long,
        )

        self._root_to_imu_quat = torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32, device=self.device)
        self._hardware = IsaacHardwareInterface(
            robot=self.robot,
            joint_ids=self.joint_ids,
            device=self.device,
            root_to_imu_quat=self._root_to_imu_quat,
        )

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

        self.scene.articulations["robot"] = Articulation(
            ArticulationCfg(
                prim_path="/World/envs/env_.*/Robot",
                spawn=sim_utils.UsdFileCfg(usd_path=self.cfg.usd_path, activate_contact_sensors=True),
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=(0.0, 0.0, 0.0),
                    rot=(1.0, 0.0, 0.0, 0.0),
                ),
                actuators=actuators,
            )
        )
        self.robot = self.scene.articulations["robot"]
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if torch.isnan(actions).any():
            raise RuntimeError("NaN detected in raw policy actions")

        self._common_step_counter += 1
        self._raw_actions[:] = actions
        self._last_actions[:] = self._actions

        clamped_actions = torch.clamp(actions, -1.0, 1.0)
        self._clamped_actions[:] = clamped_actions

        self._action_buffer = torch.roll(self._action_buffer, shifts=1, dims=1)
        self._action_buffer[:, 0, :] = clamped_actions

        delayed_actions = self._action_buffer[:, self._action_delay_steps, :]
        self._actions[:] = delayed_actions
        self._action_rate[:] = delayed_actions - self._last_actions

    def _build_control_packet(self, env_ids: Sequence[int] | None = None) -> ControlPacket:
        if env_ids is None:
            q_des = self._q_des
            kp = self._kp_fixed
            kd = self._kd_fixed
            tau_ff = self._tau_ff
            kp_gains = self._kp_gains
            kd_gains = self._kd_gains
        else:
            env_ids_t = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
            q_des = self._q_des[env_ids_t]
            kp = self._kp_fixed[env_ids_t]
            kd = self._kd_fixed[env_ids_t]
            tau_ff = self._tau_ff[env_ids_t]
            kp_gains = self._kp_gains[env_ids_t]
            kd_gains = self._kd_gains[env_ids_t]

        return ControlPacket(
            joint_names=list(self.robot.joint_names),
            q_des=q_des.clone(),
            kp=kp.clone(),
            kd=kd.clone(),
            tau_ff=tau_ff.clone(),
            kp_gains=kp_gains.clone(),
            kd_gains=kd_gains.clone(),
        )

    def _apply_action(self) -> None:
        q_des = self._standing_q.unsqueeze(0) + self._action_scale * self._actions
        q_des = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)

        self._q_des[:] = q_des
        self._joint_pos_targets[:] = q_des
        self._tau_ff.zero_()
        self._hardware.write_control_packet(self._build_control_packet())

    def get_mit_style_control_packets(self, env_ids: Sequence[int] | None = None) -> dict:
        packet = self._build_control_packet(env_ids)
        return {
            "joint_names": packet.joint_names,
            "q_des": packet.q_des,
            "Kp": packet.kp,
            "Kd": packet.kd,
            "tau_ff": packet.tau_ff,
        }

    def _get_observations(self) -> dict:
        packet = self._hardware.read_observation_packet()
        q_rel = packet.joint_pos - self._standing_q.unsqueeze(0)
        q_target_err = self._joint_pos_targets - packet.joint_pos

        fields = (
            ("q_rel", q_rel),
            ("qd", packet.joint_vel),
            ("q_target_err", q_target_err),
            ("joint_effort", packet.joint_effort),
            ("projected_gravity_b", packet.projected_gravity_b),
            ("imu_gyro_b", packet.imu_gyro_b),
            ("last_actions", self._last_actions),
        )
        obs = torch.cat(tuple(value for _, value in fields), dim=-1)

        expected_dim = sum(OBS_LAYOUT[name] for name, _ in fields)
        if obs.shape[1] != expected_dim or expected_dim != self.cfg.observation_space:
            raise RuntimeError(
                f"Observation size mismatch. Expected {self.cfg.observation_space}, got {obs.shape[1]}"
            )
        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observations")

        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        packet = self._hardware.read_observation_packet()
        q_err = self.robot.data.joint_pos - self._standing_q.unsqueeze(0)
        qd = self.robot.data.joint_vel

        tilt_metric = torch.sum(packet.projected_gravity_b[:, :2] ** 2, dim=1)
        upright = torch.exp(-self.cfg.upright_k * tilt_metric)
        pose = torch.exp(-self.cfg.pose_k * torch.mean(q_err ** 2, dim=1))
        joint_vel_pen = torch.mean(qd ** 2, dim=1)
        gyro_pen = torch.mean(packet.imu_gyro_b ** 2, dim=1)
        raw_action_pen = torch.mean(self._raw_actions ** 2, dim=1)
        action_rate_pen = torch.mean(self._action_rate ** 2, dim=1)
        survival = torch.full((self.num_envs,), self.cfg.survival_reward, device=self.device)

        reward = (
            survival
            + self.cfg.reward_scales["upright"] * upright
            + self.cfg.reward_scales["pose"] * pose
            - self.cfg.reward_scales["joint_vel"] * joint_vel_pen
            - self.cfg.reward_scales["gyro"] * gyro_pen
            - self.cfg.reward_scales["raw_action"] * raw_action_pen
            - self.cfg.reward_scales["action_rate"] * action_rate_pen
        )

        if torch.isnan(reward).any():
            raise RuntimeError("NaN detected in rewards")

        self._reward_debug_counter += 1
        if self._reward_debug_counter % self.cfg.reward_debug_interval == 0:
            saturation_pct = 100.0 * (torch.abs(self._raw_actions) > 1.0).float().mean().item()
            print(
                "smooth stand reward | "
                f"upright: {(self.cfg.reward_scales['upright'] * upright).mean().item():.4f} | "
                f"pose: {(self.cfg.reward_scales['pose'] * pose).mean().item():.4f} | "
                f"joint_vel_pen: {(self.cfg.reward_scales['joint_vel'] * joint_vel_pen).mean().item():.4f} | "
                f"gyro_pen: {(self.cfg.reward_scales['gyro'] * gyro_pen).mean().item():.4f} | "
                f"raw_action_pen: {(self.cfg.reward_scales['raw_action'] * raw_action_pen).mean().item():.4f} | "
                f"action_rate_pen: {(self.cfg.reward_scales['action_rate'] * action_rate_pen).mean().item():.4f} | "
                f"saturation_pct: {saturation_pct:.2f}% | "
                f"total: {reward.mean().item():.4f} | "
                f"raw_min/max: {self._raw_actions.min().item():+.3f}/{self._raw_actions.max().item():+.3f} | "
                f"raw_abs_mean: {torch.abs(self._raw_actions).mean().item():.3f} | "
                f"clamped_min/max: {self._clamped_actions.min().item():+.3f}/{self._clamped_actions.max().item():+.3f} | "
                f"q_des_min/max: {self._q_des.min().item():+.3f}/{self._q_des.max().item():+.3f}"
            )

        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        packet = self._hardware.read_observation_packet()
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel

        tilt_metric = torch.sum(packet.projected_gravity_b[:, :2] ** 2, dim=1)
        over_tilted = tilt_metric > self.cfg.tilt_limit
        bad_state = (
            torch.isnan(q).any(dim=1)
            | torch.isnan(qd).any(dim=1)
            | torch.isnan(packet.projected_gravity_b).any(dim=1)
            | torch.isnan(packet.imu_gyro_b).any(dim=1)
            | torch.isnan(self._raw_actions).any(dim=1)
        )
        forbidden_body_heights = self.robot.data.body_pos_w[:, self._forbidden_body_ids, 2]
        body_hit_ground = torch.any(forbidden_body_heights < self.cfg.forbidden_body_height_limit, dim=1)

        terminated = over_tilted | bad_state | body_hit_ground
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        else:
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        super()._reset_idx(env_ids)

        default_root_state = self.robot.data.default_root_state[env_ids].clone()
        default_root_state[:, :3] = self.scene.env_origins[env_ids]
        default_root_state[:, 2] += self.cfg.base_height

        joint_pos = self._standing_q.unsqueeze(0).repeat(len(env_ids), 1)
        joint_pos += self.cfg.reset_joint_pos_noise * torch.randn_like(joint_pos)
        joint_pos = torch.max(torch.min(joint_pos, self._joint_upper[env_ids]), self._joint_lower[env_ids])
        joint_vel = self.cfg.reset_joint_vel_noise * torch.randn((len(env_ids), self.num_dofs), device=self.device)

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)
        self.robot.set_joint_effort_target(torch.zeros_like(joint_pos), env_ids=env_ids)

        self._raw_actions[env_ids] = 0.0
        self._clamped_actions[env_ids] = 0.0
        self._actions[env_ids] = 0.0
        self._last_actions[env_ids] = 0.0
        self._action_rate[env_ids] = 0.0
        self._action_buffer[env_ids] = 0.0
        self._q_des[env_ids] = joint_pos
        self._joint_pos_targets[env_ids] = joint_pos
        self._tau_ff[env_ids] = 0.0
        self._hardware.write_control_packet(self._build_control_packet(env_ids), env_ids=env_ids)

    def get_action_diagnostics(self) -> dict[str, float]:
        saturation_pct = 100.0 * (torch.abs(self._raw_actions) > 1.0).float().mean().item()
        return {
            "raw_action_min": self._raw_actions.min().item(),
            "raw_action_max": self._raw_actions.max().item(),
            "raw_action_abs_mean": torch.abs(self._raw_actions).mean().item(),
            "action_saturation_pct": saturation_pct,
            "clamped_action_min": self._clamped_actions.min().item(),
            "clamped_action_max": self._clamped_actions.max().item(),
            "q_des_min": self._q_des.min().item(),
            "q_des_max": self._q_des.max().item(),
        }
