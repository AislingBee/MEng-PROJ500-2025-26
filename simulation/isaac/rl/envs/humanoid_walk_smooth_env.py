from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import isaaclab.sim as sim_utils
import torch
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_rotate_inverse

from simulation.isaac.configuration.humanoid_walk_smooth_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
)
from simulation.isaac.configuration.walking_actuator_config import WALKING_ACTUATOR_SETTINGS
from simulation.isaac.rl.interface.hardware_interface import ControlPacket
from simulation.isaac.rl.interface.isaac_hardware_interface import IsaacHardwareInterface


USD_PATH = Path(__file__).resolve().parents[2] / "assets" / "usd_generated" / "robot" / "robot.usd"


@configclass
class HumanoidWalkSmoothEnvCfg(DirectRLEnvCfg):
    decimation: int = CONTRACT.decimation
    episode_length_s: float = 10.0
    action_delay_steps: int = 0

    sim: SimulationCfg = SimulationCfg(dt=CONTRACT.sim_dt_s, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=256,
        env_spacing=3.0,
        replicate_physics=True,
    )

    action_space: int = CONTRACT.action_dim
    observation_space: int = CONTRACT.obs_dim
    state_space: int = 0

    action_scale: tuple[float, ...] = CONTRACT.action_scale
    usd_path: str = str(USD_PATH)
    base_height: float = 0.0
    gait_frequency_hz: float = CONTRACT.default_gait_frequency_hz
    command_value: float = CONTRACT.default_command_value

    contact_force_threshold: float = 2.0
    foot_slide_contact_threshold: float = 1.0
    command_active_threshold: float = 0.02

    target_step_width: float = 0.24
    min_step_width: float = 0.18
    target_foot_y_abs: float = 0.12
    min_foot_y_abs: float = 0.09

    feet_air_time_threshold_min: float = 0.12
    feet_air_time_threshold_max: float = 0.45
    single_stance_threshold_min: float = 0.04
    single_stance_threshold_max: float = 0.35
    swing_height_min: float = 0.035
    swing_height_target: float = 0.09

    upright_k: float = 5.0
    vel_tracking_k: float = 8.0
    pose_k: float = 1.0

    reward_scales = {
        "vel_track": 6.0,
        "torso_forward": 0.45,
        "upright": 1.60,
        "survival": 0.6,
        "pose": 0.03,
        "feet_air_time": 0.25,
        "single_stance": 1.40,
        "swing_clearance": 0.12,
        "com_align": 3.0,
        "forward_step": 0.0,
        "phase_single_stance": 0.50,
        "phase_forward_step": 0.50,
        "ang_vel": 0.16,
        "joint_vel": 0.015,
        "action_rate": 0.065,
        "raw_action": 0.06,
        "lin_vel_y": 12.5,
        "yaw_rate": 3.25,
        "roll_lean": 4.2,
        "pitch_lean": 1.15,
        "backward_vel": 5.0,
        "feet_slide": 5.6,
        "double_swing": 1.5,
        "bad_weight_shift": 3.0,
        "foot_tilt": 9.5,
        "swing_foot_tilt": 4.2,
        "lateral_step": 0.5,
        "step_width": 1.5,
        "narrow_step": 50.0,
        "foot_side": 1.8,
        "foot_centerline": 50.0,
        "pelvis_lateral": 3.0,
        "step_x_asymmetry": 0.45,
        "air_time_imbalance": 1.10,
        "contact_time_imbalance": 1.10,
    }

    tilt_limit: float = 0.25
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
    forbidden_body_height_limit: float = 0.075

    reset_joint_pos_noise: float = 0.01
    reset_joint_vel_noise: float = 0.03
    reward_debug_interval: int = 100


class HumanoidWalkSmoothEnv(DirectRLEnv):
    cfg: HumanoidWalkSmoothEnvCfg

    def __init__(self, cfg: HumanoidWalkSmoothEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode=render_mode, **kwargs)

        self.robot: Articulation = self.scene.articulations["robot"]
        self.left_foot_contact_sensor: ContactSensor = self.scene.sensors["left_foot_contact"]
        self.right_foot_contact_sensor: ContactSensor = self.scene.sensors["right_foot_contact"]

        self.num_dofs = self.robot.num_joints
        self.joint_ids = torch.arange(self.num_dofs, device=self.device, dtype=torch.long)

        if self.num_dofs != CONTRACT.action_dim:
            raise RuntimeError(f"Expected {CONTRACT.action_dim} joints, got {self.num_dofs}")
        if tuple(self.robot.joint_names) != CONTRACT.joint_names:
            raise RuntimeError("Robot joint_names do not match the smooth walking policy contract")

        self._step_dt = float(self.cfg.sim.dt * self.cfg.decimation)
        self._common_step_counter = 0
        self._reward_debug_counter = 0

        self._raw_actions = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float32, device=self.device)
        self._clamped_actions = torch.zeros_like(self._raw_actions)
        self._actions = torch.zeros_like(self._raw_actions)
        self._last_actions = torch.zeros_like(self._raw_actions)
        self._action_rate = torch.zeros_like(self._raw_actions)
        self._action_delay_steps = int(self.cfg.action_delay_steps)
        self._action_buffer = torch.zeros(
            (self.num_envs, self._action_delay_steps + 1, self.num_dofs),
            dtype=torch.float32,
            device=self.device,
        )

        self._commands = torch.full((self.num_envs, 1), self.cfg.command_value, dtype=torch.float32, device=self.device)
        self._joint_pos_targets = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float32, device=self.device)

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
        self._gravity_vec_w = torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32, device=self.device).repeat(
            self.num_envs, 1
        )

        soft_joint_lower = self.robot.data.soft_joint_pos_limits[..., 0]
        soft_joint_upper = self.robot.data.soft_joint_pos_limits[..., 1]
        if not torch.allclose(soft_joint_lower[0], self._joint_lower[0], atol=1e-5, rtol=0.0):
            raise RuntimeError("Robot soft joint lower limits do not match the smooth walking policy contract")
        if not torch.allclose(soft_joint_upper[0], self._joint_upper[0], atol=1e-5, rtol=0.0):
            raise RuntimeError("Robot soft joint upper limits do not match the smooth walking policy contract")

        kp_fixed, kd_fixed = build_fixed_gains(device=self.device)
        self._kp_fixed = kp_fixed.unsqueeze(0).repeat(self.num_envs, 1)
        self._kd_fixed = kd_fixed.unsqueeze(0).repeat(self.num_envs, 1)
        self._kp_gains = self._kp_fixed.clone()
        self._kd_gains = self._kd_fixed.clone()
        self._tau_ff = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float32, device=self.device)
        self._q_des = self._standing_q.unsqueeze(0).repeat(self.num_envs, 1)
        self._joint_pos_targets[:] = self._q_des

        name_to_body_idx = {name: i for i, name in enumerate(self.robot.body_names)}
        missing_bodies = [name for name in self.cfg.forbidden_body_names if name not in name_to_body_idx]
        if missing_bodies:
            raise RuntimeError(f"Forbidden body names not found in robot.body_names: {missing_bodies}")
        self._forbidden_body_ids = torch.tensor(
            [name_to_body_idx[name] for name in self.cfg.forbidden_body_names],
            device=self.device,
            dtype=torch.long,
        )

        foot_name_candidates = {
            "left": ["l_foot_link", "robot_l_foot_link"],
            "right": ["r_foot_link", "robot_r_foot_link"],
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

        self._prev_left_contact = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self._prev_right_contact = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self._left_air_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._right_air_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._left_contact_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._right_contact_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)

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

        self.scene.sensors["left_foot_contact"] = ContactSensor(
            ContactSensorCfg(
                prim_path="/World/envs/env_.*/Robot/l_foot_link",
                update_period=0.0,
                history_length=3,
                debug_vis=False,
            )
        )
        self.scene.sensors["right_foot_contact"] = ContactSensor(
            ContactSensorCfg(
                prim_path="/World/envs/env_.*/Robot/r_foot_link",
                update_period=0.0,
                history_length=3,
                debug_vis=False,
            )
        )

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

    def _squeeze_sensor_field(self, value: torch.Tensor | None, fallback: torch.Tensor) -> torch.Tensor:
        if value is None:
            return fallback
        if value.dim() == 1:
            return value
        return value.reshape(value.shape[0], -1).amax(dim=1)

    def _get_foot_contact_state(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        left_force_history = getattr(self.left_foot_contact_sensor.data, "net_forces_w_history", None)
        right_force_history = getattr(self.right_foot_contact_sensor.data, "net_forces_w_history", None)

        if left_force_history is not None:
            left_force = torch.max(torch.norm(left_force_history, dim=-1), dim=1)[0].amax(dim=1)
        else:
            left_force = torch.norm(self.left_foot_contact_sensor.data.net_forces_w, dim=-1).amax(dim=1)

        if right_force_history is not None:
            right_force = torch.max(torch.norm(right_force_history, dim=-1), dim=1)[0].amax(dim=1)
        else:
            right_force = torch.norm(self.right_foot_contact_sensor.data.net_forces_w, dim=-1).amax(dim=1)

        left_contact = left_force > self.cfg.contact_force_threshold
        right_contact = right_force > self.cfg.contact_force_threshold
        return left_contact, right_contact, left_force, right_force

    def _get_foot_kinematics(self) -> dict[str, torch.Tensor]:
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

    def _get_phase_clock(self) -> tuple[torch.Tensor, torch.Tensor]:
        phase = torch.remainder(
            self.episode_length_buf.float() * self._step_dt * self.cfg.gait_frequency_hz,
            1.0,
        )
        phase_angle = 2.0 * torch.pi * phase
        return torch.sin(phase_angle).unsqueeze(1), torch.cos(phase_angle).unsqueeze(1)

    def _get_observations(self) -> dict:
        packet = self._hardware.read_observation_packet()

        q_rel = packet.joint_pos - self._standing_q.unsqueeze(0)
        q_target_err = self._joint_pos_targets - packet.joint_pos
        phase_sin, phase_cos = self._get_phase_clock()
        foot_pos_b = packet.foot_pos_b
        if foot_pos_b.shape[-1] != 6:
            raise RuntimeError(f"FK foot_pos_b must have trailing dim 6, got {foot_pos_b.shape[-1]}")

        obs = torch.cat(
            (
                q_rel,
                packet.joint_vel,
                q_target_err,
                packet.joint_effort,
                packet.projected_gravity_b,
                packet.imu_gyro_b,
                self._commands,
                phase_sin,
                phase_cos,
                foot_pos_b,
                self._last_actions,
            ),
            dim=-1,
        )

        expected_obs_dim = sum(CONTRACT.obs_layout.values())
        if expected_obs_dim != CONTRACT.obs_dim:
            raise RuntimeError(
                f"Smooth walking observation contract mismatch: layout={expected_obs_dim}, contract={CONTRACT.obs_dim}"
            )
        if obs.shape[1] != self.cfg.observation_space:
            raise RuntimeError(f"Observation size mismatch. Expected {self.cfg.observation_space}, got {obs.shape[1]}")
        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observations")

        return {"policy": obs}

    def _sensor_first_contact(self, sensor: ContactSensor, fallback: torch.Tensor) -> torch.Tensor:
        try:
            first_contact = sensor.compute_first_contact(self._step_dt)
        except Exception:
            return fallback.float()
        return self._squeeze_sensor_field(first_contact, fallback.float())

    def _compute_feet_air_time_reward(
        self,
        left_contact: torch.Tensor,
        right_contact: torch.Tensor,
        command_active: torch.Tensor,
    ) -> torch.Tensor:
        fallback_left_first = left_contact & (~self._prev_left_contact)
        fallback_right_first = right_contact & (~self._prev_right_contact)

        left_first_contact = self._sensor_first_contact(self.left_foot_contact_sensor, fallback_left_first)
        right_first_contact = self._sensor_first_contact(self.right_foot_contact_sensor, fallback_right_first)

        fallback_left_last_air = self._left_air_steps.float() * self._step_dt
        fallback_right_last_air = self._right_air_steps.float() * self._step_dt
        left_last_air = self._squeeze_sensor_field(
            getattr(self.left_foot_contact_sensor.data, "last_air_time", None),
            fallback_left_last_air,
        )
        right_last_air = self._squeeze_sensor_field(
            getattr(self.right_foot_contact_sensor.data, "last_air_time", None),
            fallback_right_last_air,
        )

        last_air_time = torch.stack((left_last_air, right_last_air), dim=1)
        first_contact = torch.stack((left_first_contact, right_first_contact), dim=1).float()
        air_time = (last_air_time - self.cfg.feet_air_time_threshold_min) * first_contact
        air_time = torch.clamp(
            air_time,
            min=0.0,
            max=self.cfg.feet_air_time_threshold_max - self.cfg.feet_air_time_threshold_min,
        )
        return torch.sum(air_time, dim=1) * command_active

    def _compute_feet_air_time_positive_biped_reward(
        self,
        left_contact: torch.Tensor,
        right_contact: torch.Tensor,
        command_active: torch.Tensor,
    ) -> torch.Tensor:
        next_left_air_steps = torch.where(left_contact, torch.zeros_like(self._left_air_steps), self._left_air_steps + 1)
        next_right_air_steps = torch.where(
            right_contact, torch.zeros_like(self._right_air_steps), self._right_air_steps + 1
        )
        next_left_contact_steps = torch.where(
            left_contact, self._left_contact_steps + 1, torch.zeros_like(self._left_contact_steps)
        )
        next_right_contact_steps = torch.where(
            right_contact, self._right_contact_steps + 1, torch.zeros_like(self._right_contact_steps)
        )

        fallback_left_air = next_left_air_steps.float() * self._step_dt
        fallback_right_air = next_right_air_steps.float() * self._step_dt
        fallback_left_contact = next_left_contact_steps.float() * self._step_dt
        fallback_right_contact = next_right_contact_steps.float() * self._step_dt

        left_air_time = self._squeeze_sensor_field(
            getattr(self.left_foot_contact_sensor.data, "current_air_time", None),
            fallback_left_air,
        )
        right_air_time = self._squeeze_sensor_field(
            getattr(self.right_foot_contact_sensor.data, "current_air_time", None),
            fallback_right_air,
        )
        left_contact_time = self._squeeze_sensor_field(
            getattr(self.left_foot_contact_sensor.data, "current_contact_time", None),
            fallback_left_contact,
        )
        right_contact_time = self._squeeze_sensor_field(
            getattr(self.right_foot_contact_sensor.data, "current_contact_time", None),
            fallback_right_contact,
        )

        air_time = torch.stack((left_air_time, right_air_time), dim=1)
        contact_time = torch.stack((left_contact_time, right_contact_time), dim=1)
        in_contact = torch.stack((left_contact, right_contact), dim=1)
        in_mode_time = torch.where(in_contact, contact_time, air_time)
        single_stance = torch.sum(in_contact.int(), dim=1) == 1

        reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
        reward = torch.clamp(reward, max=self.cfg.single_stance_threshold_max)
        reward *= reward > self.cfg.single_stance_threshold_min
        reward *= command_active
        return reward

    def _compute_feet_slide_penalty(self, left_vel_w: torch.Tensor, right_vel_w: torch.Tensor) -> torch.Tensor:
        left_force_history = getattr(self.left_foot_contact_sensor.data, "net_forces_w_history", None)
        right_force_history = getattr(self.right_foot_contact_sensor.data, "net_forces_w_history", None)

        if left_force_history is not None:
            left_contact = torch.max(torch.norm(left_force_history, dim=-1), dim=1)[0].amax(dim=1)
            left_contact = left_contact > self.cfg.foot_slide_contact_threshold
        else:
            left_contact, _, _, _ = self._get_foot_contact_state()

        if right_force_history is not None:
            right_contact = torch.max(torch.norm(right_force_history, dim=-1), dim=1)[0].amax(dim=1)
            right_contact = right_contact > self.cfg.foot_slide_contact_threshold
        else:
            _, right_contact, _, _ = self._get_foot_contact_state()

        left_foot_vel_xy_w = torch.norm(left_vel_w[:, :2], dim=1)
        right_foot_vel_xy_w = torch.norm(right_vel_w[:, :2], dim=1)
        return left_contact.float() * left_foot_vel_xy_w + right_contact.float() * right_foot_vel_xy_w

    def _update_contact_timers(self, left_contact: torch.Tensor, right_contact: torch.Tensor) -> None:
        self._left_air_steps = torch.where(left_contact, torch.zeros_like(self._left_air_steps), self._left_air_steps + 1)
        self._right_air_steps = torch.where(
            right_contact, torch.zeros_like(self._right_air_steps), self._right_air_steps + 1
        )
        self._left_contact_steps = torch.where(
            left_contact, self._left_contact_steps + 1, torch.zeros_like(self._left_contact_steps)
        )
        self._right_contact_steps = torch.where(
            right_contact, self._right_contact_steps + 1, torch.zeros_like(self._right_contact_steps)
        )
        self._prev_left_contact[:] = left_contact
        self._prev_right_contact[:] = right_contact

    def _reward_scale(self, name: str) -> float:
        return float(self.cfg.reward_scales.get(name, 0.0))

    def _get_rewards(self) -> torch.Tensor:
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        root_lin_vel_b = self.robot.data.root_lin_vel_b
        packet = self._hardware.read_observation_packet()
        root_ang_vel_b = packet.imu_gyro_b
        projected_gravity_b = packet.projected_gravity_b

        foot_kin = self._get_foot_kinematics()
        left_pos_w = foot_kin["left_pos_w"]
        right_pos_w = foot_kin["right_pos_w"]
        left_pos_b = foot_kin["left_pos_b"]
        right_pos_b = foot_kin["right_pos_b"]
        left_vel_w = foot_kin["left_vel_w"]
        right_vel_w = foot_kin["right_vel_w"]
        left_vel_b = foot_kin["left_vel_b"]
        right_vel_b = foot_kin["right_vel_b"]
        left_quat_w = foot_kin["left_quat_w"]
        right_quat_w = foot_kin["right_quat_w"]

        left_contact, right_contact, _, _ = self._get_foot_contact_state()
        command_active = (torch.abs(self._commands[:, 0]) > self.cfg.command_active_threshold).float()
        phase = torch.remainder(
            self.episode_length_buf.float() * self._step_dt * self.cfg.gait_frequency_hz,
            1.0,
        )
        left_swing_phase = phase < 0.5
        right_swing_phase = ~left_swing_phase

        q_err = q - self._standing_q.unsqueeze(0)
        raw_action_pen = torch.mean(self._raw_actions ** 2, dim=1)
        action_rate_pen = torch.mean(self._action_rate ** 2, dim=1)

        tilt_metric = torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)
        r_upright = torch.exp(-self.cfg.upright_k * tilt_metric)

        lin_vel_error = self._commands[:, 0] - root_lin_vel_b[:, 0]
        r_vel_track = torch.exp(-self.cfg.vel_tracking_k * lin_vel_error**2)
        r_torso_forward = command_active * torch.clamp(root_lin_vel_b[:, 0], min=0.0, max=0.25)
        r_pose = torch.exp(-self.cfg.pose_k * torch.mean(q_err**2, dim=1))

        r_feet_air_time = self._compute_feet_air_time_reward(left_contact, right_contact, command_active)
        r_single_stance = self._compute_feet_air_time_positive_biped_reward(left_contact, right_contact, command_active)

        left_air_time = self._left_air_steps.float() * self._step_dt
        right_air_time = self._right_air_steps.float() * self._step_dt
        left_contact_time = self._left_contact_steps.float() * self._step_dt
        right_contact_time = self._right_contact_steps.float() * self._step_dt
        p_contact_time_imbalance = command_active * torch.abs(left_contact_time - right_contact_time)
        p_air_time_imbalance = command_active * torch.abs(left_air_time - right_air_time)

        step_gate = torch.clamp(r_single_stance / 0.10, min=0.0, max=1.0)
        r_vel_track = r_vel_track * (0.25 + 0.75 * step_gate)

        left_clearance = torch.clamp(
            (left_pos_w[:, 2] - self.cfg.swing_height_min)
            / (self.cfg.swing_height_target - self.cfg.swing_height_min),
            min=0.0,
            max=1.0,
        )
        right_clearance = torch.clamp(
            (right_pos_w[:, 2] - self.cfg.swing_height_min)
            / (self.cfg.swing_height_target - self.cfg.swing_height_min),
            min=0.0,
            max=1.0,
        )

        step_width = torch.abs(left_pos_b[:, 1] - right_pos_b[:, 1])
        r_step_width = torch.exp(-20.0 * (step_width - self.cfg.target_step_width) ** 2)
        p_narrow_step = torch.clamp(self.cfg.min_step_width - step_width, min=0.0)

        left_y_abs = torch.abs(left_pos_b[:, 1])
        right_y_abs = torch.abs(right_pos_b[:, 1])
        r_foot_side = 0.5 * (
            torch.exp(-40.0 * (left_y_abs - self.cfg.target_foot_y_abs) ** 2)
            + torch.exp(-40.0 * (right_y_abs - self.cfg.target_foot_y_abs) ** 2)
        )
        p_foot_centerline = (
            torch.clamp(self.cfg.min_foot_y_abs - left_y_abs, min=0.0)
            + torch.clamp(self.cfg.min_foot_y_abs - right_y_abs, min=0.0)
        )
        foot_mid_y = 0.5 * (left_pos_b[:, 1] + right_pos_b[:, 1])
        p_pelvis_lateral = foot_mid_y**2

        left_swing = (~left_contact) & right_contact
        right_swing = (~right_contact) & left_contact
        left_stance = left_contact & (~right_contact)
        right_stance = right_contact & (~left_contact)
        left_phase_single_stance = left_swing_phase & left_swing
        right_phase_single_stance = right_swing_phase & right_swing
        r_phase_single_stance = command_active * (
            left_phase_single_stance.float() + right_phase_single_stance.float()
        )

        pelvis_y_b = torch.zeros_like(left_pos_b[:, 1])
        com_to_left_foot_y = torch.abs(pelvis_y_b - left_pos_b[:, 1])
        com_to_right_foot_y = torch.abs(pelvis_y_b - right_pos_b[:, 1])

        r_com_align = command_active * (
            left_stance.float() * torch.exp(-20.0 * com_to_left_foot_y**2)
            + right_stance.float() * torch.exp(-20.0 * com_to_right_foot_y**2)
        )
        p_bad_weight_shift = command_active * (
            left_stance.float() * torch.clamp(com_to_left_foot_y - 0.07, min=0.0)
            + right_stance.float() * torch.clamp(com_to_right_foot_y - 0.07, min=0.0)
        )

        left_gravity_foot = quat_rotate_inverse(left_quat_w, self._gravity_vec_w)
        right_gravity_foot = quat_rotate_inverse(right_quat_w, self._gravity_vec_w)
        left_foot_tilt = torch.sum(left_gravity_foot[:, :2] ** 2, dim=1)
        right_foot_tilt = torch.sum(right_gravity_foot[:, :2] ** 2, dim=1)
        p_foot_tilt = command_active * (
            left_contact.float() * left_foot_tilt + right_contact.float() * right_foot_tilt
        )

        near_ground_left = torch.clamp((0.08 - left_pos_w[:, 2]) / 0.08, min=0.0, max=1.0)
        near_ground_right = torch.clamp((0.08 - right_pos_w[:, 2]) / 0.08, min=0.0, max=1.0)
        p_swing_foot_tilt = command_active * (
            left_swing.float() * near_ground_left * left_foot_tilt
            + right_swing.float() * near_ground_right * right_foot_tilt
        )

        r_phase_forward_step = command_active * (
            left_phase_single_stance.float() * torch.clamp(left_vel_b[:, 0], min=0.0, max=0.75)
            + right_phase_single_stance.float() * torch.clamp(right_vel_b[:, 0], min=0.0, max=0.75)
        )
        p_lateral_step = command_active * (
            left_swing.float() * torch.abs(left_vel_b[:, 1])
            + right_swing.float() * torch.abs(right_vel_b[:, 1])
        )
        r_swing_clearance = command_active * (
            left_swing.float() * left_clearance + right_swing.float() * right_clearance
        )
        p_step_x_asymmetry = command_active * torch.abs(left_pos_b[:, 0] + right_pos_b[:, 0])

        p_ang_vel = torch.mean(root_ang_vel_b**2, dim=1)
        p_joint_vel = torch.mean(qd**2, dim=1)
        p_lin_vel_y = root_lin_vel_b[:, 1] ** 2
        p_yaw_rate = root_ang_vel_b[:, 2] ** 2
        p_roll_lean = projected_gravity_b[:, 1] ** 2
        p_pitch_lean = projected_gravity_b[:, 0] ** 2
        p_backward_vel = torch.clamp(-root_lin_vel_b[:, 0], min=0.0)
        p_feet_slide = self._compute_feet_slide_penalty(left_vel_w, right_vel_w)
        p_double_swing = ((~left_contact) & (~right_contact)).float() * command_active

        survival_term = torch.ones(self.num_envs, device=self.device)
        reward = (
            self._reward_scale("vel_track") * r_vel_track
            + self._reward_scale("torso_forward") * r_torso_forward
            + self._reward_scale("upright") * r_upright
            + self._reward_scale("survival") * survival_term
            + self._reward_scale("pose") * r_pose
            + self._reward_scale("feet_air_time") * r_feet_air_time
            + self._reward_scale("single_stance") * r_single_stance
            + self._reward_scale("phase_single_stance") * r_phase_single_stance
            + self._reward_scale("swing_clearance") * r_swing_clearance
            + self._reward_scale("com_align") * r_com_align
            + self._reward_scale("phase_forward_step") * r_phase_forward_step
            + self._reward_scale("step_width") * r_step_width
            + self._reward_scale("foot_side") * r_foot_side
            - self._reward_scale("ang_vel") * p_ang_vel
            - self._reward_scale("joint_vel") * p_joint_vel
            - self._reward_scale("action_rate") * action_rate_pen
            - self._reward_scale("raw_action") * raw_action_pen
            - self._reward_scale("lin_vel_y") * p_lin_vel_y
            - self._reward_scale("yaw_rate") * p_yaw_rate
            - self._reward_scale("roll_lean") * p_roll_lean
            - self._reward_scale("pitch_lean") * p_pitch_lean
            - self._reward_scale("backward_vel") * p_backward_vel
            - self._reward_scale("feet_slide") * p_feet_slide
            - self._reward_scale("double_swing") * p_double_swing
            - self._reward_scale("narrow_step") * p_narrow_step
            - self._reward_scale("foot_centerline") * p_foot_centerline
            - self._reward_scale("pelvis_lateral") * p_pelvis_lateral
            - self._reward_scale("step_x_asymmetry") * p_step_x_asymmetry
            - self._reward_scale("bad_weight_shift") * p_bad_weight_shift
            - self._reward_scale("foot_tilt") * p_foot_tilt
            - self._reward_scale("lateral_step") * p_lateral_step
            - self._reward_scale("air_time_imbalance") * p_air_time_imbalance
            - self._reward_scale("contact_time_imbalance") * p_contact_time_imbalance
            - self._reward_scale("swing_foot_tilt") * p_swing_foot_tilt
        )

        self._update_contact_timers(left_contact, right_contact)

        if torch.isnan(reward).any():
            raise RuntimeError("NaN detected in rewards")

        self._reward_debug_counter += 1
        if self._reward_debug_counter % self.cfg.reward_debug_interval == 0:
            diag = self.get_action_diagnostics()
            print(
                "smooth walk reward | "
                f"vel_track: {(self._reward_scale('vel_track') * r_vel_track).mean().item():.4f} | "
                f"upright: {(self._reward_scale('upright') * r_upright).mean().item():.4f} | "
                f"single_stance: {(self._reward_scale('single_stance') * r_single_stance).mean().item():.4f} | "
                f"phase_single_stance: {(self._reward_scale('phase_single_stance') * r_phase_single_stance).mean().item():.4f} | "
                f"phase_forward_step: {(self._reward_scale('phase_forward_step') * r_phase_forward_step).mean().item():.4f} | "
                f"foot_clearance: {(self._reward_scale('swing_clearance') * r_swing_clearance).mean().item():.4f} | "
                f"step_x_asym_pen: {(self._reward_scale('step_x_asymmetry') * p_step_x_asymmetry).mean().item():.4f} | "
                f"action_rate_pen: {(self._reward_scale('action_rate') * action_rate_pen).mean().item():.4f} | "
                f"raw_action_pen: {(self._reward_scale('raw_action') * raw_action_pen).mean().item():.4f} | "
                f"left_contact_ratio: {left_contact.float().mean().item():.3f} | "
                f"right_contact_ratio: {right_contact.float().mean().item():.3f} | "
                f"saturation_pct: {diag['action_saturation_pct']:.2f}% | "
                f"raw_abs_mean: {diag['raw_action_abs_mean']:.3f} | "
                f"q_des_min/max: {diag['q_des_min']:+.3f}/{diag['q_des_max']:+.3f} | "
                f"total_reward: {reward.mean().item():.4f}"
            )

        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        packet = self._hardware.read_observation_packet()
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
        self._commands[env_ids] = self.cfg.command_value
        self._q_des[env_ids] = joint_pos
        self._joint_pos_targets[env_ids] = joint_pos
        self._tau_ff[env_ids] = 0.0
        self._hardware.write_control_packet(self._build_control_packet(env_ids), env_ids=env_ids)

        self._prev_left_contact[env_ids] = False
        self._prev_right_contact[env_ids] = False
        self._left_air_steps[env_ids] = 0
        self._right_air_steps[env_ids] = 0
        self._left_contact_steps[env_ids] = 0
        self._right_contact_steps[env_ids] = 0

    def get_action_diagnostics(self) -> dict[str, float]:
        saturation_pct = 100.0 * (torch.abs(self._raw_actions) > 1.0).float().mean().item()
        return {
            "raw_action_min": self._raw_actions.min().item(),
            "raw_action_max": self._raw_actions.max().item(),
            "raw_action_abs_mean": torch.abs(self._raw_actions).mean().item(),
            "clamped_action_min": self._clamped_actions.min().item(),
            "clamped_action_max": self._clamped_actions.max().item(),
            "action_saturation_pct": saturation_pct,
            "q_des_min": self._q_des.min().item(),
            "q_des_max": self._q_des.max().item(),
        }

    def get_runtime_debug_metrics(self) -> dict[str, float]:
        phase_sin, phase_cos = self._get_phase_clock()
        phase = torch.atan2(phase_sin[:, 0], phase_cos[:, 0]) / (2.0 * torch.pi)
        phase = torch.remainder(phase, 1.0)
        metrics = self.get_action_diagnostics()
        metrics.update(
            {
                "command_mean": self._commands[:, 0].mean().item(),
                "phase_mean": phase.mean().item(),
                "phase_sin_mean": phase_sin[:, 0].mean().item(),
                "phase_cos_mean": phase_cos[:, 0].mean().item(),
            }
        )
        return metrics
