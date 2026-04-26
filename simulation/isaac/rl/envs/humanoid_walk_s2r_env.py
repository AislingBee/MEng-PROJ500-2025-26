from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_rotate_inverse

from ...configuration.walking_actuator_config import WALKING_ACTUATOR_SETTINGS
from simulation.isaac.configuration.standing_s2r_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
)
from ..interface.hardware_interface import ControlPacket
from ..interface.isaac_hardware_interface import IsaacHardwareInterface


USD_PATH = Path(__file__).resolve().parents[2] / "assets" / "usd_generated" / "robot" / "robot.usd"


@configclass
class HumanoidWalkEnvS2rCfg(DirectRLEnvCfg):
    decimation: int = CONTRACT.decimation
    episode_length_s: float = 10.0

    sim: SimulationCfg = SimulationCfg(
        dt=CONTRACT.sim_dt_s,
        render_interval=decimation,
    )

    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=256,
        env_spacing=3.0,
        replicate_physics=True,
    )

    action_space: int = CONTRACT.action_dim
    # q_rel(12) + qd(12) + target_err(12) + joint_effort(12) + projected_gravity(3)
    # + root_lin_vel_b(3) + root_ang_vel_b(3) + foot_pos_b(6) + foot_vel_b(6)
    # + command(1) + last_actions(12)
    observation_space: int = 82
    state_space: int = 0

    action_scale: tuple[float, ...] = CONTRACT.action_scale

    usd_path: str = str(USD_PATH)
    base_height: float = 0.0

    # Velocity command curriculum.  The policy starts slow and is only asked for
    # faster walking once the velocity-tracking term is consistently high.
    command_lin_vel_x_min: float = 0.05
    command_lin_vel_x_max: float = 0.15
    command_lin_vel_x_max_final: float = 0.50
    command_lin_vel_x_increment: float = 0.05
    command_curriculum_interval_steps: int = 2000
    command_curriculum_success_threshold: float = 0.70
    zero_command_prob: float = 0.0
    enable_command_curriculum: bool = False

    # Contact / gait logic.
    contact_force_threshold: float = 2.0
    foot_slide_contact_threshold: float = 1.0
    command_active_threshold: float = 0.10
    reset_mirror_prob: float = 0.0  # Keep disabled until the joint sign convention is verified.

    # Berkeley-style feet air-time reward settings.
    feet_air_time_threshold_min: float = 0.12
    feet_air_time_threshold_max: float = 0.45
    single_stance_threshold_min: float = 0.04
    single_stance_threshold_max: float = 0.35

    # Additional swing shaping.  This is kept light because the contact-time
    # reward should drive stepping; clearance only discourages toe dragging.
    swing_height_min: float = 0.035
    swing_height_target: float = 0.090

    # Reward kernels.
    upright_k: float = 5.0
    vel_tracking_k: float = 8.0
    pose_k: float = 1.0

    # The important change from the previous version is that the gait reward is
    # now contact-time based, not hand-built touchdown logic.  This mirrors the
    # Berkeley reward scripts more closely for a biped.
    reward_scales = {
        "vel_track": 1.5,
        "upright": 1.0,
        "survival": 0.5,
        "pose": 0.05,
        "feet_air_time": 4.0,
        "single_stance": 3.0,
        "swing_clearance": 1.5,
        "ang_vel": 0.10,
        "joint_vel": 0.02,
        "action_rate": 0.05,
        "lin_vel_y": 1.5,
        "yaw_rate": 1.5,
        "roll_lean": 2.0,
        "pitch_lean": 0.5,
        "backward_vel": 0.5,
        "feet_slide": 3.0,
        "double_swing": 0.5,
    }

    # Curriculum equivalent of modify_reward_weight(...).  The gait terms are
    # ramped in after the policy can stand and track low forward speed.
    enable_reward_curriculum: bool = True
    reward_curriculum_start_step: int = 1000
    reward_curriculum_ramp_steps: int = 4000
    reward_curriculum_terms = (
        "feet_air_time",
        "single_stance",
        "swing_clearance",
    )

    # Push-force curriculum scaffold.  Keep disabled for first walking rebuild.
    # Enable only after flat-ground walking is stable.
    enable_push_curriculum: bool = False
    push_start_step: int = 20000
    push_interval_steps: int = 600
    push_probability: float = 0.15
    push_velocity_xy_initial: float = 0.0
    push_velocity_xy_max: float = 0.6
    push_velocity_xy_increment_factor: float = 1.5
    push_velocity_xy_decrement: float = 0.2

    # Terrain curriculum is not active in this DirectRLEnv because the scene is
    # currently a flat GroundPlaneCfg, not an Isaac Lab TerrainImporter.
    enable_terrain_curriculum: bool = False

    # Termination.
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


class HumanoidWalkEnvS2r(DirectRLEnv):
    cfg: HumanoidWalkEnvS2rCfg

    def __init__(self, cfg: HumanoidWalkEnvS2rCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode=render_mode, **kwargs)

        self.robot: Articulation = self.scene.articulations["robot"]
        self.left_foot_contact_sensor: ContactSensor = self.scene.sensors["left_foot_contact"]
        self.right_foot_contact_sensor: ContactSensor = self.scene.sensors["right_foot_contact"]

        self.num_dofs = self.robot.num_joints
        self.joint_ids = torch.arange(self.num_dofs, device=self.device, dtype=torch.long)

        if self.num_dofs != self.cfg.action_space:
            raise RuntimeError(
                f"Expected {self.cfg.action_space} joints, got {self.num_dofs}. "
                f"Check USD / standing pose / actuator config alignment."
            )
        if tuple(self.robot.joint_names) != CONTRACT.joint_names:
            raise RuntimeError(
                "Robot joint_names do not match the shared S2R policy contract. "
                "Walking and standing must use the same deployment joint order."
            )

        self._step_dt = float(self.cfg.sim.dt * self.cfg.decimation)
        self._common_step_counter = 0

        self._actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._last_actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._commands = torch.zeros((self.num_envs, 1), device=self.device)
        self._joint_pos_targets = torch.zeros((self.num_envs, self.num_dofs), device=self.device)

        self._action_scale = torch.tensor(self.cfg.action_scale, dtype=torch.float32, device=self.device).unsqueeze(0)
        self._gravity_vec_w = torch.tensor([0.0, 0.0, -1.0], device=self.device).repeat(self.num_envs, 1)

        self._standing_q = build_standing_q(device=self.device)
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
            raise RuntimeError("Robot soft joint lower limits do not match the shared S2R policy contract")
        if not torch.allclose(soft_joint_upper[0], self._joint_upper[0], atol=1e-5, rtol=0.0):
            raise RuntimeError("Robot soft joint upper limits do not match the shared S2R policy contract")

        kp_fixed, kd_fixed = build_fixed_gains(device=self.device)
        self._kp_fixed = kp_fixed.unsqueeze(0).repeat(self.num_envs, 1)
        self._kd_fixed = kd_fixed.unsqueeze(0).repeat(self.num_envs, 1)
        self._tau_ff = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float32, device=self.device)
        self._q_des = torch.zeros((self.num_envs, self.num_dofs), dtype=torch.float32, device=self.device)

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

        # Contact timers used as fallback when ContactSensor timer fields are not present.
        self._prev_left_contact = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self._prev_right_contact = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self._left_air_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._right_air_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._left_contact_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._right_contact_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)

        # Episode-level curriculum statistics.
        self._episode_vel_track_sum = torch.zeros(self.num_envs, device=self.device)
        self._episode_alive_steps = torch.zeros(self.num_envs, device=self.device)
        self._last_terminated = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self._last_time_out = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self._last_command_curriculum_step = 0
        self._last_push_curriculum_step = 0
        self._current_command_lin_vel_x_max = float(self.cfg.command_lin_vel_x_max)
        self._current_push_velocity_xy = float(self.cfg.push_velocity_xy_initial)

        # From URDF fixed joint rpy="0 -1.5707963267948963 0"
        # Quaternion order is [w, x, y, z]
        self._root_to_imu_quat = torch.tensor(
            [1.0, 0.0, 0.0, 0.0],
            dtype=torch.float32,
            device=self.device,
        )

        # Once the hardware data has been sorted, change this to = RobotHardwareInterface(...)
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
            raise RuntimeError("NaN detected in actions")

        self._common_step_counter += 1
        self._last_actions[:] = self._actions
        self._actions = torch.clamp(actions, -1.0, 1.0)
        self._apply_random_pushes()

    def _build_control_packet(self, env_ids: Sequence[int] | None = None) -> ControlPacket:
        if env_ids is None:
            q_des = self._q_des
            kp = self._kp_fixed
            kd = self._kd_fixed
            tau_ff = self._tau_ff
        else:
            env_ids_t = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
            q_des = self._q_des[env_ids_t]
            kp = self._kp_fixed[env_ids_t]
            kd = self._kd_fixed[env_ids_t]
            tau_ff = self._tau_ff[env_ids_t]

        return ControlPacket(
            joint_names=list(self.robot.joint_names),
            q_des=q_des.clone(),
            kp=kp.clone(),
            kd=kd.clone(),
            tau_ff=tau_ff.clone(),
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
        packet = self._hardware.read_observation_packet()

        q = packet.joint_pos
        qd = packet.joint_vel
        q_rel = q - self._standing_q.unsqueeze(0)
        q_target_err = self._joint_pos_targets - q

        root_lin_vel_b = self.robot.data.root_lin_vel_b
        root_ang_vel_b = packet.imu_gyro_b
        projected_gravity_b = packet.projected_gravity_b
        joint_effort = packet.joint_effort
        foot_kin = self._get_foot_kinematics()
        foot_pos_b = torch.cat((foot_kin["left_pos_b"], foot_kin["right_pos_b"]), dim=-1)
        foot_vel_b = torch.cat((foot_kin["left_vel_b"], foot_kin["right_vel_b"]), dim=-1)

        obs = torch.cat(
            (
                q_rel,
                qd,
                q_target_err,
                joint_effort,
                projected_gravity_b,
                root_lin_vel_b,
                root_ang_vel_b,
                foot_pos_b,
                foot_vel_b,
                self._commands,
                self._last_actions,
            ),
            dim=-1,
        )

        if obs.shape[1] != self.cfg.observation_space:
            raise RuntimeError(
                f"Observation size mismatch. Expected {self.cfg.observation_space}, got {obs.shape[1]}"
            )

        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observations")

        return {"policy": obs}

    def _reward_scale(self, name: str) -> float:
        scale = float(self.cfg.reward_scales.get(name, 0.0))
        if not self.cfg.enable_reward_curriculum:
            return scale
        if name not in self.cfg.reward_curriculum_terms:
            return scale
        if self._common_step_counter < self.cfg.reward_curriculum_start_step:
            return 0.0
        ramp_steps = max(int(self.cfg.reward_curriculum_ramp_steps), 1)
        ramp_progress = (self._common_step_counter - self.cfg.reward_curriculum_start_step) / ramp_steps
        ramp_progress = max(0.0, min(1.0, ramp_progress))
        return scale * ramp_progress

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
        # This is the DirectRLEnv equivalent of feet_air_time(...): reward only
        # when a foot first contacts after being airborne for long enough.
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
        # This is the DirectRLEnv equivalent of feet_air_time_positive_biped(...).
        # It rewards one-foot contact, one-foot swing, and enough time in that mode.
        next_left_air_steps = torch.where(left_contact, torch.zeros_like(self._left_air_steps), self._left_air_steps + 1)
        next_right_air_steps = torch.where(right_contact, torch.zeros_like(self._right_air_steps), self._right_air_steps + 1)
        next_left_contact_steps = torch.where(left_contact, self._left_contact_steps + 1, torch.zeros_like(self._left_contact_steps))
        next_right_contact_steps = torch.where(right_contact, self._right_contact_steps + 1, torch.zeros_like(self._right_contact_steps))

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

    def _compute_feet_slide_penalty(
        self,
        left_vel_w: torch.Tensor,
        right_vel_w: torch.Tensor,
    ) -> torch.Tensor:
        # Direct equivalent of feet_slide(...): foot horizontal velocity while in contact.
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
        self._left_air_steps = torch.where(
            left_contact,
            torch.zeros_like(self._left_air_steps),
            self._left_air_steps + 1,
        )
        self._right_air_steps = torch.where(
            right_contact,
            torch.zeros_like(self._right_air_steps),
            self._right_air_steps + 1,
        )
        self._left_contact_steps = torch.where(
            left_contact,
            self._left_contact_steps + 1,
            torch.zeros_like(self._left_contact_steps),
        )
        self._right_contact_steps = torch.where(
            right_contact,
            self._right_contact_steps + 1,
            torch.zeros_like(self._right_contact_steps),
        )
        self._prev_left_contact[:] = left_contact
        self._prev_right_contact[:] = right_contact

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
        left_vel_w = foot_kin["left_vel_w"]
        right_vel_w = foot_kin["right_vel_w"]

        left_contact, right_contact, _, _ = self._get_foot_contact_state()
        command_active = (torch.abs(self._commands[:, 0]) > self.cfg.command_active_threshold).float()

        q_err = q - self._standing_q.unsqueeze(0)
        action_rate = self._actions - self._last_actions

        tilt_metric = torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)
        r_upright = torch.exp(-self.cfg.upright_k * tilt_metric)

        # Forward-only walking: do not reward overspeeding, but punish being below command.
        lin_vel_error = torch.clamp(self._commands[:, 0] - root_lin_vel_b[:, 0], min=0.0)
        r_vel_track = torch.exp(-self.cfg.vel_tracking_k * lin_vel_error**2)
        r_pose = torch.exp(-self.cfg.pose_k * torch.mean(q_err**2, dim=1))

        r_feet_air_time = self._compute_feet_air_time_reward(left_contact, right_contact, command_active)
        r_single_stance = self._compute_feet_air_time_positive_biped_reward(left_contact, right_contact, command_active)

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
        left_swing = (~left_contact) & right_contact
        right_swing = (~right_contact) & left_contact
        r_swing_clearance = command_active * (
            left_swing.float() * left_clearance
            + right_swing.float() * right_clearance
        )

        p_ang_vel = torch.mean(root_ang_vel_b**2, dim=1)
        p_joint_vel = torch.mean(qd**2, dim=1)
        p_action_rate = torch.mean(action_rate**2, dim=1)
        p_lin_vel_y = root_lin_vel_b[:, 1] ** 2
        p_yaw_rate = root_ang_vel_b[:, 2] ** 2
        p_roll_lean = projected_gravity_b[:, 1] ** 2
        p_pitch_lean = projected_gravity_b[:, 0] ** 2
        p_backward_vel = torch.clamp(-root_lin_vel_b[:, 0], min=0.0)
        p_feet_slide = self._compute_feet_slide_penalty(left_vel_w, right_vel_w)
        p_double_swing = ((~left_contact) & (~right_contact)).float() * command_active

        survival_term = torch.ones(self.num_envs, device=self.device)

        positive_terms = {
            "vel_track": r_vel_track,
            "upright": r_upright,
            "survival": survival_term,
            "pose": r_pose,
            "feet_air_time": r_feet_air_time,
            "single_stance": r_single_stance,
            "swing_clearance": r_swing_clearance,
        }
        penalty_terms = {
            "ang_vel": p_ang_vel,
            "joint_vel": p_joint_vel,
            "action_rate": p_action_rate,
            "lin_vel_y": p_lin_vel_y,
            "yaw_rate": p_yaw_rate,
            "roll_lean": p_roll_lean,
            "pitch_lean": p_pitch_lean,
            "backward_vel": p_backward_vel,
            "feet_slide": p_feet_slide,
            "double_swing": p_double_swing,
        }

        reward = torch.zeros(self.num_envs, device=self.device)
        for name, value in positive_terms.items():
            reward += self._reward_scale(name) * value
        for name, value in penalty_terms.items():
            reward -= self._reward_scale(name) * value

        self._episode_vel_track_sum += r_vel_track.detach()
        self._episode_alive_steps += 1.0
        self._update_contact_timers(left_contact, right_contact)

        if torch.isnan(reward).any():
            raise RuntimeError("NaN detected in rewards")

        if not hasattr(self, "_reward_debug_counter"):
            self._reward_debug_counter = 0
        self._reward_debug_counter += 1

        if self._reward_debug_counter % 100 == 0:
            print(
                "reward contrib | "
                f"vel: {(self._reward_scale('vel_track') * r_vel_track).mean().item():.4f} | "
                f"upright: {(self._reward_scale('upright') * r_upright).mean().item():.4f} | "
                f"air_time: {(self._reward_scale('feet_air_time') * r_feet_air_time).mean().item():.4f} | "
                f"single_stance: {(self._reward_scale('single_stance') * r_single_stance).mean().item():.4f} | "
                f"clearance: {(self._reward_scale('swing_clearance') * r_swing_clearance).mean().item():.4f} | "
                f"feet_slide_pen: {(self._reward_scale('feet_slide') * p_feet_slide).mean().item():.4f} | "
                f"action_pen: {(self._reward_scale('action_rate') * p_action_rate).mean().item():.4f} | "
                f"lat_vel_pen: {(self._reward_scale('lin_vel_y') * p_lin_vel_y).mean().item():.4f} | "
                f"yaw_pen: {(self._reward_scale('yaw_rate') * p_yaw_rate).mean().item():.4f} | "
                f"command_max: {self._current_command_lin_vel_x_max:.2f} | "
                f"push_max: {self._current_push_velocity_xy:.2f} | "
                f"total: {reward.mean().item():.4f}"
            )

        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        packet = self._hardware.read_observation_packet()
        projected_gravity_b = packet.projected_gravity_b
        tilt_metric = torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)
        over_tilted = tilt_metric > self.cfg.tilt_limit

        bad_state = (
            torch.isnan(q).any(dim=1)
            | torch.isnan(qd).any(dim=1)
            | torch.isnan(projected_gravity_b).any(dim=1)
        )

        forbidden_body_heights = self.robot.data.body_pos_w[:, self._forbidden_body_ids, 2]
        body_hit_ground = torch.any(
            forbidden_body_heights < self.cfg.forbidden_body_height_limit,
            dim=1,
        )

        terminated = over_tilted | bad_state | body_hit_ground
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        self._last_terminated[:] = terminated
        self._last_time_out[:] = time_out
        return terminated, time_out

    def _update_command_curriculum(self, env_ids: torch.Tensor) -> None:
        if not self.cfg.enable_command_curriculum:
            return
        if self._common_step_counter < self.cfg.command_curriculum_interval_steps:
            return
        if self._common_step_counter - self._last_command_curriculum_step < self.cfg.command_curriculum_interval_steps:
            return
        if self._current_command_lin_vel_x_max >= self.cfg.command_lin_vel_x_max_final:
            return

        alive_steps = torch.clamp(self._episode_alive_steps[env_ids], min=1.0)
        mean_vel_track = torch.mean(self._episode_vel_track_sum[env_ids] / alive_steps)
        if mean_vel_track > self.cfg.command_curriculum_success_threshold:
            self._current_command_lin_vel_x_max = min(
                self._current_command_lin_vel_x_max + self.cfg.command_lin_vel_x_increment,
                self.cfg.command_lin_vel_x_max_final,
            )
            self._last_command_curriculum_step = self._common_step_counter
            print(f"[walk curriculum] command_lin_vel_x_max -> {self._current_command_lin_vel_x_max:.2f} m/s")

    def _update_push_curriculum(self, env_ids: torch.Tensor) -> None:
        if not self.cfg.enable_push_curriculum:
            return
        if self._common_step_counter < self.cfg.push_start_step:
            return
        if self._common_step_counter - self._last_push_curriculum_step < self.cfg.push_interval_steps:
            return

        terminated_count = torch.sum(self._last_terminated[env_ids]).item()
        timeout_count = torch.sum(self._last_time_out[env_ids]).item()

        if terminated_count < timeout_count * 2:
            if self._current_push_velocity_xy <= 0.0:
                self._current_push_velocity_xy = 0.05
            else:
                self._current_push_velocity_xy *= self.cfg.push_velocity_xy_increment_factor
            self._current_push_velocity_xy = min(self._current_push_velocity_xy, self.cfg.push_velocity_xy_max)
        elif terminated_count > timeout_count / 2:
            self._current_push_velocity_xy = max(
                self._current_push_velocity_xy - self.cfg.push_velocity_xy_decrement,
                0.0,
            )

        self._last_push_curriculum_step = self._common_step_counter
        print(f"[walk curriculum] push_velocity_xy -> {self._current_push_velocity_xy:.2f} m/s")

    def _apply_random_pushes(self) -> None:
        if not self.cfg.enable_push_curriculum:
            return
        if self._common_step_counter < self.cfg.push_start_step:
            return
        if self._current_push_velocity_xy <= 0.0:
            return
        if self._common_step_counter % self.cfg.push_interval_steps != 0:
            return

        push_mask = torch.rand(self.num_envs, device=self.device) < self.cfg.push_probability
        if not push_mask.any():
            return

        env_ids = torch.nonzero(push_mask, as_tuple=False).flatten()
        root_vel = self.robot.data.root_vel_w[env_ids].clone()
        push = (2.0 * torch.rand((len(env_ids), 2), device=self.device) - 1.0) * self._current_push_velocity_xy
        root_vel[:, 0:2] += push
        self.robot.write_root_velocity_to_sim(root_vel, env_ids)

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        else:
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        self._update_command_curriculum(env_ids)
        self._update_push_curriculum(env_ids)

        super()._reset_idx(env_ids)

        default_root_state = self.robot.data.default_root_state[env_ids].clone()
        default_root_state[:, :3] = self.scene.env_origins[env_ids]
        default_root_state[:, 2] += self.cfg.base_height

        joint_pos = self._standing_q.unsqueeze(0).repeat(len(env_ids), 1)
        joint_pos += 0.02 * torch.randn_like(joint_pos)

        # mirror_mask = torch.rand(len(env_ids), device=self.device) < self.cfg.reset_mirror_prob
        # if mirror_mask.any():
        #     mirrored_joint_pos = self._mirror_leg_joint_positions(joint_pos)
        #     joint_pos[mirror_mask] = mirrored_joint_pos[mirror_mask]

        joint_vel = 0.05 * torch.randn((len(env_ids), self.num_dofs), device=self.device)
        joint_pos = torch.max(torch.min(joint_pos, self._joint_upper[env_ids]), self._joint_lower[env_ids])

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)
        self.robot.set_joint_effort_target(torch.zeros_like(joint_pos), env_ids=env_ids)

        self._actions[env_ids] = 0.0
        self._last_actions[env_ids] = 0.0
        self._joint_pos_targets[env_ids] = joint_pos
        self._q_des[env_ids] = joint_pos
        self._tau_ff[env_ids] = 0.0
        self._hardware.write_control_packet(self._build_control_packet(env_ids), env_ids=env_ids)

        self._prev_left_contact[env_ids] = False
        self._prev_right_contact[env_ids] = False
        self._left_air_steps[env_ids] = 0
        self._right_air_steps[env_ids] = 0
        self._left_contact_steps[env_ids] = 0
        self._right_contact_steps[env_ids] = 0
        self._episode_vel_track_sum[env_ids] = 0.0
        self._episode_alive_steps[env_ids] = 0.0

        num_resets = len(env_ids)
        commands = torch.rand((num_resets, 1), device=self.device)
        commands = (
            self.cfg.command_lin_vel_x_min
            + (self._current_command_lin_vel_x_max - self.cfg.command_lin_vel_x_min) * commands
        )
        zero_mask = torch.rand((num_resets, 1), device=self.device) < self.cfg.zero_command_prob
        commands[zero_mask] = 0.0
        self._commands[env_ids] = commands
