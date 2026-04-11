from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import math

import torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_rotate_inverse

from ...configuration.walking_actuator_config import WALKING_ACTUATOR_SETTINGS
from simulation.isaac.configuration.standing_pose import STANDING_TARGETS_DEG


USD_PATH = Path(__file__).resolve().parents[2] / "assets" / "usd_generated" / "skeleton" / "skeleton.usd"


@configclass
class HumanoidWalkEnvCfg(DirectRLEnvCfg):
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
    # q_rel(12) + qd(12) + target_err(12) + joint_effort(12) + projected_gravity(3)
    # + root_lin_vel_b(3) + root_ang_vel_b(3) + foot_pos_b(6) + foot_vel_b(6)
    # + command(1) + last_actions(12)
    observation_space: int = 82
    state_space: int = 0

    action_scale: tuple[float, ...] = (
        0.05, 0.05, 0.15, 0.20, 0.10, 0.06,
        0.05, 0.05, 0.15, 0.20, 0.10, 0.06,
    )

    usd_path: str = str(USD_PATH)
    base_height: float = 0.0

    # Velocity command curriculum
    command_lin_vel_x_min: float = 0.05
    command_lin_vel_x_max: float = 0.15
    zero_command_prob: float = 0.0

    # Contact / gait logic
    contact_force_threshold: float = 2.0
    step_reward_command_threshold: float = 0.04
    min_step_forward_vel: float = 0
    step_cooldown_steps: int = 18
    min_swing_air_steps: int = 4
    touchdown_forward_margin: float = 0.08

    # Swing shaping
    swing_height_min: float = 0.045
    swing_height_target: float = 0.090

    # Reward kernels
    upright_k: float = 5.0
    vel_tracking_k: float = 8.0
    pose_k: float = 1.0
    reward_scales = {
        "vel_track": 3.0,
        "upright": 1.0,
        "pose": 0.0,
        "ang_vel": 0.10,
        "joint_vel": 0.02,
        "action_rate": 0.05,
        "lin_vel_y": 1.5,
        "yaw_rate": 1.5,
        "roll_lean": 2.0,
        "touchdown": 2.0,
        "step_alternation": 3.0,
        "stance_slip": 2.5,
        "stance_tilt": 1.0,
        "swing_clearance": 1.2,
        "survival": 1.0,
        "double_swing": 0.5,
        "repeat_step": 0.75,
        "forward_step": 1.0,
        "loaded_swing": 0.01,
        "lateral_step": 1.0,
    }

    # Termination
    tilt_limit: float = 0.25
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


class HumanoidWalkEnv(DirectRLEnv):
    cfg: HumanoidWalkEnvCfg

    def __init__(self, cfg: HumanoidWalkEnvCfg, render_mode: str | None = None, **kwargs):
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

        self._actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._last_actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._commands = torch.zeros((self.num_envs, 1), device=self.device)
        self._joint_pos_targets = torch.zeros((self.num_envs, self.num_dofs), device=self.device)

        self._action_scale = torch.tensor(self.cfg.action_scale, dtype=torch.float32, device=self.device).unsqueeze(0)
        self._gravity_vec_w = torch.tensor([0.0, 0.0, -1.0], device=self.device).repeat(self.num_envs, 1)

        self._standing_q = self._build_standing_pose_tensor()
        self._joint_lower = self.robot.data.soft_joint_pos_limits[..., 0].clone()
        self._joint_upper = self.robot.data.soft_joint_pos_limits[..., 1].clone()

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

        self._left_step_cooldown = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._right_step_cooldown = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._last_step_side = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._prev_left_contact = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self._prev_right_contact = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self._left_air_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._right_air_steps = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self._prev_left_touchdown_x = torch.zeros(self.num_envs, device=self.device)
        self._prev_right_touchdown_x = torch.zeros(self.num_envs, device=self.device)

    def _setup_scene(self):
        ground_cfg = sim_utils.GroundPlaneCfg()
        ground_cfg.func("/World/ground", ground_cfg)

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0)
        light_cfg.func("/World/light", light_cfg)

        actuators = {}
        for group_name, cfg in WALKING_ACTUATOR_SETTINGS.items():
            actuators[group_name] = DCMotorCfg(
                joint_names_expr=cfg["joint_names"],
                saturation_effort=cfg["saturation_effort"],
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

        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])

        self.scene.sensors["left_foot_contact"] = ContactSensor(
            ContactSensorCfg(
                prim_path="/World/envs/env_.*/Robot/robot_l_foot_link",
                update_period=0.0,
                history_length=3,
                debug_vis=False,
            )
        )
        self.scene.sensors["right_foot_contact"] = ContactSensor(
            ContactSensorCfg(
                prim_path="/World/envs/env_.*/Robot/robot_r_foot_link",
                update_period=0.0,
                history_length=3,
                debug_vis=False,
            )
        )



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
        targets = self._standing_q.unsqueeze(0) + self._action_scale * self._actions
        targets = torch.max(torch.min(targets, self._joint_upper), self._joint_lower)
        self._joint_pos_targets[:] = targets
        self.robot.set_joint_position_target(targets, joint_ids=self.joint_ids)

    def _get_joint_effort_obs(self) -> torch.Tensor:
        applied_torque = getattr(self.robot.data, "applied_torque", None)
        if applied_torque is not None:
            return applied_torque
        computed_torque = getattr(self.robot.data, "computed_torque", None)
        if computed_torque is not None:
            return computed_torque
        return torch.zeros_like(self.robot.data.joint_pos)

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
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        q_rel = q - self._standing_q.unsqueeze(0)
        q_target_err = self._joint_pos_targets - q

        root_quat_w = self.robot.data.root_quat_w
        root_ang_vel_b = self.robot.data.root_ang_vel_b
        root_lin_vel_b = self.robot.data.root_lin_vel_b
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)

        joint_effort = self._get_joint_effort_obs()
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

        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observations")

        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:

        # left_sensor = self.scene.sensors["left_foot_contact"]
        # print(left_sensor.data.net_forces_w.shape)
        # print(left_sensor.data.net_forces_w[0])

        self._left_step_cooldown = torch.clamp(self._left_step_cooldown - 1, min=0)
        self._right_step_cooldown = torch.clamp(self._right_step_cooldown - 1, min=0)

        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel
        root_quat_w = self.robot.data.root_quat_w
        root_lin_vel_b = self.robot.data.root_lin_vel_b
        root_ang_vel_b = self.robot.data.root_ang_vel_b
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)

        foot_kin = self._get_foot_kinematics()
        left_pos_w = foot_kin["left_pos_w"]
        right_pos_w = foot_kin["right_pos_w"]
        left_pos_b = foot_kin["left_pos_b"]
        right_pos_b = foot_kin["right_pos_b"]
        left_vel_w = foot_kin["left_vel_w"]
        right_vel_w = foot_kin["right_vel_w"]
        left_quat_w = foot_kin["left_quat_w"]
        right_quat_w = foot_kin["right_quat_w"]

        left_sensor = self.scene.sensors["left_foot_contact"]
        right_sensor = self.scene.sensors["right_foot_contact"]

        left_force = left_sensor.data.net_forces_w[:, 0, 2]
        right_force = right_sensor.data.net_forces_w[:, 0, 2]

        left_contact = left_force > self.cfg.contact_force_threshold
        right_contact = right_force > self.cfg.contact_force_threshold

        command_active = (self._commands[:, 0] > self.cfg.step_reward_command_threshold).float()
        command_active_bool = self._commands[:, 0] > self.cfg.step_reward_command_threshold

        q_err = q - self._standing_q.unsqueeze(0)
        action_rate = self._actions - self._last_actions

        tilt_metric = torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)
        r_upright = torch.exp(-self.cfg.upright_k * tilt_metric)

        lin_vel_error = torch.clamp(self._commands[:, 0] - root_lin_vel_b[:, 0], min=0.0)
        r_vel_track = torch.exp(-self.cfg.vel_tracking_k * lin_vel_error**2)
        r_pose = torch.exp(-self.cfg.pose_k * torch.mean(q_err**2, dim=1))

        p_ang_vel = torch.mean(root_ang_vel_b**2, dim=1)
        p_joint_vel = torch.mean(qd**2, dim=1)
        p_action_rate = torch.mean(action_rate**2, dim=1)
        p_lin_vel_y = root_lin_vel_b[:, 1] ** 2
        p_yaw_rate = root_ang_vel_b[:, 2] ** 2
        p_roll_lean = projected_gravity_b[:, 1] ** 2

        left_foot_vel_xy_w = torch.norm(left_vel_w[:, :2], dim=1)
        right_foot_vel_xy_w = torch.norm(right_vel_w[:, :2], dim=1)
        p_stance_slip = left_contact.float() * left_foot_vel_xy_w + right_contact.float() * right_foot_vel_xy_w

        left_foot_gravity = quat_rotate_inverse(left_quat_w, self._gravity_vec_w)
        right_foot_gravity = quat_rotate_inverse(right_quat_w, self._gravity_vec_w)
        left_stance_tilt = torch.sum(left_foot_gravity[:, :2] ** 2, dim=1)
        right_stance_tilt = torch.sum(right_foot_gravity[:, :2] ** 2, dim=1)
        p_stance_tilt = left_contact.float() * left_stance_tilt + right_contact.float() * right_stance_tilt

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

        left_supported_swing = (~left_contact) & right_contact
        right_supported_swing = (~right_contact) & left_contact

        r_swing_clearance = command_active * (
                left_supported_swing.float() * left_clearance
                + right_supported_swing.float() * right_clearance
        )

        left_forward_swing = left_supported_swing.float() * torch.clamp(foot_kin["left_vel_b"][:, 0], min=0.0, max=0.5)
        right_forward_swing = right_supported_swing.float() * torch.clamp(foot_kin["right_vel_b"][:, 0], min=0.0, max=0.5)
        r_forward_swing = left_forward_swing + right_forward_swing

        left_bad_loaded_swing = left_supported_swing.float() * torch.clamp(left_force - 20.0, min=0.0)
        right_bad_loaded_swing = right_supported_swing.float() * torch.clamp(right_force - 20.0, min=0.0)

        p_loaded_swing = left_bad_loaded_swing + right_bad_loaded_swing

        prev_left_air_steps = self._left_air_steps.clone()
        prev_right_air_steps = self._right_air_steps.clone()

        left_touchdown = (
            left_contact
            & (~self._prev_left_contact)
            & (prev_left_air_steps >= self.cfg.min_swing_air_steps)
        )
        right_touchdown = (
            right_contact
            & (~self._prev_right_contact)
            & (prev_right_air_steps >= self.cfg.min_swing_air_steps)
        )

        left_rewarded_touchdown = left_touchdown & right_contact
        right_rewarded_touchdown = right_touchdown & left_contact

        r_touchdown = left_rewarded_touchdown.float() + right_rewarded_touchdown.float()

        left_repeat = left_rewarded_touchdown & (self._last_step_side == 1)
        right_repeat = right_rewarded_touchdown & (self._last_step_side == 2)
        p_repeat_step = left_repeat.float() + right_repeat.float()

        left_alt = left_rewarded_touchdown & (self._last_step_side != 1)
        right_alt = right_rewarded_touchdown & (self._last_step_side != 2)
        r_step_alternation = left_alt.float() + right_alt.float()

        self._last_step_side[left_alt] = 1
        self._last_step_side[right_alt] = 2

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

        self._prev_left_contact[:] = left_contact
        self._prev_right_contact[:] = right_contact

        both_feet_airborne = (~left_contact) & (~right_contact)
        p_double_swing = both_feet_airborne.float()

        left_lateral_error = torch.abs(left_pos_b[:, 1])
        right_lateral_error = torch.abs(right_pos_b[:, 1])

        p_lateral_step = (
                left_rewarded_touchdown.float() * left_lateral_error +
                right_rewarded_touchdown.float() * right_lateral_error
        )

        survival_term = torch.ones(self.num_envs, device=self.device)

        reward = (
            self.cfg.reward_scales["vel_track"] * r_vel_track
            + self.cfg.reward_scales["upright"] * r_upright
            + self.cfg.reward_scales["pose"] * r_pose
            + self.cfg.reward_scales["touchdown"] * r_touchdown
            + self.cfg.reward_scales["step_alternation"] * r_step_alternation
            + self.cfg.reward_scales["swing_clearance"] * r_swing_clearance
            + self.cfg.reward_scales["survival"] * survival_term
            + self.cfg.reward_scales["forward_step"] * r_forward_swing
            - self.cfg.reward_scales["ang_vel"] * p_ang_vel
            - self.cfg.reward_scales["joint_vel"] * p_joint_vel
            - self.cfg.reward_scales["action_rate"] * p_action_rate
            - self.cfg.reward_scales["lin_vel_y"] * p_lin_vel_y
            - self.cfg.reward_scales["yaw_rate"] * p_yaw_rate
            - self.cfg.reward_scales["roll_lean"] * p_roll_lean
            - self.cfg.reward_scales["stance_slip"] * p_stance_slip
            - self.cfg.reward_scales["stance_tilt"] * p_stance_tilt
            - self.cfg.reward_scales["double_swing"] * p_double_swing
            - self.cfg.reward_scales["repeat_step"] * p_repeat_step
            - self.cfg.reward_scales["loaded_swing"] * p_loaded_swing
            - self.cfg.reward_scales["lateral_step"] * p_lateral_step
        )

        if torch.isnan(reward).any():
            raise RuntimeError("NaN detected in rewards")

        if not hasattr(self, "_reward_debug_counter"):
            self._reward_debug_counter = 0
        self._reward_debug_counter += 1

        if self._reward_debug_counter % 100 == 0:
            vel_term = self.cfg.reward_scales["vel_track"] * r_vel_track
            upright_term = self.cfg.reward_scales["upright"] * r_upright
            pose_term = self.cfg.reward_scales["pose"] * r_pose
            ang_term = self.cfg.reward_scales["ang_vel"] * p_ang_vel
            joint_term = self.cfg.reward_scales["joint_vel"] * p_joint_vel
            action_term = self.cfg.reward_scales["action_rate"] * p_action_rate
            lin_y_term = self.cfg.reward_scales["lin_vel_y"] * p_lin_vel_y
            yaw_term = self.cfg.reward_scales["yaw_rate"] * p_yaw_rate
            roll_lean_term = self.cfg.reward_scales["roll_lean"] * p_roll_lean
            stance_slip_term = self.cfg.reward_scales["stance_slip"] * p_stance_slip
            stance_tilt_term = self.cfg.reward_scales["stance_tilt"] * p_stance_tilt
            swing_clearance_term = self.cfg.reward_scales["swing_clearance"] * r_swing_clearance
            touchdown_term = self.cfg.reward_scales["touchdown"] * r_touchdown
            step_alt_term = self.cfg.reward_scales["step_alternation"] * r_step_alternation
            survival_term = self.cfg.reward_scales["survival"] * survival_term
            double_swing_term = self.cfg.reward_scales["double_swing"] * p_double_swing
            repeat_step_term = self.cfg.reward_scales["repeat_step"] * p_repeat_step
            forward_step_term = self.cfg.reward_scales["forward_step"] * r_forward_swing
            loaded_swing_term = self.cfg.reward_scales["loaded_swing"] * p_loaded_swing
            laterial_step_term = self.cfg.reward_scales["lateral_step"] * p_lateral_step

            print(
                "reward contrib | "
                f"vel: {vel_term.mean().item():.4f} | "
                f"upright: {upright_term.mean().item():.4f} | "
                f"pose: {pose_term.mean().item():.4f} | "
                f"ang_pen: {ang_term.mean().item():.4f} | "
                f"joint_pen: {joint_term.mean().item():.4f} | "
                f"action_pen: {action_term.mean().item():.4f} | "
                f"lin_vel_pen: {lin_y_term.mean().item():.4f} | "
                f"yaw_rate_pen: {yaw_term.mean().item():.4f} | "
                f"roll_lean_pen: {roll_lean_term.mean().item():.4f} | "
                f"stance_slip_pen: {stance_slip_term.mean().item():.4f} | "
                f"stance_tilt_pen: {stance_tilt_term.mean().item():.4f} | "
                f"swing_clear: {swing_clearance_term.mean().item():.4f} | "
                f"touchdown: {touchdown_term.mean().item():.4f} | "
                f"step_alt: {step_alt_term.mean().item():.4f} | "
                f"survival: {survival_term.mean().item():.4f} | "
                f"double_swing: {double_swing_term.mean().item():.4f} | "
                f"repeat_step: {repeat_step_term.mean().item():.4f} | "
                f"forward_step: {forward_step_term.mean().item():.4f} | "
                f"loaded_swing: {loaded_swing_term.mean().item():.4f} | "
                f"laterial_step: {laterial_step_term.mean().item():.4f} | "
                f"total: {reward.mean().item():.4f}"
            )

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
        body_hit_ground = torch.any(
            forbidden_body_heights < self.cfg.forbidden_body_height_limit,
            dim=1,
        )

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
        joint_pos += 0.02 * torch.randn_like(joint_pos)

        joint_vel = 0.05 * torch.randn((len(env_ids), self.num_dofs), device=self.device)
        joint_pos = torch.max(torch.min(joint_pos, self._joint_upper[env_ids]), self._joint_lower[env_ids])

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)

        self._actions[env_ids] = 0.0
        self._last_actions[env_ids] = 0.0
        self._joint_pos_targets[env_ids] = joint_pos
        self._left_step_cooldown[env_ids] = 0
        self._right_step_cooldown[env_ids] = 0
        self._last_step_side[env_ids] = 0
        self._prev_left_contact[env_ids] = False
        self._prev_right_contact[env_ids] = False
        self._left_air_steps[env_ids] = 0
        self._right_air_steps[env_ids] = 0
        self._prev_left_touchdown_x[env_ids] = 0.0
        self._prev_right_touchdown_x[env_ids] = 0.0

        num_resets = len(env_ids)
        commands = torch.rand((num_resets, 1), device=self.device)
        commands = (
            self.cfg.command_lin_vel_x_min
            + (self.cfg.command_lin_vel_x_max - self.cfg.command_lin_vel_x_min) * commands
        )
        zero_mask = torch.rand((num_resets, 1), device=self.device) < self.cfg.zero_command_prob
        commands[zero_mask] = 0.0
        self._commands[env_ids] = commands
