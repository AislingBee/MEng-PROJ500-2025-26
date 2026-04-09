from __future__ import annotations

from asyncio import wait_for
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
    observation_space: int = 46

    action_scale: tuple[float, ...] = (
        0.10, 0.08, 0.15, 0.20, 0.12, 0.08,
        0.10, 0.08, 0.15, 0.20, 0.12, 0.08,
    )

    state_space: int = 0

    usd_path: str = str(USD_PATH)
    base_height: float = 0

    # Velocity parameters and zero probability
    command_lin_vel_x_min: float = 0.0
    command_lin_vel_x_max: float = 0.20
    zero_command_prob: float = 0.2

    # Reward Variables
    upright_k: float = 5.0
    vel_tracking_k: float = 8.0
    pose_k: float = 1.0
    reward_scales = {
        "vel_track": 5.0,
        "upright": 0.3,
        "pose": 0.1,
        "ang_vel": 0.1,
        "joint_vel": 0.02,
        "action_rate": 0.03,
        "lin_vel_y": 2.0,
        "yaw_rate": 2.0,
        "roll_lean": 1.5,
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


class HumanoidWalkEnv(DirectRLEnv):
    cfg: HumanoidWalkEnvCfg

    def __init__(self, cfg: HumanoidWalkEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode=render_mode, **kwargs)

        self.robot: Articulation = self.scene.articulations["robot"]
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

        self._action_scale = torch.tensor(
            self.cfg.action_scale, dtype=torch.float32, device=self.device
        ).unsqueeze(0)

        self._gravity_vec_w = torch.tensor([0.0, 0.0, -1.0], device=self.device).repeat(self.num_envs, 1)

        self._standing_q = self._build_standing_pose_tensor()
        self._joint_lower = self.robot.data.soft_joint_pos_limits[..., 0].clone()
        self._joint_upper = self.robot.data.soft_joint_pos_limits[..., 1].clone()

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
        targets = self._standing_q.unsqueeze(0) + self._action_scale * self._actions
        targets = torch.max(torch.min(targets, self._joint_upper), self._joint_lower)
        self.robot.set_joint_position_target(targets, joint_ids=self.joint_ids)

    def _get_observations(self) -> dict:
        q = self.robot.data.joint_pos # Tells the policy what the joint positions are.
        qd = self.robot.data.joint_vel
        q_rel = q - self._standing_q.unsqueeze(0)

        root_quat_w = self.robot.data.root_quat_w
        root_ang_vel_b = self.robot.data.root_ang_vel_b
        root_lin_vel_b = self.robot.data.root_lin_vel_b
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)

        obs = torch.cat(
            (
                q_rel,  # 12 # Joint positions relative to standing pose.
                qd,  # 12 # Tells the policy how fast the joints are moving.
                projected_gravity_b,  # 3 # Orientation relative to gravity
                root_lin_vel_b,  # 3 # Tells the policy whether it is actually translating.
                root_ang_vel_b,  # 3 # Tells the policy if it is rotating too much.
                self._commands, #1 # Defines the task target
                self._last_actions,  # 12 # Averaging smoother by providing action history
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
        root_lin_vel_b = self.robot.data.root_lin_vel_b
        root_ang_vel_b = self.robot.data.root_ang_vel_b
        projected_gravity_b = quat_rotate_inverse(root_quat_w, self._gravity_vec_w)

        q_err = q - self._standing_q.unsqueeze(0)
        action_rate = self._actions - self._last_actions

        # Upright term
        tilt_metric = torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)
        r_upright = torch.exp(-self.cfg.upright_k * tilt_metric)

        # Forward velocity tracking term
        lin_vel_error = root_lin_vel_b[:, 0] - self._commands[:, 0]
        r_vel_track = torch.exp(-self.cfg.vel_tracking_k * lin_vel_error ** 2)

        # Weak pose regularisation
        r_pose = torch.exp(-self.cfg.pose_k * torch.mean(q_err ** 2, dim=1))

        # Penalties
        p_ang_vel = torch.mean(root_ang_vel_b ** 2, dim=1)
        p_joint_vel = torch.mean(qd ** 2, dim=1)
        p_action_rate = torch.mean(action_rate ** 2, dim=1)
        p_lin_vel_y = root_lin_vel_b[:, 1] ** 2
        p_yaw_rate = root_ang_vel_b[:, 2] ** 2
        p_roll_lean = projected_gravity_b[:, 1] ** 2

        survival_reward = 0 # temporary while debugging the gait issue.

        reward = (
            survival_reward
            + self.cfg.reward_scales["vel_track"] * r_vel_track
            + self.cfg.reward_scales["upright"] * r_upright
            + self.cfg.reward_scales["pose"] * r_pose
            - self.cfg.reward_scales["ang_vel"] * p_ang_vel
            - self.cfg.reward_scales["joint_vel"] * p_joint_vel
            - self.cfg.reward_scales["action_rate"] * p_action_rate
            - self.cfg.reward_scales["lin_vel_y"] * p_lin_vel_y
            - self.cfg.reward_scales["yaw_rate"] * p_yaw_rate
            - self.cfg.reward_scales["roll_lean"] * p_roll_lean
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
                f"roll_lean_pen: {roll_lean_term.mean().item():.4f}"
                f"survival: {survival_reward:.4f} | "
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

        # print("cmd:", self._commands[:8, 0])
        # print("vx :", self.robot.data.root_lin_vel_b[:8, 0])

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

        num_resets = len(env_ids)

        commands = torch.rand((num_resets, 1), device=self.device)
        commands = (
            self.cfg.command_lin_vel_x_min
            + (self.cfg.command_lin_vel_x_max - self.cfg.command_lin_vel_x_min) * commands
        )

        zero_mask = torch.rand((num_resets, 1), device=self.device) < self.cfg.zero_command_prob
        commands[zero_mask] = 0.0

        self._commands[env_ids] = commands

        # print("command sample:", self._commands[env_ids][:5].squeeze(-1))

