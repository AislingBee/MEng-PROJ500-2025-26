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
from isaaclab.utils.math import quat_rotate_inverse

from ...configuration.walking_actuator_config import (
    WALKING_ACTUATOR_SETTINGS,
)
from simulation.isaac.configuration.standing_s2r_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
)
from ..interface.hardware_interface import ControlPacket
from ..interface.isaac_hardware_interface import IsaacHardwareInterface


USD_PATH = Path(__file__).resolve().parents[2] / "assets" / "usd_generated" / "robot" / "robot.usd"


@configclass
class HumanoidStandEnvS2rCfg(DirectRLEnvCfg):
    decimation: int = CONTRACT.decimation
    episode_length_s: float = 10.0
    action_delay_steps: int = 0

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
    observation_space: int = CONTRACT.obs_dim
    state_space: int = 0

    action_scale: tuple[float, ...] = CONTRACT.action_scale
    default_command_value: float = CONTRACT.default_command_value

    usd_path: str = str(USD_PATH)
    base_height: float = 0.00

    upright_k: float = 8.0
    pose_k: float = 4.0
    reward_scales = {
        "upright": 2.0,
        "pose": 1.5,
        "ang_vel": 0.05,
        "joint_vel": 0.02,
        "action_rate": 0.05,
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
    forbidden_body_height_limit: float = -1.0


class HumanoidStandEnvS2r(DirectRLEnv):
    cfg: HumanoidStandEnvS2rCfg

    def __init__(self, cfg: HumanoidStandEnvS2rCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode=render_mode, **kwargs)

        self.robot: Articulation = self.scene.articulations["robot"]
        self.num_dofs = self.robot.num_joints
        self.joint_ids = torch.arange(self.num_dofs, device=self.device, dtype=torch.long)

        if self.num_dofs != self.cfg.action_space:
            raise RuntimeError(
                f"Expected {self.cfg.action_space} joints, got {self.num_dofs}. "
                f"Check USD / standing pose / actuator config alignment."
            )
        if tuple(self.robot.joint_names) != CONTRACT.joint_names:
            raise RuntimeError(
                "Robot joint_names do not match the standing S2R policy contract. "
                "Training and deployment joint order must stay identical."
            )

        self._actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._last_actions = torch.zeros((self.num_envs, self.num_dofs), device=self.device)
        self._action_delay_steps = self.cfg.action_delay_steps
        self._action_buffer = torch.zeros(
            (self.num_envs, self._action_delay_steps + 1, self.num_dofs),
            device=self.device,
        )
        self._commands = torch.full(
            (self.num_envs, 1),
            fill_value=self.cfg.default_command_value,
            device=self.device,
        )
        self._joint_pos_targets = torch.zeros((self.num_envs, self.num_dofs), device=self.device)

        self._action_scale = torch.tensor(
            self.cfg.action_scale, dtype=torch.float32, device=self.device
        ).unsqueeze(0)

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
            raise RuntimeError("Robot soft joint lower limits do not match the standing S2R policy contract")
        if not torch.allclose(soft_joint_upper[0], self._joint_upper[0], atol=1e-5, rtol=0.0):
            raise RuntimeError("Robot soft joint upper limits do not match the standing S2R policy contract")

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

        # From URDF fixed joint rpy="0 -1.5707963267948963 0"
        # Quaternion order is [w, x, y, z]
        self._root_to_imu_quat = torch.tensor(
            # [0.7071068, 0.0, -0.7071068, 0.0],
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

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if torch.isnan(actions).any():
            raise RuntimeError("NaN detected in actions")

        self._last_actions[:] = self._actions
        actions = torch.clamp(actions, -1.0, 1.0)

        self._action_buffer = torch.roll(self._action_buffer, shifts=1, dims=1)
        self._action_buffer[:, 0, :] = actions

        delayed_actions = self._action_buffer[:, self._action_delay_steps, :]
        self._actions[:] = delayed_actions

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

    def _get_observations(self) -> dict:
        packet = self._hardware.read_observation_packet()

        q_rel = packet.joint_pos - self._standing_q.unsqueeze(0)

        obs = torch.cat(
            (
                q_rel,
                packet.joint_vel,
                packet.joint_effort,
                packet.projected_gravity_b,
                packet.imu_gyro_b,
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

    def _get_rewards(self) -> torch.Tensor:
        q = self.robot.data.joint_pos
        qd = self.robot.data.joint_vel

        packet = self._hardware.read_observation_packet()
        projected_gravity_b = packet.projected_gravity_b
        imu_gyro_b = packet.imu_gyro_b

        q_err = q - self._standing_q.unsqueeze(0)
        action_rate = self._actions - self._last_actions

        tilt_metric = torch.sum(projected_gravity_b[:, :2] ** 2, dim=1)

        r_upright = torch.exp(-self.cfg.upright_k * tilt_metric)
        r_pose = torch.exp(-self.cfg.pose_k * torch.mean(q_err ** 2, dim=1))

        p_ang_vel = torch.mean(imu_gyro_b ** 2, dim=1)
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
        # terminated = bad_state | body_hit_ground
        time_out = self.episode_length_buf >= self.max_episode_length - 1

        # if torch.any(over_tilted | bad_state | body_hit_ground):
        #     idx = torch.where(over_tilted | bad_state | body_hit_ground)[0][0]
        #
        #     print("\n=== RESET DEBUG ===")
        #     print(f"env_id: {idx.item()}")
        #     print(f"over_tilted      : {over_tilted[idx].item()}")
        #     print(f"bad_state        : {bad_state[idx].item()}")
        #     print(f"body_hit_ground  : {body_hit_ground[idx].item()}")
        #     print(f"tilt_metric      : {tilt_metric[idx].item()}")
        #
        #     print(f"root_z           : {self.robot.data.root_pos_w[idx, 2].item()}")
        #
        #     body_heights = self.robot.data.body_pos_w[idx, self._forbidden_body_ids, 2]
        #     print(f"min_body_z       : {body_heights.min().item()}")
        #
        #     print(f"projected_gravity_b: {projected_gravity_b[idx].detach().cpu().numpy()}")
        #     print(f"root_quat_w        : {self.robot.data.root_quat_w[idx].detach().cpu().numpy()}")
        #
        #     print("===================\n")

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

        joint_vel = 0.05 * torch.randn(
            (len(env_ids), self.num_dofs),
            device=self.device,
        )

        joint_pos = torch.max(
            torch.min(joint_pos, self._joint_upper[env_ids]),
            self._joint_lower[env_ids],
        )

        self.robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
        self.robot.set_joint_position_target(joint_pos, env_ids=env_ids)

        self._actions[env_ids] = 0.0
        self._last_actions[env_ids] = 0.0
        self._action_buffer[env_ids] = 0.0
        self._joint_pos_targets[env_ids] = joint_pos
        self._q_des[env_ids] = joint_pos
        self._tau_ff[env_ids] = 0.0

        # print("Reset!-----------------------------------------")
