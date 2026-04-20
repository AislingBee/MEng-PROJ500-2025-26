# Notion Documenation for this script.
# https://josephandrews.notion.site/Thor-Policy-Runner-Sim-to-Real-Deployment-Standing-Task-3456b3c9bc7680e99d5af51064cacd0f?source=copy_link

from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import torch

from simulation.isaac.configuration.standing_pose import STANDING_TARGETS_DEG
from simulation.isaac.configuration.walking_actuator_config import (
    build_per_joint_walking_actuator_cfg,
)
from simulation.isaac.rl.interface.hardware_interface import ControlPacket
from simulation.isaac.rl.interface.robot_hardware_interface import (
    RobotCommandMessage,
    RobotHardwareInterface,
    RobotInterfaceConfig,
    RobotStateSample,
)


Tensor = torch.Tensor


@dataclass
class ThorPolicyRunnerConfig:
    policy_path: str
    joint_names: tuple[str, ...]
    joint_lower_rad: tuple[float, ...]
    joint_upper_rad: tuple[float, ...]
    action_scale: tuple[float, ...] = (
        0.10, 0.08, 0.15, 0.20, 0.12, 0.08,
        0.10, 0.08, 0.15, 0.20, 0.12, 0.08,
    )
    command_value: float = 0.0
    device: str = "cpu"
    loop_hz: float = 60.0
    send_standing_pose_on_exit: bool = True

    def __post_init__(self) -> None:
        n = len(self.joint_names)
        if n != 12:
            raise ValueError(f"Expected 12 joints, got {n}")
        if len(self.joint_lower_rad) != n:
            raise ValueError("joint_lower_rad length must match joint_names")
        if len(self.joint_upper_rad) != n:
            raise ValueError("joint_upper_rad length must match joint_names")
        if len(self.action_scale) != n:
            raise ValueError("action_scale length must match joint_names")
        if self.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")


class DeployablePolicy:
    """Loads a deployable actor module for inference.

    Supported forms:
      1) TorchScript module via torch.jit.load(...)
      2) torch.load(...) returning a callable torch.nn.Module

    This runner intentionally does not rebuild a training-time network from an
    RSL-RL checkpoint dict. Thor deployment should consume an exported actor,
    not a full training checkpoint.
    """

    def __init__(self, policy_path: str | Path, device: str | torch.device = "cpu"):
        self.device = torch.device(device)
        self.policy = self._load(policy_path)
        self.policy.eval()

    def _load(self, policy_path: str | Path):
        policy_path = str(policy_path)

        try:
            model = torch.jit.load(policy_path, map_location=self.device)
            return model
        except Exception:
            pass

        obj = torch.load(policy_path, map_location=self.device)
        if isinstance(obj, torch.nn.Module):
            obj.to(self.device)
            return obj

        if isinstance(obj, dict):
            raise RuntimeError(
                "Policy file loaded as a checkpoint dict, not a deployable actor module. "
                "Export the trained actor to TorchScript first, then use that .pt here."
            )

        raise RuntimeError(
            f"Unsupported policy file contents in '{policy_path}'. "
            "Expected TorchScript or a serialized torch.nn.Module."
        )

    @torch.inference_mode()
    def act(self, obs: Tensor) -> Tensor:
        out = self.policy(obs)

        if isinstance(out, dict):
            if "actions" in out:
                out = out["actions"]
            else:
                raise RuntimeError("Policy dict output does not contain 'actions'")
        elif isinstance(out, (tuple, list)):
            if not out:
                raise RuntimeError("Policy returned an empty tuple/list")
            out = out[0]

        out = torch.as_tensor(out, dtype=torch.float32, device=self.device)
        if out.ndim == 1:
            out = out.unsqueeze(0)
        if out.shape != (1, 12):
            raise RuntimeError(f"Expected policy output shape (1, 12), got {tuple(out.shape)}")
        if torch.isnan(out).any():
            raise RuntimeError("NaN detected in policy output")
        return out


class ThorStandingPolicyRunner:
    def __init__(
        self,
        runner_cfg: ThorPolicyRunnerConfig,
        hardware_cfg: RobotInterfaceConfig,
        state_reader: Callable[[], RobotStateSample],
        command_writer: Callable[[RobotCommandMessage], None],
    ):
        self.cfg = runner_cfg
        self.device = torch.device(runner_cfg.device)

        self.hardware = RobotHardwareInterface(
            cfg=hardware_cfg,
            state_reader=state_reader,
            command_writer=command_writer,
            device=self.device,
        )
        self.policy = DeployablePolicy(runner_cfg.policy_path, device=self.device)

        self._joint_names = list(runner_cfg.joint_names)
        self._standing_q = self._build_standing_pose_tensor(self._joint_names)
        self._action_scale = torch.tensor(
            runner_cfg.action_scale, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_lower = torch.tensor(
            runner_cfg.joint_lower_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_upper = torch.tensor(
            runner_cfg.joint_upper_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._commands = torch.tensor(
            [[runner_cfg.command_value]], dtype=torch.float32, device=self.device
        )
        self._last_actions = torch.zeros((1, 12), dtype=torch.float32, device=self.device)
        self._tau_ff = torch.zeros((1, 12), dtype=torch.float32, device=self.device)

        per_joint_cfg = build_per_joint_walking_actuator_cfg(self._joint_names)
        self._kp_fixed = torch.tensor(
            per_joint_cfg["stiffness"], dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._kd_fixed = torch.tensor(
            per_joint_cfg["damping"], dtype=torch.float32, device=self.device
        ).unsqueeze(0)

    @staticmethod
    def _build_standing_pose_tensor(joint_names: Sequence[str]) -> Tensor:
        q = torch.zeros((12,), dtype=torch.float32)
        for i, joint_name in enumerate(joint_names):
            q[i] = math.radians(STANDING_TARGETS_DEG.get(joint_name, 0.0))
        return q

    def build_observation(self) -> Tensor:
        packet = self.hardware.read_observation_packet()

        q_rel = packet.joint_pos - self._standing_q.unsqueeze(0).to(self.device)
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

        obs = obs.to(self.device, dtype=torch.float32)
        if obs.shape != (1, 55):
            raise RuntimeError(f"Expected observation shape (1, 55), got {tuple(obs.shape)}")
        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observations")
        return obs

    def generate_control_packet(self, actions: Tensor) -> ControlPacket:
        actions = torch.clamp(actions, -1.0, 1.0)
        q_des = self._standing_q.unsqueeze(0).to(self.device) + self._action_scale * actions
        q_des = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)

        return ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des.clone(),
            kp=self._kp_fixed.clone(),
            kd=self._kd_fixed.clone(),
            tau_ff=self._tau_ff.clone(),
        )

    def step(self) -> ControlPacket:
        obs = self.build_observation()
        actions = self.policy.act(obs)
        packet = self.generate_control_packet(actions)
        self.hardware.write_control_packet(packet)
        self._last_actions = torch.clamp(actions, -1.0, 1.0).detach().clone()
        return packet

    def send_standing_pose(self) -> None:
        packet = ControlPacket(
            joint_names=self._joint_names,
            q_des=self._standing_q.unsqueeze(0).to(self.device).clone(),
            kp=self._kp_fixed.clone(),
            kd=self._kd_fixed.clone(),
            tau_ff=self._tau_ff.clone(),
        )
        self.hardware.write_control_packet(packet)

    def run(self) -> None:
        period_s = 1.0 / self.cfg.loop_hz
        next_t = time.monotonic()

        try:
            while True:
                self.step()
                next_t += period_s
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_t = time.monotonic()
        except KeyboardInterrupt:
            if self.cfg.send_standing_pose_on_exit:
                self.send_standing_pose()


# -----------------------------------------------------------------------------
# Wiring example
# Replace these with the actual Thor ROS/CAN hooks.
# -----------------------------------------------------------------------------

def example_state_reader() -> RobotStateSample:
    raise NotImplementedError("Inject your real Thor state reader here")



def example_command_writer(msg: RobotCommandMessage) -> None:
    raise NotImplementedError("Inject your real Thor command writer here")



def main() -> None:
    joint_names = (
        "robot_l_hip_yaw_joint",
        "robot_l_hip_roll_joint",
        "robot_l_hip_pitch_joint",
        "robot_l_knee_joint",
        "robot_l_ankle_pitch_joint",
        "robot_l_ankle_roll_joint",
        "robot_r_hip_yaw_joint",
        "robot_r_hip_roll_joint",
        "robot_r_hip_pitch_joint",
        "robot_r_knee_joint",
        "robot_r_ankle_pitch_joint",
        "robot_r_ankle_roll_joint",
    )

    # Fill these from the same trained robot definition used by Isaac.
    # They must match the soft joint position limits used during training.
    joint_lower_rad = (
        -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
        -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
    )
    joint_upper_rad = (
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    )

    runner_cfg = ThorPolicyRunnerConfig(
        policy_path="exports/standing_policy.pt",
        joint_names=joint_names,
        joint_lower_rad=joint_lower_rad,
        joint_upper_rad=joint_upper_rad,
        device="cpu",
        loop_hz=60.0,
        command_value=0.0,
    )

    hardware_cfg = RobotInterfaceConfig(
        joint_names=joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in joint_names),
        joint_signs=tuple(1.0 for _ in joint_names),
    )

    runner = ThorStandingPolicyRunner(
        runner_cfg=runner_cfg,
        hardware_cfg=hardware_cfg,
        state_reader=example_state_reader,
        command_writer=example_command_writer,
    )
    runner.run()


if __name__ == "__main__":
    main()
