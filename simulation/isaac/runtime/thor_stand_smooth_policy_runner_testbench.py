from __future__ import annotations

import argparse
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import torch

from simulation.isaac.configuration.humanoid_stand_smooth_ppo_cfg import (
    SMOOTH_STAND_DEPLOYMENT_CFG,
)
from simulation.isaac.configuration.standing_s2r_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
)
from simulation.isaac.rl.interface.hardware_interface import ControlPacket
from simulation.isaac.rl.interface.robot_hardware_interface import (
    RobotCommandMessage,
    RobotHardwareInterface,
    RobotInterfaceConfig,
    RobotStateSample,
)


SMOOTH_STAND_OBS_LAYOUT: dict[str, int] = {
    "q_rel": 12,
    "qd": 12,
    "q_target_err": 12,
    "joint_effort": 12,
    "projected_gravity_b": 3,
    "imu_gyro_b": 3,
    "last_actions": 12,
}
SMOOTH_STAND_OBS_DIM = sum(SMOOTH_STAND_OBS_LAYOUT.values())


@dataclass
class SmoothThorStandingRunnerConfig:
    policy_path: str
    joint_names: tuple[str, ...] = CONTRACT.joint_names
    joint_lower_rad: tuple[float, ...] = CONTRACT.joint_lower_limits_rad
    joint_upper_rad: tuple[float, ...] = CONTRACT.joint_upper_limits_rad
    action_scale: tuple[float, ...] = tuple(0.65 * x for x in CONTRACT.action_scale)
    use_obs_normalization: bool = SMOOTH_STAND_DEPLOYMENT_CFG.use_obs_normalization
    obs_normalizer_required: bool = SMOOTH_STAND_DEPLOYMENT_CFG.obs_normalizer_required
    obs_normalizer_artifact_name: str = SMOOTH_STAND_DEPLOYMENT_CFG.obs_normalizer_artifact_name
    obs_normalizer_path: str | None = None
    device: str = "cpu"
    loop_hz: float = CONTRACT.policy_loop_hz
    action_delay_steps: int = 2
    debug_print_every_n_steps: int = 1

    def __post_init__(self) -> None:
        if self.joint_names != CONTRACT.joint_names:
            raise ValueError("joint_names must match CONTRACT.joint_names")
        if len(self.action_scale) != CONTRACT.action_dim:
            raise ValueError("action_scale length must match CONTRACT.action_dim")
        if self.action_delay_steps < 0:
            raise ValueError("action_delay_steps must be non-negative")
        if self.debug_print_every_n_steps <= 0:
            raise ValueError("debug_print_every_n_steps must be positive")


class DeployableObsNormalizer:
    def __init__(self, normalizer_path: str | Path, device: str | torch.device = "cpu"):
        self.device = torch.device(device)
        self.path = Path(normalizer_path).expanduser().resolve()
        self.normalizer = torch.jit.load(str(self.path), map_location=self.device)
        self.normalizer.eval()

    @torch.inference_mode()
    def normalize(self, obs: torch.Tensor) -> torch.Tensor:
        normalized_obs = self.normalizer(obs)
        normalized_obs = torch.as_tensor(normalized_obs, dtype=torch.float32, device=self.device)
        if normalized_obs.shape != obs.shape:
            raise RuntimeError(
                f"Observation normalizer returned shape {tuple(normalized_obs.shape)}, expected {tuple(obs.shape)}"
            )
        if torch.isnan(normalized_obs).any():
            raise RuntimeError("NaN detected in normalized observations")
        return normalized_obs


class DeployablePolicy:
    def __init__(
        self,
        policy_path: str | Path,
        device: str | torch.device = "cpu",
        obs_normalizer: DeployableObsNormalizer | None = None,
    ):
        self.device = torch.device(device)
        self.path = Path(policy_path).expanduser().resolve()
        self.policy = torch.jit.load(str(self.path), map_location=self.device)
        self.policy.eval()
        self.obs_normalizer = obs_normalizer

    @torch.inference_mode()
    def act(self, obs: torch.Tensor) -> torch.Tensor:
        if self.obs_normalizer is not None:
            obs = self.obs_normalizer.normalize(obs)
        out = self.policy({"policy": obs})
        if isinstance(out, dict):
            out = out["actions"]
        elif isinstance(out, (tuple, list)):
            out = out[0]
        out = torch.as_tensor(out, dtype=torch.float32, device=self.device)
        if out.ndim == 1:
            out = out.unsqueeze(0)
        if out.shape != (1, CONTRACT.action_dim):
            raise RuntimeError(f"Expected action shape {(1, CONTRACT.action_dim)}, got {tuple(out.shape)}")
        if torch.isnan(out).any():
            raise RuntimeError("NaN detected in policy output")
        return out


class SmoothThorStandingPolicyRunner:
    def __init__(
        self,
        runner_cfg: SmoothThorStandingRunnerConfig,
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
        self.obs_normalizer = self._load_obs_normalizer()
        self.policy = DeployablePolicy(
            runner_cfg.policy_path,
            device=self.device,
            obs_normalizer=self.obs_normalizer,
        )

        self._standing_q = build_standing_q(device=self.device)
        self._action_scale = torch.tensor(runner_cfg.action_scale, dtype=torch.float32, device=self.device).unsqueeze(0)
        self._joint_lower = torch.tensor(runner_cfg.joint_lower_rad, dtype=torch.float32, device=self.device).unsqueeze(0)
        self._joint_upper = torch.tensor(runner_cfg.joint_upper_rad, dtype=torch.float32, device=self.device).unsqueeze(0)
        kp_fixed, kd_fixed = build_fixed_gains(device=self.device)
        self._kp_fixed = kp_fixed.unsqueeze(0)
        self._kd_fixed = kd_fixed.unsqueeze(0)
        self._tau_ff = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)

        self._actions = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)
        self._last_actions = torch.zeros_like(self._actions)
        self._action_buffer = torch.zeros(
            (1, runner_cfg.action_delay_steps + 1, CONTRACT.action_dim),
            dtype=torch.float32,
            device=self.device,
        )
        self._joint_pos_targets = self._standing_q.unsqueeze(0).clone()
        self._step_count = 0
        self._last_raw_actions = torch.zeros_like(self._actions)
        self._last_clamped_actions = torch.zeros_like(self._actions)
        self._last_obs = torch.zeros((1, SMOOTH_STAND_OBS_DIM), dtype=torch.float32, device=self.device)
        self._last_saturation_pct = 0.0
        self._print_startup_summary()

    def _resolve_obs_normalizer_path(self) -> Path:
        if self.cfg.obs_normalizer_path is not None:
            return Path(self.cfg.obs_normalizer_path).expanduser().resolve()
        return Path(self.cfg.policy_path).expanduser().resolve().with_name(self.cfg.obs_normalizer_artifact_name)

    def _load_obs_normalizer(self) -> DeployableObsNormalizer | None:
        if not self.cfg.use_obs_normalization:
            return None

        normalizer_path = self._resolve_obs_normalizer_path()
        if not normalizer_path.is_file():
            if self.cfg.obs_normalizer_required:
                raise RuntimeError(
                    "Smooth standing policy requires observation normalization, "
                    f"but normalizer artifact was not found at: {normalizer_path}"
                )
            return None
        return DeployableObsNormalizer(normalizer_path, device=self.device)

    def _print_startup_summary(self) -> None:
        normalizer_path = "None"
        if self.obs_normalizer is not None:
            normalizer_path = str(self.obs_normalizer.path)
        print(
            "[SMOOTH STAND RUNNER] "
            f"policy_path={self.policy.path} | "
            f"use_obs_normalization={self.cfg.use_obs_normalization} | "
            f"normalizer_path={normalizer_path} | "
            f"observation_dim={SMOOTH_STAND_OBS_DIM} | "
            f"action_dim={CONTRACT.action_dim}"
        )

    def build_observation(self) -> torch.Tensor:
        packet = self.hardware.read_observation_packet()
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
        obs = torch.cat(tuple(value for _, value in fields), dim=-1).to(self.device, dtype=torch.float32)
        if obs.shape != (1, SMOOTH_STAND_OBS_DIM):
            raise RuntimeError(f"Expected obs shape {(1, SMOOTH_STAND_OBS_DIM)}, got {tuple(obs.shape)}")
        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observation")
        self._last_obs = obs.detach().clone()
        return obs

    def generate_control_packet(self, raw_actions: torch.Tensor) -> ControlPacket:
        self._last_actions[:] = self._actions
        self._last_raw_actions = raw_actions.detach().clone()
        clamped_actions = torch.clamp(raw_actions, -1.0, 1.0)
        self._last_clamped_actions = clamped_actions.detach().clone()
        self._last_saturation_pct = 100.0 * (torch.abs(raw_actions) > 1.0).float().mean().item()

        self._action_buffer = torch.roll(self._action_buffer, shifts=1, dims=1)
        self._action_buffer[:, 0, :] = clamped_actions
        delayed_actions = self._action_buffer[:, self.cfg.action_delay_steps, :]
        self._actions[:] = delayed_actions

        q_des = self._standing_q.unsqueeze(0) + self._action_scale * delayed_actions
        q_des = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)
        self._joint_pos_targets = q_des.detach().clone()

        return ControlPacket(
            joint_names=list(self.cfg.joint_names),
            q_des=q_des.clone(),
            kp=self._kp_fixed.clone(),
            kd=self._kd_fixed.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp_fixed.clone(),
            kd_gains=self._kd_fixed.clone(),
        )

    def step(self) -> ControlPacket:
        obs = self.build_observation()
        raw_actions = self.policy.act(obs)
        packet = self.generate_control_packet(raw_actions)
        self.hardware.write_control_packet(packet)
        self._step_count += 1
        self._debug_print(packet)
        return packet

    def _debug_print(self, packet: ControlPacket) -> None:
        if self._step_count % self.cfg.debug_print_every_n_steps != 0:
            return
        raw = self._last_raw_actions.detach().cpu()
        clamped = self._last_clamped_actions.detach().cpu()
        q_des = packet.q_des.detach().cpu()
        print(
            f"[SMOOTH STAND DEBUG] step={self._step_count} | "
            f"raw_min={raw.min().item():+.4f} raw_max={raw.max().item():+.4f} | "
            f"raw_abs_mean={raw.abs().mean().item():.4f} | "
            f"action_saturation_pct={self._last_saturation_pct:.2f}% | "
            f"clamped_min={clamped.min().item():+.4f} clamped_max={clamped.max().item():+.4f} | "
            f"q_des_min={q_des.min().item():+.4f} q_des_max={q_des.max().item():+.4f}"
        )


def rad_to_encoder_counts(q_rad: list[float]) -> list[int]:
    counts = []
    for q in q_rad:
        q_0_2pi = q % (2.0 * math.pi)
        count = int(round((q_0_2pi / (2.0 * math.pi)) * 16384.0))
        counts.append(max(0, min(16383, count)))
    return counts


def fake_state_reader() -> RobotStateSample:
    q = build_standing_q(device="cpu")
    return RobotStateSample(
        encoder_counts=rad_to_encoder_counts(q.tolist()),
        projected_gravity_b=[0.0, 0.0, -1.0],
        imu_gyro_b=[0.0, 0.0, 0.0],
        joint_vel=[0.0] * CONTRACT.action_dim,
        joint_effort=[0.0] * CONTRACT.action_dim,
        timestamp_s=time.monotonic(),
    )


def fake_command_writer(msg: RobotCommandMessage) -> None:
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Run smooth standing Thor deployment path diagnostics.")
    parser.add_argument("--policy", required=True, help="Path to exported smooth standing policy_jit.pt")
    parser.add_argument(
        "--obs-normalizer",
        default=None,
        help="Optional path to obs_normalizer.pt. Defaults to a sibling of --policy.",
    )
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--delay", type=int, default=2)
    args = parser.parse_args()

    runner_cfg = SmoothThorStandingRunnerConfig(
        policy_path=args.policy,
        obs_normalizer_path=args.obs_normalizer,
        action_delay_steps=args.delay,
        debug_print_every_n_steps=1,
    )
    hardware_cfg = RobotInterfaceConfig(
        joint_names=CONTRACT.joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in CONTRACT.joint_names),
        joint_signs=tuple(1.0 for _ in CONTRACT.joint_names),
    )
    runner = SmoothThorStandingPolicyRunner(
        runner_cfg=runner_cfg,
        hardware_cfg=hardware_cfg,
        state_reader=fake_state_reader,
        command_writer=fake_command_writer,
    )
    for _ in range(args.steps):
        runner.step()
        time.sleep(1.0 / runner_cfg.loop_hz)


if __name__ == "__main__":
    main()
