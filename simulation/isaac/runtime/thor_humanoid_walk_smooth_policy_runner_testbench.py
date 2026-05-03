from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

import torch

from simulation.isaac.configuration.humanoid_walk_smooth_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
)
from simulation.isaac.rl.interface.hardware_interface import ControlPacket, ObservationPacket
from simulation.isaac.rl.interface.robot_hardware_interface import (
    RobotCommandMessage,
    RobotHardwareInterface,
    RobotInterfaceConfig,
    RobotStateSample,
)


Tensor = torch.Tensor
REPO_ROOT = Path(__file__).resolve().parents[3]
SMOOTH_WALK_LOG_ROOT = REPO_ROOT / "logs" / "rsl_rl" / "humanoid_walk_smooth"
DEFAULT_SMOOTH_WALK_POLICY_PATH = r"hardware\policy\walking_smooth_policy.pt"


def rad_to_encoder_counts(q_rad: list[float]) -> list[int]:
    counts = []
    for q in q_rad:
        q_0_2pi = q % (2.0 * math.pi)
        count = int(round((q_0_2pi / (2.0 * math.pi)) * 16384.0))
        counts.append(max(0, min(16383, count)))
    return counts


TEST_CASE = {
    "name": "upright_walk_ready",
    "gravity_b": [0.0, 0.0, -1.0],
    "gyro_b": [0.0, 0.0, 0.0],
    "joint_offset_rad": [0.0] * CONTRACT.action_dim,
    "joint_vel": [0.0] * CONTRACT.action_dim,
    "joint_effort": [0.0] * CONTRACT.action_dim,
}


def fake_state_reader() -> RobotStateSample:
    standing_q = build_standing_q(device="cpu")
    joint_offset = torch.tensor(TEST_CASE["joint_offset_rad"], dtype=torch.float32)
    q = standing_q + joint_offset

    return RobotStateSample(
        encoder_counts=rad_to_encoder_counts(q.tolist()),
        projected_gravity_b=TEST_CASE["gravity_b"],
        imu_gyro_b=TEST_CASE["gyro_b"],
        joint_vel=TEST_CASE["joint_vel"],
        joint_effort=TEST_CASE["joint_effort"],
        timestamp_s=time.monotonic(),
    )


def fake_command_writer(msg: RobotCommandMessage) -> None:
    return None


def _latest_exported_policy() -> Path:
    if not SMOOTH_WALK_LOG_ROOT.exists():
        raise FileNotFoundError(
            f"No smooth walking log directory found at {SMOOTH_WALK_LOG_ROOT.resolve()}. "
            "Train a smooth walking policy first or pass --policy explicitly."
        )

    run_dirs = sorted(
        (path for path in SMOOTH_WALK_LOG_ROOT.iterdir() if path.is_dir()),
        key=lambda path: (path.stat().st_mtime, path.name),
    )
    for run_dir in reversed(run_dirs):
        candidate = run_dir / "exported" / "policy_jit.pt"
        if candidate.is_file():
            return candidate.resolve()

    raise FileNotFoundError(
        f"No exported smooth walking policy_jit.pt found under {SMOOTH_WALK_LOG_ROOT.resolve()}. "
        "Train/export a smooth walking policy first or pass --policy explicitly."
    )


@dataclass
class ThorHumanoidWalkSmoothRunnerConfig:
    policy_path: str = DEFAULT_SMOOTH_WALK_POLICY_PATH
    obs_normalizer_path: str | None = None
    device: str = "cpu"
    loop_hz: float = CONTRACT.policy_loop_hz
    action_delay_steps: int = 2
    command_value: float = CONTRACT.default_command_value
    max_command_value: float = 0.50
    debug_print_every_n_steps: int = 1

    def __post_init__(self) -> None:
        if self.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")
        if self.max_command_value < 0.0:
            raise ValueError("max_command_value must be non-negative")
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
    def normalize(self, obs: Tensor) -> Tensor:
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
    def act(self, obs: Tensor) -> Tensor:
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
        expected_shape = (1, CONTRACT.action_dim)
        if out.shape != expected_shape:
            raise RuntimeError(f"Expected action shape {expected_shape}, got {tuple(out.shape)}")
        if torch.isnan(out).any():
            raise RuntimeError("NaN detected in policy output")
        return out


class ThorHumanoidWalkSmoothPolicyRunnerTestHarness:
    def __init__(
        self,
        runner_cfg: ThorHumanoidWalkSmoothRunnerConfig,
        hardware_cfg: RobotInterfaceConfig,
    ) -> None:
        self.cfg = runner_cfg
        self.device = torch.device(runner_cfg.device)
        self.hardware = RobotHardwareInterface(
            cfg=hardware_cfg,
            state_reader=fake_state_reader,
            command_writer=fake_command_writer,
            device=self.device,
        )
        self.obs_normalizer = self._load_obs_normalizer()
        self.policy = DeployablePolicy(runner_cfg.policy_path, device=self.device, obs_normalizer=self.obs_normalizer)

        self._joint_names = list(CONTRACT.joint_names)
        self._standing_q = build_standing_q(device=self.device)
        self._action_scale = torch.tensor(CONTRACT.action_scale, dtype=torch.float32, device=self.device).unsqueeze(0)
        self._joint_lower = torch.tensor(
            CONTRACT.joint_lower_limits_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_upper = torch.tensor(
            CONTRACT.joint_upper_limits_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._commands = torch.zeros((1, 1), dtype=torch.float32, device=self.device)
        self._actions = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)
        self._last_actions = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)
        self._action_buffer = torch.zeros(
            (1, runner_cfg.action_delay_steps + 1, CONTRACT.action_dim),
            dtype=torch.float32,
            device=self.device,
        )
        self._joint_pos_targets = self._standing_q.unsqueeze(0).clone()
        self._tau_ff = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)

        kp_fixed, kd_fixed = build_fixed_gains(device=self.device)
        self._kp_fixed = kp_fixed.unsqueeze(0)
        self._kd_fixed = kd_fixed.unsqueeze(0)
        self._kp_gains = self._kp_fixed.clone()
        self._kd_gains = self._kd_fixed.clone()

        self._step_count = 0
        self._last_action_saturation_pct = 0.0
        self.set_command_value(runner_cfg.command_value)
        self._print_startup_summary()

    def _resolve_obs_normalizer_path(self) -> Path:
        if self.cfg.obs_normalizer_path is not None:
            return Path(self.cfg.obs_normalizer_path).expanduser().resolve()
        return Path(self.cfg.policy_path).expanduser().resolve().with_name(CONTRACT.obs_normalizer_artifact_name)

    def _load_obs_normalizer(self) -> DeployableObsNormalizer | None:
        if not CONTRACT.use_obs_normalization:
            return None

        normalizer_path = self._resolve_obs_normalizer_path()
        if not normalizer_path.is_file():
            if CONTRACT.obs_normalizer_required:
                raise RuntimeError(
                    "Smooth walking policy requires observation normalization, "
                    f"but normalizer artifact was not found at: {normalizer_path}"
                )
            return None
        return DeployableObsNormalizer(normalizer_path, device=self.device)

    def _print_startup_summary(self) -> None:
        normalizer_path = "None"
        if self.obs_normalizer is not None:
            normalizer_path = str(self.obs_normalizer.path)
        print(
            "[SMOOTH WALK RUNNER] "
            f"policy_path={self.policy.path} | "
            f"use_obs_normalization={CONTRACT.use_obs_normalization} | "
            f"normalizer_path={normalizer_path} | "
            f"observation_dim={CONTRACT.obs_dim} | "
            f"action_dim={CONTRACT.action_dim}"
        )

    def set_command_value(self, command_value: float) -> None:
        clamped_value = min(max(float(command_value), 0.0), self.cfg.max_command_value)
        self._commands[0, 0] = clamped_value

    def _get_phase_clock(self) -> tuple[Tensor, Tensor]:
        phase = (self._step_count / self.cfg.loop_hz * CONTRACT.default_gait_frequency_hz) % 1.0
        phase_angle = 2.0 * math.pi * phase
        phase_sin = torch.tensor([[math.sin(phase_angle)]], dtype=torch.float32, device=self.device)
        phase_cos = torch.tensor([[math.cos(phase_angle)]], dtype=torch.float32, device=self.device)
        return phase_sin, phase_cos

    def build_observation(self, observation_packet: ObservationPacket) -> Tensor:
        q_rel = observation_packet.joint_pos - self._standing_q.unsqueeze(0)
        q_target_err = self._joint_pos_targets - observation_packet.joint_pos
        phase_sin, phase_cos = self._get_phase_clock()

        obs = torch.cat(
            (
                q_rel,
                observation_packet.joint_vel,
                q_target_err,
                observation_packet.joint_effort,
                observation_packet.projected_gravity_b,
                observation_packet.imu_gyro_b,
                self._commands,
                phase_sin,
                phase_cos,
                observation_packet.foot_pos_b,
                self._last_actions,
            ),
            dim=-1,
        ).to(self.device, dtype=torch.float32)

        expected_shape = (1, CONTRACT.obs_dim)
        if obs.shape != expected_shape:
            raise RuntimeError(f"Expected smooth walking observation shape {expected_shape}, got {tuple(obs.shape)}")
        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in smooth walking observation")
        return obs

    def _process_policy_actions(self, raw_actions: Tensor) -> tuple[Tensor, Tensor]:
        self._last_actions[:] = self._actions
        clamped_actions = torch.clamp(raw_actions, -1.0, 1.0)
        saturated = raw_actions != clamped_actions
        self._last_action_saturation_pct = 100.0 * saturated.float().mean().item()

        self._action_buffer = torch.roll(self._action_buffer, shifts=1, dims=1)
        self._action_buffer[:, 0, :] = clamped_actions
        delayed_actions = self._action_buffer[:, self.cfg.action_delay_steps, :]
        self._actions[:] = delayed_actions
        return clamped_actions, delayed_actions

    def generate_control_packet(self, actions: Tensor) -> tuple[ControlPacket, Tensor, Tensor]:
        clamped_actions, applied_actions = self._process_policy_actions(actions)
        q_des = self._standing_q.unsqueeze(0) + self._action_scale * applied_actions
        q_des = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)
        self._joint_pos_targets = q_des.detach().clone()

        packet = ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des.clone(),
            kp=self._kp_fixed.clone(),
            kd=self._kd_fixed.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp_gains.clone(),
            kd_gains=self._kd_gains.clone(),
        )
        return packet, clamped_actions, applied_actions

    def step(self) -> ControlPacket:
        observation_packet = self.hardware.read_observation_packet()
        obs = self.build_observation(observation_packet)
        actions = self.policy.act(obs)
        packet, clamped_actions, applied_actions = self.generate_control_packet(actions)
        self.hardware.write_control_packet(packet)
        self._step_count += 1
        self._debug_print_step(obs, actions, clamped_actions, applied_actions, packet)
        return packet

    def _debug_print_step(
        self,
        obs: Tensor,
        raw_actions: Tensor,
        clamped_actions: Tensor,
        applied_actions: Tensor,
        packet: ControlPacket,
    ) -> None:
        if self._step_count % self.cfg.debug_print_every_n_steps != 0:
            return

        phase_sin, phase_cos = self._get_phase_clock()
        print("\n" + "=" * 90)
        print(f"[SMOOTH WALKING DEBUG] step={self._step_count}")
        print("-" * 90)
        print("obs shape:", tuple(obs.shape))
        print("command:", self._commands.detach().cpu())
        print("phase_sin:", phase_sin.detach().cpu())
        print("phase_cos:", phase_cos.detach().cpu())
        print("raw_actions:", raw_actions.detach().cpu())
        print("clamped_actions:", clamped_actions.detach().cpu())
        print("applied_actions_after_delay:", applied_actions.detach().cpu())
        print(f"action_saturation_pct: {self._last_action_saturation_pct:.2f}%")
        print("joint order:")
        q_des = packet.q_des.detach().cpu()
        for i, name in enumerate(packet.joint_names):
            print(f"{i:02d} {name:40s} q_des={q_des[0, i]:+.5f} rad")
        print("=" * 90)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run smooth walking Thor deployment path diagnostics.")
    parser.add_argument(
        "--policy",
        default=None,
        help="Path to exported smooth walking policy_jit.pt. Defaults to runner_cfg.policy_path, then falls back to latest export.",
    )
    parser.add_argument(
        "--obs-normalizer",
        default=None,
        help="Optional path to obs_normalizer.pt. Defaults to a sibling of --policy.",
    )
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--delay", type=int, default=2)
    parser.add_argument("--command", type=float, default=CONTRACT.default_command_value)
    args = parser.parse_args()
    default_runner_cfg = ThorHumanoidWalkSmoothRunnerConfig()
    policy_path = args.policy if args.policy is not None else default_runner_cfg.policy_path
    policy_path_obj = Path(policy_path).expanduser()
    if not policy_path_obj.is_absolute():
        policy_path_obj = (REPO_ROOT / policy_path_obj).resolve()
    if not policy_path_obj.is_file():
        policy_path_obj = _latest_exported_policy()

    runner_cfg = ThorHumanoidWalkSmoothRunnerConfig(
        policy_path=str(policy_path_obj),
        obs_normalizer_path=args.obs_normalizer,
        action_delay_steps=args.delay,
        command_value=args.command,
        debug_print_every_n_steps=1,
    )
    hardware_cfg = RobotInterfaceConfig(
        joint_names=CONTRACT.joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in CONTRACT.joint_names),
        joint_signs=tuple(1.0 for _ in CONTRACT.joint_names),
    )
    runner = ThorHumanoidWalkSmoothPolicyRunnerTestHarness(
        runner_cfg=runner_cfg,
        hardware_cfg=hardware_cfg,
    )

    print("\n--- SMOOTH WALKING DEBUG LOOP TEST ---")
    for _ in range(args.steps):
        runner.step()
        time.sleep(1.0 / runner_cfg.loop_hz)


if __name__ == "__main__":
    main()
