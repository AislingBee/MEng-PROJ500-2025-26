from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
import math
import time

import torch

from .hardware_interface import BaseHardwareInterface, ControlPacket, ObservationPacket


@dataclass
class RobotStateSample:
    """Single hardware state sample for the real robot.

    Expected inputs are the hardware-facing signals discussed for the standing
    S2R policy:
      - 12 joint encoder counts
      - optional joint velocity estimate in rad/s
      - optional joint effort estimate in Nm
      - projected gravity in body frame
      - gyro in body frame in rad/s
      - optional timestamp for velocity estimation

    Shapes should be flat per-robot arrays, not batched tensors.
    """

    encoder_counts: Sequence[float | int]
    projected_gravity_b: Sequence[float]
    imu_gyro_b: Sequence[float]
    joint_vel: Sequence[float] | None = None
    joint_effort: Sequence[float] | None = None
    timestamp_s: float | None = None


@dataclass
class RobotCommandMessage:
    """Hardware command message ready for the ROS/CAN layer."""

    joint_names: tuple[str, ...]
    q_des: list[float]
    qd_des: list[float]
    kp: list[float]
    kd: list[float]
    tau_ff: list[float]


@dataclass
class RobotInterfaceConfig:
    """Configuration for the real robot hardware backend.

    This keeps the environment contract fixed while allowing the actual ROS2/CAN
    transport details to be filled in later.
    """

    joint_names: tuple[str, ...]
    encoder_cpr: int = 16384
    encoder_min: int = 0
    encoder_max: int = 16383
    encoder_offsets_rad: tuple[float, ...] | None = None
    joint_signs: tuple[float, ...] | None = None
    default_qd_des_rad_s: float = 0.0
    velocity_clip_rad_s: float = 25.0
    effort_clip_nm: float = 150.0
    gravity_eps: float = 1e-6
    min_velocity_dt_s: float = 1e-4
    max_velocity_dt_s: float = 0.25
    enforce_single_robot: bool = True

    def __post_init__(self) -> None:
        n = len(self.joint_names)
        if self.encoder_offsets_rad is None:
            self.encoder_offsets_rad = tuple(0.0 for _ in range(n))
        if self.joint_signs is None:
            self.joint_signs = tuple(1.0 for _ in range(n))
        if len(self.encoder_offsets_rad) != n:
            raise ValueError("encoder_offsets_rad length must match joint_names")
        if len(self.joint_signs) != n:
            raise ValueError("joint_signs length must match joint_names")
        if self.encoder_cpr <= 0:
            raise ValueError("encoder_cpr must be positive")


class RobotHardwareInterface(BaseHardwareInterface):
    """Real robot backend for the standing S2R environment.

    Design intent:
      - keep the environment unchanged
      - convert raw hardware state into the common ObservationPacket
      - convert policy control output into a transport-ready command message

    Transport is injected through two callables:
      - state_reader(): returns RobotStateSample
      - command_writer(msg): sends RobotCommandMessage to ROS/CAN layer

    This avoids hard-coding ROS2 types here while still giving you working
    Stage 1 interface code.
    """

    def __init__(
        self,
        cfg: RobotInterfaceConfig,
        state_reader: Callable[[], RobotStateSample],
        command_writer: Callable[[RobotCommandMessage], None],
        device: str | torch.device = "cpu",
    ):
        self.cfg = cfg
        self.state_reader = state_reader
        self.command_writer = command_writer
        self.device = torch.device(device)
        self.num_joints = len(cfg.joint_names)

        self._last_q: torch.Tensor | None = None
        self._last_timestamp_s: float | None = None

    def _resolve_env_ids(self, env_ids: Sequence[int] | None) -> torch.Tensor:
        if env_ids is None:
            return torch.tensor([0], dtype=torch.long, device=self.device)

        env_ids_t = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        if self.cfg.enforce_single_robot:
            if env_ids_t.numel() != 1 or int(env_ids_t[0].item()) != 0:
                raise ValueError(
                    "RobotHardwareInterface only supports a single real robot instance. "
                    "Use env_ids=None or env_ids=[0]."
                )
        return env_ids_t

    def _counts_to_joint_pos_rad(self, encoder_counts: Sequence[float | int]) -> torch.Tensor:
        counts = torch.as_tensor(encoder_counts, dtype=torch.float32, device=self.device)
        if counts.numel() != self.num_joints:
            raise ValueError(
                f"Expected {self.num_joints} encoder counts, got {counts.numel()}"
            )

        # 14-bit encoder format: 0..16383 = 0..2pi.
        q = (counts / float(self.cfg.encoder_cpr)) * (2.0 * math.pi)

        # Wrap to [-pi, pi) before applying sign and calibrated zero offset.
        q = torch.remainder(q + math.pi, 2.0 * math.pi) - math.pi

        signs = torch.as_tensor(self.cfg.joint_signs, dtype=torch.float32, device=self.device)
        offsets = torch.as_tensor(
            self.cfg.encoder_offsets_rad, dtype=torch.float32, device=self.device
        )
        q = signs * q - offsets
        return q

    def _estimate_joint_velocity(self, q: torch.Tensor, sample: RobotStateSample) -> torch.Tensor:
        if sample.joint_vel is not None:
            qd = torch.as_tensor(sample.joint_vel, dtype=torch.float32, device=self.device)
            if qd.numel() != self.num_joints:
                raise ValueError(
                    f"Expected {self.num_joints} joint velocities, got {qd.numel()}"
                )
            return torch.clamp(qd, -self.cfg.velocity_clip_rad_s, self.cfg.velocity_clip_rad_s)

        now_s = sample.timestamp_s if sample.timestamp_s is not None else time.monotonic()
        if self._last_q is None or self._last_timestamp_s is None:
            self._last_q = q.clone()
            self._last_timestamp_s = now_s
            return torch.zeros_like(q)

        dt = now_s - self._last_timestamp_s
        if dt < self.cfg.min_velocity_dt_s or dt > self.cfg.max_velocity_dt_s:
            qd = torch.zeros_like(q)
        else:
            dq = q - self._last_q
            dq = torch.remainder(dq + math.pi, 2.0 * math.pi) - math.pi
            qd = dq / dt
            qd = torch.clamp(qd, -self.cfg.velocity_clip_rad_s, self.cfg.velocity_clip_rad_s)

        self._last_q = q.clone()
        self._last_timestamp_s = now_s
        return qd

    def _joint_effort_nm(self, sample: RobotStateSample) -> torch.Tensor:
        if sample.joint_effort is None:
            return torch.zeros((self.num_joints,), dtype=torch.float32, device=self.device)

        tau = torch.as_tensor(sample.joint_effort, dtype=torch.float32, device=self.device)
        if tau.numel() != self.num_joints:
            raise ValueError(
                f"Expected {self.num_joints} joint efforts, got {tau.numel()}"
            )
        return torch.clamp(tau, -self.cfg.effort_clip_nm, self.cfg.effort_clip_nm)

    def _projected_gravity_b(self, sample: RobotStateSample) -> torch.Tensor:
        gravity = torch.as_tensor(
            sample.projected_gravity_b, dtype=torch.float32, device=self.device
        )
        if gravity.numel() != 3:
            raise ValueError(
                f"Expected projected_gravity_b with 3 values, got {gravity.numel()}"
            )

        norm = torch.linalg.norm(gravity)
        if float(norm.item()) < self.cfg.gravity_eps:
            return torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32, device=self.device)
        return gravity / norm

    def _imu_gyro_b(self, sample: RobotStateSample) -> torch.Tensor:
        gyro = torch.as_tensor(sample.imu_gyro_b, dtype=torch.float32, device=self.device)
        if gyro.numel() != 3:
            raise ValueError(f"Expected imu_gyro_b with 3 values, got {gyro.numel()}")
        return gyro

    def read_observation_packet(self, env_ids: Sequence[int] | None = None) -> ObservationPacket:
        self._resolve_env_ids(env_ids)
        sample = self.state_reader()

        q = self._counts_to_joint_pos_rad(sample.encoder_counts)
        qd = self._estimate_joint_velocity(q, sample)
        tau = self._joint_effort_nm(sample)
        gravity_b = self._projected_gravity_b(sample)
        gyro_b = self._imu_gyro_b(sample)

        # Return batched tensors so the env sees the same shape contract as sim.
        return ObservationPacket(
            joint_pos=q.unsqueeze(0).clone(),
            joint_vel=qd.unsqueeze(0).clone(),
            joint_effort=tau.unsqueeze(0).clone(),
            projected_gravity_b=gravity_b.unsqueeze(0).clone(),
            imu_gyro_b=gyro_b.unsqueeze(0).clone(),
        )

    def _flatten_control_tensor(self, x: torch.Tensor, field_name: str) -> torch.Tensor:
        x = torch.as_tensor(x, dtype=torch.float32, device=self.device)
        if x.ndim == 2:
            if x.shape[0] != 1:
                raise ValueError(
                    f"Real robot backend expects batch size 1 for {field_name}, got shape {tuple(x.shape)}"
                )
            x = x[0]
        if x.numel() != self.num_joints:
            raise ValueError(
                f"Expected {self.num_joints} values for {field_name}, got {x.numel()}"
            )
        return x

    def write_control_packet(self, packet: ControlPacket, env_ids: Sequence[int] | None = None) -> None:
        self._resolve_env_ids(env_ids)

        q_des = self._flatten_control_tensor(packet.q_des, "q_des")
        kp = self._flatten_control_tensor(packet.kp, "kp")
        kd = self._flatten_control_tensor(packet.kd, "kd")
        tau_ff = self._flatten_control_tensor(packet.tau_ff, "tau_ff")
        qd_des = torch.full_like(q_des, fill_value=self.cfg.default_qd_des_rad_s)

        msg = RobotCommandMessage(
            joint_names=tuple(packet.joint_names),
            q_des=q_des.detach().cpu().tolist(),
            qd_des=qd_des.detach().cpu().tolist(),
            kp=kp.detach().cpu().tolist(),
            kd=kd.detach().cpu().tolist(),
            tau_ff=tau_ff.detach().cpu().tolist(),
        )
        self.command_writer(msg)
