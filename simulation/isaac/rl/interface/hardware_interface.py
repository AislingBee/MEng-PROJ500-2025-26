from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Sequence

import torch


@dataclass
class ObservationPacket:
    """Common observation packet shared by sim and real hardware backends."""

    joint_pos: torch.Tensor
    joint_vel: torch.Tensor
    joint_effort: torch.Tensor
    projected_gravity_b: torch.Tensor
    imu_gyro_b: torch.Tensor


@dataclass
class ControlPacket:
    """MIT-style control packet shared by sim and real hardware backends."""

    joint_names: list[str]
    q_des: torch.Tensor
    kp: torch.Tensor
    kd: torch.Tensor
    tau_ff: torch.Tensor


class BaseHardwareInterface(ABC):
    """Abstract hardware interface used by the RL environment.

    The goal is that the environment reads observations and writes control through
    this interface only. The sim backend and robot backend must both satisfy the
    same contract.
    """

    @abstractmethod
    def read_observation_packet(self, env_ids: Sequence[int] | None = None) -> ObservationPacket:
        raise NotImplementedError

    @abstractmethod
    def write_control_packet(
        self, packet: ControlPacket, env_ids: Sequence[int] | None = None
    ) -> None:
        raise NotImplementedError
