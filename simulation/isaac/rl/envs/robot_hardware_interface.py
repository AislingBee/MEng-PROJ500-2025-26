from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import torch

from .hardware_interface import BaseHardwareInterface, ControlPacket, ObservationPacket


@dataclass
class RobotInterfaceConfig:
    """Placeholder configuration for the real robot backend.

    Update these fields once the final ROS2 / CAN topics, scaling, offsets, and
    control loop timing are known.
    """

    joint_names: tuple[str, ...]
    control_dt: float
    encoder_source: str = ""
    imu_source: str = ""
    effort_source: str = ""
    command_sink: str = ""


class RobotHardwareInterface(BaseHardwareInterface):
    """Placeholder real-robot backend.

    The functions are intentionally in place now so the environment contract does
    not need to change later. Replace the TODO blocks with the actual transport,
    scaling, offsets, filtering, and safety checks when hardware details are known.
    """

    def __init__(self, cfg: RobotInterfaceConfig, device: str | torch.device = "cpu"):
        self.cfg = cfg
        self.device = torch.device(device)
        self.num_joints = len(cfg.joint_names)

    def read_observation_packet(self, env_ids: Sequence[int] | None = None) -> ObservationPacket:
        # TODO:
        # - read encoder positions
        # - estimate / read joint velocities
        # - read or estimate joint efforts
        # - read IMU gyro
        # - compute projected gravity in body frame from IMU quaternion or accel fusion
        # - apply scaling, offsets, filtering, and unit conversion
        raise NotImplementedError(
            "RobotHardwareInterface.read_observation_packet() is a placeholder. "
            "Update it with the real encoder / IMU / effort pipeline."
        )

    def write_control_packet(self, packet: ControlPacket, env_ids: Sequence[int] | None = None) -> None:
        # TODO:
        # - pack q_des, kp, kd, tau_ff into the final ROS2 / CAN command format
        # - enforce joint order and scaling
        # - add command saturation and safety interlocks
        # - transmit at the required control loop rate
        raise NotImplementedError(
            "RobotHardwareInterface.write_control_packet() is a placeholder. "
            "Update it with the real ROS2 / CAN control transport."
        )
