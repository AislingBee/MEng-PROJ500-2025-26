from __future__ import annotations

from collections.abc import Sequence

import torch
from isaaclab.assets import Articulation
from isaaclab.utils.math import quat_rotate_inverse

from .hardware_interface import BaseHardwareInterface, ControlPacket, ObservationPacket


class IsaacHardwareInterface(BaseHardwareInterface):
    """Isaac-backed implementation of the common hardware interface."""

    def __init__(self, robot: Articulation, joint_ids: torch.Tensor, device: torch.device):
        self.robot = robot
        self.joint_ids = joint_ids
        self.device = device
        self._gravity_vec_w = torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32, device=device)

    def _resolve_env_ids(self, env_ids: Sequence[int] | None) -> torch.Tensor:
        if env_ids is None:
            return torch.arange(self.robot.data.joint_pos.shape[0], device=self.device, dtype=torch.long)
        return torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

    def _get_joint_effort_obs(self) -> torch.Tensor:
        applied_effort = getattr(self.robot.data, "applied_effort", None)
        if applied_effort is not None:
            return applied_effort

        computed_effort = getattr(self.robot.data, "computed_effort", None)
        if computed_effort is not None:
            return computed_effort

        applied_torque = getattr(self.robot.data, "applied_torque", None)
        if applied_torque is not None:
            return applied_torque

        computed_torque = getattr(self.robot.data, "computed_torque", None)
        if computed_torque is not None:
            return computed_torque

        return torch.zeros_like(self.robot.data.joint_pos)

    def read_observation_packet(self, env_ids: Sequence[int] | None = None) -> ObservationPacket:
        env_ids_t = self._resolve_env_ids(env_ids)

        root_quat_w = self.robot.data.root_quat_w[env_ids_t]
        projected_gravity_b = quat_rotate_inverse(
            root_quat_w,
            self._gravity_vec_w.unsqueeze(0).expand(len(env_ids_t), -1),
        )

        return ObservationPacket(
            joint_pos=self.robot.data.joint_pos[env_ids_t].clone(),
            joint_vel=self.robot.data.joint_vel[env_ids_t].clone(),
            joint_effort=self._get_joint_effort_obs()[env_ids_t].clone(),
            projected_gravity_b=projected_gravity_b.clone(),
            imu_gyro_b=self.robot.data.root_ang_vel_b[env_ids_t].clone(),
        )

    def write_control_packet(self, packet: ControlPacket, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            self.robot.set_joint_position_target(packet.q_des, joint_ids=self.joint_ids)
            self.robot.set_joint_effort_target(packet.tau_ff, joint_ids=self.joint_ids)
            return

        env_ids_t = self._resolve_env_ids(env_ids)
        self.robot.set_joint_position_target(packet.q_des, env_ids=env_ids_t, joint_ids=self.joint_ids)
        self.robot.set_joint_effort_target(packet.tau_ff, env_ids=env_ids_t, joint_ids=self.joint_ids)
