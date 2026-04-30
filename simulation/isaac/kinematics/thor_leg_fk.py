from __future__ import annotations

import torch


# Joint order must match simulation.isaac.configuration.walking_s2r_policy_contract.CONTRACT.joint_names.
# These transforms are copied from simulation/isaac/assets/robot/robot.urdf. TODO: regenerate these constants
# from the robot URDF/CAD source when link geometry changes instead of editing them by hand.
_LEFT_CHAIN = (
    ((0.11999999999233989, 3.066148575681922e-12, -0.012), (-3.1415926535897754, 0.0, -1.570796326794897), (0.0, 0.0, 1.0), 0),
    ((0.0, -0.07499999999999755, 0.13630000000000136), (0.0, 1.5707963267948786, 0.0), (0.0, 1.0, 0.0), 2),
    ((0.0, 0.075, 0.0012999999999999908), (-1.5707963267948966, -1.5707963267948963, 0.0), (1.0, 0.0, 0.0), 4),
    ((0.003499999999999997, -0.3, -0.013000000000000008), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 6),
    ((0.0, -0.3, 0.0), (1.5707963267948968, 0.0, 0.0), (0.0, 1.0, 0.0), 8),
    ((0.07402717, 0.009000000000000005, 3.743000000000052e-05), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 10),
)

_RIGHT_CHAIN = (
    ((-0.12000000000765933, 3.0662609857631655e-12, -0.012), (-3.1415926535897754, 0.0, -1.570796326794897), (0.0, 0.0, -1.0), 1),
    ((0.0, 0.07500000000000245, 0.13629999999999867), (0.0, 1.5707963267948786, 0.0), (0.0, 1.0, 0.0), 3),
    ((0.0, -0.075, 0.0013000000000000093), (-1.5707963267948966, -1.5707963267948963, 0.0), (-1.0, 0.0, 0.0), 5),
    ((0.0035000000000000027, -0.3, 0.013000000000000008), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0), 7),
    ((0.0, -0.3, 0.0), (-1.5707963267948966, 0.0, 0.0), (0.0, -1.0, 0.0), 9),
    ((0.07397279, 0.009000000000000005, 3.7410000000051014e-05), (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), 11),
)


def _constant_tensor(values, *, like: torch.Tensor) -> torch.Tensor:
    return torch.tensor(values, dtype=like.dtype, device=like.device)


def _rpy_matrix(rpy, *, like: torch.Tensor) -> torch.Tensor:
    roll, pitch, yaw = _constant_tensor(rpy, like=like).unbind()
    cr, sr = torch.cos(roll), torch.sin(roll)
    cp, sp = torch.cos(pitch), torch.sin(pitch)
    cy, sy = torch.cos(yaw), torch.sin(yaw)

    row0 = torch.stack((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr))
    row1 = torch.stack((sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr))
    row2 = torch.stack((-sp, cp * sr, cp * cr))
    return torch.stack((row0, row1, row2), dim=0)


def _axis_angle_matrix(axis, angle: torch.Tensor) -> torch.Tensor:
    axis_t = _constant_tensor(axis, like=angle)
    axis_t = axis_t / torch.linalg.norm(axis_t)
    x, y, z = axis_t.unbind()
    c = torch.cos(angle)
    s = torch.sin(angle)
    one_c = 1.0 - c
    zeros = torch.zeros_like(angle)

    row0 = torch.stack((c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s), dim=-1)
    row1 = torch.stack((y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s), dim=-1)
    row2 = torch.stack((z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c), dim=-1)
    return torch.stack((row0, row1, row2), dim=-2) + zeros[..., None, None] * 0.0


def _compute_chain_pos(joint_pos_flat: torch.Tensor, chain) -> torch.Tensor:
    batch = joint_pos_flat.shape[0]
    rot = torch.eye(3, dtype=joint_pos_flat.dtype, device=joint_pos_flat.device).expand(batch, 3, 3).clone()
    pos = torch.zeros((batch, 3), dtype=joint_pos_flat.dtype, device=joint_pos_flat.device)

    for xyz, rpy, axis, joint_idx in chain:
        origin_xyz = _constant_tensor(xyz, like=joint_pos_flat).expand(batch, 3)
        origin_rot = _rpy_matrix(rpy, like=joint_pos_flat).expand(batch, 3, 3)
        pos = pos + torch.matmul(rot, origin_xyz.unsqueeze(-1)).squeeze(-1)
        rot = torch.matmul(rot, origin_rot)
        rot = torch.matmul(rot, _axis_angle_matrix(axis, joint_pos_flat[:, joint_idx]))

    return pos


def compute_foot_pos_b(joint_pos: torch.Tensor) -> torch.Tensor:
    """
    Compute left/right foot positions in pelvis/root body frame from joint angles.

    Args:
        joint_pos: shape (..., 12), ordered exactly as CONTRACT.joint_names.

    Returns:
        foot_pos_b: shape (..., 6), ordered:
            left_foot_x, left_foot_y, left_foot_z,
            right_foot_x, right_foot_y, right_foot_z
    """
    if joint_pos.shape[-1] != 12:
        raise RuntimeError(f"compute_foot_pos_b expects joint_pos.shape[-1] == 12, got {joint_pos.shape[-1]}")

    original_shape = joint_pos.shape[:-1]
    joint_pos_flat = joint_pos.reshape(-1, 12)
    left_pos = _compute_chain_pos(joint_pos_flat, _LEFT_CHAIN)
    right_pos = _compute_chain_pos(joint_pos_flat, _RIGHT_CHAIN)
    foot_pos_b = torch.cat((left_pos, right_pos), dim=-1).reshape(*original_shape, 6)
    if foot_pos_b.shape[-1] != 6:
        raise RuntimeError(f"FK foot_pos_b must have trailing dim 6, got {foot_pos_b.shape[-1]}")
    return foot_pos_b
