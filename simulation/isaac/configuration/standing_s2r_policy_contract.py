from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Final

import torch

from simulation.isaac.configuration.standing_pose import STANDING_TARGETS_DEG
from simulation.isaac.configuration.walking_actuator_config import (
    build_per_joint_walking_actuator_cfg,
)

# Shared standing sim-to-real policy contract.
# Import this into:
#   - humanoid_stand_s2r_env.py
#   - export_deployable_policy(...)
#   - thor_policy_runner.py
#
# This keeps deployment-critical constants aligned between training and Thor.


JOINT_NAMES: Final[tuple[str, ...]] = (
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

ACTION_SCALE: Final[tuple[float, ...]] = (
    0.10, 0.08, 0.15, 0.20, 0.12, 0.08,
    0.10, 0.08, 0.15, 0.20, 0.12, 0.08,
)

SIM_DT_S: Final[float] = 1.0 / 120.0
DECIMATION: Final[int] = 2
POLICY_LOOP_HZ: Final[float] = 1.0 / (SIM_DT_S * DECIMATION)

DEFAULT_COMMAND_VALUE: Final[float] = 0.0

ACTION_DIM: Final[int] = 12
OBS_DIM: Final[int] = 55

OBS_LAYOUT: Final[dict[str, int]] = {
    "q_rel": 12,
    "joint_vel": 12,
    "joint_effort": 12,
    "projected_gravity_b": 3,
    "imu_gyro_b": 3,
    "command": 1,
    "last_actions": 12,
}

if sum(OBS_LAYOUT.values()) != OBS_DIM:
    raise RuntimeError(f"Observation layout sums to {sum(OBS_LAYOUT.values())}, expected {OBS_DIM}")

_JOINT_LIMITS_PATH = Path(__file__).with_name("joint_limits_config.json")
with _JOINT_LIMITS_PATH.open("r", encoding="utf-8") as f:
    _JOINT_LIMITS_DATA = json.load(f)

JOINT_LIMIT_NAMES: Final[tuple[str, ...]] = tuple(_JOINT_LIMITS_DATA["joint_names"])
JOINT_LOWER_LIMITS_RAD: Final[tuple[float, ...]] = tuple(_JOINT_LIMITS_DATA["joint_lower_limits"])
JOINT_UPPER_LIMITS_RAD: Final[tuple[float, ...]] = tuple(_JOINT_LIMITS_DATA["joint_upper_limits"])

if len(JOINT_LIMIT_NAMES) != ACTION_DIM:
    raise RuntimeError(f"Joint limit config has {len(JOINT_LIMIT_NAMES)} joints, expected {ACTION_DIM}")


@dataclass(frozen=True)
class StandingS2RPolicyContract:
    joint_names: tuple[str, ...] = JOINT_NAMES
    action_scale: tuple[float, ...] = ACTION_SCALE
    sim_dt_s: float = SIM_DT_S
    decimation: int = DECIMATION
    default_command_value: float = DEFAULT_COMMAND_VALUE
    action_dim: int = ACTION_DIM
    obs_dim: int = OBS_DIM
    joint_lower_limits_rad: tuple[float, ...] = JOINT_LOWER_LIMITS_RAD
    joint_upper_limits_rad: tuple[float, ...] = JOINT_UPPER_LIMITS_RAD

    @property
    def policy_loop_hz(self) -> float:
        return 1.0 / (self.sim_dt_s * self.decimation)

    def build_standing_q(self, device: str | torch.device = "cpu") -> torch.Tensor:
        q = torch.zeros((len(self.joint_names),), dtype=torch.float32, device=device)
        for i, joint_name in enumerate(self.joint_names):
            q[i] = math.radians(STANDING_TARGETS_DEG.get(joint_name, 0.0))
        return q

    def build_fixed_gains(
        self, device: str | torch.device = "cpu"
    ) -> tuple[torch.Tensor, torch.Tensor]:
        per_joint_cfg = build_per_joint_walking_actuator_cfg(self.joint_names)
        kp = torch.tensor(per_joint_cfg["stiffness"], dtype=torch.float32, device=device)
        kd = torch.tensor(per_joint_cfg["damping"], dtype=torch.float32, device=device)
        return kp, kd


CONTRACT: Final[StandingS2RPolicyContract] = StandingS2RPolicyContract()


def build_standing_q(device: str | torch.device = "cpu") -> torch.Tensor:
    return CONTRACT.build_standing_q(device=device)


def build_fixed_gains(
    device: str | torch.device = "cpu",
) -> tuple[torch.Tensor, torch.Tensor]:
    return CONTRACT.build_fixed_gains(device=device)


def get_thor_runner_defaults() -> dict:
    return {
        "joint_names": CONTRACT.joint_names,
        "action_scale": CONTRACT.action_scale,
        "loop_hz": CONTRACT.policy_loop_hz,
        "command_value": CONTRACT.default_command_value,
        "observation_dim": CONTRACT.obs_dim,
        "action_dim": CONTRACT.action_dim,
        "joint_lower_limits_rad": CONTRACT.joint_lower_limits_rad,
        "joint_upper_limits_rad": CONTRACT.joint_upper_limits_rad,
    }


# Joint limits are intentionally not hard-coded here because the correct values
# must match the exact articulation limits that Isaac used during training.
# Populate these from the trained robot definition / runtime dump and keep the
# ordering aligned with JOINT_NAMES.
#
# Example:
# THOR_JOINT_LIMITS = {
#     "lower_rad": (... 12 values ...),
#     "upper_rad": (... 12 values ...),
# }
