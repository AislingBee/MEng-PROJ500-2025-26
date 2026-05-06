#!/usr/bin/env python3
"""Pretty Robot Moves – Thor choreography runner.

Smoothly interpolates between a sequence of named joint-space poses using
cosine ease-in/ease-out.  No policy, no RL – just direct PD position control.

--------------------------------------------------------------------------------
HOW TO ADD A NEW POSE
--------------------------------------------------------------------------------
1.  Add an entry to NAMED_POSES below.  All values are in RADIANS.
    Only specify joints that differ from standing; omitted joints stay at their
    standing value.

HOW TO ADD A NEW MOVE TO THE SEQUENCE
--------------------------------------------------------------------------------
2.  Add a MoveStep to MOVE_SEQUENCE.
    MoveStep(pose="YOUR_POSE_NAME", transition_s=3.0, hold_s=2.0)
       transition_s  – seconds to sweep from the previous pose to this one
       hold_s        – seconds to dwell at this pose before the next move

KEYBOARD CONTROLS (while running)
--------------------------------------------------------------------------------
    q / Ctrl-C  – safe stop (sends standing pose on exit)
    p           – pause / resume
    n           – skip to next move immediately
    l           – toggle looping (default: on)
    +/-         – increase / decrease global speed by 10 %
--------------------------------------------------------------------------------
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import NamedTuple

import torch

from simulation.isaac.configuration.hardware_motor_direction_config import (
    joint_feedback_tuple,
    motor_direction_tuple,
)
from simulation.isaac.configuration.standing_s2r_policy_contract import CONTRACT
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
from hardware.thor.thor_policy_runner import (
    _shutdown_ros2_bridge,
    ros2_command_writer,
    ros2_state_reader,
)

Tensor = torch.Tensor

# ---------------------------------------------------------------------------
# Load per-joint soft limits from the dedicated config file
# ---------------------------------------------------------------------------
_LIMITS_FILE = Path(__file__).with_name("pretty_robot_moves_joint_limits.json")


class _PrettyMovesLimits:
    """Loads pretty_robot_moves_joint_limits.json and exposes tensors for
    soft-limit abort and warn-zone proximity checks."""

    def __init__(self, path: Path, joint_names: tuple[str, ...], device: torch.device) -> None:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        joints_by_name = {j["name"]: j for j in data["joints"]}
        missing = [n for n in joint_names if n not in joints_by_name]
        if missing:
            raise ValueError(
                f"pretty_robot_moves_joint_limits.json is missing entries for: {missing}"
            )

        soft_lower, soft_upper = [], []
        warn_lower, warn_upper = [], []
        for name in joint_names:
            j = joints_by_name[name]
            soft_lower.append(j["soft_lower"])
            soft_upper.append(j["soft_upper"])
            warn_lower.append(j["warn_lower"])
            warn_upper.append(j["warn_upper"])

        self.soft_lower = torch.tensor(soft_lower, dtype=torch.float32, device=device)
        self.soft_upper = torch.tensor(soft_upper, dtype=torch.float32, device=device)
        self.warn_lower = torch.tensor(warn_lower, dtype=torch.float32, device=device)
        self.warn_upper = torch.tensor(warn_upper, dtype=torch.float32, device=device)
        self._joint_names = list(joint_names)

    def check(self, q: Tensor, label: str) -> None:
        """Raise RuntimeError on soft-limit violation; print a warning in the warn zone."""
        q_flat = q.squeeze(0) if q.dim() == 2 else q

        # Soft-limit abort
        lower_viol = q_flat < self.soft_lower
        upper_viol = q_flat > self.soft_upper
        if lower_viol.any() or upper_viol.any():
            msgs = []
            for i, name in enumerate(self._joint_names):
                if lower_viol[i]:
                    msgs.append(
                        f"  {name}: {q_flat[i].item():.4f} < soft_lower {self.soft_lower[i].item():.4f} rad"
                    )
                elif upper_viol[i]:
                    msgs.append(
                        f"  {name}: {q_flat[i].item():.4f} > soft_upper {self.soft_upper[i].item():.4f} rad"
                    )
            raise RuntimeError(
                f"[PRETTY MOVES] Soft joint limit exceeded during '{label}':\n" + "\n".join(msgs)
            )

        # Warn-zone proximity alert (print once per violation)
        in_warn = (q_flat < self.warn_lower) | (q_flat > self.warn_upper)
        if in_warn.any():
            for i, name in enumerate(self._joint_names):
                if q_flat[i] < self.warn_lower[i]:
                    print(
                        f"[PRETTY MOVES] WARN '{label}' {name}: "
                        f"{q_flat[i].item():.4f} rad approaching soft_lower "
                        f"{self.soft_lower[i].item():.4f} rad"
                    )
                elif q_flat[i] > self.warn_upper[i]:
                    print(
                        f"[PRETTY MOVES] WARN '{label}' {name}: "
                        f"{q_flat[i].item():.4f} rad approaching soft_upper "
                        f"{self.soft_upper[i].item():.4f} rad"
                    )


# ---------------------------------------------------------------------------
# Soft joint limits in radians (from pretty_robot_moves_joint_limits.json)
# ---------------------------------------------------------------------------
#   pelvis_yaw_l      : -0.40 → +0.50
#   pelvis_yaw_r      : -0.20 → +0.40
#   hip_pitch_l       : -0.50 → +0.50
#   hip_pitch_r       : -0.40 → +0.50
#   hip_roll_l        : -0.15 → +0.15
#   hip_roll_r        : -0.10 → +0.40
#   knee_l            : -0.70 → +0.60
#   knee_r            : -0.70 → +0.50
#   ankle_pitch_l     : -1.50 →  0.00  (negative convention)
#   ankle_pitch_r     :  0.00 → +1.50
#   ankle_roll_l      : -0.10 → +0.40
#   ankle_roll_r      : -0.30 → +0.20

# ---------------------------------------------------------------------------
# STANDING reference in radians (mirrors standing_pose.py)
# ---------------------------------------------------------------------------
_STANDING_RAD: dict[str, float] = {
    "pelvis_link_l_yaw_joint":          0.0000,
    "pelvis_link_r_yaw_joint":          0.0000,
    "l_hip_yaw_link_l_pitch_joint":     0.0000,   # 13°
    "r_hip_yaw_link_r_pitch_joint":     0.0000,   # 13°
    "l_hip_pitch_link_l_roll_joint":    0.0000,
    "r_hip_pitch_link_r_roll_joint":    0.0000,
    "l_thigh_link_l_knee_joint":        0.0000,   # 25°
    "r_thigh_link_r_knee_joint":        0.0000,   # 25°
    "l_shank_link_l_ankle_joint":      -0.2000,   # -13° (negative convention for left ankle)
    "r_shank_link_r_ankle_joint":       0.2000,   # +13°
    "l_ankle_link_l_foot_joint":        0.0000,
    "r_ankle_link_r_foot_joint":        0.0000,
}

# ---------------------------------------------------------------------------
# ============================================================
#   ADD / EDIT NAMED POSES HERE  (all values in RADIANS)
#   Omitted joints default to their STANDING value.
# ============================================================
# ---------------------------------------------------------------------------
NAMED_POSES: dict[str, dict[str, float]] = {

    # ------------------------------------------------------------------
    # Standing – reference pose (slight knee bend, hip pitch for shape)
    # ------------------------------------------------------------------
    "STANDING": {
        # All at standing defaults – no overrides needed.
    },

    # ------------------------------------------------------------------
    # Legs straight – hang fully extended with toes slightly pointed
    # ------------------------------------------------------------------
    "LEGS_STRAIGHT": {
        "l_hip_yaw_link_l_pitch_joint":  0.00,
        "r_hip_yaw_link_r_pitch_joint":  0.00,
        "l_thigh_link_l_knee_joint":     0.00,
        "r_thigh_link_r_knee_joint":     0.00,
        "l_shank_link_l_ankle_joint":   -0.10,   # slight toe point (safe zone)
        "r_shank_link_r_ankle_joint":    0.10,   # slight toe point (safe zone)
    },

    # ------------------------------------------------------------------
    # Kick left – left leg swings forward with extended knee and pointed toe
    # ------------------------------------------------------------------
    "KICK_LEFT": {
        "l_hip_yaw_link_l_pitch_joint":  0.45,   # swing forward  (limit  0.50)
        "r_hip_yaw_link_r_pitch_joint":  0.00,   # right leg neutral
        "l_thigh_link_l_knee_joint":    -0.55,   # extend knee    (limit -0.70)
        "r_thigh_link_r_knee_joint":     0.00,
        "l_shank_link_l_ankle_joint":   -1.60,   # point left toe (limit -1.60)
        "r_shank_link_r_ankle_joint":    0.10,
    },

    # ------------------------------------------------------------------
    # Kick right – right leg swings forward with extended knee and pointed toe
    # ------------------------------------------------------------------
    "KICK_RIGHT": {
        "l_hip_yaw_link_l_pitch_joint":  0.00,
        "r_hip_yaw_link_r_pitch_joint":  0.45,   # swing forward  (limit  0.50)
        "l_thigh_link_l_knee_joint":     0.00,
        "r_thigh_link_r_knee_joint":    -0.55,   # extend knee    (limit -0.70)
        "l_shank_link_l_ankle_joint":   -0.10,
        "r_shank_link_r_ankle_joint":    1.50,   # point right toe (limit 1.50)
    },

    # ------------------------------------------------------------------
    # Knees up – both hips pitched forward, knees pulled up, toes dangling
    # ------------------------------------------------------------------
    "KNEES_UP": {
        "l_hip_yaw_link_l_pitch_joint":  0.45,   # forward        (limit  0.50)
        "r_hip_yaw_link_r_pitch_joint":  0.45,   # forward        (limit  0.50)
        "l_thigh_link_l_knee_joint":     0.55,   # bent up        (limit  0.60)
        "r_thigh_link_r_knee_joint":     0.46,   # bent up        (limit  0.50)
        "l_shank_link_l_ankle_joint":   -1.60,   # toes fully pointed
        "r_shank_link_r_ankle_joint":    1.50,
    },

    # ------------------------------------------------------------------
    # Legs back – both legs sweep behind, knees fold naturally
    # ------------------------------------------------------------------
    "LEGS_BACK": {
        "l_hip_yaw_link_l_pitch_joint": -0.45,   # sweep back     (limit -0.50)
        "r_hip_yaw_link_r_pitch_joint": -0.35,   # sweep back     (limit -0.40)
        "l_thigh_link_l_knee_joint":     0.45,   # fold back
        "r_thigh_link_r_knee_joint":     0.40,
        "l_shank_link_l_ankle_joint":   -0.10,
        "r_shank_link_r_ankle_joint":    0.10,
    },

    # ------------------------------------------------------------------
    # Foot taps – alternate pointing left/right ankle pitch (joints 9 & 10).
    # TAP_L fully points the left toe; TAP_R fully points the right toe.
    # Used in a rapid alternating sequence below.
    # ------------------------------------------------------------------
    "FOOT_TAP_L": {
        "l_shank_link_l_ankle_joint": -1.60,   # left toe fully pointed  (limit -1.60)
        "r_shank_link_r_ankle_joint":  0.10,   # right ankle neutral
    },
    "FOOT_TAP_R": {
        "l_shank_link_l_ankle_joint": -0.10,   # left ankle neutral
        "r_shank_link_r_ankle_joint":  1.50,   # right toe fully pointed (limit  1.50)
    },

}

# ---------------------------------------------------------------------------
# ============================================================
#   NAMED SEQUENCES  –  pick one with  --sequence <name>
#   Each is a short self-contained loop.
# ============================================================
# ---------------------------------------------------------------------------
class MoveStep(NamedTuple):
    pose: str            # key from NAMED_POSES
    transition_s: float  # seconds to sweep from previous pose to this one
    hold_s: float        # seconds to hold this pose before the next step


SEQUENCES: dict[str, list[MoveStep]] = {

    # ------------------------------------------------------------------
    # kicks  –  left kick, right kick, back to neutral
    # ------------------------------------------------------------------
    "kicks": [
        MoveStep("STANDING",      transition_s=4.0, hold_s=1.0),
        MoveStep("LEGS_STRAIGHT", transition_s=3.0, hold_s=1.0),
        MoveStep("KICK_LEFT",     transition_s=2.5, hold_s=2.0),
        MoveStep("LEGS_STRAIGHT", transition_s=2.0, hold_s=0.5),
        MoveStep("KICK_RIGHT",    transition_s=2.5, hold_s=2.0),
        MoveStep("STANDING",      transition_s=3.0, hold_s=1.0),
    ],

    # ------------------------------------------------------------------
    # taps  –  rapid alternating ankle-point taps
    # ------------------------------------------------------------------
    "taps": [
        MoveStep("STANDING",   transition_s=3.0, hold_s=0.5),
        MoveStep("FOOT_TAP_L", transition_s=0.35, hold_s=0.20),
        MoveStep("FOOT_TAP_R", transition_s=0.35, hold_s=0.20),
        MoveStep("FOOT_TAP_L", transition_s=0.35, hold_s=0.20),
        MoveStep("FOOT_TAP_R", transition_s=0.35, hold_s=0.20),
        MoveStep("FOOT_TAP_L", transition_s=0.35, hold_s=0.20),
        MoveStep("FOOT_TAP_R", transition_s=0.35, hold_s=0.20),
        MoveStep("STANDING",   transition_s=2.0, hold_s=1.0),
    ],

    # ------------------------------------------------------------------
    # shapes  –  knees up and legs back
    # ------------------------------------------------------------------
    "shapes": [
        MoveStep("STANDING",      transition_s=4.0, hold_s=1.0),
        MoveStep("KNEES_UP",      transition_s=3.0, hold_s=2.5),
        MoveStep("LEGS_STRAIGHT", transition_s=2.5, hold_s=1.0),
        MoveStep("LEGS_BACK",     transition_s=3.5, hold_s=2.5),
        MoveStep("STANDING",      transition_s=4.0, hold_s=1.0),
    ],

}

DEFAULT_SEQUENCE = "taps"


# ===========================================================================
# Implementation – you normally do not need to edit below this line
# ===========================================================================

@dataclass
class PrettyMovesConfig:
    loop: bool = True
    loop_hz: float = CONTRACT.policy_loop_hz
    kp_scale: float = 0.20      # conservative – same as startup ramp
    kd_scale: float = 1.00
    effort_scale: float = 0.25
    max_position_error_rad: float = 0.90
    speed_multiplier: float = 0.3
    debug_print_every_n_steps: int = 100
    device: str = "cpu"


def _resolve_pose(partial_pose_rad: dict[str, float], joint_names: tuple[str, ...]) -> dict[str, float]:
    """Fill in any missing joints with their STANDING value (radians)."""
    resolved: dict[str, float] = {}
    for name in joint_names:
        resolved[name] = partial_pose_rad.get(name, _STANDING_RAD[name])
    return resolved


def _pose_to_tensor(
    pose_rad: dict[str, float],
    joint_names: tuple[str, ...],
    device: torch.device,
) -> Tensor:
    q = torch.zeros(len(joint_names), dtype=torch.float32, device=device)
    for i, name in enumerate(joint_names):
        q[i] = pose_rad[name]
    return q


def _cosine_alpha(t: float, duration: float) -> float:
    """Smooth cosine ease-in/ease-out: 0 → 1 over [0, duration]."""
    if duration <= 0.0:
        return 1.0
    progress = min(max(t / duration, 0.0), 1.0)
    return 0.5 * (1.0 - math.cos(math.pi * progress))


def _validate_poses(limits: _PrettyMovesLimits) -> None:
    """Fail fast at startup if any pose or sequence reference is invalid."""
    for pose_name, partial_pose in NAMED_POSES.items():
        resolved = _resolve_pose(partial_pose, CONTRACT.joint_names)
        for i, joint_name in enumerate(CONTRACT.joint_names):
            val_rad = resolved[joint_name]
            sl = float(limits.soft_lower[i].item())
            su = float(limits.soft_upper[i].item())
            if val_rad < sl - 1e-4 or val_rad > su + 1e-4:
                raise ValueError(
                    f"Pose '{pose_name}' joint '{joint_name}' = {val_rad:.4f} rad "
                    f"exceeds soft limits [{sl:.4f}, {su:.4f}] rad"
                )
    for seq_name, steps in SEQUENCES.items():
        for step_index, step in enumerate(steps):
            if step.pose not in NAMED_POSES:
                raise ValueError(
                    f"SEQUENCES['{seq_name}'][{step_index}] references unknown pose '{step.pose}'. "
                    f"Available poses: {list(NAMED_POSES.keys())}"
                )


class ThorPrettyMovesRunner:
    def __init__(
        self,
        cfg: PrettyMovesConfig,
        hardware_cfg: RobotInterfaceConfig,
        state_reader,
        command_writer,
        sequence: list[MoveStep],
    ) -> None:
        self.cfg = cfg
        self.device = torch.device(cfg.device)
        self._sequence = sequence

        self.hardware = RobotHardwareInterface(
            cfg=hardware_cfg,
            state_reader=state_reader,
            command_writer=command_writer,
            device=self.device,
        )

        self._joint_names = list(CONTRACT.joint_names)

        # Load soft limits from the dedicated config file
        self._limits = _PrettyMovesLimits(_LIMITS_FILE, CONTRACT.joint_names, self.device)

        # Hard clamp tensors (still used as a last resort in _send_q_des)
        self._joint_lower = torch.tensor(
            CONTRACT.joint_lower_limits_rad, dtype=torch.float32, device=self.device
        )
        self._joint_upper = torch.tensor(
            CONTRACT.joint_upper_limits_rad, dtype=torch.float32, device=self.device
        )

        per_joint = build_per_joint_walking_actuator_cfg(CONTRACT.joint_names)
        self._kp = (
            torch.tensor(per_joint["stiffness"], dtype=torch.float32, device=self.device)
            * float(cfg.kp_scale)
        ).unsqueeze(0)
        self._kd = (
            torch.tensor(per_joint["damping"], dtype=torch.float32, device=self.device)
            * float(cfg.kd_scale)
        ).unsqueeze(0)
        self._tau_ff = torch.zeros(
            (1, len(self._joint_names)), dtype=torch.float32, device=self.device
        )

        # Pre-build all pose tensors
        self._pose_tensors: dict[str, Tensor] = {}
        for pose_name, partial_pose in NAMED_POSES.items():
            resolved = _resolve_pose(partial_pose, CONTRACT.joint_names)
            self._pose_tensors[pose_name] = _pose_to_tensor(resolved, CONTRACT.joint_names, self.device)

        # Runtime state
        self._step_count = 0
        self._paused = False
        self._skip_requested = False
        self._loop = cfg.loop
        self._speed = cfg.speed_multiplier
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # Control helpers
    # ------------------------------------------------------------------
    def request_stop(self) -> None:
        self._stop_event.set()

    def toggle_pause(self) -> None:
        self._paused = not self._paused
        print(f"[PRETTY MOVES] {'PAUSED' if self._paused else 'RESUMED'}")

    def request_skip(self) -> None:
        self._skip_requested = True

    def toggle_loop(self) -> None:
        self._loop = not self._loop
        print(f"[PRETTY MOVES] Looping {'ON' if self._loop else 'OFF'}")

    def adjust_speed(self, delta: float) -> None:
        self._speed = max(0.1, min(5.0, self._speed + delta))
        print(f"[PRETTY MOVES] Speed ×{self._speed:.2f}")

    # ------------------------------------------------------------------
    # Hardware helpers
    # ------------------------------------------------------------------
    def _send_zero_torque(self) -> None:
        """Emergency safe-state: zero all kp, kd, and torque feedforward.

        Called before any RuntimeError abort so that motors are de-energised
        before the exception propagates.
        """
        zeros = torch.zeros(
            (1, len(self._joint_names)), dtype=torch.float32, device=self.device
        )
        packet = ControlPacket(
            joint_names=self._joint_names,
            q_des=zeros.clone(),
            kp=zeros.clone(),
            kd=zeros.clone(),
            tau_ff=zeros.clone(),
            kp_gains=zeros.clone(),
            kd_gains=zeros.clone(),
        )
        try:
            self.hardware.write_control_packet(packet)
            print("[PRETTY MOVES] ABORT: zero kp/kd/torque sent to all motors.")
        except Exception as _e:
            print(f"[PRETTY MOVES] WARNING: could not send zero-torque packet: {_e}")

    def _read_current_q(self) -> Tensor:
        obs = self.hardware.read_observation_packet()
        return obs.joint_pos.to(self.device, dtype=torch.float32).squeeze(0)

    def _send_q_des(self, q_des: Tensor, label: str = "") -> None:
        """Check soft limits, clamp to hard limits, validate no NaN, write to hardware."""
        if torch.isnan(q_des).any():
            self._send_zero_torque()
            raise RuntimeError("NaN detected in q_des before sending")
        # Soft-limit check: abort + warn-zone proximity alert
        try:
            self._limits.check(q_des, label or "send")
        except RuntimeError:
            self._send_zero_torque()
            raise
        # Hard clamp as final safety net (should never trigger if soft limits are respected)
        q_clamped = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)
        packet = ControlPacket(
            joint_names=self._joint_names,
            q_des=q_clamped.unsqueeze(0).clone(),
            kp=self._kp.clone(),
            kd=self._kd.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp.clone(),
            kd_gains=self._kd.clone(),
        )
        self.hardware.write_control_packet(packet)

    def _check_tracking_error(self, q_actual: Tensor, q_des: Tensor, label: str) -> None:
        max_err = float(torch.max(torch.abs(q_actual - q_des)).item())
        if max_err > self.cfg.max_position_error_rad:
            self._send_zero_torque()
            raise RuntimeError(
                f"[PRETTY MOVES] Tracking error abort during '{label}': "
                f"max|q_actual - q_des| = {max_err:.4f} rad "
                f"(limit {self.cfg.max_position_error_rad:.4f} rad)"
            )

    # ------------------------------------------------------------------
    # Core interpolation loop
    # ------------------------------------------------------------------
    def _interpolate_to_pose(
        self,
        q_from: Tensor,
        q_to: Tensor,
        duration_s: float,
        label: str,
    ) -> Tensor:
        """Move from q_from → q_to over duration_s seconds (cosine ease).

        Returns the actual position when the sweep finishes (= q_actual at end).
        """
        effective_duration = max(0.01, duration_s / self._speed)
        period_s = 1.0 / self.cfg.loop_hz
        start_t = time.monotonic()
        next_t = start_t
        last_q_des = q_from.clone()

        while not self._stop_event.is_set():
            if self._paused:
                time.sleep(0.05)
                next_t = time.monotonic()
                continue

            elapsed = time.monotonic() - start_t
            alpha = _cosine_alpha(elapsed, effective_duration)
            q_des = q_from + alpha * (q_to - q_from)
            last_q_des = q_des.clone()

            self._send_q_des(q_des, label)
            q_actual = self._read_current_q()
            self._check_tracking_error(q_actual, q_des, label)

            self._step_count += 1
            if self._step_count % self.cfg.debug_print_every_n_steps == 0:
                print(
                    f"[PRETTY MOVES] move='{label}' α={alpha:.3f} "
                    f"max_err={float(torch.max(torch.abs(q_actual - q_des)).item()):.4f} rad"
                )

            if self._skip_requested or elapsed >= effective_duration:
                self._skip_requested = False
                break

            next_t += period_s
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

        return last_q_des

    def _hold_pose(self, q_des: Tensor, duration_s: float, label: str) -> None:
        """Hold q_des for duration_s seconds."""
        effective_duration = max(0.0, duration_s / self._speed)
        period_s = 1.0 / self.cfg.loop_hz
        start_t = time.monotonic()
        next_t = start_t

        while not self._stop_event.is_set():
            if self._paused:
                time.sleep(0.05)
                next_t = time.monotonic()
                continue

            self._send_q_des(q_des, f"{label} (hold)")
            q_actual = self._read_current_q()
            self._check_tracking_error(q_actual, q_des, f"{label} (hold)")

            if self._skip_requested or (time.monotonic() - start_t) >= effective_duration:
                self._skip_requested = False
                break

            next_t += period_s
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

    # ------------------------------------------------------------------
    # Main run loop
    # ------------------------------------------------------------------
    def run(self) -> None:
        print("\n" + "=" * 70)
        print("  THOR PRETTY MOVES")
        print(f"  Sequence: {[s.pose for s in self._sequence]}")
        print(f"  Looping:  {self._loop}")
        print(f"  Speed:    ×{self._speed:.2f}")
        print("  Controls: [q] quit  [p] pause  [n] next  [l] loop  [+/-] speed")
        print("=" * 70 + "\n")

        # ------ Move from current hardware position to first pose ------
        first_pose_name = self._sequence[0].pose
        q_current = self._read_current_q()
        q_first = self._pose_tensors[first_pose_name]

        print(f"[PRETTY MOVES] Ramping to first pose '{first_pose_name}' ...")
        q_current = self._interpolate_to_pose(
            q_current, q_first,
            duration_s=self._sequence[0].transition_s,
            label=f"→ {first_pose_name}",
        )

        # ------ Sequence loop ------------------------------------------
        sequence_index = 0

        try:
            while not self._stop_event.is_set():
                step = self._sequence[sequence_index]
                q_target = self._pose_tensors[step.pose]

                if sequence_index != 0:
                    # Transition to this pose
                    print(
                        f"[PRETTY MOVES] [{sequence_index + 1}/{len(self._sequence)}] "
                        f"'{step.pose}'  transition={step.transition_s:.1f}s  hold={step.hold_s:.1f}s"
                    )
                    q_current = self._interpolate_to_pose(
                        q_current, q_target,
                        duration_s=step.transition_s,
                        label=step.pose,
                    )
                else:
                    # Already moved to first pose above; just update q_current
                    q_current = q_first

                if self._stop_event.is_set():
                    break

                # Hold
                self._hold_pose(q_current, step.hold_s, label=step.pose)

                # Advance sequence
                sequence_index += 1
                if sequence_index >= len(self._sequence):
                    if self._loop:
                        sequence_index = 0
                        print("[PRETTY MOVES] Looping sequence from start.")
                    else:
                        print("[PRETTY MOVES] Sequence complete.")
                        break

        except KeyboardInterrupt:
            pass
        finally:
            # Safe exit – return to standing
            print("\n[PRETTY MOVES] Returning to standing pose ...")
            q_standing = self._pose_tensors["STANDING"]
            try:
                self._interpolate_to_pose(
                    q_current, q_standing,
                    duration_s=4.0,
                    label="EXIT → STANDING",
                )
            except Exception:
                # Best effort on exit – just send standing directly
                self._send_q_des(q_standing, "EXIT → STANDING")
            print("[PRETTY MOVES] Done.")


# ---------------------------------------------------------------------------
# Keyboard input thread
# ---------------------------------------------------------------------------
def _keyboard_thread(runner: ThorPrettyMovesRunner, stop_event: threading.Event) -> None:
    print("[PRETTY MOVES] Keyboard input active. Type a command and press Enter.")
    while not stop_event.is_set():
        try:
            key = input().strip().lower()
        except EOFError:
            break
        if key in ("q", "quit", "exit"):
            print("[PRETTY MOVES] Quit requested.")
            runner.request_stop()
            break
        elif key == "p":
            runner.toggle_pause()
        elif key == "n":
            runner.request_skip()
            print("[PRETTY MOVES] Skipping to next move.")
        elif key == "l":
            runner.toggle_loop()
        elif key == "+":
            runner.adjust_speed(+0.1)
        elif key == "-":
            runner.adjust_speed(-0.1)
        else:
            print(f"[PRETTY MOVES] Unknown key '{key}'. Valid: q p n l + -")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Thor Pretty Moves – choreograph smooth joint-space motion sequences."
    )
    parser.add_argument(
        "--sequence",
        type=str,
        default=DEFAULT_SEQUENCE,
        choices=list(SEQUENCES.keys()),
        help=f"Which movement sequence to run. Choices: {list(SEQUENCES.keys())}. Default: {DEFAULT_SEQUENCE}",
    )
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument(
        "--loop-hz", type=float, default=CONTRACT.policy_loop_hz,
        help="Control loop frequency in Hz.",
    )
    parser.add_argument(
        "--kp-scale", type=float, default=0.20,
        help="Scale applied to stiffness gains (keep ≤0.25 for safety).",
    )
    parser.add_argument("--kd-scale", type=float, default=1.00)
    parser.add_argument(
        "--no-loop", action="store_true",
        help="Play the sequence once then exit instead of looping.",
    )
    parser.add_argument(
        "--speed", type=float, default=0.3,
        help="Global speed multiplier (>1 = faster, <1 = slower).",
    )
    parser.add_argument(
        "--max-position-error-rad", type=float, default=0.90,
        help="Abort if any joint tracking error exceeds this value (rad).",
    )
    parser.add_argument(
        "--debug-every", type=int, default=100,
        help="Print debug line every N control steps.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Load limits and validate all poses before touching hardware
    _startup_limits = _PrettyMovesLimits(
        _LIMITS_FILE, CONTRACT.joint_names, torch.device("cpu")
    )
    _validate_poses(_startup_limits)

    sequence = SEQUENCES[args.sequence]
    print(f"[PRETTY MOVES] Using sequence: '{args.sequence}' ({len(sequence)} steps)")

    joint_names = CONTRACT.joint_names
    cfg = PrettyMovesConfig(
        loop=not args.no_loop,
        loop_hz=args.loop_hz,
        kp_scale=args.kp_scale,
        kd_scale=args.kd_scale,
        max_position_error_rad=args.max_position_error_rad,
        speed_multiplier=args.speed,
        debug_print_every_n_steps=args.debug_every,
        device=args.device,
    )

    hardware_cfg = RobotInterfaceConfig(
        joint_names=joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in joint_names),
        joint_signs=joint_feedback_tuple(joint_names),
        motor_direction_signs=motor_direction_tuple(joint_names),
    )

    runner = ThorPrettyMovesRunner(
        cfg=cfg,
        hardware_cfg=hardware_cfg,
        state_reader=ros2_state_reader,
        command_writer=ros2_command_writer,
        sequence=sequence,
    )

    stop_event = threading.Event()
    kb_thread = threading.Thread(
        target=_keyboard_thread, args=(runner, stop_event), daemon=True
    )
    kb_thread.start()

    try:
        runner.run()
    finally:
        stop_event.set()
        _shutdown_ros2_bridge()


if __name__ == "__main__":
    main()
