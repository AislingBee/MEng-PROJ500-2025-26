from __future__ import annotations

import argparse
import math
import time
from collections.abc import Callable
from dataclasses import dataclass

import torch

# Select policy contract.
# Use exactly one active import.
# Standing and walking use the same hardware interface.
# The selected contract defines the hardware joint order and joint limits.
from simulation.isaac.configuration.standing_s2r_policy_contract import CONTRACT

# from simulation.isaac.configuration.walking_s2r_policy_contract import CONTRACT

from simulation.isaac.configuration.standing_pose import STANDING_TARGETS_DEG
from simulation.isaac.configuration.walking_actuator_config import (
    build_per_joint_walking_actuator_cfg,
)
from simulation.isaac.configuration.zero_pose import ZERO_POSE_DEG
from simulation.isaac.rl.interface.hardware_interface import ControlPacket
from simulation.isaac.rl.interface.robot_hardware_interface import (
    RobotHardwareInterface,
    RobotInterfaceConfig,
)
from hardware.thor.thor_policy_runner import (
    _shutdown_ros2_bridge,
    ros2_command_writer,
    ros2_state_reader,
)


Tensor = torch.Tensor


@dataclass
class ThorStartupToStandingConfig:
    joint_names: tuple[str, ...] = CONTRACT.joint_names
    joint_lower_rad: tuple[float, ...] = CONTRACT.joint_lower_limits_rad
    joint_upper_rad: tuple[float, ...] = CONTRACT.joint_upper_limits_rad
    loop_hz: float = CONTRACT.policy_loop_hz
    device: str = "cpu"
    ramp_time_s: float = 8.0
    kp_scale: float = 0.20
    kd_scale: float = 1.00
    effort_scale: float = 0.25
    position_tolerance_rad: float = 0.05
    velocity_tolerance_rad_s: float = 0.10
    max_position_error_rad: float = 0.75
    send_standing_pose_on_exit: bool = True
    debug_print_every_n_steps: int = 50

    def __post_init__(self) -> None:
        joint_count = len(self.joint_names)
        if self.joint_names != CONTRACT.joint_names:
            raise ValueError("joint_names must match the selected S2R policy contract")
        if joint_count != CONTRACT.action_dim:
            raise ValueError(f"Expected {CONTRACT.action_dim} joints, got {joint_count}")
        if len(self.joint_lower_rad) != joint_count:
            raise ValueError("joint_lower_rad length must match joint_names")
        if len(self.joint_upper_rad) != joint_count:
            raise ValueError("joint_upper_rad length must match joint_names")
        if tuple(self.joint_lower_rad) != CONTRACT.joint_lower_limits_rad:
            raise ValueError("joint_lower_rad must match the selected S2R policy contract")
        if tuple(self.joint_upper_rad) != CONTRACT.joint_upper_limits_rad:
            raise ValueError("joint_upper_rad must match the selected S2R policy contract")
        if self.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")
        if self.ramp_time_s <= 0.0:
            raise ValueError("ramp_time_s must be positive")
        if self.kp_scale < 0.0:
            raise ValueError("kp_scale must be non-negative")
        if self.kd_scale < 0.0:
            raise ValueError("kd_scale must be non-negative")
        if self.effort_scale < 0.0:
            raise ValueError("effort_scale must be non-negative")
        if self.position_tolerance_rad < 0.0:
            raise ValueError("position_tolerance_rad must be non-negative")
        if self.velocity_tolerance_rad_s < 0.0:
            raise ValueError("velocity_tolerance_rad_s must be non-negative")
        if self.max_position_error_rad <= 0.0:
            raise ValueError("max_position_error_rad must be positive")
        if self.debug_print_every_n_steps <= 0:
            raise ValueError("debug_print_every_n_steps must be positive")


def _build_pose_tensor_from_degrees(
    pose_degrees: dict[str, float],
    joint_names: tuple[str, ...],
    pose_name: str,
    device: torch.device,
) -> Tensor:
    missing_joints = [joint_name for joint_name in joint_names if joint_name not in pose_degrees]
    if missing_joints:
        raise ValueError(
            f"{pose_name} config is missing required joints: {', '.join(missing_joints)}"
        )

    q = torch.zeros((len(joint_names),), dtype=torch.float32, device=device)
    for joint_index, joint_name in enumerate(joint_names):
        angle_deg = pose_degrees[joint_name]
        q[joint_index] = math.radians(float(angle_deg))
    return q


def _print_pose_comparison_table(
    joint_names: tuple[str, ...],
    q_zero: Tensor,
    q_standing: Tensor,
) -> None:
    header = (
        f"{'joint name':<34} | {'zero rad':>10} | {'standing rad':>12} | "
        f"{'delta rad':>10} | {'delta deg':>10}"
    )
    print(header)
    print("-" * len(header))
    for joint_index, joint_name in enumerate(joint_names):
        zero_rad = float(q_zero[joint_index].item())
        standing_rad = float(q_standing[joint_index].item())
        delta_rad = standing_rad - zero_rad
        delta_deg = math.degrees(delta_rad)
        print(
            f"{joint_name:<34} | {zero_rad:>10.6f} | {standing_rad:>12.6f} | "
            f"{delta_rad:>10.6f} | {delta_deg:>10.3f}"
        )


class ThorStartupToStandingRunner:
    def __init__(
        self,
        startup_cfg: ThorStartupToStandingConfig,
        hardware_cfg: RobotInterfaceConfig,
        state_reader: Callable[[], RobotStateSample],
        command_writer: Callable[[RobotCommandMessage], None],
    ) -> None:
        self.cfg = startup_cfg
        self.device = torch.device(startup_cfg.device)
        self.hardware = RobotHardwareInterface(
            cfg=hardware_cfg,
            state_reader=state_reader,
            command_writer=command_writer,
            device=self.device,
        )

        self._joint_names = list(startup_cfg.joint_names)
        self._joint_lower = torch.tensor(
            startup_cfg.joint_lower_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_upper = torch.tensor(
            startup_cfg.joint_upper_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._q_zero = _build_pose_tensor_from_degrees(
            ZERO_POSE_DEG,
            startup_cfg.joint_names,
            pose_name="zero_pose",
            device=self.device,
        )
        self._q_standing = _build_pose_tensor_from_degrees(
            STANDING_TARGETS_DEG,
            startup_cfg.joint_names,
            pose_name="standing_pose",
            device=self.device,
        )

        per_joint_actuator_cfg = build_per_joint_walking_actuator_cfg(startup_cfg.joint_names)
        self._kp = (
            torch.tensor(per_joint_actuator_cfg["stiffness"], dtype=torch.float32, device=self.device)
            * float(startup_cfg.kp_scale)
        ).unsqueeze(0)
        self._kd = (
            torch.tensor(per_joint_actuator_cfg["damping"], dtype=torch.float32, device=self.device)
            * float(startup_cfg.kd_scale)
        ).unsqueeze(0)
        self._effort_limit = (
            torch.tensor(per_joint_actuator_cfg["effort_limit"], dtype=torch.float32, device=self.device)
            * float(startup_cfg.effort_scale)
        ).unsqueeze(0)
        self._tau_ff = torch.zeros((1, len(self._joint_names)), dtype=torch.float32, device=self.device)

        self._step_count = 0

    def print_pose_comparison_table(self) -> None:
        _print_pose_comparison_table(self.cfg.joint_names, self._q_zero, self._q_standing)

    def _check_for_nan(self, tensor: Tensor, tensor_name: str) -> None:
        if torch.isnan(tensor).any():
            raise RuntimeError(f"NaN detected in {tensor_name}")

    def _validate_joint_targets(self, q_des: Tensor) -> None:
        if (q_des < self._joint_lower).any() or (q_des > self._joint_upper).any():
            violating = []
            for joint_index, joint_name in enumerate(self._joint_names):
                q_des_value = float(q_des[0, joint_index].item())
                lower_value = float(self._joint_lower[0, joint_index].item())
                upper_value = float(self._joint_upper[0, joint_index].item())
                if q_des_value < lower_value or q_des_value > upper_value:
                    violating.append(
                        f"{joint_name}: target={q_des_value:.6f} rad, "
                        f"limits=[{lower_value:.6f}, {upper_value:.6f}]"
                    )
            raise RuntimeError(
                "Joint target exceeds policy contract joint limits: " + "; ".join(violating)
            )

    def _build_control_packet(self, q_des: Tensor) -> ControlPacket:
        self._validate_joint_targets(q_des)
        # TODO: ControlPacket currently has no effort-limit field. Keep tau_ff at
        # zero during startup and apply effort scaling here once the hardware
        # command path supports explicit effort limiting.
        _ = self._effort_limit

        return ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des.clone(),
            kp=self._kp.clone(),
            kd=self._kd.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp.clone(),
            kd_gains=self._kd.clone(),
        )

    def send_standing_pose(self) -> None:
        observation_packet = self.hardware.read_observation_packet()
        q_actual = observation_packet.joint_pos.to(self.device, dtype=torch.float32)
        q_standing = self._q_standing.unsqueeze(0).clone()
        self._check_for_nan(q_actual, "q_actual")
        self._check_for_nan(q_standing, "q_des")
        self._validate_joint_targets(q_standing)
        packet = self._build_control_packet(q_standing)
        self.hardware.write_control_packet(packet)

    def _debug_print_step(
        self,
        alpha: float,
        q_actual: Tensor,
        q_des: Tensor,
        joint_vel: Tensor,
        mode: str,
    ) -> None:
        if self._step_count % self.cfg.debug_print_every_n_steps != 0:
            return

        max_position_error_to_standing = torch.max(
            torch.abs(self._q_standing.unsqueeze(0) - q_actual)
        ).item()
        max_joint_velocity = torch.max(torch.abs(joint_vel)).item()
        settled_in_position = max_position_error_to_standing <= self.cfg.position_tolerance_rad
        settled_in_velocity = max_joint_velocity <= self.cfg.velocity_tolerance_rad_s

        print(
            "[THOR STARTUP DEBUG] "
            f"step={self._step_count} | "
            f"mode={mode} | "
            f"alpha={alpha:.3f} | "
            f"max position error to q_standing={max_position_error_to_standing:.6f} rad | "
            f"max joint velocity={max_joint_velocity:.6f} rad/s | "
            f"within position tolerance={settled_in_position} | "
            f"within velocity tolerance={settled_in_velocity}"
        )

    def run(self) -> None:
        self.print_pose_comparison_table()
        print("\nStarting Thor startup ramp toward standing pose. Press Ctrl+C to stop.\n")

        startup_packet = self.hardware.read_observation_packet()
        q_start = startup_packet.joint_pos.to(self.device, dtype=torch.float32).clone()
        self._check_for_nan(q_start, "q_actual")

        period_s = 1.0 / self.cfg.loop_hz
        start_time_s = time.monotonic()
        next_time_s = start_time_s
        interrupted = False

        try:
            while True:
                observation_packet = self.hardware.read_observation_packet()
                q_actual = observation_packet.joint_pos.to(self.device, dtype=torch.float32)
                joint_vel = observation_packet.joint_vel.to(self.device, dtype=torch.float32)
                self._check_for_nan(q_actual, "q_actual")

                elapsed_time_s = time.monotonic() - start_time_s
                alpha = min(max(elapsed_time_s / self.cfg.ramp_time_s, 0.0), 1.0)
                mode = "RAMP" if alpha < 1.0 else "HOLD"

                q_des = q_start + alpha * (self._q_standing.unsqueeze(0) - q_start)
                self._check_for_nan(q_des, "q_des")
                self._validate_joint_targets(q_des)

                max_position_error = torch.max(torch.abs(q_actual - q_des)).item()
                if max_position_error > self.cfg.max_position_error_rad:
                    raise RuntimeError(
                        f"Startup aborted: max abs(q_actual - q_des)={max_position_error:.6f} rad "
                        f"exceeds threshold {self.cfg.max_position_error_rad:.6f} rad"
                    )
                packet = self._build_control_packet(q_des)
                self.hardware.write_control_packet(packet)

                self._step_count += 1
                self._debug_print_step(alpha, q_actual, q_des, joint_vel, mode)

                next_time_s += period_s
                sleep_s = next_time_s - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_time_s = time.monotonic()
        except KeyboardInterrupt:
            interrupted = True
        finally:
            if interrupted and self.cfg.send_standing_pose_on_exit:
                self.send_standing_pose()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move Thor from calibrated zero pose into the standing policy pose."
    )
    parser.add_argument("--device", type=str, default="cpu", help="Torch device for tensor operations.")
    parser.add_argument(
        "--loop-hz",
        type=float,
        default=CONTRACT.policy_loop_hz,
        help="Hardware control loop frequency in Hz.",
    )
    parser.add_argument(
        "--ramp-time-s",
        type=float,
        default=8.0,
        help="Ramp time in seconds for interpolating from the measured pose to standing.",
    )
    parser.add_argument("--kp-scale", type=float, default=0.20, help="Scale factor applied to stiffness gains.")
    parser.add_argument("--kd-scale", type=float, default=1.00, help="Scale factor applied to damping gains.")
    parser.add_argument(
        "--effort-scale",
        type=float,
        default=0.25,
        help="Scale factor applied where the command path supports effort limiting.",
    )
    parser.add_argument(
        "--position-tolerance-rad",
        type=float,
        default=0.05,
        help="Position tolerance used for live standing-settled debug checks.",
    )
    parser.add_argument(
        "--velocity-tolerance-rad-s",
        type=float,
        default=0.10,
        help="Velocity tolerance used for live standing-settled debug checks.",
    )
    parser.add_argument(
        "--max-position-error-rad",
        type=float,
        default=0.75,
        help="Abort if max abs(q_actual - q_des) exceeds this threshold after startup begins.",
    )
    parser.add_argument(
        "--debug-print-every-n-steps",
        type=int,
        default=50,
        help="Print live debug data every N control steps.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    joint_names = CONTRACT.joint_names

    startup_cfg = ThorStartupToStandingConfig(
        joint_names=joint_names,
        joint_lower_rad=CONTRACT.joint_lower_limits_rad,
        joint_upper_rad=CONTRACT.joint_upper_limits_rad,
        loop_hz=args.loop_hz,
        device=args.device,
        ramp_time_s=args.ramp_time_s,
        kp_scale=args.kp_scale,
        kd_scale=args.kd_scale,
        effort_scale=args.effort_scale,
        position_tolerance_rad=args.position_tolerance_rad,
        velocity_tolerance_rad_s=args.velocity_tolerance_rad_s,
        max_position_error_rad=args.max_position_error_rad,
        debug_print_every_n_steps=args.debug_print_every_n_steps,
    )

    hardware_cfg = RobotInterfaceConfig(
        joint_names=joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in joint_names),
        joint_signs=tuple(1.0 for _ in joint_names),
    )

    runner = ThorStartupToStandingRunner(
        startup_cfg=startup_cfg,
        hardware_cfg=hardware_cfg,
        state_reader=ros2_state_reader,
        command_writer=ros2_command_writer,
    )
    try:
        runner.run()
    finally:
        _shutdown_ros2_bridge()


if __name__ == "__main__":
    main()
