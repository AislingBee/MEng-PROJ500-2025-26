from __future__ import annotations

import argparse
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import torch

# Select policy contract.
# Use exactly one active import.
# Standing and walking use the same hardware interface.
# The selected contract defines the policy observation layout.
# DONT NOT DELETE THIS COMMENTED PART
# from simulation.isaac.configuration.walking_s2r_policy_contract import (
#     CONTRACT,
#     build_fixed_gains,
#     build_standing_q,
# )

from simulation.isaac.configuration.hardware_motor_direction_config import (
    joint_feedback_tuple,
    motor_direction_tuple,
)

# from simulation.isaac.configuration.stand_smooth_s2r_policy_contract import (
#     CONTRACT,
#     build_fixed_gains,
#     build_standing_q,
# )

from simulation.isaac.configuration.stand_smooth_s2r_policy_contract import (
    CONTRACT,
    build_fixed_gains,
    build_standing_q,
)



from simulation.isaac.rl.interface.hardware_interface import ControlPacket, ObservationPacket
from simulation.isaac.rl.interface.robot_hardware_interface import (
    RobotHardwareInterface,
    RobotInterfaceConfig, RobotStateSample, RobotCommandMessage,
)

from hardware.thor.thor_policy_runner import (
    DeployableObsNormalizer,
    DeployablePolicy,
    _shutdown_ros2_bridge,
    ros2_command_writer,
    ros2_state_reader,
)


Tensor = torch.Tensor
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_POLICY_PATH = (
    "hardware/policy/standing_policy.pt"
    if CONTRACT.obs_dim == 55 and CONTRACT.default_command_value == 0.0
    else "hardware/policy/standing_policy.pt"
)

MODE_STARTUP_RAMP = "STARTUP_RAMP"
MODE_STANDING_HOLD = "STANDING_HOLD"
MODE_POLICY = "POLICY"
MODE_EXIT = "EXIT"


@dataclass
class ThorStartupThenPolicyRunnerConfig:
    policy_path: str
    obs_normalizer_path: str | None = None
    joint_names: tuple[str, ...] = CONTRACT.joint_names
    joint_lower_rad: tuple[float, ...] = CONTRACT.joint_lower_limits_rad
    joint_upper_rad: tuple[float, ...] = CONTRACT.joint_upper_limits_rad
    action_scale: tuple[float, ...] = CONTRACT.action_scale
    command_value: float = CONTRACT.default_command_value
    max_command_value: float = 0.50
    device: str = "cpu"
    loop_hz: float = CONTRACT.policy_loop_hz
    action_delay_steps: int = 2
    ramp_time_s: float = 8.0
    startup_kp_scale: float = 0.20
    startup_kd_scale: float = 1.00
    position_tolerance_rad: float = 0.05
    velocity_tolerance_rad_s: float = 0.10
    max_position_error_rad: float = 0.75
    send_standing_pose_on_exit: bool = True
    debug_print_every_n_steps: int = 200  # 1 s at 200 Hz

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
        if len(self.action_scale) != joint_count:
            raise ValueError("action_scale length must match joint_names")
        if tuple(self.joint_lower_rad) != CONTRACT.joint_lower_limits_rad:
            raise ValueError("joint_lower_rad must match the selected S2R policy contract")
        if tuple(self.joint_upper_rad) != CONTRACT.joint_upper_limits_rad:
            raise ValueError("joint_upper_rad must match the selected S2R policy contract")
        if self.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")
        if self.ramp_time_s <= 0.0:
            raise ValueError("ramp_time_s must be positive")
        if self.startup_kp_scale < 0.0:
            raise ValueError("startup_kp_scale must be non-negative")
        if self.startup_kd_scale < 0.0:
            raise ValueError("startup_kd_scale must be non-negative")
        if self.position_tolerance_rad < 0.0:
            raise ValueError("position_tolerance_rad must be non-negative")
        if self.velocity_tolerance_rad_s < 0.0:
            raise ValueError("velocity_tolerance_rad_s must be non-negative")
        if self.max_position_error_rad <= 0.0:
            raise ValueError("max_position_error_rad must be positive")
        if self.max_command_value < 0.0:
            raise ValueError("max_command_value must be non-negative")
        if self.action_delay_steps < 0:
            raise ValueError("action_delay_steps must be non-negative")
        if self.debug_print_every_n_steps <= 0:
            raise ValueError("debug_print_every_n_steps must be positive")


class ThorStartupThenPolicyRunner:
    def __init__(
        self,
        runner_cfg: ThorStartupThenPolicyRunnerConfig,
        hardware_cfg: RobotInterfaceConfig,
        state_reader: Callable[[], RobotStateSample],
        command_writer: Callable[[RobotCommandMessage], None],
    ) -> None:
        self.cfg = runner_cfg
        self.device = torch.device(runner_cfg.device)

        self.hardware = RobotHardwareInterface(
            cfg=hardware_cfg,
            state_reader=state_reader,
            command_writer=command_writer,
            device=self.device,
        )
        self._policy_has_embedded_obs_normalizer = self._policy_embeds_obs_normalizer(
            runner_cfg.policy_path
        )
        self.obs_normalizer = self._load_obs_normalizer()
        self.policy = DeployablePolicy(
            runner_cfg.policy_path,
            device=self.device,
            obs_normalizer=self.obs_normalizer,
        )

        self._joint_names = list(runner_cfg.joint_names)
        self._standing_q = build_standing_q(device=self.device)
        self._action_scale = torch.tensor(
            runner_cfg.action_scale, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_lower = torch.tensor(
            runner_cfg.joint_lower_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_upper = torch.tensor(
            runner_cfg.joint_upper_rad, dtype=torch.float32, device=self.device
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
        self._kp_policy = kp_fixed.unsqueeze(0)
        self._kd_policy = kd_fixed.unsqueeze(0)
        self._kp_startup = self._kp_policy * float(runner_cfg.startup_kp_scale)
        self._kd_startup = self._kd_policy * float(runner_cfg.startup_kd_scale)

        # Per-joint MIT gains follow the policy runner during policy control.
        self._kp_gains_policy = torch.full(
            (1, CONTRACT.action_dim), 30.0, dtype=torch.float32, device=self.device
        )
        self._kd_gains_policy = torch.full(
            (1, CONTRACT.action_dim), 2.0, dtype=torch.float32, device=self.device
        )
        self._kp_gains_startup = self._kp_startup.clone()
        self._kd_gains_startup = self._kd_startup.clone()
        self.set_command_value(runner_cfg.command_value)

        self._mode = MODE_STARTUP_RAMP
        self._step_count = 0
        self._q_start: Tensor | None = None
        self._ramp_start_time_s: float | None = None

        self._request_lock = threading.Lock()
        self._pending_hold = False
        self._pending_policy = False
        self._pending_exit = False
        self._pending_command_value: float | None = None
        self._last_raw_actions_debug: Tensor | None = None
        self._last_clamped_actions_debug: Tensor | None = None
        self._last_applied_actions_debug: Tensor | None = None
        self._last_action_saturation_pct: float = 0.0

        self._print_startup_summary()

    def set_command_value(self, command_value: float) -> None:
        clamped_value = min(max(float(command_value), 0.0), self.cfg.max_command_value)
        self._commands[0, 0] = clamped_value

    def _policy_embeds_obs_normalizer(self, policy_path: str) -> bool:
        try:
            policy = torch.jit.load(str(Path(policy_path).expanduser().resolve()), map_location="cpu")
        except Exception:
            return False
        actor = getattr(policy, "actor", None)
        return actor is not None and hasattr(actor, "obs_normalizer")

    def _resolve_obs_normalizer_path(self) -> str:
        if self.cfg.obs_normalizer_path is not None:
            return self.cfg.obs_normalizer_path
        normalizer_name = getattr(CONTRACT, "obs_normalizer_artifact_name", "obs_normalizer.pt")
        return str(Path(self.cfg.policy_path).expanduser().resolve().with_name(normalizer_name))

    def _load_obs_normalizer(self) -> DeployableObsNormalizer | None:
        if not getattr(CONTRACT, "use_obs_normalization", False):
            return None
        if self._policy_has_embedded_obs_normalizer:
            return None

        normalizer_path = self._resolve_obs_normalizer_path()
        if not Path(normalizer_path).is_file():
            if getattr(CONTRACT, "obs_normalizer_required", False):
                raise RuntimeError(
                    "Selected policy contract requires observation normalization, "
                    f"but normalizer artifact was not found at: {normalizer_path}"
                )
            return None
        return DeployableObsNormalizer(normalizer_path, device=self.device)

    def _print_startup_summary(self) -> None:
        normalizer_path = "None"
        if self._policy_has_embedded_obs_normalizer:
            normalizer_path = "embedded-in-policy"
        elif self.obs_normalizer is not None:
            normalizer_path = str(self.obs_normalizer.path)
        print(
            "[THOR STARTUP+POLICY] "
            f"policy_path={self.policy.path} | "
            f"use_obs_normalization={getattr(CONTRACT, 'use_obs_normalization', False)} | "
            f"normalizer_path={normalizer_path} | "
            f"observation_dim={CONTRACT.obs_dim} | "
            f"action_dim={CONTRACT.action_dim}"
        )

    def set_stand_mode(self) -> None:
        self.set_command_value(0.0)

    def set_walk_mode(self) -> None:
        self.set_command_value(0.05)

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

    def _build_startup_control_packet(self, q_des: Tensor) -> ControlPacket:
        self._check_for_nan(q_des, "q_des")
        # No joint-limit validation here: the ramp starts from the real motor
        # positions which may be outside policy limits. Validation runs once the
        # robot reaches STANDING_HOLD.
        return ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des.clone(),
            kp=self._kp_startup.clone(),
            kd=self._kd_startup.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp_gains_startup.clone(),
            kd_gains=self._kd_gains_startup.clone(),
        )

    def _build_standing_policy_packet(self) -> ControlPacket:
        q_des = self._standing_q.unsqueeze(0).clone()
        self._check_for_nan(q_des, "q_des")
        self._validate_joint_targets(q_des)
        return ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des,
            kp=self._kp_policy.clone(),
            kd=self._kd_policy.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp_gains_policy.clone(),
            kd_gains=self._kd_gains_policy.clone(),
        )

    def _get_phase_clock(self) -> tuple[Tensor, Tensor]:
        gait_frequency_hz = getattr(CONTRACT, "default_gait_frequency_hz", None)
        if gait_frequency_hz is None:
            raise RuntimeError("Selected policy contract does not define a walking gait phase clock")

        phase = (self._step_count / self.cfg.loop_hz * gait_frequency_hz) % 1.0
        phase_angle = 2.0 * math.pi * phase
        phase_sin = torch.tensor([[math.sin(phase_angle)]], dtype=torch.float32, device=self.device)
        phase_cos = torch.tensor([[math.cos(phase_angle)]], dtype=torch.float32, device=self.device)
        return phase_sin, phase_cos

    def _build_observation_fields(
        self, observation_packet: ObservationPacket
    ) -> tuple[tuple[str, Tensor], ...]:
        q_rel = observation_packet.joint_pos - self._standing_q.unsqueeze(0)
        standing_base_fields = (
            ("q_rel", q_rel),
            ("joint_vel", observation_packet.joint_vel),
        )
        tail_fields = (
            ("joint_effort", observation_packet.joint_effort),
            ("projected_gravity_b", observation_packet.projected_gravity_b),
            ("imu_gyro_b", observation_packet.imu_gyro_b),
            ("command", self._commands),
            ("last_actions", self._last_actions),
        )
        standing_fields = standing_base_fields + tail_fields
        if sum(field.shape[1] for _, field in standing_fields) == CONTRACT.obs_dim:
            return standing_fields

        q_target_err = self._joint_pos_targets - observation_packet.joint_pos
        phase_sin, phase_cos = self._get_phase_clock()
        foot_pos_b = observation_packet.foot_pos_b
        if foot_pos_b.shape[-1] != 6:
            raise RuntimeError(f"FK foot_pos_b must have trailing dim 6, got {foot_pos_b.shape[-1]}")
        walking_fields = (
            ("q_rel", q_rel),
            ("qd", observation_packet.joint_vel),
            ("q_target_err", q_target_err),
            ("joint_effort", observation_packet.joint_effort),
            ("projected_gravity_b", observation_packet.projected_gravity_b),
            ("imu_gyro_b", observation_packet.imu_gyro_b),
            ("command", self._commands),
            ("phase_sin", phase_sin),
            ("phase_cos", phase_cos),
            ("foot_pos_b", foot_pos_b),
            ("last_actions", self._last_actions),
        )

        if sum(field.shape[1] for _, field in walking_fields) == CONTRACT.obs_dim:
            return walking_fields

        raise RuntimeError(f"Selected policy contract has unsupported observation dim {CONTRACT.obs_dim}")

    def build_observation(self, observation_packet: ObservationPacket) -> Tensor:
        fields = self._build_observation_fields(observation_packet)
        obs = torch.cat(tuple(field for _, field in fields), dim=-1)
        obs = obs.to(self.device, dtype=torch.float32)
        expected_shape = (1, CONTRACT.obs_dim)
        if obs.shape != expected_shape:
            raise RuntimeError(f"Expected observation shape {expected_shape}, got {tuple(obs.shape)}")
        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in observations")
        return obs

    def _process_policy_actions(self, raw_actions: Tensor) -> Tensor:
        self._last_actions[:] = self._actions
        clamped_actions = torch.clamp(raw_actions, -1.0, 1.0)
        saturated = raw_actions != clamped_actions
        self._last_action_saturation_pct = 100.0 * saturated.float().mean().item()

        self._action_buffer = torch.roll(self._action_buffer, shifts=1, dims=1)
        self._action_buffer[:, 0, :] = clamped_actions
        delayed_actions = self._action_buffer[:, self.cfg.action_delay_steps, :]
        self._actions[:] = delayed_actions

        self._last_raw_actions_debug = raw_actions.detach().clone()
        self._last_clamped_actions_debug = clamped_actions.detach().clone()
        self._last_applied_actions_debug = delayed_actions.detach().clone()
        return delayed_actions

    def _generate_policy_control_packet(self, actions: Tensor) -> ControlPacket:
        applied_actions = self._process_policy_actions(actions)
        q_des = self._standing_q.unsqueeze(0) + self._action_scale * applied_actions
        q_des = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)
        self._check_for_nan(q_des, "q_des")
        self._validate_joint_targets(q_des)
        self._joint_pos_targets = q_des.detach().clone()

        return ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des.clone(),
            kp=self._kp_policy.clone(),
            kd=self._kd_policy.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp_gains_policy.clone(),
            kd_gains=self._kd_gains_policy.clone(),
        )

    def _standing_metrics(self, q_actual: Tensor, joint_vel: Tensor) -> tuple[float, float]:
        max_standing_error = torch.max(torch.abs(q_actual - self._standing_q.unsqueeze(0))).item()
        max_joint_velocity = torch.max(torch.abs(joint_vel)).item()
        return max_standing_error, max_joint_velocity

    def _max_position_error_details(
        self, q_actual: Tensor, q_des: Tensor
    ) -> tuple[float, int, str, float, float]:
        position_errors = torch.abs(q_actual - q_des)
        flat_joint_index = int(torch.argmax(position_errors).item())
        joint_index = flat_joint_index % len(self._joint_names)
        motor_id = joint_index + 1
        return (
            float(position_errors.flatten()[flat_joint_index].item()),
            motor_id,
            self._joint_names[joint_index],
            float(q_actual.flatten()[flat_joint_index].item()),
            float(q_des.flatten()[flat_joint_index].item()),
        )

    def _request_hold(self) -> None:
        with self._request_lock:
            self._pending_hold = True

    def _request_policy(self) -> None:
        with self._request_lock:
            self._pending_policy = True

    def _request_exit(self) -> None:
        with self._request_lock:
            self._pending_exit = True

    def _request_command_value(self, command_value: float) -> None:
        with self._request_lock:
            self._pending_command_value = float(command_value)

    def _process_pending_requests(self, q_actual: Tensor, joint_vel: Tensor) -> None:
        with self._request_lock:
            pending_exit = self._pending_exit
            pending_hold = self._pending_hold
            pending_policy = self._pending_policy
            pending_command_value = self._pending_command_value
            self._pending_exit = False
            self._pending_hold = False
            self._pending_policy = False
            self._pending_command_value = None

        if pending_exit:
            self._mode = MODE_EXIT
            return

        if pending_hold:
            self._mode = MODE_STANDING_HOLD
            self.set_stand_mode()
            print("Keyboard: switching to STANDING_HOLD.")

        if pending_policy:
            if self._mode != MODE_STANDING_HOLD:
                print(
                    f"Keyboard: policy handover refused because current mode is {self._mode}. "
                    "Wait for STANDING_HOLD or press h first."
                )
            else:
                max_standing_error, max_joint_velocity = self._standing_metrics(q_actual, joint_vel)
                if max_standing_error >= self.cfg.position_tolerance_rad:
                    print(
                        "Keyboard: policy handover refused because "
                        f"max abs(q_actual - q_standing)={max_standing_error:.6f} rad "
                        f">= position_tolerance_rad={self.cfg.position_tolerance_rad:.6f} rad."
                    )
                elif max_joint_velocity >= self.cfg.velocity_tolerance_rad_s:
                    print(
                        "Keyboard: policy handover refused because "
                        f"max abs(joint_vel)={max_joint_velocity:.6f} rad/s "
                        f">= velocity_tolerance_rad_s={self.cfg.velocity_tolerance_rad_s:.6f} rad/s."
                    )
                else:
                    self._mode = MODE_POLICY
                    self._joint_pos_targets = self._standing_q.unsqueeze(0).clone()
                    self._actions.zero_()
                    self._last_actions.zero_()
                    self._action_buffer.zero_()
                    print("Keyboard: switching to POLICY.")

        if pending_command_value is not None:
            if self._mode != MODE_POLICY:
                print(
                    "Keyboard: command value change ignored because command keys are only active in POLICY mode."
                )
            else:
                self.set_command_value(pending_command_value)
                print(f"Keyboard: command_value set to {float(self._commands[0, 0].item()):.3f}.")

    def _debug_print_step(self, mode: str, alpha: float, q_actual: Tensor, joint_vel: Tensor) -> None:
        if self._step_count % self.cfg.debug_print_every_n_steps != 0:
            return

        max_standing_error, max_joint_velocity = self._standing_metrics(q_actual, joint_vel)

        # Compact per-joint command summary (q_des and gains)
        if mode == "STARTUP_RAMP":
            kp_now = self._kp_startup
            kd_now = self._kd_startup
            q_des_now = self._q_start if self._q_start is not None else q_actual
        else:
            kp_now = self._kp_policy
            kd_now = self._kd_policy
            q_des_now = self._standing_q

        q_des_str   = " ".join(f"{v:+.3f}" for v in q_des_now.flatten().tolist())
        q_actual_str = " ".join(f"{v:+.3f}" for v in q_actual.flatten().tolist())
        kp_str      = " ".join(f"{v:.1f}" for v in kp_now.flatten().tolist())
        kd_str      = " ".join(f"{v:.2f}" for v in kd_now.flatten().tolist())

        action_debug = ""
        if (
            self._last_raw_actions_debug is not None
            and self._last_clamped_actions_debug is not None
            and self._last_applied_actions_debug is not None
        ):
            action_debug = (
                f"\n  raw_actions        : {self._last_raw_actions_debug.detach().cpu()}"
                f"\n  clamped_actions    : {self._last_clamped_actions_debug.detach().cpu()}"
                f"\n  applied_actions    : {self._last_applied_actions_debug.detach().cpu()}"
                f"\n  action_saturation  : {self._last_action_saturation_pct:.2f}%"
            )
        print(
            f"\n[THOR DEBUG] step={self._step_count}  mode={mode}  alpha={alpha:.3f}"
            f"\n  q_des    [rad] : {q_des_str}"
            f"\n  q_actual [rad] : {q_actual_str}"
            f"\n  kp       [N/r] : {kp_str}"
            f"\n  kd     [Ns/r]  : {kd_str}"
            f"\n  max pos error  : {max_standing_error:.4f} rad"
            f"\n  max velocity   : {max_joint_velocity:.4f} rad/s"
            f"\n  command value  : {float(self._commands[0, 0].item()):.3f}"
            f"{action_debug}",
            flush=True,
        )

    def send_standing_pose_once(self) -> None:
        if self._mode == MODE_POLICY:
            packet = self._build_standing_policy_packet()
        else:
            packet = self._build_startup_control_packet(self._standing_q.unsqueeze(0).clone())
        self.hardware.write_control_packet(packet)

    def _send_zero_torque_hold(self, q_actual: Tensor) -> None:
        """Send a single zero-gain packet at current joint positions to make motors compliant."""
        zero_kp = torch.zeros_like(self._kp_startup)
        zero_kd = torch.zeros_like(self._kd_startup)
        packet = ControlPacket(
            joint_names=self._joint_names,
            q_des=q_actual.clone(),
            kp=zero_kp,
            kd=zero_kd,
            tau_ff=torch.zeros_like(self._tau_ff),
            kp_gains=zero_kp.clone(),
            kd_gains=zero_kd.clone(),
        )
        self.hardware.write_control_packet(packet)

    def run(self, stop_event: threading.Event | None = None) -> None:
        print("Thor startup + policy runner keyboard commands: Enter/p=start policy, h=hold, x=exit")

        startup_packet = self.hardware.read_observation_packet()
        q_start = startup_packet.joint_pos.to(self.device, dtype=torch.float32).clone()
        self._check_for_nan(q_start, "q_actual")
        self._q_start = q_start
        self._ramp_start_time_s = time.monotonic()

        period_s = 1.0 / self.cfg.loop_hz
        next_t = time.monotonic()
        should_send_exit_pose = True
        _abort_q_actual: Tensor | None = None

        try:
            while stop_event is None or not stop_event.is_set():
                observation_packet = self.hardware.read_observation_packet()
                q_actual = observation_packet.joint_pos.to(self.device, dtype=torch.float32)
                joint_vel = observation_packet.joint_vel.to(self.device, dtype=torch.float32)
                self._check_for_nan(q_actual, "q_actual")
                self._check_for_nan(joint_vel, "joint_vel")

                self._process_pending_requests(q_actual, joint_vel)
                if self._mode == MODE_EXIT:
                    break

                alpha = 1.0
                if self._mode == MODE_STARTUP_RAMP:
                    if self._q_start is None or self._ramp_start_time_s is None:
                        raise RuntimeError("Startup ramp state was not initialized")
                    elapsed_time_s = time.monotonic() - self._ramp_start_time_s
                    alpha = min(max(elapsed_time_s / self.cfg.ramp_time_s, 0.0), 1.0)
                    q_des = self._q_start + alpha * (self._standing_q.unsqueeze(0) - self._q_start)
                    self._check_for_nan(q_des, "q_des")
                    # Intentionally no _validate_joint_targets here: the ramp
                    # starts from the real motor positions which may be outside
                    # policy limits. Validation is deferred to STANDING_HOLD.

                    (
                        max_position_error,
                        worst_motor_id,
                        worst_joint_name,
                        worst_q_actual,
                        worst_q_des,
                    ) = self._max_position_error_details(q_actual, q_des)
                    if max_position_error > self.cfg.max_position_error_rad:
                        _abort_q_actual = q_actual.clone()
                        self._send_zero_torque_hold(q_actual)
                        raise RuntimeError(
                            f"Startup aborted: max abs(q_actual - q_des)={max_position_error:.6f} rad "
                            f"at motor CAN ID {worst_motor_id} / joint {worst_joint_name} "
                            f"(q_actual={worst_q_actual:+.6f} rad, q_des={worst_q_des:+.6f} rad) "
                            f"exceeds threshold {self.cfg.max_position_error_rad:.6f} rad"
                        )

                    packet = self._build_startup_control_packet(q_des)
                    self.hardware.write_control_packet(packet)

                    if alpha >= 1.0:
                        self._mode = MODE_STANDING_HOLD
                        self.set_stand_mode()
                        print("Startup ramp complete. Entering STANDING_HOLD. Press Enter (or 'p') to start policy.")

                elif self._mode == MODE_STANDING_HOLD:
                    q_des = self._standing_q.unsqueeze(0).clone()
                    self._check_for_nan(q_des, "q_des")
                    self._validate_joint_targets(q_des)

                    (
                        max_position_error,
                        worst_motor_id,
                        worst_joint_name,
                        worst_q_actual,
                        worst_q_des,
                    ) = self._max_position_error_details(q_actual, q_des)
                    if max_position_error > self.cfg.max_position_error_rad:
                        _abort_q_actual = q_actual.clone()
                        self._send_zero_torque_hold(q_actual)
                        raise RuntimeError(
                            f"Startup hold aborted: max abs(q_actual - q_des)={max_position_error:.6f} rad "
                            f"at motor CAN ID {worst_motor_id} / joint {worst_joint_name} "
                            f"(q_actual={worst_q_actual:+.6f} rad, q_des={worst_q_des:+.6f} rad) "
                            f"exceeds threshold {self.cfg.max_position_error_rad:.6f} rad"
                        )

                    packet = self._build_startup_control_packet(q_des)
                    self.hardware.write_control_packet(packet)

                elif self._mode == MODE_POLICY:
                    obs = self.build_observation(observation_packet)
                    actions = self.policy.act(obs)
                    packet = self._generate_policy_control_packet(actions)
                    self.hardware.write_control_packet(packet)

                else:
                    raise RuntimeError(f"Unsupported mode: {self._mode}")

                self._step_count += 1
                self._debug_print_step(self._mode, alpha, q_actual, joint_vel)

                next_t += period_s
                sleep_s = next_t - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_t = time.monotonic()
        except KeyboardInterrupt:
            self._mode = MODE_EXIT
        finally:
            if should_send_exit_pose and self.cfg.send_standing_pose_on_exit:
                try:
                    if _abort_q_actual is not None:
                        # Already sent zero-torque before the raise; send once more in case
                        # the bridge flushed the TX queue during exception handling.
                        self._send_zero_torque_hold(_abort_q_actual)
                    else:
                        self.send_standing_pose_once()
                except Exception as exc:
                    print(f"[THOR] Warning: could not send exit pose (ROS2 context may be gone): {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run Thor startup ramp, hold standing, then hand over to policy control."
    )
    parser.add_argument(
        "--policy-path",
        type=str,
        default=DEFAULT_POLICY_PATH,
        help="Path to the deployable policy module.",
    )
    parser.add_argument(
        "--obs-normalizer",
        type=str,
        default=None,
        help="Optional path to obs_normalizer.pt. Defaults to a sibling of --policy-path.",
    )
    parser.add_argument("--device", type=str, default="cpu", help="Torch device for tensor operations.")
    parser.add_argument(
        "--loop-hz",
        type=float,
        default=CONTRACT.policy_loop_hz,
        help="Hardware control loop frequency in Hz.",
    )
    parser.add_argument(
        "--action-delay-steps",
        type=int,
        default=2,
        help="Number of policy steps to delay clamped actions before q_des generation.",
    )
    parser.add_argument(
        "--ramp-time-s",
        type=float,
        default=8.0,
        help="Ramp time in seconds for interpolating from the measured pose to standing.",
    )
    parser.add_argument(
        "--startup-kp-scale",
        type=float,
        default=0.20,
        help="Scale factor applied to policy stiffness gains during startup and hold.",
    )
    parser.add_argument(
        "--startup-kd-scale",
        type=float,
        default=1.00,
        help="Scale factor applied to policy damping gains during startup and hold.",
    )
    parser.add_argument(
        "--position-tolerance-rad",
        type=float,
        default=0.05,
        help="Maximum standing pose error allowed before policy handover.",
    )
    parser.add_argument(
        "--velocity-tolerance-rad-s",
        type=float,
        default=0.10,
        help="Maximum joint velocity allowed before policy handover.",
    )
    parser.add_argument(
        "--max-position-error-rad",
        type=float,
        default=0.75,
        help="Abort if max abs(q_actual - q_des) exceeds this threshold during startup or hold.",
    )
    parser.add_argument(
        "--debug-print-every-n-steps",
        type=int,
        default=50,
        help="Print mode and standing-status debug data every N control steps.",
    )
    return parser.parse_args()


def _resolve_repo_path(path_value: str | None) -> str | None:
    if path_value is None:
        return None
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    return str(path.resolve())


def main() -> None:
    args = parse_args()
    joint_names = CONTRACT.joint_names

    runner_cfg = ThorStartupThenPolicyRunnerConfig(
        policy_path=_resolve_repo_path(args.policy_path),
        obs_normalizer_path=_resolve_repo_path(args.obs_normalizer),
        joint_names=joint_names,
        joint_lower_rad=CONTRACT.joint_lower_limits_rad,
        joint_upper_rad=CONTRACT.joint_upper_limits_rad,
        action_scale=CONTRACT.action_scale,
        command_value=CONTRACT.default_command_value,
        device=args.device,
        loop_hz=args.loop_hz,
        action_delay_steps=args.action_delay_steps,
        ramp_time_s=args.ramp_time_s,
        startup_kp_scale=args.startup_kp_scale,
        startup_kd_scale=args.startup_kd_scale,
        position_tolerance_rad=args.position_tolerance_rad,
        velocity_tolerance_rad_s=args.velocity_tolerance_rad_s,
        max_position_error_rad=args.max_position_error_rad,
        debug_print_every_n_steps=args.debug_print_every_n_steps,
    )

    hardware_cfg = RobotInterfaceConfig(
        joint_names=joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in joint_names),
        joint_signs=joint_feedback_tuple(joint_names),
        motor_direction_signs=motor_direction_tuple(joint_names),
    )

    runner = ThorStartupThenPolicyRunner(
        runner_cfg=runner_cfg,
        hardware_cfg=hardware_cfg,
        state_reader=ros2_state_reader,
        command_writer=ros2_command_writer,
    )
    stop_event = threading.Event()

    def keyboard_loop() -> None:
        while not stop_event.is_set():
            try:
                key = input().strip().lower()
            except EOFError:
                break

            if key == "h":
                runner._request_hold()
            elif key in ("", "p"):
                runner._request_policy()
            # elif key == "s":
            #     runner._request_command_value(0.0)
            # elif key == "w":
            #     runner._request_command_value(0.05)
            elif key == "x":
                runner._request_exit()
                stop_event.set()
                break

    keyboard_thread = threading.Thread(target=keyboard_loop, daemon=True)
    keyboard_thread.start()
    try:
        runner.run(stop_event=stop_event)
    finally:
        _shutdown_ros2_bridge()


if __name__ == "__main__":
    main()
