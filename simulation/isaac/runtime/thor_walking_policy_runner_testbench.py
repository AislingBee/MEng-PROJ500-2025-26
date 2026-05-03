from __future__ import annotations

import math
import time
from dataclasses import dataclass

import torch

from hardware.thor.thor_policy_runner import DeployablePolicy
from simulation.isaac.configuration.walking_s2r_policy_contract import (
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


def rad_to_encoder_counts(q_rad: list[float]) -> list[int]:
    counts = []
    for q in q_rad:
        q_0_2pi = q % (2.0 * math.pi)
        count = int(round((q_0_2pi / (2.0 * math.pi)) * 16384.0))
        count = max(0, min(16383, count))
        counts.append(count)
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
    pass


@dataclass
class ThorWalkingPolicyRunnerConfig:
    policy_path: str = "hardware/policy/walking_policy_200.pt"
    device: str = "cpu"
    loop_hz: float = CONTRACT.policy_loop_hz
    command_value: float = CONTRACT.default_command_value
    max_command_value: float = 0.50
    debug_print_every_n_steps: int = 1

    def __post_init__(self) -> None:
        if self.loop_hz <= 0.0:
            raise ValueError("loop_hz must be positive")
        if self.max_command_value < 0.0:
            raise ValueError("max_command_value must be non-negative")
        if self.debug_print_every_n_steps <= 0:
            raise ValueError("debug_print_every_n_steps must be positive")


class ThorWalkingPolicyRunnerTestHarness:
    def __init__(
        self,
        runner_cfg: ThorWalkingPolicyRunnerConfig,
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
        self.policy = DeployablePolicy(runner_cfg.policy_path, device=self.device)

        self._joint_names = list(CONTRACT.joint_names)
        self._standing_q = build_standing_q(device=self.device)
        self._action_scale = torch.tensor(
            CONTRACT.action_scale, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_lower = torch.tensor(
            CONTRACT.joint_lower_limits_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._joint_upper = torch.tensor(
            CONTRACT.joint_upper_limits_rad, dtype=torch.float32, device=self.device
        ).unsqueeze(0)
        self._commands = torch.zeros((1, 1), dtype=torch.float32, device=self.device)
        self._last_actions = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)
        self._joint_pos_targets = self._standing_q.unsqueeze(0).clone()
        self._tau_ff = torch.zeros((1, CONTRACT.action_dim), dtype=torch.float32, device=self.device)

        kp_fixed, kd_fixed = build_fixed_gains(device=self.device)
        self._kp_fixed = kp_fixed.unsqueeze(0)
        self._kd_fixed = kd_fixed.unsqueeze(0)
        self._kp_gains = torch.full((1, CONTRACT.action_dim), 30.0, dtype=torch.float32, device=self.device)
        self._kd_gains = torch.full((1, CONTRACT.action_dim), 2.0, dtype=torch.float32, device=self.device)

        self._step_count = 0
        self.set_command_value(runner_cfg.command_value)

    def set_command_value(self, command_value: float) -> None:
        clamped_value = min(max(float(command_value), 0.0), self.cfg.max_command_value)
        self._commands[0, 0] = clamped_value

    def _get_phase_clock(self) -> tuple[Tensor, Tensor]:
        phase = (
            self._step_count
            / self.cfg.loop_hz
            * CONTRACT.default_gait_frequency_hz
        ) % 1.0
        phase_angle = 2.0 * math.pi * phase
        phase_sin = torch.tensor([[math.sin(phase_angle)]], dtype=torch.float32, device=self.device)
        phase_cos = torch.tensor([[math.cos(phase_angle)]], dtype=torch.float32, device=self.device)
        return phase_sin, phase_cos

    def build_observation(self, observation_packet: ObservationPacket) -> Tensor:
        q_rel = observation_packet.joint_pos - self._standing_q.unsqueeze(0)
        q_target_err = self._joint_pos_targets - observation_packet.joint_pos
        phase_sin, phase_cos = self._get_phase_clock()

        fields = (
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
        )
        obs = torch.cat(fields, dim=-1).to(self.device, dtype=torch.float32)
        expected_shape = (1, CONTRACT.obs_dim)
        if obs.shape != expected_shape:
            raise RuntimeError(f"Expected walking observation shape {expected_shape}, got {tuple(obs.shape)}")
        if torch.isnan(obs).any():
            raise RuntimeError("NaN detected in walking observation")
        return obs

    def generate_control_packet(self, actions: Tensor) -> ControlPacket:
        actions = torch.clamp(actions, -1.0, 1.0)
        q_des = self._standing_q.unsqueeze(0) + self._action_scale * actions
        q_des = torch.max(torch.min(q_des, self._joint_upper), self._joint_lower)
        self._joint_pos_targets = q_des.detach().clone()
        self._last_actions = actions.detach().clone()

        return ControlPacket(
            joint_names=self._joint_names,
            q_des=q_des.clone(),
            kp=self._kp_fixed.clone(),
            kd=self._kd_fixed.clone(),
            tau_ff=self._tau_ff.clone(),
            kp_gains=self._kp_gains.clone(),
            kd_gains=self._kd_gains.clone(),
        )

    def step(self) -> ControlPacket:
        observation_packet = self.hardware.read_observation_packet()
        obs = self.build_observation(observation_packet)
        actions = self.policy.act(obs)
        packet = self.generate_control_packet(actions)
        self.hardware.write_control_packet(packet)
        self._step_count += 1
        self._debug_print_step(obs, actions, packet)
        return packet

    def _debug_print_step(self, obs: Tensor, actions: Tensor, packet: ControlPacket) -> None:
        if self._step_count % self.cfg.debug_print_every_n_steps != 0:
            return

        print("\n" + "=" * 90)
        print(f"[THOR WALKING DEBUG] step={self._step_count}")
        print("-" * 90)
        print("obs shape:", tuple(obs.shape))
        print("obs full:", obs.detach().cpu())
        print("q_rel:", obs[:, 0:12])
        print("qd:", obs[:, 12:24])
        print("q_target_err:", obs[:, 24:36])
        print("joint_effort:", obs[:, 36:48])
        print("projected_gravity_b:", obs[:, 48:51])
        print("imu_gyro_b:", obs[:, 51:54])
        print("command:", obs[:, 54:55])
        print("phase_sin:", obs[:, 55:56])
        print("phase_cos:", obs[:, 56:57])
        print("foot_pos_b:", obs[:, 57:63])
        print("last_actions:", obs[:, 63:75])
        print("actions:", actions.detach().cpu())
        print("actions min/max:", actions.min().item(), actions.max().item())
        print("q_des min/max:", packet.q_des.min().item(), packet.q_des.max().item())
        print("joint order:")
        q_des = packet.q_des.detach().cpu()
        for i, name in enumerate(packet.joint_names):
            print(f"{i:02d} {name:40s} q_des={q_des[0, i]:+.5f} rad")
        print("=" * 90)


def main() -> None:
    runner_cfg = ThorWalkingPolicyRunnerConfig()
    hardware_cfg = RobotInterfaceConfig(
        joint_names=CONTRACT.joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in CONTRACT.joint_names),
        joint_signs=tuple(1.0 for _ in CONTRACT.joint_names),
    )
    runner = ThorWalkingPolicyRunnerTestHarness(
        runner_cfg=runner_cfg,
        hardware_cfg=hardware_cfg,
    )

    print("\n--- WALKING DEBUG LOOP TEST ---")
    for _ in range(10):
        runner.step()
        time.sleep(1.0 / runner_cfg.loop_hz)


if __name__ == "__main__":
    main()
