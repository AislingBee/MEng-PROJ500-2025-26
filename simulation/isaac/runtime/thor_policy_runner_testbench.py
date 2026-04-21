from __future__ import annotations

import torch
import math

from simulation.isaac.configuration.standing_s2r_policy_contract import (
    CONTRACT,
    build_standing_q,
    get_thor_runner_defaults,
)
from simulation.isaac.rl.interface.robot_hardware_interface import (
    RobotCommandMessage,
    RobotInterfaceConfig,
    RobotStateSample,
)
from hardware.thor.thor_policy_runner import (
    ThorPolicyRunnerConfig,
    ThorStandingPolicyRunner,
)


def rad_to_encoder_counts(q_rad: list[float]) -> list[int]:
    counts = []
    for q in q_rad:
        q_0_2pi = q % (2.0 * math.pi)
        count = int(round((q_0_2pi / (2.0 * math.pi)) * 16384.0))
        count = max(0, min(16383, count))
        counts.append(count)
    return counts


def fake_state_reader() -> RobotStateSample:
    standing_q = build_standing_q(device="cpu")
    encoder_counts = rad_to_encoder_counts(standing_q.tolist())

    return RobotStateSample(
        encoder_counts=encoder_counts,
        joint_vel=[0.0] * CONTRACT.action_dim,
        joint_effort=[0.0] * CONTRACT.action_dim,
        projected_gravity_b=[0.0, 0.0, -1.0],
        imu_gyro_b=[0.0, 0.0, 0.0],
    )


def fake_command_writer(msg: RobotCommandMessage) -> None:
    print("\n--- Command written ---")
    print("joint_names:", msg.joint_names)
    print("q_des:", msg.q_des)
    print("kp:", msg.kp)
    print("kd:", msg.kd)
    print("tau_ff:", msg.tau_ff)


def main() -> None:
    contract_defaults = get_thor_runner_defaults()
    joint_names = contract_defaults["joint_names"]

    runner_cfg = ThorPolicyRunnerConfig(
        policy_path=r"hardware\policy\policy_jit.pt",
        joint_names=joint_names,
        joint_lower_rad=contract_defaults["joint_lower_limits_rad"],
        joint_upper_rad=contract_defaults["joint_upper_limits_rad"],
        device="cpu",
        loop_hz=contract_defaults["loop_hz"],
        command_value=contract_defaults["command_value"],
    )

    hardware_cfg = RobotInterfaceConfig(
        joint_names=joint_names,
        encoder_offsets_rad=tuple(0.0 for _ in joint_names),
        joint_signs=tuple(1.0 for _ in joint_names),
    )

    runner = ThorStandingPolicyRunner(
        runner_cfg=runner_cfg,
        hardware_cfg=hardware_cfg,
        state_reader=fake_state_reader,
        command_writer=fake_command_writer,
    )

    obs = runner.build_observation()
    print("obs shape:", tuple(obs.shape))
    print("obs:", obs)

    packet = runner.step()

    print("\n--- Returned control packet ---")
    print("q_des shape:", tuple(packet.q_des.shape))
    print("q_des min/max:", packet.q_des.min().item(), packet.q_des.max().item())
    print("kp shape:", tuple(packet.kp.shape))
    print("kd shape:", tuple(packet.kd.shape))
    print("tau_ff shape:", tuple(packet.tau_ff.shape))


if __name__ == "__main__":
    main()