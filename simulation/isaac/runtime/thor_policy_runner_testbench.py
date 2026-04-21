from __future__ import annotations

import torch
import math
import time

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


TEST_CASE = {
    "name": "upright",
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

    encoder_counts = rad_to_encoder_counts(q.tolist())

    return RobotStateSample(
        encoder_counts=encoder_counts,
        projected_gravity_b=TEST_CASE["gravity_b"],
        imu_gyro_b=TEST_CASE["gyro_b"],
        joint_vel=TEST_CASE["joint_vel"],
        joint_effort=TEST_CASE["joint_effort"],
        timestamp_s=time.monotonic(),
    )


def fake_command_writer(msg: RobotCommandMessage) -> None:
    print("\n--- Command written ---")
    print("joint_names:", msg.joint_names)
    print("q_des min/max:", min(msg.q_des), max(msg.q_des))
    print("first 4 q_des:", msg.q_des[:4])
    print("first 4 kp:", msg.kp[:4])
    print("first 4 kd:", msg.kd[:4])


TEST_CASES = [
    {
        "name": "upright",
        "gravity_b": [0.0, 0.0, -1.0],
        "gyro_b": [0.0, 0.0, 0.0],
        "joint_offset_rad": [0.0] * CONTRACT.action_dim,
        "joint_vel": [0.0] * CONTRACT.action_dim,
        "joint_effort": [0.0] * CONTRACT.action_dim,
    },
    {
        "name": "roll_tilt_left",
        "gravity_b": [0.10, 0.0, -0.995],
        "gyro_b": [0.0, 0.0, 0.0],
        "joint_offset_rad": [0.0] * CONTRACT.action_dim,
        "joint_vel": [0.0] * CONTRACT.action_dim,
        "joint_effort": [0.0] * CONTRACT.action_dim,
    },
    {
        "name": "pitch_tilt_forward",
        "gravity_b": [0.0, 0.10, -0.995],
        "gyro_b": [0.0, 0.0, 0.0],
        "joint_offset_rad": [0.0] * CONTRACT.action_dim,
        "joint_vel": [0.0] * CONTRACT.action_dim,
        "joint_effort": [0.0] * CONTRACT.action_dim,
    },
    {
        "name": "yaw_rate",
        "gravity_b": [0.0, 0.0, -1.0],
        "gyro_b": [0.0, 0.0, 0.5],
        "joint_offset_rad": [0.0] * CONTRACT.action_dim,
        "joint_vel": [0.0] * CONTRACT.action_dim,
        "joint_effort": [0.0] * CONTRACT.action_dim,
    },
    {
        "name": "left_knee_offset",
        "gravity_b": [0.0, 0.0, -1.0],
        "gyro_b": [0.0, 0.0, 0.0],
        "joint_offset_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
        "joint_vel": [0.0] * CONTRACT.action_dim,
        "joint_effort": [0.0] * CONTRACT.action_dim,
    },
]

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

    # print("\n --- STAGE A: STEP TEST VALIDATION ---")
    # print("\n --- TEST A --- STEP TEST ---")
    #
    # obs = runner.build_observation()
    # print("obs shape:", tuple(obs.shape))
    # print("obs:", obs)
    #
    # packet = runner.step()
    #
    # print("joint_names:", packet.joint_names)
    #
    # print("\n--- Returned control packet ---")
    # print("q_des shape:", tuple(packet.q_des.shape))
    # print("q_des min/max:", packet.q_des.min().item(), packet.q_des.max().item())
    # print("kp shape:", tuple(packet.kp.shape))
    # print("kd shape:", tuple(packet.kd.shape))
    # print("tau_ff shape:", tuple(packet.tau_ff.shape))
    #
    # actions = runner.policy.act(obs)
    #
    # print("actions:", actions)
    # print("actions min/max:", actions.min().item(), actions.max().item())
    #
    # print("\n --- TEST B --- LOOP TEST ---")
    #
    # for i in range(10): # Increase for better test (200)
    #     packet = runner.step()
    #
    #     if i % 20 == 0:
    #         print(f"step {i} | q_des min/max:",
    #               packet.q_des.min().item(),
    #               packet.q_des.max().item())
    #
    #     time.sleep(1.0 / runner_cfg.loop_hz)

    print("\n--- STAGE B: CONTROLLED VALIDATION ---")
    print("joint order from runner:")
    for i, name in enumerate(joint_names):
        print(f"{i:02d}: {name}")

    for case in TEST_CASES:
        global TEST_CASE
        TEST_CASE = case

        print(f"\n===== CASE: {case['name']} =====")
        obs = runner.build_observation()
        actions = runner.policy.act(obs)
        packet = runner.generate_control_packet(actions)

        print("obs shape:", tuple(obs.shape))
        print("actions:", actions)
        print("actions min/max:", actions.min().item(), actions.max().item())
        print("q_des min/max:", packet.q_des.min().item(), packet.q_des.max().item())

        q_rel = obs[:, :12]
        print("q_rel first 6:", q_rel[0, :6])

        gravity_slice = obs[:, 36:39]
        gyro_slice = obs[:, 39:42]
        print("gravity in obs:", gravity_slice)
        print("gyro in obs:", gyro_slice)

        runner.step()



if __name__ == "__main__":
    main()