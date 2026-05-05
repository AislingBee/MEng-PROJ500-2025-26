from __future__ import annotations

# Motor direction signs map policy/sim joint commands into the real actuator direction convention.
# +1.0 = actuator command direction matches policy/sim convention
# -1.0 = actuator command direction is inverted on the real robot
#
# Joint names must match rcu_protocol.py MOTOR_JOINT_NAMES (CAN IDs 1–12).

MOTOR_DIRECTION_SIGNS: dict[str, float] = {
    # CAN ID 1  — Left yaw
    "pelvis_link_l_yaw_joint":          +1.0,
    # CAN ID 2  — Right yaw
    "pelvis_link_r_yaw_joint":          +1.0,
    # CAN ID 3  — Left hip pitch
    "l_hip_yaw_link_l_pitch_joint":     +1.0,
    # CAN ID 4  — Right hip pitch
    "r_hip_yaw_link_r_pitch_joint":     +1.0,
    # CAN ID 5  — Left hip roll
    "l_hip_pitch_link_l_roll_joint":    +1.0,
    # CAN ID 6  — Right hip roll
    "r_hip_pitch_link_r_roll_joint":    +1.0,
    # CAN ID 7  — Left knee
    "l_thigh_link_l_knee_joint":        +1.0,
    # CAN ID 8  — Right knee
    "r_thigh_link_r_knee_joint":        +1.0,
    # CAN ID 9  — Left ankle
    "l_shank_link_l_ankle_joint":       +1.0,
    # CAN ID 10 — Right ankle
    "r_shank_link_r_ankle_joint":       +1.0,
    # CAN ID 11 — Left foot roll
    "l_ankle_link_l_foot_joint":        +1.0,
    # CAN ID 12 — Right foot roll
    "r_ankle_link_r_foot_joint":        +1.0,
}

# IMPORTANT: All signs are +1.0 (identity) until physically verified.
# Use sequential_motor_zero.py to test one motor at a time.
# For each motor, command it to 0.0 rad and observe the physical direction:
#   - If it moves in the CORRECT direction (toward sim zero): sign = +1.0
#   - If it moves in the WRONG direction (away from sim zero): sign = -1.0
# Update this dict with confirmed values before running startup_then_policy_runner.


def motor_direction_tuple(joint_names: tuple[str, ...]) -> tuple[float, ...]:
    """Return motor direction signs ordered to match joint_names."""

    missing = [name for name in joint_names if name not in MOTOR_DIRECTION_SIGNS]
    if missing:
        raise KeyError(f"Missing motor direction signs for joints: {missing}")

    invalid = {
        name: value
        for name, value in MOTOR_DIRECTION_SIGNS.items()
        if value not in (-1.0, +1.0)
    }
    if invalid:
        raise ValueError(f"Motor direction signs must be +1.0 or -1.0: {invalid}")

    return tuple(float(MOTOR_DIRECTION_SIGNS[name]) for name in joint_names)


def motor_direction_tuple(joint_names: tuple[str, ...]) -> tuple[float, ...]:
    """Return motor direction signs ordered to match joint_names."""

    missing = [name for name in joint_names if name not in MOTOR_DIRECTION_SIGNS]
    if missing:
        raise KeyError(f"Missing motor direction signs for joints: {missing}")

    invalid = {
        name: value
        for name, value in MOTOR_DIRECTION_SIGNS.items()
        if value not in (-1.0, +1.0)
    }
    if invalid:
        raise ValueError(f"Motor direction signs must be +1.0 or -1.0: {invalid}")

    return tuple(float(MOTOR_DIRECTION_SIGNS[name]) for name in joint_names)
