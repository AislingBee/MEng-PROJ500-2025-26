from __future__ import annotations

# Motor direction signs map policy/sim joint commands into the real actuator direction convention.
# +1.0 = actuator command direction matches policy/sim convention
# -1.0 = actuator command direction is inverted on the real robot
#
# Joint names must match rcu_protocol.py MOTOR_JOINT_NAMES (CAN IDs 1-12).

MOTOR_DIRECTION_SIGNS: dict[str, float] = {
    "pelvis_link_l_yaw_joint":          -1.0, # CAN ID 1  - Left yaw        URDF + = anticlock
    "pelvis_link_r_yaw_joint":          +1.0, # CAN ID 2  - Right yaw       URDF + = clock

    "l_hip_yaw_link_l_pitch_joint":     -1.0, # CAN ID 3  - Left hip pitch  URDF + = clock
    "r_hip_yaw_link_r_pitch_joint":     +1.0, # CAN ID 4  - Right hip pitch URDF + = anticlock

    "l_hip_pitch_link_l_roll_joint":    +1.0, # CAN ID 5  - Left hip roll   URDF + = clock
    "r_hip_pitch_link_r_roll_joint":    -1.0, # CAN ID 6  - Right hip roll  URDF + = anticlock

    "l_thigh_link_l_knee_joint":        -1.0, # CAN ID 7  - Left knee       URDF + = anticlock
    "r_thigh_link_r_knee_joint":        +1.0, # CAN ID 8  - Right knee      URDF + = clock

    "l_shank_link_l_ankle_joint":       +1.0, # CAN ID 9  - Left ankle      URDF + = anticlock
    "r_shank_link_r_ankle_joint":       -1.0, # CAN ID 10 - Right ankle     URDF + = clock

    "l_ankle_link_l_foot_joint":        -1.0, # CAN ID 11 - Left foot roll  URDF + = anticlock
    "r_ankle_link_r_foot_joint":        +1.0, # CAN ID 12 - Right foot roll URDF + = clock
}

# Feedback signs map real hardware sensor readings into the policy/sim joint convention.
# +1.0 = hardware feedback direction matches policy/sim convention
# -1.0 = hardware feedback direction is inverted relative to policy/sim convention

JOINT_FEEDBACK_SIGNS: dict[str, float] = MOTOR_DIRECTION_SIGNS.copy()


def _sign_tuple(
    signs: dict[str, float],
    joint_names: tuple[str, ...],
    label: str,
) -> tuple[float, ...]:
    """Return signs ordered to match joint_names."""

    missing = [name for name in joint_names if name not in signs]
    if missing:
        raise KeyError(f"Missing {label} signs for joints: {missing}")

    invalid = {
        name: value
        for name, value in signs.items()
        if value not in (-1.0, +1.0)
    }
    if invalid:
        raise ValueError(f"{label} signs must be +1.0 or -1.0: {invalid}")

    return tuple(float(signs[name]) for name in joint_names)


def motor_direction_tuple(joint_names: tuple[str, ...]) -> tuple[float, ...]:
    """Return motor direction signs ordered to match joint_names."""

    return _sign_tuple(MOTOR_DIRECTION_SIGNS, joint_names, "motor direction")


def joint_feedback_tuple(joint_names: tuple[str, ...]) -> tuple[float, ...]:
    """Return joint feedback signs ordered to match joint_names."""

    return _sign_tuple(JOINT_FEEDBACK_SIGNS, joint_names, "joint feedback")
