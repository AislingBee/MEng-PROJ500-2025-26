from __future__ import annotations

# Motor direction signs map policy/sim joint commands into the real actuator direction convention.
# +1.0 = actuator command direction matches policy/sim convention
# -1.0 = actuator command direction is inverted on the real robot

MOTOR_DIRECTION_SIGNS: dict[str, float] = {
    # Left leg
    "pelvis_link_l_yaw_joint": -1.0,
    "pelvis_link_l_hip_roll_joint": +1.0,
    "pelvis_link_l_hip_pitch_joint": +1.0,
    "left_knee_joint": +1.0,
    "left_ankle_pitch_joint": -1.0,
    "left_ankle_roll_joint": -1.0,

    # Right leg
    "pelvis_link_r_yaw_joint": +1.0,
    "pelvis_link_r_hip_roll_joint": -1.0,
    "pelvis_link_r_hip_pitch_joint": -1.0,
    "right_knee_joint": -1.0,
    "right_ankle_pitch_joint": +1.0,
    "right_ankle_roll_joint": +1.0,
}

#     # Left leg
#     "pelvis_link_l_yaw_joint": +1.0,
#     "pelvis_link_l_hip_roll_joint": -1.0,
#     "pelvis_link_l_hip_pitch_joint": -1.0,
#     "left_knee_joint": -1.0,
#     "left_ankle_pitch_joint": +1.0,
#     "left_ankle_roll_joint": +1.0,
#
#     # Right leg
#     "pelvis_link_r_yaw_joint": -1.0,
#     "pelvis_link_r_hip_roll_joint": +1.0,
#     "pelvis_link_r_hip_pitch_joint": +1.0,
#     "right_knee_joint": +1.0,
#     "right_ankle_pitch_joint": -1.0,
#     "right_ankle_roll_joint": -1.0,
# }


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
