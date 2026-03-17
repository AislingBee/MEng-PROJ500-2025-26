ACTUATOR_SETTINGS = {
    "pelvis_yaw": {
        "joint_names": [
            "robot_pelvis_link_l_yaw_joint",
            "robot_pelvis_link_r_yaw_joint",
        ],
        "effort_limit": 50.0,
        "velocity_limit": 8.0,
        "stiffness": 130.0,
        "damping": 4.0,
    },
    "hip_yaw": {
        "joint_names": [
            "robot_l_hip_yaw_link_l_pitch_joint",
            "robot_r_hip_yaw_link_r_pitch_joint",
        ],
        "effort_limit": 80.0,
        "velocity_limit": 8.0,
        "stiffness": 220.0,
        "damping": 8.0,
    },
    "hip_roll": {
        "joint_names": [
            "robot_l_hip_pitch_link_l_roll_joint",
            "robot_r_hip_pitch_link_r_roll_joint",
        ],
        "effort_limit": 75.0,
        "velocity_limit": 8.0,
        "stiffness": 190.0,
        "damping": 7.0,
    },
    "knees": {
        "joint_names": [
            "robot_l_thigh_link_l_knee_joint",
            "robot_r_thigh_link_r_knee_joint",
        ],
        "effort_limit": 90.0,
        "velocity_limit": 8.0,
        "stiffness": 280.0,
        "damping": 10.0,
    },
    "ankle_pitch": {
        "joint_names": [
            "robot_l_shank_link_l_ankle_joint",
            "robot_r_shank_link_r_ankle_joint",
        ],
        "effort_limit": 60.0,
        "velocity_limit": 10.0,
        "stiffness": 160.0,
        "damping": 6.0,
    },
    "ankle_roll": {
        "joint_names": [
            "robot_l_ankle_link_l_foot_joint",
            "robot_r_ankle_link_r_foot_joint",
        ],
        "effort_limit": 40.0,
        "velocity_limit": 10.0,
        "stiffness": 120.0,
        "damping": 5.0,
    },
}


def build_per_joint_limits_and_gains(joint_names):
    effort_limits = []
    velocity_limits = []
    stiffnesses = []
    dampings = []

    for joint_name in joint_names:
        matched = False
        for cfg in ACTUATOR_SETTINGS.values():
            if joint_name in cfg["joint_names"]:
                effort_limits.append(cfg["effort_limit"])
                velocity_limits.append(cfg["velocity_limit"])
                stiffnesses.append(cfg["stiffness"])
                dampings.append(cfg["damping"])
                matched = True
                break

        if not matched:
            raise RuntimeError(f"No actuator settings found for joint: {joint_name}")

    return effort_limits, velocity_limits, stiffnesses, dampings