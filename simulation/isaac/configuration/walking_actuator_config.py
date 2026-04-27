"""Per-joint actuator configuration for the humanoid walking task.

This keeps the old grouped/per-joint structure, but carries over the newer
walking-task actuator parameters so the environment can build per-joint arrays
instead of applying one blanket regex profile to every joint.
"""

from __future__ import annotations


WALKING_ACTUATOR_SETTINGS = {
    "pelvis_yaw": {
        "joint_names": [
            "pelvis_link_l_yaw_joint",
            "pelvis_link_r_yaw_joint",
        ],
        "saturation_effort": 120.0,
        "effort_limit": 50.0,
        "effort_limit_sim": 50.0,
        "velocity_limit": 8.0,
        "velocity_limit_sim": 8.0,
        "stiffness": 130.0,
        "damping": 4.0,
        "armature": 0.04,
        "friction": 0.2,
        "dynamic_friction": 0.0,
        "viscous_friction": 0.0,
    },
    "hip_yaw": {
        "joint_names": [
            "l_hip_yaw_link_l_pitch_joint",
            "r_hip_yaw_link_r_pitch_joint",
        ],
        "saturation_effort": 120.0,
        "effort_limit": 80.0,
        "effort_limit_sim": 80.0,
        "velocity_limit": 8.0,
        "velocity_limit_sim": 8.0,
        "stiffness": 220.0,
        "damping": 8.0,
        "armature": 0.04,
        "friction": 0.2,
        "dynamic_friction": 0.0,
        "viscous_friction": 0.0,
    },
    "hip_roll": {
        "joint_names": [
            "l_hip_pitch_link_l_roll_joint",
            "r_hip_pitch_link_r_roll_joint",
        ],
        "saturation_effort": 120.0,
        "effort_limit": 75.0,
        "effort_limit_sim": 75.0,
        "velocity_limit": 8.0,
        "velocity_limit_sim": 8.0,
        "stiffness": 190.0,
        "damping": 7.0,
        "armature": 0.04,
        "friction": 0.2,
        "dynamic_friction": 0.0,
        "viscous_friction": 0.0,
    },
    "knees": {
        "joint_names": [
            "l_thigh_link_l_knee_joint",
            "r_thigh_link_r_knee_joint",
        ],
        "saturation_effort": 120.0,
        "effort_limit": 90.0,
        "effort_limit_sim": 90.0,
        "velocity_limit": 8.0,
        "velocity_limit_sim": 8.0,
        "stiffness": 280.0,
        "damping": 10.0,
        "armature": 0.04,
        "friction": 0.2,
        "dynamic_friction": 0.0,
        "viscous_friction": 0.0,
    },
    "ankle_pitch": {
        "joint_names": [
            "l_shank_link_l_ankle_joint",
            "r_shank_link_r_ankle_joint",
        ],
        "saturation_effort": 120.0,
        "effort_limit": 100.0,
        "effort_limit_sim": 100.0,
        "velocity_limit": 10.0,
        "velocity_limit_sim": 10.0,
        "stiffness": 300.0,
        "damping": 12.0,
        "armature": 0.04,
        "friction": 0.2,
        "dynamic_friction": 0.0,
        "viscous_friction": 0.0,
    },
    "ankle_roll": {
        "joint_names": [
            "l_ankle_link_l_foot_joint",
            "r_ankle_link_r_foot_joint",
        ],
        "saturation_effort": 120.0,
        "effort_limit": 40.0,
        "effort_limit_sim": 40.0,
        "velocity_limit": 10.0,
        "velocity_limit_sim": 10.0,
        "stiffness": 120.0,
        "damping": 5.0,
        "armature": 0.04,
        "friction": 0.2,
        "dynamic_friction": 0.0,
        "viscous_friction": 0.0,
    },
}


def build_per_joint_walking_actuator_cfg(joint_names):
    saturation_efforts = []
    effort_limits = []
    effort_limit_sims = []
    velocity_limits = []
    velocity_limit_sims = []
    stiffnesses = []
    dampings = []
    armatures = []
    frictions = []
    dynamic_frictions = []
    viscous_frictions = []

    for joint_name in joint_names:
        matched = False

        for cfg in WALKING_ACTUATOR_SETTINGS.values():
            if joint_name in cfg["joint_names"]:
                saturation_efforts.append(cfg["saturation_effort"])
                effort_limits.append(cfg["effort_limit"])
                effort_limit_sims.append(cfg["effort_limit_sim"])
                velocity_limits.append(cfg["velocity_limit"])
                velocity_limit_sims.append(cfg["velocity_limit_sim"])
                stiffnesses.append(cfg["stiffness"])
                dampings.append(cfg["damping"])
                armatures.append(cfg["armature"])
                frictions.append(cfg["friction"])
                dynamic_frictions.append(cfg["dynamic_friction"])
                viscous_frictions.append(cfg["viscous_friction"])
                matched = True
                break

        if not matched:
            raise RuntimeError(f"No walking actuator settings found for joint: {joint_name}")

    return {
        "saturation_effort": saturation_efforts,
        "effort_limit": effort_limits,
        "effort_limit_sim": effort_limit_sims,
        "velocity_limit": velocity_limits,
        "velocity_limit_sim": velocity_limit_sims,
        "stiffness": stiffnesses,
        "damping": dampings,
        "armature": armatures,
        "friction": frictions,
        "dynamic_friction": dynamic_frictions,
        "viscous_friction": viscous_frictions,
    }