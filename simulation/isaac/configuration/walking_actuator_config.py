"""Actuator configuration for the humanoid walking task.

This is set up around a single RobStride RS04-120Nm actuator profile shared across all
12 lower-body joints. It uses an explicit DC motor actuator in Isaac Lab so the walk
policy can observe actuator effort/torque, while the reward still uses privileged foot
contact in simulation only.
"""

from __future__ import annotations

# Practical initial gains for position-target walking around the standing pose.
# These are intentionally conservative and should be tuned after the first short test run.
WALKING_ACTUATOR_SETTINGS = {
    "lower_body_rs04": {
        "joint_names": [".*"],
        "saturation_effort": 120.0,
        "effort_limit": 120.0,
        "effort_limit_sim": 120.0,
        "velocity_limit": 17.488,
        "velocity_limit_sim": 17.488,
        "stiffness": 250.0,
        "damping": 12.0,
        "armature": 0.04,
        "friction": 0.2,
        "dynamic_friction": 0.0,
        "viscous_friction": 0.0,
    }
}
