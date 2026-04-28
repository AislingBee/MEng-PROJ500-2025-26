# Full robot observation aggregated from motor feedback and IMU.
# Published by robot_observation_bridge.py, consumed by Ros2RobotBridge
# (simulation/isaac/rl/interface/ros2_robot_bridge.py).

float64 timestamp_s           # monotonic timestamp in seconds

# Joint state (one element per joint, same order as motor_names.json)
float64[] joint_pos_rad       # joint positions  [rad]   as received from hardware
float64[] joint_vel_rad_s     # joint velocities [rad/s] as received from hardware
float64[] joint_effort_nm     # joint efforts    [Nm]    (empty if unavailable)

# IMU-derived body-frame signals (3 elements each)
float64[] projected_gravity_b  # unit gravity vector projected into body frame
float64[] imu_gyro_b           # angular velocity in body frame [rad/s]
