# Full multi-joint MIT-style control command sent by the RL policy.
# One message covers all robot joints in the same order as joint_names.
#
# Packed into UInt8MultiArray by robot_command_bridge.py (16 bytes per joint)
# before forwarding to ethernet_can_bridge.py → STM32.

string[] joint_names   # ordered list of joint names
float64[] q_des        # desired joint positions   [rad]
float64[] qd_des       # desired joint velocities  [rad/s]
float64[] kp           # position gains            [Nm/rad]
float64[] kd           # velocity gains            [Nm·s/rad]
float64[] tau_ff       # feedforward torques        [Nm]
