from launch import LaunchDescription
from launch.actions import Node
from launch.substitutions import LaunchConfiguration

# Defaults
_MOTOR_COUNT = 2
_JOINT_NAMES = 'motor_1,motor_2'
_STATES = '[[0.0, 0.0], [0.5, 0.5], [-0.5, -0.5]]'
_KP = 50.0
_KD = 2.0
_STATE_DURATION = 3.0
_RATE_HZ = 50.0


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_test',
            executable='multi_state_motor_test',
            name='multi_state_motor_test',
            output='screen',
            parameters=[
                {'motor_count': LaunchConfiguration('motor_count', default=_MOTOR_COUNT)},
                {'joint_names': LaunchConfiguration('joint_names', default=_JOINT_NAMES)},
                {'states_json': LaunchConfiguration('states_json', default=_STATES)},
                {'kp': LaunchConfiguration('kp', default=_KP)},
                {'kd': LaunchConfiguration('kd', default=_KD)},
                {'state_duration': LaunchConfiguration('state_duration', default=_STATE_DURATION)},
                {'rate_hz': LaunchConfiguration('rate_hz', default=_RATE_HZ)},
                {'command_topic': 'robot_command'},
                {'feedback_topic': 'motor_feedback'},
            ],
        ),
    ])
