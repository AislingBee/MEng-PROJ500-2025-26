from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Defaults
_SERIAL_PORT = 'COM6'
_BAUD_RATE = '921600'
_MOTOR_COUNT = '2'
_JOINT_NAMES = ''
_CAN_ID_BASE = '1'
_STATES = '[]'
_KP = '50.0'
_KD = '2.0'
_TAU_FF = '0.0'
_STATE_DURATION = '3.0'
_RATE_HZ = '50.0'
_CONFIG_FILE = PathJoinSubstitution([
    FindPackageShare('motor_test'),
    'config',
    'json',
    'multi_state_test_config.json',
])


def generate_launch_description():
    # Launch arguments make the file reusable across setups without edits.
    args = [
        DeclareLaunchArgument('serial_port', default_value=_SERIAL_PORT),
        DeclareLaunchArgument('baud_rate', default_value=_BAUD_RATE),
        DeclareLaunchArgument('motor_count', default_value=_MOTOR_COUNT),
        DeclareLaunchArgument('config_file', default_value=_CONFIG_FILE),
        DeclareLaunchArgument('joint_names', default_value=_JOINT_NAMES),
        DeclareLaunchArgument('can_id_base', default_value=_CAN_ID_BASE),
        DeclareLaunchArgument('states_json', default_value=_STATES),
        DeclareLaunchArgument('kp', default_value=_KP),
        DeclareLaunchArgument('kd', default_value=_KD),
        DeclareLaunchArgument('tau_ff', default_value=_TAU_FF),
        DeclareLaunchArgument('state_duration', default_value=_STATE_DURATION),
        DeclareLaunchArgument('rate_hz', default_value=_RATE_HZ),
    ]

    # 1) Multi-state command generator.
    multi_state_test_node = Node(
        package='motor_test',
        executable='multi_state_motor_test.py',
        name='multi_state_motor_test',
        output='screen',
        parameters=[{
            'motor_count': LaunchConfiguration('motor_count'),
            'config_file': LaunchConfiguration('config_file'),
            'joint_names': LaunchConfiguration('joint_names'),
            'states_json': LaunchConfiguration('states_json'),
            'kp': LaunchConfiguration('kp'),
            'kd': LaunchConfiguration('kd'),
            'tau_ff': LaunchConfiguration('tau_ff'),
            'state_duration': LaunchConfiguration('state_duration'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'command_topic': 'robot_command',
            'feedback_topic': 'motor_feedback',
        }],
    )

    # 2) Command bridge: /robot_command -> /motor_can_tx
    robot_command_bridge_node = Node(
        package='motor_test',
        executable='robot_command_bridge.py',
        name='robot_command_bridge',
        output='screen',
        parameters=[{
            'command_topic': 'robot_command',
            'can_tx_topic': 'motor_can_tx',
            'all_logging_info': False,
        }],
    )

    # 3) Serial bridge: /motor_can_tx <-> STM32 <-> /motor_can_feedback
    serial_can_bridge_node = Node(
        package='motor_test',
        executable='serial_can_bridge.py',
        name='serial_can_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'command_topic': 'motor_can_tx',
            'feedback_topic': 'motor_can_feedback',
            'can_id_per_joint': True,
            'can_id_base': LaunchConfiguration('can_id_base'),
            'all_logging_info': False,
        }],
    )

    # 4) Feedback bridge: /motor_can_feedback -> /motor_feedback
    motor_feedback_listener_node = Node(
        package='motor_test',
        executable='motor_feedback_listener.py',
        name='motor_feedback_listener',
        output='screen',
        parameters=[{
            'input': 'motor_can_feedback',
            'motor_count': LaunchConfiguration('motor_count'),
            'names_file': 'motor_names.json',
            'all_logging_info': False,
        }],
    )

    return LaunchDescription(args + [
        multi_state_test_node,
        robot_command_bridge_node,
        serial_can_bridge_node,
        motor_feedback_listener_node,
    ])
