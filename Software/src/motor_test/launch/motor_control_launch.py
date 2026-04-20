# Remember to build and source your workspace before launching:
#   colcon build --packages-select motor_test
#   source install/setup.bash
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_controller_test_node = Node(
        package='motor_test',
        executable='motor_controller_test.py',
        name='motor_controller_test',
        output='screen',
        parameters=[
            {
                'command_topic': 'motor_params',
                'feedback_topic': 'motor_feedback',
                'rate': 10.0,
                'target_q': 0.4,
                'kp': 20.0,
                'kd': 1.0,
                'tau': 0.0,
                'step_duration': 4.0,
                'motor_index': 0,
                'all_logging_info': True,
            }
        ],
    )

    json_can_convert_node = Node(
        package='motor_test',
        executable='motor_can_bridge.py',
        name='motor_params_can_bridge',
        output='screen',
        parameters=[
            {
                'inputs': 'motor_params',
                'output': 'motor_can',
                'frame': 0x201,
                'all_logging_info': True,
            }
        ],
    )

    serial_can_bridge_node = Node(
        package='motor_test',
        executable='serial_can_bridge.py',
        name='serial_can_bridge',
        output='screen',
        parameters=[
            {
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 115200,
                'timeout': 0.1,
                'command_topic': 'motor_can',
                'feedback_topic': 'motor_can_feedback',
                'can_id': 0x7F,
                'all_logging_info': True,
            }
        ],
    )

    motor_feedback_node = Node(
        package='motor_test',
        executable='motor_feedback_listener.py',
        name='motor_feedback_listener',
        output='screen',
        parameters=[
            {
                'input': 'motor_can_feedback',
                'motor_count': 1,
                'names_file': 'motor_names.json',
                'all_logging_info': True,
            }
        ],
    )

    motor_sub_node = Node(
        package='motor_test',
        executable='motor_sub',
        name='motor_sub',
        output='screen',
        parameters=[{'input': 'motor_params'}],
    )

    return LaunchDescription([
        motor_controller_test_node,
        json_can_convert_node,
        serial_can_bridge_node,
        motor_feedback_node,
        motor_sub_node,
    ])
