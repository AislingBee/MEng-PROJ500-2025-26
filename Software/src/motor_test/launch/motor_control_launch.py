# Remember to build and source your workspace before launching:
#   colcon build --packages-select motor_test
#   source install/setup.bash
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_pub_node = Node(
        package='motor_test',
        executable='motor_pub.py',
        name='motor_pub',
        output='screen',
        parameters=[
            {
                'topic': 'motor_params',
                'rate': 10.0,
                'q': 1.0,
                'kp': 10.0,
                'kd': 0.5,
                'tau': 0.0,
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

    motor_feedback_node = Node(
        package='motor_test',
        executable='motor_feedback_listener.py',
        name='motor_feedback_listener',
        output='screen',
        parameters=[
            {
                'input': 'motor_can',
                'motor_count': 13,
                'names_file': 'motor_names.json',
                'all_logging_info': True,
            }
        ],
    )

    motor_can_publisher_node = Node(
        package='motor_test',
        executable='motor_can_publisher.py',
        name='motor_can_publisher',
        output='screen',
        parameters=[
            {
                'input': 'motor_can',
                'output': 'motor_can_tx',
                'frame': 0x201,
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
        motor_pub_node,
        json_can_convert_node,
        motor_feedback_node,
        motor_can_publisher_node,
        motor_sub_node,
    ])
