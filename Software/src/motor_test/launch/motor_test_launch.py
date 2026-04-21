# Remember to build and source  workspace before launching:
#   colcon build --packages-select motor_test
#   source install/setup.bash

# TODO: WRITE TESTING README EXPLAINATION

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: add notes as to what needs to be changed for tests
    # TODO: Add way to change params in testing
    motor_pub_node = Node(
        package='motor_test',
        executable='motor_pub.py',
        name='motor_pub',
        output='screen',
        parameters=[
            {
                'topic': 'test_motor_params',
                'rate': 5.0,
                'q': 1.0,
                'kp': 10.0,
                'kd': 0.5,
                'tau': 0.0,
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
                'input': 'test_motor_params',
                'output': 'motor_can',
                'frame': 0x201,
                'all_logging_info': True,
            }
        ],
    )

    ethernet_can_bridge_node = Node(
        package='motor_test',
        executable='ethernet_can_bridge.py',
        name='ethernet_can_bridge',
        output='screen',
        parameters=[
            {
                'stm32_ip':       '192.168.1.100',
                'stm32_port':     7777,
                'listen_port':    7777,
                'command_topic':  'motor_can',
                'feedback_topic': 'motor_can_feedback',
                'can_id':         0x7F,
                'all_logging_info': True,
            }
        ],
    )

    motor_feedback_listener_node = Node(
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

    return LaunchDescription([
        motor_pub_node,
        motor_can_publisher_node,
        ethernet_can_bridge_node,
        motor_feedback_listener_node,
    ])
