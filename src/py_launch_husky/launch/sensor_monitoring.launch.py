
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='talker',
            name='digital_state_display'
        ),
        Node(
            package='py_pubsub',
            executable='experiment',
            name='husky_state_publisher'
        ),
        Node(
            package='py_pubsub',
            executable='hardware',
            name='led_stack_controller'
        )
    ])
