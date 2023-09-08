from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim_plus',
            executable='turtlesim_plus_node',
            name='turtlesim_plus'
        ),
        Node(
            package='turtlesim_plus',
            executable='controller_py',
            name='controller'
        ),
        Node(
            package='turtlesim_plus',
            executable='pizza_on_click',
            name='spawn_pizza_on_click'
        )
    ])

