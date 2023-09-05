from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim_plus',
            executable='turtlesim_plus_node',
            name='turtlesim_plus',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/turtle1/cmd_vel','/turtle_controller/cmd_vel'),
            ],
        ),

        Node(
            package='turtlesim_plus',
            executable='controller_py',
            name='controller',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='turtlesim_plus',
            executable='pizza_on_click',
            name='spawn_pizza_on_click',
            output='screen',
            emulate_tty=True,
        )
    ])


