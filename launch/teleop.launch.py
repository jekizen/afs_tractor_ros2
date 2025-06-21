from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='gnome-terminal --'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller'],
            output='screen'
        )
    ])
