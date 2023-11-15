from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='meu_ros',
            executable='nav_waypoints',
            name='nav_waypoints',
            prefix = 'gnome-terminal --',
            output='screen'
        )
    ])