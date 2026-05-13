from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_drive_robot',
            executable='mission_server.py',
            name='mission_server',
            output='screen',
        ),
        Node(
            package='diff_drive_robot',
            executable='handoff_coordinator.py',
            name='handoff_coordinator',
            output='screen',
        ),
    ])
