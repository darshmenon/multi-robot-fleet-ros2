"""
Launch the LLM planner node.

Usage:
    ros2 launch ur_llm_planner llm_planner.launch.py
    ros2 launch ur_llm_planner llm_planner.launch.py model:=claude-sonnet-4-6
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='claude-haiku-4-5-20251001',
            description='Claude model ID for motion planning',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        Node(
            package='ur_llm_planner',
            executable='llm_planner_node',
            name='llm_planner_node',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'anthropic_api_key': EnvironmentVariable('ANTHROPIC_API_KEY', default_value=''),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
