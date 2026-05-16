"""
Launch the LLM planner node.

Usage:
    # Ollama (default, llama2 model)
    ros2 launch ur_llm_planner llm_planner.launch.py

    # Different Ollama model
    ros2 launch ur_llm_planner llm_planner.launch.py model:=mistral

    # Anthropic Claude backend
    ros2 launch ur_llm_planner llm_planner.launch.py backend:=anthropic model:=claude-haiku-4-5-20251001
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='ollama',
            description='LLM backend: ollama or anthropic',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='llama2',
            description='Model name (e.g. llama2, mistral for ollama; claude-haiku-4-5-20251001 for anthropic)',
        ),
        DeclareLaunchArgument(
            'ollama_base_url',
            default_value='http://localhost:11434',
            description='Ollama REST API base URL',
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
                'backend': LaunchConfiguration('backend'),
                'model': LaunchConfiguration('model'),
                'ollama_base_url': LaunchConfiguration('ollama_base_url'),
                'anthropic_api_key': EnvironmentVariable('ANTHROPIC_API_KEY', default_value=''),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
