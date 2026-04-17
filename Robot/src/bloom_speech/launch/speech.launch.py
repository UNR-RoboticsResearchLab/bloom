from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bloom_speech',
            executable='tts_node',
            name='tts_node',
            output='screen',
        ),
        Node(
            package='bloom_speech',
            executable='llm_node',
            name='llm_node',
            output='screen',
        ),
    ])