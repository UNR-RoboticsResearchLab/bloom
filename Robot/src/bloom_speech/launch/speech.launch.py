import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    robot_dir = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..'
    )
    robot_dir = os.path.realpath(robot_dir)

    return LaunchDescription([
        SetEnvironmentVariable(
            name='PYTHONPATH',
            value=robot_dir + ':' + os.environ.get('PYTHONPATH', '')
        ),
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