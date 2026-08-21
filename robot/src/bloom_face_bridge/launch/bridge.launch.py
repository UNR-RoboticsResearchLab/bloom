from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bloom_face_bridge',
            executable='bridge_node',
            name='bloom_face_bridge',
            output='screen',
        ),
    ])
