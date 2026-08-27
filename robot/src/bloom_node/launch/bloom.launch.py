from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/bloom_config.yaml',
            description='Configuration file to load'
        ),
        DeclareLaunchArgument(
            'base_url',
            default_value='https://bloom.unr.dev/',
            description='Web service base URL'
        ),
        Node(
            package='bloom_node',
            executable='bloom_node',
            name='bloom_node',
            parameters=[{
                'base_url': LaunchConfiguration('base_url'),
                # Other parameters...
            }],
            output='screen'
        )
    ])
