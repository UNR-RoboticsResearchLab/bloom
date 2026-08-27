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
            default_value='',
            description='Web service base URL. Leave empty (default) to use '
                         'the value from the config file; pass base_url:=... '
                         'on the command line to override it.'
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
