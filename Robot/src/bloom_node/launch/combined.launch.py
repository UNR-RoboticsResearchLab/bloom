#!/usr/bin/env python3
"""
Combined launch file for Bloom robot driver and OpenHMI Blossom system
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate combined launch description for Bloom driver and Blossom system."""

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for motor communication'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='1000000',
        description='Baudrate for serial communication'
    )

    enable_idle_arg = DeclareLaunchArgument(
        'enable_idle',
        default_value='true',
        description='Enable idle behavior'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/bloom_config.yaml',
        description='Configuration file to load'
    )

    base_url_arg = DeclareLaunchArgument(
        'base_url',
        default_value='http://localhost:5000',
        description='Web service base URL'
    )

    # Include OpenHMI Blossom launch file with arguments
    blossom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('openhmi_blossom'),
                'launch',
                'blossom.launch.py'
            ])
        ),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'enable_idle': LaunchConfiguration('enable_idle'),
        }.items()
    )

    # Include bloom_node launch file
    bloom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bloom_node'),
                'launch',
                'bloom.launch.py'
            ])
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'base_url': LaunchConfiguration('base_url'),
        }.items()
    )

    # Include bloom_speech launch file
    speech_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bloom_speech'),
                'launch',
                'speech.launch.py'
            ])
        )
    )

    # Run bloom_face face_node
    face_launch = Node(
        package='bloom_face',
        executable='face_node',
        additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        enable_idle_arg,
        config_file_arg,
        base_url_arg,
        blossom_launch,
        bloom_launch,
        speech_launch,
        face_launch,
    ])
