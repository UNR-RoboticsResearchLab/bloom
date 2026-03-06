#!/usr/bin/env python3
"""
Combined launch file for Bloom robot driver and OpenHMI Blossom system
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        enable_idle_arg,
        blossom_launch,
    ])
