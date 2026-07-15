#!/usr/bin/env python3
"""
Combined launch file for Bloom robot driver and the M-SARG servo/face system
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate combined launch description for Bloom driver and M-SARG system."""

    # Declare launch arguments
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

    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='Skip real I2C/PCA9685 hardware (for dev-machine testing)'
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

    # Include m_sarg launch file (servo_driver_node + face_node)
    m_sarg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('m_sarg'),
                'launch',
                'm_sarg.launch.py'
            ])
        ),
        launch_arguments={
            'dry_run': LaunchConfiguration('dry_run'),
        }.items()
    )

    # Include bloom_speech launch file (tts_node drives face visemes/emotion
    # during speech; without it the face never animates past its idle state)
    speech_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bloom_speech'),
                'launch',
                'speech.launch.py'
            ])
        )
    )

    return LaunchDescription([
        config_file_arg,
        base_url_arg,
        dry_run_arg,
        bloom_launch,
        m_sarg_launch,
        speech_launch,
    ])
