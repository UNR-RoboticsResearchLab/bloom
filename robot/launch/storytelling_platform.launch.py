"""Start storytelling, researcher controls, M's face, and nod preview.

m-sarg remains a separate checkout.
This launch does not start the motor driver.
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    voice_defaults = {
        "robot_root": "/workspace/bloom/robot",
        "python_executable": "/workspace/venvs/bloom-llm/bin/python",
        "participant": "TEST",
        "condition": "uncustomized",
        "robot_name": "M",
        "prompt_id": "pet_care_expository",
        "completion_check": "llm",
        "log_dir": "/workspace/study_logs/storytelling",
        "silence_seconds": "4.0",
        "thinking_seconds": "20.0",
        "max_followups": "2",
        "voice": "en-US-AvaNeural",
    }
    platform_defaults = {
        "default_name": "M",
        "web_host": "0.0.0.0",
        "web_port": "8770",
        "enable_face": "true",
        "m_sarg_root": "/workspace/m-sarg",
        "face_port": "8771",
        "face_kiosk": "false",
        "enable_motion": "true",
        "motion_preview": "true",
        "motion_status_topic": "/story_voice/status",
        "motion_speech_event_topic": "/story_voice/speech_event",
        "motion_action": "/m_sarg_play_behavior",
        "first_nod_seconds": "8.0",
        "nod_interval_seconds": "8.0",
        "speech_activity_window_seconds": "2.0",
        "nod_amplitude": "5.0",
        "nod_speed_scale": "0.5",
    }
    defaults = {**voice_defaults, **platform_defaults}

    voice_launch = Path(__file__).with_name(
        "storytelling_voice.launch.py"
    )
    activity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(voice_launch)),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in voice_defaults
        }.items(),
    )

    webpage = ExecuteProcess(
        cmd=[
            LaunchConfiguration("python_executable"),
            "-m",
            "llm_module.storytelling_web",
            "--mode",
            "voice",
            "--default-name",
            LaunchConfiguration("default_name"),
            "--host",
            LaunchConfiguration("web_host"),
            "--port",
            LaunchConfiguration("web_port"),
        ],
        cwd=LaunchConfiguration("robot_root"),
        output="screen",
    )

    face = ExecuteProcess(
        cmd=[
            LaunchConfiguration("python_executable"),
            "-m",
            "m_sarg.face_node",
            "--ros-args",
            "-p",
            ["port:=", LaunchConfiguration("face_port")],
            "-p",
            ["kiosk:=", LaunchConfiguration("face_kiosk")],
            "-p",
            "wifi_enabled:=false",
        ],
        cwd=LaunchConfiguration("m_sarg_root"),
        additional_env={
            "PYTHONPATH": [
                LaunchConfiguration("m_sarg_root"),
                "/src/m_sarg",
                os.pathsep,
                EnvironmentVariable("PYTHONPATH", default_value=""),
            ],
        },
        condition=IfCondition(LaunchConfiguration("enable_face")),
        output="screen",
    )

    motion_parameters = {
        "preview": "motion_preview",
        "status_topic": "motion_status_topic",
        "speech_event_topic": "motion_speech_event_topic",
        "action_name": "motion_action",
        "first_nod_seconds": "first_nod_seconds",
        "nod_interval_seconds": "nod_interval_seconds",
        "speech_activity_window_seconds": "speech_activity_window_seconds",
        "amplitude": "nod_amplitude",
        "speed_scale": "nod_speed_scale",
    }
    motion_command = [
        LaunchConfiguration("python_executable"),
        "-m",
        "llm_module.storytelling_motion_node",
        "--ros-args",
    ]
    for parameter, argument in motion_parameters.items():
        motion_command.extend([
            "-p",
            [f"{parameter}:=", LaunchConfiguration(argument)],
        ])

    motion = ExecuteProcess(
        cmd=motion_command,
        cwd=LaunchConfiguration("robot_root"),
        condition=IfCondition(LaunchConfiguration("enable_motion")),
        output="screen",
    )

    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in defaults.items()
        ],
        RegisterEventHandler(
            OnProcessExit(
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="A storytelling platform process exited."
                        )
                    )
                ]
            )
        ),
        activity,
        webpage,
        face,
        motion,
    ])