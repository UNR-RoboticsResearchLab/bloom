"""Quiet ROS test. Uses isolated topics and Python modules; no colcon build yet."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = LaunchConfiguration("robot_root")
    python = LaunchConfiguration("python_executable")
    remaps = {
        "/tts/speak": "/study_test/tts/speak",
        "/tts/done": "/study_test/tts/done",
        "/tts/interrupt": "/study_test/tts/interrupt",
        "/stt/enable": "/study_test/stt_enable",
        "/robot/state_cmd": "/study_test/robot_state_cmd",
        "/vosk/result": "/study_test/child_text",
        "/study/stt_activity": "/study_test/stt_activity",
        "/study/status": "/study_test/status",
        **{f"/study/{name}": f"/study_test/{name}"
           for name in ("start", "finish", "pause", "resume", "stop", "reset")},
    }
    remap_args = []
    for old, new in remaps.items():
        remap_args += ["-r", f"{old}:={new}"]
    parameters = []
    for name in ("participant", "condition", "robot_name", "prompt_id",
                 "completion_check", "log_dir", "silence_seconds"):
        parameters += ["-p", [name, ":=", LaunchConfiguration(name)]]
    controller = ExecuteProcess(
        cmd=[python, "-m", "llm_module.storytelling_controller_node",
             "--ros-args", *parameters, *remap_args],
        cwd=root, output="screen",
    )
    mock = ExecuteProcess(
        cmd=[python, "-m", "llm_module.storytelling_mock_tts", "--ros-args", *remap_args],
        cwd=root, output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot_root", default_value="/workspace/bloom/robot"),
        DeclareLaunchArgument("python_executable", default_value="/workspace/venvs/bloom-llm/bin/python"),
        DeclareLaunchArgument("participant", default_value="TEST"),
        DeclareLaunchArgument("condition", default_value="uncustomized"),
        DeclareLaunchArgument("robot_name", default_value="Luna"),
        DeclareLaunchArgument("prompt_id", default_value="pet_care_expository"),
        DeclareLaunchArgument("completion_check", default_value="rules"),
        DeclareLaunchArgument("log_dir", default_value="/workspace/study_logs/storytelling"),
        DeclareLaunchArgument("silence_seconds", default_value="10.0"),
        mock, controller,
    ])