"""Real microphone and audible TTS on the lab computer; no motors or face nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = LaunchConfiguration("robot_root")
    python = LaunchConfiguration("python_executable")
    remaps = {
        "/tts/speak": "/story_voice/tts/speak",
        "/tts/done": "/story_voice/tts/done",
        "/tts/interrupt": "/story_voice/tts/interrupt",
        "/stt/enable": "/story_voice/stt_enable",
        "/robot/state_cmd": "/story_voice/robot_state_cmd",
        "/vosk/result": "/story_voice/legacy_child_text",
        "/study/stt_activity": "/story_voice/legacy_stt_activity",
        "/study/speech_error": "/story_voice/legacy_speech_error",
        "/study/speech_event": "/story_voice/speech_event",
        "/study/status": "/story_voice/status",
        **{f"/study/{name}": f"/story_voice/{name}"
           for name in ("start", "finish", "pause", "resume", "stop", "reset")},
    }
    remap_args = []
    for old, new in remaps.items():
        remap_args += ["-r", f"{old}:={new}"]
    params = []
    for name in ("participant", "condition", "robot_name", "prompt_id", "completion_check",
                 "log_dir", "silence_seconds", "thinking_seconds", "max_followups"):
        params += ["-p", [name, ":=", LaunchConfiguration(name)]]
    defaults = {
        "robot_root": "/workspace/bloom/robot",
        "python_executable": "/workspace/venvs/bloom-llm/bin/python",
        "participant": "TEST", "condition": "uncustomized", "robot_name": "Luna",
        "prompt_id": "pet_care_expository", "completion_check": "llm",
        "log_dir": "/workspace/study_logs/storytelling",
        "silence_seconds": "8.0", "thinking_seconds": "20.0", "max_followups": "2",
        "voice": "en-US-AvaNeural",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        ExecuteProcess(
            cmd=[python, "-m", "llm_module.storytelling_speech_node", "--ros-args",
                 "-p", ["voice:=", LaunchConfiguration("voice")], *remap_args],
            cwd=root, output="screen"),
        ExecuteProcess(
            cmd=[python, "-m", "llm_module.storytelling_controller_node", "--ros-args",
                 *params, *remap_args],
            cwd=root, output="screen"),
    ])