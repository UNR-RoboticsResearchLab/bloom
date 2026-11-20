# Storytelling platform

## Companion repository

Requires a separate checkout of:
https://github.com/UNR-RoboticsResearchLab/m-sarg

Version used during development:
01162f6834cb450e741ecd1128f7904a5d48227e

## Working development setup

- ROS 2 Jazzy / Python 3.12
- Python environment with access to ROS system packages
- Python dependencies from llm_module/requirements.txt and
  tts_module/requirements.txt, plus aiohttp
- PulseAudio playback through paplay
- Azure credentials configured locally; never commit credentials
- m_sarg_msgs built in the m-sarg workspace

## Start on the development computer

source /opt/ros/jazzy/setup.bash
source /workspace/m-sarg/install/local_setup.bash
source /workspace/bloom/robot/install/local_setup.bash
source /workspace/venvs/bloom-llm/bin/activate
cd /workspace/bloom/robot

ros2 launch /workspace/bloom/robot/launch/storytelling_platform.launch.py motion_preview:=true

Researcher controls: http://localhost:8770/
Robot face: http://localhost:8771/

Preview mode prints nod requests without sending motor commands.
The launch does not start a motor driver.

## Moving to M

Override robot_root, m_sarg_root, python_executable, and log_dir
with the actual paths on M.

Use face_kiosk:=true to request automatic full-screen face opening.
Verify the display session, microphone, speaker, network access,
and physical movements on M before study use.

Real nod requests require motion_preview:=false and a running
m-sarg servo driver. Physical nod settings remain unverified on M.
