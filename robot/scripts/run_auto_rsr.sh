#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$workspace/install/local_setup.bash"
cd "$workspace"
# Explicit URL prevents a development/local URL from leaking into study mode.
exec ros2 launch "$workspace/src/bloom_node/launch/m_sarg_combined.launch.py" \
    base_url:=https://dev-bloom.unr.dev kiosk:=true
