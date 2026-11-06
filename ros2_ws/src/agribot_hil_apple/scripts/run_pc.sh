#!/usr/bin/env bash
set -eo pipefail
ws="${AGRIBOT_WS:-$HOME/agribot_ws/ros2_ws}"
source "$ws/install/setup.bash"
source "$HOME/.local/state/agribot-hil-20261105/install/setup.bash"
source "$ws/src/agribot_hil_apple/scripts/hil_env.sh" pc
export DISPLAY="${DISPLAY:-:0}"
exec ros2 launch agribot_hil_apple pc.launch.py "$@"
