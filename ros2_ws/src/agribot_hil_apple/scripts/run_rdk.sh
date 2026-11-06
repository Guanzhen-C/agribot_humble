#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/agribot_ws/ros2_ws/install/setup.bash"
root="$HOME/agribot_hil_ws"
if [[ -r "$root/install/setup.bash" ]]; then
  source "$root/install/setup.bash"
fi
source "$root/src/agribot_hil_apple/scripts/hil_env.sh" rdk
exec ros2 launch "$root/rdk.launch.py" "$@"
