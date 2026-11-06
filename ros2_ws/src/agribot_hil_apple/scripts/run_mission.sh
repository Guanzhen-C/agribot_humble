#!/usr/bin/env bash
set -eo pipefail
output="${1:?Usage: run_mission.sh REPORT_DIRECTORY}"
source /opt/ros/humble/setup.bash
source "$HOME/agribot_ws/ros2_ws/install/setup.bash"
root="$HOME/agribot_hil_ws"
source "$root/install/setup.bash"
source "$root/src/agribot_hil_apple/scripts/hil_env.sh" rdk
exec python3 -m agribot_hil_apple.mission --execute --minimum-fruit 2 --maximum-fruit 2 --output "$output"
