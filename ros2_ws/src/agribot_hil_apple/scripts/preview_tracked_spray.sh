#!/usr/bin/env bash
set -eo pipefail
seconds="${1:-60}"
[[ "$seconds" =~ ^[0-9]+$ ]] && (( seconds >= 1 && seconds <= 300 )) || {
  printf 'usage: %s [duration_seconds: 1..300]\n' "$0" >&2
  exit 2
}
package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${AGRIBOT_WS:-$HOME/agribot_ws/ros2_ws}/install/setup.bash"
source "$HOME/.local/state/agribot-isobus-hil-20261109/install/setup.bash"
source "$package/scripts/hil_env.sh" pc
# The X5 ISOBUS client still sends this intent through the physical 6750 chain.
# Never publish actuator RPM or drive commands from the visual preview.
trap 'timeout 4 ros2 param set /hil_tracked_gazebo_can_bridge work_authorized false || true' EXIT
trap 'exit 0' INT TERM
timeout 5 ros2 param set /hil_tracked_gazebo_can_bridge motion_authorized false
timeout 5 ros2 param set /hil_tracked_gazebo_can_bridge work_authorized true
ros2 topic pub --rate 10 --times "$(( seconds*10 ))" /hil/work/command std_msgs/msg/String \
  '{data: "{\"fan_gear\":3,\"pump_gear\":4,\"shredder_level\":0}"}' >/dev/null
