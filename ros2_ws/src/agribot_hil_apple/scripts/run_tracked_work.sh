#!/usr/bin/env bash
set -eo pipefail
mode="${1:-}"
role="${mode#orchard_}"
[[ "$mode" == pc || "$mode" == rdk || "$mode" == orchard_pc || "$mode" == orchard_rdk ]] || {
  printf 'usage: %s pc|rdk|orchard_pc|orchard_rdk [launch arguments]\n' "$0" >&2; exit 2;
}
shift
package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${AGRIBOT_WS:-$HOME/agribot_ws/ros2_ws}/install/setup.bash"
if [[ "$role" == pc ]]; then
  source "$HOME/.local/state/agribot-isobus-hil-20261109/install/setup.bash"
else
  source "$HOME/agribot_isobus_hil_ws/install/setup.bash"
fi
launch="tracked_work_${role}.launch.py"
[[ "$mode" == orchard_* ]] && launch="tracked_orchard_${role}.launch.py"
source "$package/scripts/hil_env.sh" "$role"
exec ros2 launch agribot_hil_apple "$launch" "$@"
