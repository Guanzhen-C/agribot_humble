#!/usr/bin/env bash
set -eo pipefail
mode="${1:-}"
role="${mode##*_}"
[[ "$mode" == pc || "$mode" == rdk || "$mode" == orchard_pc || "$mode" == orchard_rdk || "$mode" == shredder_pc || "$mode" == shredder_rdk ]] || {
  printf 'usage: %s pc|rdk|orchard_pc|orchard_rdk|shredder_pc|shredder_rdk [launch arguments]\n' "$0" >&2; exit 2;
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
if [[ "$mode" == shredder_* ]]; then
  launch="tracked_orchard_${role}.launch.py"
  if [[ "$role" == pc ]]; then
    set -- "scene_root:=$HOME/.cache/agribot/isobus/tracked_shredder_orchard" "$@"
  else
    set -- "scene_root:=$HOME/agribot_isobus_hil_ws/scenario_shredder" "$@"
  fi
fi
source "$package/scripts/hil_env.sh" "$role"
exec ros2 launch agribot_hil_apple "$launch" "$@"
