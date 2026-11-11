#!/usr/bin/env bash
set -eo pipefail
hitch_role="${1:-}"
[[ "$hitch_role" == pc || "$hitch_role" == rdk ]] || { printf 'usage: %s pc|rdk [launch arguments]\n' "$0" >&2; exit 2; }
shift
package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${AGRIBOT_WS:-$HOME/agribot_ws/ros2_ws}/install/setup.bash"
if [[ "$hitch_role" == pc ]]; then
  source "$HOME/.local/state/agribot-isobus-hil-20261109/install/setup.bash"
  hitch_prefix="${HITCH_HIL_STATE:-$HOME/.local/state/agribot-hitch-hil-20261111}/install"
  source "$hitch_prefix/share/agribot_hil_hitch/local_setup.bash"
  export AMENT_PREFIX_PATH="$hitch_prefix:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$hitch_prefix/lib:${LD_LIBRARY_PATH:-}"
  export GAZEBO_PLUGIN_PATH="$hitch_prefix/lib:${GAZEBO_PLUGIN_PATH:-}"
else
  source "$HOME/agribot_isobus_hil_ws/install/setup.bash"
fi
source "$(ros2 pkg prefix --share agribot_hil_apple)/scripts/hil_env.sh" "$hitch_role"
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=0
export GAZEBO_MASTER_URI=http://127.0.0.1:11358
if [[ "${1:-}" == mission ]]; then
  [[ "$hitch_role" == rdk ]] || { printf 'Run the work mission on X5.\n' >&2; exit 2; }
  shift
  exec /usr/bin/python3 "$package/scripts/mission.py" \
    --manifest "$HOME/agribot_hitch_hil_ws/scenario/mission.json" "$@"
fi
if [[ "${1:-}" == verify ]]; then
  shift
  exec /usr/bin/python3 "$package/scripts/verify_physical_hil.py" "$@"
fi
if [[ "${1:-}" == monitor ]]; then
  shift
  exec /usr/bin/python3 "$package/scripts/monitor_powertrain.py" "$@"
fi
if [[ "${1:-}" == verify-powertrain ]]; then
  shift
  exec /usr/bin/python3 "$package/scripts/verify_powertrain.py" --physical "$@"
fi
exec ros2 launch "$package/launch/$hitch_role.launch.py" "$@"
