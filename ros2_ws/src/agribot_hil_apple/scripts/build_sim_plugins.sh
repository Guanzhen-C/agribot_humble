#!/usr/bin/env bash
set -eo pipefail
package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
root="${HIL_OVERLAY_ROOT:-$HOME/.local/state/agribot-hil-20261105}"
source /opt/ros/humble/setup.bash
source "${AGRIBOT_WS:-$HOME/agribot_ws/ros2_ws}/install/setup.bash"
export PATH=/usr/bin:/bin
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}:/opt/ros/humble"
cmake -S "$package/plugins" -B "$root/build/hil_sim_plugins" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="$root/plugins"
cmake --build "$root/build/hil_sim_plugins" --parallel 2
