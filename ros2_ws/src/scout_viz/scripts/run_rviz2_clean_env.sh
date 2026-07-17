#!/usr/bin/env bash

set -eo pipefail

if [ "$#" -lt 1 ]; then
  echo "usage: $0 <rviz_config>" >&2
  exit 2
fi

RVIZ_CONFIG="$1"

# RViz2 crashes in this workspace when it inherits the ROS 1 overlay.
# Launch it from a clean Humble environment and keep only the GUI variables.
export HOME="${HOME:-$(getent passwd "$(id -u)" | cut -d: -f6)}"
export USER="${USER:-$(id -un)}"
export LOGNAME="${LOGNAME:-$USER}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

select_display() {
  if command -v xdpyinfo >/dev/null 2>&1; then
    for candidate in "${DISPLAY:-}" ":1" ":0"; do
      if [ -n "$candidate" ] && xdpyinfo -display "$candidate" >/dev/null 2>&1; then
        echo "$candidate"
        return 0
      fi
    done
  fi

  if [ -n "${DISPLAY:-}" ]; then
    echo "$DISPLAY"
  else
    echo ":0"
  fi
}

export DISPLAY="$(select_display)"

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset ROS_DISTRO
unset ROS_ETC_DIR
unset ROS_MASTER_URI
unset ROS_PACKAGE_PATH
unset ROS_PYTHON_VERSION
unset ROS_ROOT
unset ROS_VERSION

source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_setup="${AGRIBOT_WORKSPACE_SETUP:-}"
if [ -z "${workspace_setup}" ]; then
  for candidate in \
    "${SCRIPT_DIR}/../../../../local_setup.bash" \
    "${SCRIPT_DIR}/../../../install/local_setup.bash"; do
    if [ -r "${candidate}" ]; then
      workspace_setup="${candidate}"
      break
    fi
  done
fi

if [ -z "${workspace_setup}" ] || [ ! -r "${workspace_setup}" ]; then
  echo "Agribot workspace setup not found. Build the workspace first." >&2
  exit 1
fi
source "${workspace_setup}"

exec rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
