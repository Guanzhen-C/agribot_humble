#!/usr/bin/env bash

_ROS2_HUMBLE_SETUP="/opt/ros/humble/setup.bash"
_ROS2_HUMBLE_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_ROS2_HUMBLE_NOUNSET=0
case $- in
  *u*) _ROS2_HUMBLE_NOUNSET=1; set +u ;;
esac

if [[ ! -r "${_ROS2_HUMBLE_SETUP}" ]]; then
  echo "ROS 2 Humble is not installed. Run ${_ROS2_HUMBLE_WORKSPACE}/install_ros2_humble.sh first." >&2
  [[ "${_ROS2_HUMBLE_NOUNSET}" -eq 1 ]] && set -u
  unset _ROS2_HUMBLE_SETUP _ROS2_HUMBLE_WORKSPACE _ROS2_HUMBLE_NOUNSET
  return 1 2>/dev/null || exit 1
fi

# Do not retain paths from another ROS distribution in this shell.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR ROS_MASTER_URI ROS_ROOT
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

# Humble on Ubuntu 22.04 uses the system Python 3.10. ROS executables that use
# `#!/usr/bin/env python3` must not resolve to a Conda interpreter.
clean_path=""
IFS=: read -ra path_entries <<< "${PATH}"
for entry in "${path_entries[@]}"; do
  if [[ "${entry}" == *miniconda* || "${entry}" == *anaconda* ]]; then
    continue
  fi
  clean_path="${clean_path:+${clean_path}:}${entry}"
done
export PATH="${clean_path}"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE
unset PYTHONHOME PYTHONPATH

source "${_ROS2_HUMBLE_SETUP}"
if [[ -r "${_ROS2_HUMBLE_WORKSPACE}/install/local_setup.bash" ]]; then
  source "${_ROS2_HUMBLE_WORKSPACE}/install/local_setup.bash"
fi

# Package hooks add workspace models to GAZEBO_MODEL_PATH. Once that variable
# is set, Gazebo 11 no longer falls back to its system model directory.
GAZEBO_SYSTEM_MODEL_PATH="/usr/share/gazebo-11/models"
if [[ -d "${GAZEBO_SYSTEM_MODEL_PATH}" ]]; then
  case ":${GAZEBO_MODEL_PATH:-}:" in
    *":${GAZEBO_SYSTEM_MODEL_PATH}:"*) ;;
    *) export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+${GAZEBO_MODEL_PATH}:}${GAZEBO_SYSTEM_MODEL_PATH}" ;;
  esac
fi

[[ "${_ROS2_HUMBLE_NOUNSET}" -eq 1 ]] && set -u
unset _ROS2_HUMBLE_SETUP _ROS2_HUMBLE_WORKSPACE _ROS2_HUMBLE_NOUNSET
unset clean_path path_entries entry GAZEBO_SYSTEM_MODEL_PATH
