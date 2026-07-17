#!/usr/bin/env bash
set -euo pipefail

readonly WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${WORKSPACE_DIR}"

# Keep Conda's Python and libraries out of CMake discovery. Humble binaries on
# Ubuntu 22.04 are built against the system Python 3.10 and system C++ ABI.
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
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

set +u
source /opt/ros/humble/setup.bash
set -u

# ROS 2 Humble on Ubuntu 22.04 uses Python 3.10. CMake must not select a
# Python interpreter from an active Conda environment.
exec /usr/bin/colcon build \
  --symlink-install \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
  "$@"
