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

# Large PCL/GTSAM translation units can each consume roughly 2 GiB while
# compiling. Bound both package-level and per-package concurrency by default so
# high-core-count hosts do not exhaust memory. Explicit environment variables
# or a command-line --parallel-workers value still override these defaults.
memory_kib=$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)
if ((memory_kib < 12 * 1024 * 1024)); then
  default_package_workers=1
  default_build_jobs=1
else
  default_package_workers=2
  default_build_jobs=2
fi

export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-${AGRIBOT_BUILD_JOBS_PER_PACKAGE:-$default_build_jobs}}"

colcon_concurrency_args=()
has_parallel_workers=false
for arg in "$@"; do
  if [[ "$arg" == "--parallel-workers" || "$arg" == --parallel-workers=* ]]; then
    has_parallel_workers=true
    break
  fi
done
if [[ "$has_parallel_workers" == false ]]; then
  colcon_concurrency_args=(
    --parallel-workers
    "${AGRIBOT_BUILD_PACKAGE_WORKERS:-$default_package_workers}"
  )
fi

# ROS 2 Humble on Ubuntu 22.04 uses Python 3.10. CMake must not select a
# Python interpreter from an active Conda environment.
exec /usr/bin/colcon build \
  --symlink-install \
  "${colcon_concurrency_args[@]}" \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
  "$@"
