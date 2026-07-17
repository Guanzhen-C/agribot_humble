#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../.." && pwd)"

exec "${workspace_dir}/build_ros2_humble.sh" \
  --packages-up-to livox_ros_driver2 "$@"
