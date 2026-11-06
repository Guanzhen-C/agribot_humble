#!/usr/bin/env bash
# Source this in HIL-only terminals; do not put it into the physical robot's bashrc.
role="${1:-pc}"
if [[ "$role" != pc && "$role" != rdk ]]; then
  printf 'usage: source hil_env.sh pc|rdk\n' >&2
  return 2 2>/dev/null || exit 2
fi
hil_share="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble is not installed.\n' >&2
  return 2 2>/dev/null || exit 2
fi
source /opt/ros/humble/setup.bash
if [[ -d "$hil_share/agribot_hil_apple" ]]; then
  export PYTHONPATH="$hil_share${PYTHONPATH:+:$PYTHONPATH}"
fi
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$hil_share/config/dds_$role.xml"
unset role hil_share
