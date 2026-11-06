#!/usr/bin/env bash
# Raise only buffer ceilings; DDS opts in using this package's private XML.
set -euo pipefail
if [[ $EUID != 0 ]]; then
  printf 'Run with sudo; this does not change addresses, routes, or ROS domains.\n' >&2
  exit 1
fi
for key in net.core.rmem_max net.core.wmem_max; do
  current=$(sysctl -n "$key")
  if (( current < 16777216 )); then
    sysctl -w "$key=16777216"
  fi
done
