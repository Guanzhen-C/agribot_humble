#!/usr/bin/env bash
set -euo pipefail
if (( EUID != 0 )); then
  printf 'Run with sudo; creates only internal vcan73, no physical CAN changes.\n' >&2
  exit 2
fi
modprobe vcan
if [[ -e /sys/class/net/vcan73 ]]; then
  ip -details link show vcan73 | grep -q 'vcan' || exit 2
else
  ip link add dev vcan73 type vcan
fi
ip link set vcan73 up
ip -details link show vcan73
