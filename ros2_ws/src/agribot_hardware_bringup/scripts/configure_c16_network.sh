#!/usr/bin/env bash
set -euo pipefail

interface="${1:-eno1}"
host_ip="${2:-192.168.1.102}"
lidar_ip="${3:-192.168.1.200}"

if [[ ${EUID} -ne 0 ]]; then
  echo "This command configures ${interface}; rerun it with sudo." >&2
  exit 1
fi

ip link set dev "${interface}" up
ip address replace "${host_ip}/32" dev "${interface}"
ip route replace "${lidar_ip}/32" dev "${interface}" src "${host_ip}"

echo "Configured ${interface}: ${host_ip}/32 -> ${lidar_ip}/32"
