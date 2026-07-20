#!/usr/bin/env bash
set -euo pipefail

interface="${1:-can0}"
bitrate="${2:-500000}"

if [[ ${EUID} -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

if [[ ! "${interface}" =~ ^can[0-9]+$ ]]; then
  echo "Invalid CAN interface: ${interface}" >&2
  exit 2
fi
if [[ ! "${bitrate}" =~ ^[0-9]+$ ]] || (( bitrate <= 0 )); then
  echo "Invalid CAN bitrate: ${bitrate}" >&2
  exit 2
fi

ip link set "${interface}" down 2>/dev/null || true
ip link set "${interface}" type can bitrate "${bitrate}" restart-ms 100
ip link set "${interface}" up
ip -details link show "${interface}"
