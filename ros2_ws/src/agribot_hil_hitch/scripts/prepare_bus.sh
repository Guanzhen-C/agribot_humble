#!/usr/bin/env bash
set -euo pipefail
[[ "$EUID" == 0 ]] || { printf 'Run with sudo after stopping the previous HIL session.\n' >&2; exit 2; }
role="${1:-}"
if [[ "$role" == pc ]]; then
  device="$(readlink -f /sys/class/net/can0/device)"
  usb="$device"
  while [[ "$usb" != / && ! -f "$usb/idVendor" ]]; do usb="$(dirname "$usb")"; done
  [[ -f "$usb/serial" && "$(<"$usb/serial")" == 003000224856511520343739 &&
      "$(basename "$(readlink -f "$device/driver")")" == gs_usb ]] || {
    printf 'Wrong physical PC USB-CAN. No interface was changed.\n' >&2; exit 2;
  }
  ip link set can0 down
  ip link set can0 type can bitrate 250000 restart-ms 0
  ip link set can0 up
  ip -details link show can0
elif [[ "$role" == rdk || "$role" == sil ]]; then
  modprobe vcan
  interfaces=(vcan74)
  [[ "$role" == sil ]] && interfaces+=(vcan75)
  for interface in "${interfaces[@]}"; do
    if ! ip link show "$interface" >/dev/null 2>&1; then
      ip link add "$interface" type vcan
    fi
    [[ "$(readlink -f "/sys/class/net/$interface")" == "/sys/devices/virtual/net/$interface" &&
        "$(cat "/sys/class/net/$interface/type")" == 280 ]] || exit 2
    ip link set "$interface" up
  done
else
  printf 'usage: %s pc|rdk|sil\n' "$0" >&2; exit 2
fi
