#!/usr/bin/env bash
set -euo pipefail
if (( $# != 2 )); then
  printf 'usage: sudo prepare_can.sh INTERFACE EXPECTED_USB_SERIAL\n' >&2
  exit 2
fi
interface="$1"
expected="$2"
[[ "$interface" =~ ^can[0-9]+$ && "$expected" =~ ^[0-9A-Fa-f]{24}$ ]] || exit 2
if (( EUID != 0 )); then
  printf 'Root is needed only to configure the CAN link.\n' >&2
  exit 2
fi
device="$(readlink -f "/sys/class/net/$interface/device")"
driver="$(basename "$(readlink -f "$device/driver")")"
[[ "$driver" == gs_usb ]] || { printf 'Refusing non-gs_usb device.\n' >&2; exit 2; }
serial="$(tr -d '\n' < "$device/../serial")"
[[ "$serial" == "$expected" ]] || { printf 'USB serial mismatch.\n' >&2; exit 2; }
flags="$(cat "/sys/class/net/$interface/flags")"
if (( flags & 1 )); then
  printf 'Interface is already active; refusing to reset an active bus.\n' >&2
  exit 2
fi
# Some gs_usb versions do not implement automatic bus-off restart. Keeping it
# disabled also prevents an unattended restart from resuming stale commands.
ip link set "$interface" type can bitrate 1000000 sample-point 0.875 restart-ms 0
ip link set "$interface" txqueuelen 1024
ip link set "$interface" up
ip -details -statistics link show "$interface"
