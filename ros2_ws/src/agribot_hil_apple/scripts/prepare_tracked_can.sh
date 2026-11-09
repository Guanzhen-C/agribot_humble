#!/usr/bin/env bash
# Only the serial-guarded PC adapter; the old 1 Mbps Ackermann helper is intact.
set -euo pipefail
interface="${1:-can0}"
expected=003000224856511520343739
device="$(readlink -f "/sys/class/net/$interface/device")"
[[ "$(basename "$(readlink -f "$device/driver")")" == gs_usb ]]
[[ "$(<"$device/../serial")" == "$expected" ]]
[[ "$EUID" == 0 ]] || { printf 'Run with sudo.\n' >&2; exit 2; }
ip link set "$interface" down
ip link set "$interface" type can bitrate 250000 restart-ms 0 fd off
ip link set "$interface" up
ip -details -statistics link show "$interface"
