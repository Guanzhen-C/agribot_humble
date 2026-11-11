#!/usr/bin/env bash
set -euo pipefail
[[ "$EUID" == 0 ]] || { printf 'Run this PC-only network preparation with sudo.\n' >&2; exit 2; }
interface="${HITCH_NET_INTERFACE:-enx207bd2b013d1}"
ip -4 -o address show dev "$interface" | awk '{print $4}' | grep -qx '192.168.1.1/24' || {
  printf 'Dedicated PC address is absent; no firewall rule was changed.\n' >&2; exit 2;
}
# Domain 74, participant indices 0..119; only this peer and wired interface.
ufw allow in on "$interface" proto udp from 192.168.1.102 to 192.168.1.1 \
  port 25900:26149 comment agribot-hitch-hil-dds-74
