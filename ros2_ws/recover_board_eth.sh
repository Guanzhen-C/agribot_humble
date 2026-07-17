#!/usr/bin/env bash

set -euo pipefail

CONNECTION_NAME="${CONNECTION_NAME:-Wired connection 1}"
INTERFACE_NAME="${INTERFACE_NAME:-enx207bd2b013d1}"
LOCAL_CIDR="${LOCAL_CIDR:-192.168.137.1/24}"
BOARD_IP="${BOARD_IP:-192.168.137.64}"
BOARD_USER="${BOARD_USER:-sunrise}"
PING_COUNT="${PING_COUNT:-3}"

echo "[1/5] Checking NetworkManager connection profile..."
if ! nmcli -g NAME connection show | grep -Fxq "${CONNECTION_NAME}"; then
    echo "Connection profile not found: ${CONNECTION_NAME}" >&2
    echo "Available profiles:" >&2
    nmcli -f NAME,TYPE,DEVICE connection show >&2
    exit 1
fi

echo "[2/5] Applying static IPv4 settings to ${CONNECTION_NAME}..."
nmcli connection modify "${CONNECTION_NAME}" \
    connection.interface-name "${INTERFACE_NAME}" \
    ipv4.method manual \
    ipv4.addresses "${LOCAL_CIDR}" \
    ipv6.method ignore \
    connection.autoconnect yes

echo "[3/5] Bringing up ${CONNECTION_NAME}..."
nmcli connection up "${CONNECTION_NAME}" >/dev/null

echo "[4/5] Verifying local interface state..."
ip -br addr show dev "${INTERFACE_NAME}"
ip route | grep -F "${LOCAL_CIDR%/*}" || true

echo "[5/5] Testing reachability to ${BOARD_IP}..."
if ping -c "${PING_COUNT}" -W 1 "${BOARD_IP}"; then
    echo
    echo "Board is reachable."
    echo "SSH test:"
    echo "  ssh ${BOARD_USER}@${BOARD_IP}"
else
    echo
    echo "Board is still unreachable." >&2
    echo "Check these items:" >&2
    echo "  1. Board power" >&2
    echo "  2. Ethernet cable / USB NIC" >&2
    echo "  3. Board IP is still ${BOARD_IP}" >&2
    exit 1
fi
