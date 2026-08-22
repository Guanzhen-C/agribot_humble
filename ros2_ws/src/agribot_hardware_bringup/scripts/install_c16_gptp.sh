#!/usr/bin/env bash
set -euo pipefail

interface="${1:-eth0}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${script_dir}/../config/time_sync/c16_gptp.cfg" ]]; then
  share_dir="$(cd -- "${script_dir}/.." && pwd)"
else
  install_prefix="$(cd -- "${script_dir}/../.." && pwd)"
  share_dir="${install_prefix}/share/agribot_hardware_bringup"
fi

if [[ ${EUID} -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

for command in chronyc ethtool ip phc2sys phc_ctl ptp4l systemctl; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "Missing required command: ${command}" >&2
    exit 1
  fi
done

if [[ ! -d "/sys/class/net/${interface}" ]]; then
  echo "Network interface does not exist: ${interface}" >&2
  exit 1
fi

timestamp_info="$(ethtool -T "${interface}")"
if ! grep -q "hardware-raw-clock" <<<"${timestamp_info}"; then
  echo "${interface} does not support a PTP hardware clock" >&2
  exit 1
fi
if grep -q "PTP Hardware Clock: none" <<<"${timestamp_info}"; then
  echo "${interface} has no usable PTP hardware clock" >&2
  exit 1
fi

install -d -m 0755 /etc/agribot /etc/default /etc/systemd/system
install -m 0644 \
  "${share_dir}/config/time_sync/c16_gptp.cfg" \
  /etc/agribot/c16_gptp.cfg
install -m 0644 \
  "${share_dir}/systemd/agribot-c16-phc.service" \
  /etc/systemd/system/agribot-c16-phc.service
install -m 0644 \
  "${share_dir}/systemd/agribot-c16-gptp.service" \
  /etc/systemd/system/agribot-c16-gptp.service
printf 'PTP_INTERFACE=%s\n' "${interface}" \
  > /etc/default/agribot-c16-gptp

systemctl daemon-reload
systemctl enable agribot-c16-phc.service agribot-c16-gptp.service
systemctl restart agribot-c16-phc.service
systemctl restart agribot-c16-gptp.service

systemctl --no-pager --full status \
  agribot-c16-phc.service agribot-c16-gptp.service
