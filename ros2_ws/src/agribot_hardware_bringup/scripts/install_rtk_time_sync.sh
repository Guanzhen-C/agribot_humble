#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${script_dir}/../config/time_sync/rtk_time_sync.env" ]]; then
  share_dir="$(cd -- "${script_dir}/.." && pwd)"
else
  install_prefix="$(cd -- "${script_dir}/../.." && pwd)"
  share_dir="${install_prefix}/share/agribot_hardware_bringup"
fi

if [[ ${EUID} -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

for command in chronyc python3 systemctl; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "缺少命令：${command}" >&2
    exit 1
  fi
done
if ! python3 -c 'import serial' >/dev/null 2>&1; then
  echo "缺少 python3-serial，请先执行：sudo apt install python3-serial" >&2
  exit 1
fi

serial_device="$(
  sed -n 's/^RTK_TIME_SERIAL_DEVICE=//p' \
    "${share_dir}/config/time_sync/rtk_time_sync.env"
)"
if [[ ! -c "${serial_device}" ]]; then
  echo "RDK UART设备不存在：${serial_device}" >&2
  echo "请确认RDK X5已启用40-pin UART1并重启。" >&2
  exit 1
fi
serial_unit="serial-getty@$(basename -- "${serial_device}").service"
if systemctl is-active --quiet "${serial_unit}"; then
  echo "${serial_device}正被${serial_unit}占用，请先关闭该串口控制台。" >&2
  exit 1
fi

install -d -m 0755 \
  /etc/chrony/conf.d /etc/default /etc/systemd/system /usr/local/sbin
install -m 0644 \
  "${share_dir}/config/time_sync/rtk_time_sync.env" \
  /etc/default/agribot-rtk-time
install -m 0644 \
  "${share_dir}/config/time_sync/rtk-pps.conf" \
  /etc/chrony/conf.d/rtk-pps.conf
install -m 0755 \
  "${script_dir}/rtk_time_feeder.py" \
  /usr/local/sbin/agribot-rtk-time-feeder
install -m 0644 \
  "${share_dir}/systemd/agribot-rtk-time.service" \
  /etc/systemd/system/agribot-rtk-time.service

systemctl stop agribot-rtk-time.service 2>/dev/null || true
systemctl daemon-reload
systemctl enable agribot-rtk-time.service
systemctl restart chrony.service
systemctl restart agribot-rtk-time.service

systemctl --no-pager --full status agribot-rtk-time.service
chronyc sources -v
