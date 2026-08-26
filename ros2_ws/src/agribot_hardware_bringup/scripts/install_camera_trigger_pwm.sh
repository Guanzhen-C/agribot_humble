#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${script_dir}/../config/time_sync/camera_trigger_pwm.env" ]]; then
  share_dir="$(cd -- "${script_dir}/.." && pwd)"
else
  install_prefix="$(cd -- "${script_dir}/../.." && pwd)"
  share_dir="${install_prefix}/share/agribot_hardware_bringup"
fi

if [[ ${EUID} -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

if systemctl is-active --quiet agribot-camera-trigger.service; then
  # Stop the old Pin 32 service with its currently loaded unit before replacing
  # the unit file. This prevents two independent trigger sources from running.
  systemctl stop agribot-camera-trigger.service
fi

install -d -m 0755 /etc/default /etc/systemd/system /usr/local/sbin
install -m 0644 \
  "${share_dir}/config/time_sync/camera_trigger_pwm.env" \
  /etc/default/agribot-camera-trigger
install -m 0755 \
  "${script_dir}/configure_camera_trigger_pwm.sh" \
  /usr/local/sbin/agribot-camera-trigger-pwm
install -m 0755 \
  "${script_dir}/camera_trigger_lpwm" \
  /usr/local/sbin/agribot-camera-trigger-lpwm
install -m 0644 \
  "${share_dir}/systemd/agribot-camera-trigger.service" \
  /etc/systemd/system/agribot-camera-trigger.service

systemctl daemon-reload
systemctl enable agribot-camera-trigger.service
systemctl restart agribot-camera-trigger.service
systemctl --no-pager --full status agribot-camera-trigger.service
