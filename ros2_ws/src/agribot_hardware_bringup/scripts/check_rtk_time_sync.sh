#!/usr/bin/env bash
set -euo pipefail

environment_file=/etc/default/agribot-rtk-time
if [[ ! -r "${environment_file}" ]]; then
  echo "授时配置不存在：${environment_file}" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${environment_file}"

failure=0
if [[ ! -c "${RTK_TIME_SERIAL_DEVICE}" ]]; then
  echo "错误：TTL串口不存在：${RTK_TIME_SERIAL_DEVICE}" >&2
  failure=1
fi
if [[ ! -c /dev/pps-rtk ]]; then
  echo "错误：PPS设备不存在：/dev/pps-rtk" >&2
  failure=1
fi
if ! systemctl is-active --quiet agribot-rtk-time.service; then
  echo "错误：agribot-rtk-time.service未运行" >&2
  failure=1
fi
if ! systemctl is-active --quiet chrony.service; then
  echo "错误：chrony.service未运行" >&2
  failure=1
fi

echo "=== RTK TTL授时服务 ==="
systemctl --no-pager --full status agribot-rtk-time.service || true
echo "=== Chrony时间源 ==="
chronyc sources -v || failure=1
echo "=== Chrony跟踪状态 ==="
chronyc tracking || failure=1
echo "=== 最近授时日志 ==="
journalctl -u agribot-rtk-time.service -n 20 --no-pager || true

exit "${failure}"
