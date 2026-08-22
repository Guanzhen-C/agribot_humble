#!/usr/bin/env bash
set -euo pipefail

config_file="${AGRIBOT_CAMERA_TRIGGER_CONFIG:-/etc/default/agribot-camera-trigger}"
if [[ -r "${config_file}" ]]; then
  # shellcheck disable=SC1090
  source "${config_file}"
fi

systemctl is-active --quiet agribot-camera-trigger.service
/usr/local/sbin/agribot-camera-trigger-pwm status

if command -v ros2 >/dev/null 2>&1 && \
  ros2 node list 2>/dev/null | grep -qx '/agribot_right_camera'; then
  echo "相机硬触发参数："
  ros2 param get /agribot_right_camera trigger_enable
  ros2 param get /agribot_right_camera trigger_selector
  ros2 param get /agribot_right_camera trigger_source
  ros2 param get /agribot_right_camera trigger_activation
  echo "相机图像频率（5秒）："
  timeout 5 ros2 topic hz /camera/rgb/image_raw --window 30 || test $? -eq 124
fi
