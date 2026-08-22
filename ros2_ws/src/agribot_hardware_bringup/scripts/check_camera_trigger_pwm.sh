#!/usr/bin/env bash
set -euo pipefail

config_file="${AGRIBOT_CAMERA_TRIGGER_CONFIG:-/etc/default/agribot-camera-trigger}"
if [[ -r "${config_file}" ]]; then
  # shellcheck disable=SC1090
  source "${config_file}"
fi

systemctl is-active --quiet agribot-camera-trigger.service
/usr/local/sbin/agribot-camera-trigger-pwm status

get_camera_parameter() {
  local name="$1"
  local expected="$2"
  local output=""
  local attempt
  for attempt in 1 2 3; do
    if output="$(timeout 8 ros2 param get /agribot_right_camera "${name}" 2>&1)"; then
      echo "${output}"
      grep -Fq "is: ${expected}" <<<"${output}" || {
        echo "错误：相机参数${name}不是期望值${expected}" >&2
        return 1
      }
      return 0
    fi
    sleep 1
  done
  echo "错误：读取相机参数${name}失败：${output}" >&2
  return 1
}

command -v ros2 >/dev/null 2>&1 || {
  echo "错误：未找到ros2，请先source ROS和工作区环境" >&2
  exit 1
}

camera_found=false
for attempt in 1 2 3; do
  if timeout 8 ros2 node list 2>/dev/null | grep -qx '/agribot_right_camera'; then
    camera_found=true
    break
  fi
  sleep 1
done
[[ "${camera_found}" == "true" ]] || {
  echo "错误：未发现/agribot_right_camera，相机节点尚未运行" >&2
  exit 1
}

echo "相机硬触发参数："
get_camera_parameter trigger_enable True
get_camera_parameter trigger_selector FrameBurstStart
get_camera_parameter trigger_source Line0
get_camera_parameter trigger_activation RisingEdge
echo "相机图像频率（5秒）："
rate_output="$(timeout 6 ros2 topic hz /camera/rgb/image_raw --window 30 2>&1)" || {
  result=$?
  [[ ${result} -eq 124 ]] || exit "${result}"
}
echo "${rate_output}"
grep -q "average rate:" <<<"${rate_output}" || {
  echo "错误：5秒内没有收到相机图像" >&2
  exit 1
}
