#!/usr/bin/env bash
set -euo pipefail

config_file="${AGRIBOT_CAMERA_TRIGGER_CONFIG:-/etc/default/agribot-camera-trigger}"
if [[ -r "${config_file}" ]]; then
  # shellcheck disable=SC1090
  source "${config_file}"
fi

backend="${CAMERA_TRIGGER_BACKEND:-pin32_pwm}"
pps_device="${CAMERA_TRIGGER_PPS_DEVICE:-/dev/pps-rtk}"
[[ -c "${pps_device}" ]] || {
  echo "错误：RTK PPS字符设备不存在：${pps_device}" >&2
  exit 1
}
case "${backend}" in
  pin32_pwm)
    helper="/usr/local/sbin/agribot-camera-trigger-pps-lock"
    description="RTK PPS -> Pin 32/PWM6（初始对相+持续PPS监测）"
    ;;
  j14_lpwm)
    helper="/usr/local/sbin/agribot-camera-trigger-lpwm"
    description="RTK PPS -> Pin 33/TIME_SYNC2 -> LPWM1 -> J14 Pin 18"
    ;;
  *)
    echo "错误：未知相机触发后端${backend}" >&2
    exit 1
    ;;
esac
[[ -x "${helper}" ]] || {
  echo "错误：相机触发程序不可执行：${helper}" >&2
  exit 1
}

systemctl is-active --quiet agribot-camera-trigger.service
sudo /usr/local/sbin/agribot-camera-trigger-pwm status
echo "相机触发：${description}"

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
echo "相机驱动内部实测取流频率："
rate_output="$(timeout 8 ros2 topic echo \
  /agribot_right_camera/frame_rate_hz std_msgs/msg/Float32 \
  --once --qos-reliability reliable --qos-durability transient_local 2>&1)" || {
  echo "错误：未收到相机驱动内部频率状态：${rate_output}" >&2
  exit 1
}
echo "${rate_output}"
measured_rate="$(awk '$1 == "data:" {print $2; exit}' <<<"${rate_output}")"
[[ "${measured_rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "错误：相机取流频率格式无效" >&2
  exit 1
}
awk -v rate="${measured_rate}" 'BEGIN {exit !(rate >= 9.5 && rate <= 10.5)}' || {
  echo "错误：相机取流频率${measured_rate} Hz不在[9.5,10.5] Hz范围内" >&2
  exit 1
}
