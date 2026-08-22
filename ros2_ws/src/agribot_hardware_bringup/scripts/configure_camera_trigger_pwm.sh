#!/usr/bin/env bash
set -euo pipefail

action="${1:-status}"
sysfs_root="${AGRIBOT_PWM_SYSFS_ROOT:-/sys/class/pwm}"
chip="${PWM_CHIP:-pwmchip0}"
channel="${PWM_CHANNEL:-0}"
expected_alias="${PWM_EXPECTED_ALIAS:-pwm3}"
period_ns="${PWM_PERIOD_NS:-100000000}"
duty_cycle_ns="${PWM_DUTY_CYCLE_NS:-1000000}"
polarity="${PWM_POLARITY:-normal}"
chip_path="${sysfs_root}/${chip}"
pwm_path="${chip_path}/pwm${channel}"

die() {
  echo "错误：$*" >&2
  exit 1
}

require_unsigned_integer() {
  local name="$1"
  local value="$2"
  [[ "${value}" =~ ^[0-9]+$ ]] || die "${name}必须是非负整数，当前值=${value}"
}

require_unsigned_integer PWM_CHANNEL "${channel}"
require_unsigned_integer PWM_PERIOD_NS "${period_ns}"
require_unsigned_integer PWM_DUTY_CYCLE_NS "${duty_cycle_ns}"
(( period_ns > 0 )) || die "PWM_PERIOD_NS必须大于0"
(( duty_cycle_ns > 0 && duty_cycle_ns < period_ns )) || \
  die "PWM_DUTY_CYCLE_NS必须大于0且小于PWM_PERIOD_NS"
[[ "${polarity}" == "normal" || "${polarity}" == "inversed" ]] || \
  die "PWM_POLARITY必须是normal或inversed"
[[ -d "${chip_path}" ]] || die "PWM控制器不存在：${chip_path}"

if [[ -n "${expected_alias}" && -r "${chip_path}/device/uevent" ]]; then
  if ! grep -qx "OF_ALIAS_0=${expected_alias}" "${chip_path}/device/uevent"; then
    die "${chip}不是期望的${expected_alias}，拒绝操作以免驱动错误引脚"
  fi
fi

wait_for_pwm_path() {
  local attempt
  for attempt in {1..50}; do
    [[ -d "${pwm_path}" ]] && return 0
    sleep 0.02
  done
  die "导出PWM通道后未出现：${pwm_path}"
}

export_channel() {
  if [[ ! -d "${pwm_path}" ]]; then
    printf '%s\n' "${channel}" > "${chip_path}/export"
    wait_for_pwm_path
  fi
}

read_value() {
  tr -d '[:space:]' < "${pwm_path}/$1"
}

show_status() {
  [[ -d "${pwm_path}" ]] || die "PWM通道尚未导出：${pwm_path}"
  local actual_period actual_duty actual_enable actual_polarity
  actual_period="$(read_value period)"
  actual_duty="$(read_value duty_cycle)"
  actual_enable="$(read_value enable)"
  actual_polarity="$(read_value polarity)"
  echo "PWM路径：${pwm_path}"
  echo "周期：${actual_period} ns"
  echo "高电平：${actual_duty} ns"
  echo "极性：${actual_polarity}"
  echo "使能：${actual_enable}"
  [[ "${actual_period}" == "${period_ns}" ]] || die "PWM周期不符合配置"
  [[ "${actual_duty}" == "${duty_cycle_ns}" ]] || die "PWM高电平时间不符合配置"
  [[ "${actual_polarity}" == "${polarity}" ]] || die "PWM极性不符合配置"
  [[ "${actual_enable}" == "1" ]] || die "PWM尚未使能"
}

start_pwm() {
  if [[ ${EUID} -ne 0 && "${sysfs_root}" == "/sys/class/pwm" ]]; then
    die "启动PWM需要root权限"
  fi
  export_channel

  if [[ "$(read_value enable)" == "1" ]]; then
    printf '0\n' > "${pwm_path}/enable"
  fi

  local current_period
  current_period="$(read_value period)"
  if [[ "${current_period}" != "0" ]]; then
    printf '0\n' > "${pwm_path}/duty_cycle"
  fi
  printf '%s\n' "${period_ns}" > "${pwm_path}/period"
  printf '%s\n' "${duty_cycle_ns}" > "${pwm_path}/duty_cycle"
  printf '%s\n' "${polarity}" > "${pwm_path}/polarity"
  printf '1\n' > "${pwm_path}/enable"
  show_status
}

stop_pwm() {
  if [[ ${EUID} -ne 0 && "${sysfs_root}" == "/sys/class/pwm" ]]; then
    die "停止PWM需要root权限"
  fi
  if [[ -d "${pwm_path}" && "$(read_value enable)" == "1" ]]; then
    printf '0\n' > "${pwm_path}/enable"
  fi
  echo "相机触发PWM已停止"
}

case "${action}" in
  start)
    start_pwm
    ;;
  stop)
    stop_pwm
    ;;
  status)
    show_status
    ;;
  *)
    die "用法：$0 {start|stop|status}"
    ;;
esac
