#!/usr/bin/env bash
set -euo pipefail

action="${1:-status}"
sysfs_root="${AGRIBOT_PWM_SYSFS_ROOT:-/sys/class/pwm}"
devmem_command="${AGRIBOT_DEVMEM_COMMAND:-devmem}"
chip="${LPWM_CHIP:-pwmchip2}"
channel="${LPWM_CHANNEL:-0}"
expected_alias="${LPWM_EXPECTED_ALIAS:-lpwm1}"
expected_driver="${LPWM_EXPECTED_DRIVER:-hobot-lpwm}"
device="${LPWM_DEVICE:-/dev/hobot-lpwm1}"
channel_id="${LPWM_CHANNEL_ID:-4}"
trigger_source="${LPWM_TRIGGER_SOURCE:-2}"
period_us="${LPWM_PERIOD_US:-100000}"
offset_us="${LPWM_OFFSET_US:-10}"
duty_us="${LPWM_DUTY_US:-1000}"
threshold_us="${LPWM_THRESHOLD_US:-0}"
adjust_step="${LPWM_ADJUST_STEP:-0}"
ready_file="${LPWM_READY_FILE:-/run/agribot-camera-trigger/ready}"
pinmux_register="${LPWM_TIME_SYNC_PINMUX_REGISTER:-0x34180080}"
pinmux_shift="${LPWM_TIME_SYNC_PINMUX_SHIFT:-22}"
pinmux_width="${LPWM_TIME_SYNC_PINMUX_WIDTH:-2}"
pinmux_value="${LPWM_TIME_SYNC_PINMUX_VALUE:-2}"
legacy_enable_path="${LPWM_LEGACY_PWM_ENABLE_PATH:-/sys/class/pwm/pwmchip0/pwm0/enable}"
chip_path="${sysfs_root}/${chip}"
config_info="${chip_path}/device/lpwm_config_info"

die() {
  echo "错误：$*" >&2
  exit 1
}

require_unsigned_integer() {
  local name="$1"
  local value="$2"
  [[ "${value}" =~ ^[0-9]+$ ]] || die "${name}必须是非负整数，当前值=${value}"
}

for item in \
  "LPWM_CHANNEL:${channel}" \
  "LPWM_CHANNEL_ID:${channel_id}" \
  "LPWM_TRIGGER_SOURCE:${trigger_source}" \
  "LPWM_PERIOD_US:${period_us}" \
  "LPWM_OFFSET_US:${offset_us}" \
  "LPWM_DUTY_US:${duty_us}" \
  "LPWM_THRESHOLD_US:${threshold_us}" \
  "LPWM_ADJUST_STEP:${adjust_step}" \
  "LPWM_TIME_SYNC_PINMUX_SHIFT:${pinmux_shift}" \
  "LPWM_TIME_SYNC_PINMUX_WIDTH:${pinmux_width}" \
  "LPWM_TIME_SYNC_PINMUX_VALUE:${pinmux_value}"; do
  require_unsigned_integer "${item%%:*}" "${item#*:}"
done

(( channel < 4 )) || die "LPWM_CHANNEL必须小于4"
(( channel_id == 4 + channel )) || \
  die "LPWM1的全局通道号应为4+LPWM_CHANNEL"
(( trigger_source <= 10 )) || die "LPWM_TRIGGER_SOURCE必须小于等于10"
(( period_us >= 2 && period_us <= 1000000 )) || \
  die "LPWM_PERIOD_US必须在[2,1000000]范围内"
(( duty_us > 0 && duty_us <= 4000 && duty_us < period_us )) || \
  die "LPWM_DUTY_US必须在[1,4000]范围内且小于周期"
(( offset_us + duty_us <= period_us )) || \
  die "LPWM_OFFSET_US与LPWM_DUTY_US之和不能超过周期"
(( threshold_us <= 65535 )) || die "LPWM_THRESHOLD_US必须小于等于65535"
(( adjust_step <= 15 )) || die "LPWM_ADJUST_STEP必须小于等于15"
(( pinmux_width > 0 && pinmux_width < 32 )) || \
  die "LPWM_TIME_SYNC_PINMUX_WIDTH必须在[1,31]范围内"
(( pinmux_value < (1 << pinmux_width) )) || \
  die "LPWM_TIME_SYNC_PINMUX_VALUE超出位宽"

require_root_for_hardware() {
  if [[ ${EUID} -ne 0 && "${sysfs_root}" == "/sys/class/pwm" ]]; then
    die "配置LPWM和TIME_SYNC2需要root权限"
  fi
}

verify_lpwm_device() {
  [[ -d "${chip_path}" ]] || die "LPWM控制器不存在：${chip_path}"
  [[ -r "${chip_path}/device/uevent" ]] || die "无法读取LPWM uevent"
  grep -qx "DRIVER=${expected_driver}" "${chip_path}/device/uevent" || \
    die "${chip}不是${expected_driver}驱动，拒绝操作"
  grep -qx "OF_ALIAS_0=${expected_alias}" "${chip_path}/device/uevent" || \
    die "${chip}不是期望的${expected_alias}，拒绝操作"
  [[ -r "${chip_path}/npwm" ]] || die "无法读取${chip}通道数"
  local channel_count
  channel_count="$(tr -d '[:space:]' < "${chip_path}/npwm")"
  [[ "${channel_count}" =~ ^[0-9]+$ ]] || die "LPWM通道数格式错误"
  (( channel < channel_count )) || die "LPWM通道${channel}不存在"
  [[ -r "${config_info}" ]] || die "LPWM配置状态不存在：${config_info}"
  if [[ "${sysfs_root}" == "/sys/class/pwm" ]]; then
    [[ -c "${device}" ]] || die "LPWM字符设备不存在：${device}"
  fi
}

read_pinmux_register() {
  local value
  value="$("${devmem_command}" "${pinmux_register}" 32)" || \
    die "读取TIME_SYNC2复用寄存器失败"
  value="$(tr -d '[:space:]' <<<"${value}")"
  [[ "${value}" =~ ^0[xX][0-9a-fA-F]+$ || "${value}" =~ ^[0-9]+$ ]] || \
    die "TIME_SYNC2复用寄存器返回值无效：${value}"
  printf '%s\n' "$((value))"
}

set_timesync2_mux() {
  command -v "${devmem_command}" >/dev/null 2>&1 || \
    [[ -x "${devmem_command}" ]] || die "未找到devmem命令：${devmem_command}"
  local current mask updated verified
  current="$(read_pinmux_register)"
  mask=$(( ((1 << pinmux_width) - 1) << pinmux_shift ))
  updated=$(( (current & ~mask) | (pinmux_value << pinmux_shift) ))
  if (( updated != current )); then
    "${devmem_command}" "${pinmux_register}" 32 "$(printf '0x%08X' "${updated}")" >/dev/null
  fi
  verified="$(read_pinmux_register)"
  (( ((verified >> pinmux_shift) & ((1 << pinmux_width) - 1)) == pinmux_value )) || \
    die "物理Pin 33未成功切换为TIME_SYNC2输入"
  printf 'TIME_SYNC2输入已就绪：Pin 33，寄存器=%s\n' \
    "$(printf '0x%08X' "${verified}")"
}

disable_legacy_pwm() {
  if [[ -w "${legacy_enable_path}" ]] && \
    [[ "$(tr -d '[:space:]' < "${legacy_enable_path}")" == "1" ]]; then
    printf '0\n' > "${legacy_enable_path}"
    echo "旧Pin 32普通PWM已停止"
  fi
}

ready_value() {
  local name="$1"
  awk -F= -v key="${name}" '$1 == key {print substr($0, length(key) + 2); exit}' \
    "${ready_file}"
}

assert_ready_value() {
  local name="$1"
  local expected="$2"
  local actual
  actual="$(ready_value "${name}")"
  [[ "${actual}" == "${expected}" ]] || \
    die "LPWM就绪状态${name}=${actual:-缺失}，期望${expected}"
}

show_status() {
  verify_lpwm_device
  [[ -r "${ready_file}" ]] || die "LPWM服务尚未发布就绪状态：${ready_file}"
  assert_ready_value device "${device}"
  assert_ready_value channel_id "${channel_id}"
  assert_ready_value trigger_source "${trigger_source}"
  assert_ready_value trigger_mode "1"
  assert_ready_value period_us "${period_us}"
  assert_ready_value offset_us "${offset_us}"
  assert_ready_value duty_us "${duty_us}"

  local pid row expected_period expected_duty
  pid="$(ready_value pid)"
  [[ "${pid}" =~ ^[0-9]+$ ]] || die "LPWM服务PID无效"
  [[ -d "/proc/${pid}" ]] || die "LPWM服务进程${pid}不存在"

  row="$(awk -v selected="${channel}" '$1 == selected {print; exit}' "${config_info}")"
  [[ -n "${row}" ]] || die "未找到LPWM通道${channel}状态"
  read -r _ actual_source actual_offset actual_period actual_duty \
    actual_threshold actual_adjust occupied <<<"${row}"
  expected_period=$((period_us - 1))
  expected_duty=$((duty_us - 1))
  [[ "${actual_source}" == "${trigger_source}" ]] || die "LPWM触发源不符合配置"
  [[ "${actual_offset}" == "${offset_us}" ]] || die "LPWM相位偏移不符合配置"
  [[ "${actual_period}" == "${expected_period}" ]] || die "LPWM周期不符合配置"
  [[ "${actual_duty}" == "${expected_duty}" ]] || die "LPWM高电平时间不符合配置"
  [[ "${actual_threshold}" == "${threshold_us}" ]] || die "LPWM同步阈值不符合配置"
  [[ "${actual_adjust}" == "${adjust_step}" ]] || die "LPWM调整步长不符合配置"
  [[ "${occupied}" == "CAMSYS" ]] || die "LPWM通道未被CAMSYS硬件触发程序占用"

  local register_value
  register_value="$(read_pinmux_register)"
  (( ((register_value >> pinmux_shift) & ((1 << pinmux_width) - 1)) == pinmux_value )) || \
    die "物理Pin 33当前不是TIME_SYNC2输入"

  echo "LPWM设备：${device}"
  echo "LPWM输出：J14 Pin 18（CAM1_TRIG_3V3）"
  echo "PPS硬触发输入：物理Pin 33（TIME_SYNC2/SGT1）"
  echo "配置：10 Hz，PPS后${offset_us} us输出${duty_us} us高电平"
  echo "驱动状态：${row}"
}

prepare() {
  require_root_for_hardware
  verify_lpwm_device
  disable_legacy_pwm
  set_timesync2_mux
  rm -f "${ready_file}" "${ready_file}.tmp"
}

cleanup() {
  require_root_for_hardware
  rm -f "${ready_file}" "${ready_file}.tmp"
  disable_legacy_pwm
}

case "${action}" in
  prepare)
    prepare
    ;;
  status)
    show_status
    ;;
  cleanup)
    cleanup
    ;;
  disable-legacy)
    require_root_for_hardware
    disable_legacy_pwm
    ;;
  *)
    die "用法：$0 {prepare|status|cleanup|disable-legacy}"
    ;;
esac
