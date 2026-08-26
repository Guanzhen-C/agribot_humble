#!/usr/bin/env bash
set -euo pipefail

config_file="${AGRIBOT_CAMERA_TRIGGER_CONFIG:-/etc/default/agribot-camera-trigger}"
if [[ -z "${CAMERA_TRIGGER_BACKEND+x}" && \
  "${AGRIBOT_PWM_SYSFS_ROOT:-/sys/class/pwm}" == "/sys/class/pwm" && \
  -r "${config_file}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${config_file}"
  set +a
fi

action="${1:-status}"
backend="${CAMERA_TRIGGER_BACKEND:-pin32_pwm}"
sysfs_root="${AGRIBOT_PWM_SYSFS_ROOT:-/sys/class/pwm}"
ready_file="${CAMERA_TRIGGER_READY_FILE:-/run/agribot-camera-trigger/ready}"

pwm_chip="${PWM_CHIP:-pwmchip0}"
pwm_channel="${PWM_CHANNEL:-0}"
pwm_expected_alias="${PWM_EXPECTED_ALIAS:-pwm3}"
pwm_period_ns="${PWM_PERIOD_NS:-100000000}"
pwm_duty_ns="${PWM_DUTY_CYCLE_NS:-1000000}"
pwm_polarity="${PWM_POLARITY:-normal}"
pwm_edge_gpio_chip="${PWM_EDGE_GPIO_CHIP:-/dev/gpiochip5}"
pwm_edge_gpio_offset="${PWM_EDGE_GPIO_OFFSET:-10}"
pwm_edge_gpio_global="${PWM_EDGE_GPIO_GLOBAL:-357}"
pwm_edge_buffer_path="${PWM_EDGE_BUFFER_PATH:-/run/agribot-camera-trigger/physical_edges.bin}"
pwm_edge_gpio_pinmux_value="${PWM_EDGE_GPIO_PINMUX_VALUE:-1}"
pwm_chip_path="${sysfs_root}/${pwm_chip}"
pwm_path="${pwm_chip_path}/pwm${pwm_channel}"

devmem_command="${AGRIBOT_DEVMEM_COMMAND:-devmem}"
lpwm_chip="${LPWM_CHIP:-pwmchip2}"
lpwm_channel="${LPWM_CHANNEL:-0}"
lpwm_expected_alias="${LPWM_EXPECTED_ALIAS:-lpwm1}"
lpwm_expected_driver="${LPWM_EXPECTED_DRIVER:-hobot-lpwm}"
lpwm_device="${LPWM_DEVICE:-/dev/hobot-lpwm1}"
lpwm_channel_id="${LPWM_CHANNEL_ID:-4}"
lpwm_trigger_source="${LPWM_TRIGGER_SOURCE:-6}"
lpwm_period_us="${LPWM_PERIOD_US:-100000}"
lpwm_offset_us="${LPWM_OFFSET_US:-10}"
lpwm_duty_us="${LPWM_DUTY_US:-1000}"
lpwm_threshold_us="${LPWM_THRESHOLD_US:-0}"
lpwm_adjust_step="${LPWM_ADJUST_STEP:-0}"
lpwm_pinmux_register="${LPWM_TIME_SYNC_PINMUX_REGISTER:-0x34180080}"
lpwm_pinmux_shift="${LPWM_TIME_SYNC_PINMUX_SHIFT:-22}"
lpwm_pinmux_width="${LPWM_TIME_SYNC_PINMUX_WIDTH:-2}"
lpwm_pinmux_value="${LPWM_TIME_SYNC_PINMUX_VALUE:-2}"
lpwm_chip_path="${sysfs_root}/${lpwm_chip}"
lpwm_config_info="${lpwm_chip_path}/device/lpwm_config_info"

die() {
  echo "错误：$*" >&2
  exit 1
}

require_unsigned_integer() {
  local name="$1"
  local value="$2"
  [[ "${value}" =~ ^[0-9]+$ ]] || die "${name}必须是非负整数，当前值=${value}"
}

require_root_for_hardware() {
  if [[ ${EUID} -ne 0 && "${sysfs_root}" == "/sys/class/pwm" ]]; then
    die "配置相机触发输出需要root权限"
  fi
}

validate_backend() {
  [[ "${backend}" == "pin32_pwm" || "${backend}" == "j14_lpwm" ]] || \
    die "CAMERA_TRIGGER_BACKEND必须是pin32_pwm或j14_lpwm"
}

read_value() {
  tr -d '[:space:]' < "$1"
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
    die "触发服务状态${name}=${actual:-缺失}，期望${expected}"
}

assert_ready_process() {
  local pid
  pid="$(ready_value pid)"
  [[ "${pid}" =~ ^[0-9]+$ ]] || die "触发服务PID无效"
  [[ -d "/proc/${pid}" ]] || die "触发服务进程${pid}不存在"
}

verify_pwm_device() {
  [[ -d "${pwm_chip_path}" ]] || die "Pin32 PWM控制器不存在：${pwm_chip_path}"
  [[ -r "${pwm_chip_path}/device/uevent" ]] || die "无法读取Pin32 PWM uevent"
  grep -qx "OF_ALIAS_0=${pwm_expected_alias}" "${pwm_chip_path}/device/uevent" || \
    die "${pwm_chip}不是期望的${pwm_expected_alias}，拒绝操作"
  [[ -r "${pwm_chip_path}/npwm" ]] || die "无法读取${pwm_chip}通道数"
  local count
  count="$(read_value "${pwm_chip_path}/npwm")"
  [[ "${count}" =~ ^[0-9]+$ ]] || die "Pin32 PWM通道数格式错误"
  (( pwm_channel < count )) || die "Pin32 PWM通道${pwm_channel}不存在"
}

wait_for_pwm_path() {
  local attempt
  for attempt in {1..50}; do
    [[ -d "${pwm_path}" ]] && return 0
    sleep 0.02
  done
  die "导出PWM通道后未出现：${pwm_path}"
}

export_pwm_channel() {
  if [[ ! -d "${pwm_path}" ]]; then
    printf '%s\n' "${pwm_channel}" > "${pwm_chip_path}/export"
    wait_for_pwm_path
  fi
}

disable_pin32_pwm() {
  if [[ -d "${pwm_path}" && "$(read_value "${pwm_path}/enable")" == "1" ]]; then
    printf '0\n' > "${pwm_path}/enable"
  fi
}

prepare_pin32_pwm() {
  verify_pwm_device
  for item in \
    "PWM_CHANNEL:${pwm_channel}" \
    "PWM_PERIOD_NS:${pwm_period_ns}" \
    "PWM_DUTY_CYCLE_NS:${pwm_duty_ns}"; do
    require_unsigned_integer "${item%%:*}" "${item#*:}"
  done
  (( pwm_period_ns > 0 )) || die "PWM_PERIOD_NS必须大于0"
  (( pwm_duty_ns > 0 && pwm_duty_ns < pwm_period_ns )) || \
    die "PWM_DUTY_CYCLE_NS必须大于0且小于PWM_PERIOD_NS"
  [[ "${pwm_polarity}" == "normal" || "${pwm_polarity}" == "inversed" ]] || \
    die "PWM_POLARITY必须是normal或inversed"

  export_pwm_channel
  disable_pin32_pwm
  if [[ "$(read_value "${pwm_path}/period")" != "0" ]]; then
    printf '0\n' > "${pwm_path}/duty_cycle"
  fi
  printf '%s\n' "${pwm_period_ns}" > "${pwm_path}/period"
  printf '%s\n' "${pwm_duty_ns}" > "${pwm_path}/duty_cycle"
  printf '%s\n' "${pwm_polarity}" > "${pwm_path}/polarity"
  if [[ "${sysfs_root}" == "/sys/class/pwm" ]]; then
    [[ -c "${pwm_edge_gpio_chip}" ]] || \
      die "Pin33 GPIO控制器不存在：${pwm_edge_gpio_chip}"
    require_unsigned_integer "PWM_EDGE_GPIO_OFFSET" "${pwm_edge_gpio_offset}"
    require_unsigned_integer "PWM_EDGE_GPIO_GLOBAL" "${pwm_edge_gpio_global}"
    if [[ -d "/sys/class/gpio/gpio${pwm_edge_gpio_global}" ]]; then
      printf '%s\n' "${pwm_edge_gpio_global}" > /sys/class/gpio/unexport
    fi
    set_pin33_gpio_mux
  fi
  echo "Pin32 PWM已准备：10 Hz；Pin33物理沿回采已准备；等待逐PPS校相"
}

show_pin32_status() {
  verify_pwm_device
  [[ -d "${pwm_path}" ]] || die "Pin32 PWM通道尚未导出：${pwm_path}"
  [[ -r "${ready_file}" ]] || die "触发服务尚未发布就绪状态：${ready_file}"
  assert_ready_value backend pin32_pwm
  assert_ready_value pwm_enable_path "${pwm_path}/enable"
  assert_ready_value period_ns "${pwm_period_ns}"
  assert_ready_value duty_cycle_ns "${pwm_duty_ns}"
  assert_ready_value polarity "${pwm_polarity}"
  assert_ready_value pps_alignment every_pps
  assert_ready_value pps_monitoring continuous
  assert_ready_value pwm_rearm before_each_pps
  assert_ready_value physical_edge_capture pin33_gpio
  assert_ready_value edge_timestamp_source gpio_v2_realtime
  assert_ready_value edge_gpio_chip "${pwm_edge_gpio_chip}"
  assert_ready_value edge_gpio_offset "${pwm_edge_gpio_offset}"
  assert_ready_value edge_buffer_path "${pwm_edge_buffer_path}"
  assert_ready_process

  [[ "$(read_value "${pwm_path}/period")" == "${pwm_period_ns}" ]] || \
    die "Pin32 PWM周期不符合配置"
  [[ "$(read_value "${pwm_path}/duty_cycle")" == "${pwm_duty_ns}" ]] || \
    die "Pin32 PWM高电平时间不符合配置"
  [[ "$(read_value "${pwm_path}/polarity")" == "${pwm_polarity}" ]] || \
    die "Pin32 PWM极性不符合配置"
  [[ "$(read_value "${pwm_path}/enable")" == "1" ]] || die "Pin32 PWM尚未使能"
  [[ -r "${pwm_edge_buffer_path}" ]] || die "Pin33物理沿缓冲不可读"

  local expected_edges
  expected_edges=$((1000000000 / pwm_period_ns))
  [[ "$(ready_value edges_previous_second)" == "${expected_edges}" ]] || \
    die "最近一个PPS周期的物理沿数不等于${expected_edges}"

  echo "当前后端：pin32_pwm"
  echo "相机触发输出：40Pin物理Pin 32（PWM6）"
  echo "物理沿回采：40Pin物理Pin 33（GPIO357，内核CLOCK_REALTIME时间戳）"
  echo "PPS对相：每个PPS前预关断并在PPS后重启，严格保持每秒${expected_edges}沿"
  echo "最近PPS序号：$(ready_value pps_sequence)"
  echo "最近物理沿相位误差：$(ready_value edge_phase_error_us) us"
  echo "最近一秒物理沿数：$(ready_value edges_previous_second)"
}

verify_lpwm_device() {
  [[ -d "${lpwm_chip_path}" ]] || die "LPWM控制器不存在：${lpwm_chip_path}"
  [[ -r "${lpwm_chip_path}/device/uevent" ]] || die "无法读取LPWM uevent"
  grep -qx "DRIVER=${lpwm_expected_driver}" "${lpwm_chip_path}/device/uevent" || \
    die "${lpwm_chip}不是${lpwm_expected_driver}驱动，拒绝操作"
  grep -qx "OF_ALIAS_0=${lpwm_expected_alias}" "${lpwm_chip_path}/device/uevent" || \
    die "${lpwm_chip}不是期望的${lpwm_expected_alias}，拒绝操作"
  [[ -r "${lpwm_chip_path}/npwm" ]] || die "无法读取${lpwm_chip}通道数"
  local count
  count="$(read_value "${lpwm_chip_path}/npwm")"
  [[ "${count}" =~ ^[0-9]+$ ]] || die "LPWM通道数格式错误"
  (( lpwm_channel < count )) || die "LPWM通道${lpwm_channel}不存在"
  [[ -r "${lpwm_config_info}" ]] || die "LPWM配置状态不存在：${lpwm_config_info}"
  if [[ "${sysfs_root}" == "/sys/class/pwm" ]]; then
    [[ -c "${lpwm_device}" ]] || die "LPWM字符设备不存在：${lpwm_device}"
  fi
}

validate_j14_configuration() {
  for item in \
    "LPWM_CHANNEL:${lpwm_channel}" \
    "LPWM_CHANNEL_ID:${lpwm_channel_id}" \
    "LPWM_TRIGGER_SOURCE:${lpwm_trigger_source}" \
    "LPWM_PERIOD_US:${lpwm_period_us}" \
    "LPWM_OFFSET_US:${lpwm_offset_us}" \
    "LPWM_DUTY_US:${lpwm_duty_us}" \
    "LPWM_THRESHOLD_US:${lpwm_threshold_us}" \
    "LPWM_ADJUST_STEP:${lpwm_adjust_step}" \
    "LPWM_TIME_SYNC_PINMUX_SHIFT:${lpwm_pinmux_shift}" \
    "LPWM_TIME_SYNC_PINMUX_WIDTH:${lpwm_pinmux_width}" \
    "LPWM_TIME_SYNC_PINMUX_VALUE:${lpwm_pinmux_value}"; do
    require_unsigned_integer "${item%%:*}" "${item#*:}"
  done

  (( lpwm_channel < 4 )) || die "LPWM_CHANNEL必须小于4"
  (( lpwm_channel_id == 4 + lpwm_channel )) || \
    die "LPWM1的全局通道号应为4+LPWM_CHANNEL"
  (( lpwm_trigger_source == 6 )) || \
    die "TIME_SYNC2的LPWM_TRIGGER_SOURCE必须为6"
  (( lpwm_period_us >= 2 && lpwm_period_us <= 1000000 )) || \
    die "LPWM_PERIOD_US必须在[2,1000000]范围内"
  (( lpwm_duty_us > 0 && lpwm_duty_us <= 4000 && \
    lpwm_duty_us < lpwm_period_us )) || \
    die "LPWM_DUTY_US必须在[1,4000]范围内且小于周期"
  (( lpwm_offset_us + lpwm_duty_us <= lpwm_period_us )) || \
    die "LPWM_OFFSET_US与LPWM_DUTY_US之和不能超过周期"
  (( lpwm_threshold_us <= 65535 )) || \
    die "LPWM_THRESHOLD_US必须小于等于65535"
  (( lpwm_adjust_step <= 15 )) || die "LPWM_ADJUST_STEP必须小于等于15"
  (( lpwm_pinmux_width > 0 && lpwm_pinmux_width < 32 )) || \
    die "LPWM_TIME_SYNC_PINMUX_WIDTH必须在[1,31]范围内"
  (( lpwm_pinmux_value < (1 << lpwm_pinmux_width) )) || \
    die "LPWM_TIME_SYNC_PINMUX_VALUE超出位宽"
}

read_pinmux_register() {
  local value
  value="$("${devmem_command}" "${lpwm_pinmux_register}" 32)" || \
    die "读取TIME_SYNC2复用寄存器失败"
  value="$(tr -d '[:space:]' <<<"${value}")"
  [[ "${value}" =~ ^0[xX][0-9a-fA-F]+$ || "${value}" =~ ^[0-9]+$ ]] || \
    die "TIME_SYNC2复用寄存器返回值无效：${value}"
  printf '%s\n' "$((value))"
}

set_pin33_gpio_mux() {
  command -v "${devmem_command}" >/dev/null 2>&1 || \
    [[ -x "${devmem_command}" ]] || die "未找到devmem命令：${devmem_command}"
  local current mask updated verified
  current="$(read_pinmux_register)"
  mask=$(( ((1 << lpwm_pinmux_width) - 1) << lpwm_pinmux_shift ))
  updated=$(( (current & ~mask) | (pwm_edge_gpio_pinmux_value << lpwm_pinmux_shift) ))
  if (( updated != current )); then
    "${devmem_command}" "${lpwm_pinmux_register}" 32 \
      "$(printf '0x%08X' "${updated}")" >/dev/null
  fi
  verified="$(read_pinmux_register)"
  (( ((verified >> lpwm_pinmux_shift) & ((1 << lpwm_pinmux_width) - 1)) == \
    pwm_edge_gpio_pinmux_value )) || die "物理Pin 33未成功切换为GPIO输入"
  printf 'Pin33 GPIO输入复用已就绪：寄存器=%s\n' \
    "$(printf '0x%08X' "${verified}")"
}

set_timesync2_mux() {
  command -v "${devmem_command}" >/dev/null 2>&1 || \
    [[ -x "${devmem_command}" ]] || die "未找到devmem命令：${devmem_command}"
  local current mask updated verified
  current="$(read_pinmux_register)"
  mask=$(( ((1 << lpwm_pinmux_width) - 1) << lpwm_pinmux_shift ))
  updated=$(( (current & ~mask) | (lpwm_pinmux_value << lpwm_pinmux_shift) ))
  if (( updated != current )); then
    "${devmem_command}" "${lpwm_pinmux_register}" 32 \
      "$(printf '0x%08X' "${updated}")" >/dev/null
  fi
  verified="$(read_pinmux_register)"
  (( ((verified >> lpwm_pinmux_shift) & ((1 << lpwm_pinmux_width) - 1)) == \
    lpwm_pinmux_value )) || die "物理Pin 33未成功切换为TIME_SYNC2输入"
  printf 'TIME_SYNC2输入已就绪：Pin 33，寄存器=%s\n' \
    "$(printf '0x%08X' "${verified}")"
}

prepare_j14_lpwm() {
  validate_j14_configuration
  verify_lpwm_device
  disable_pin32_pwm
  set_timesync2_mux
  echo "J14 LPWM已准备，等待硬件PPS触发"
}

show_j14_status() {
  validate_j14_configuration
  verify_lpwm_device
  [[ -r "${ready_file}" ]] || die "LPWM服务尚未发布就绪状态：${ready_file}"
  assert_ready_value backend j14_lpwm
  assert_ready_value device "${lpwm_device}"
  assert_ready_value channel_id "${lpwm_channel_id}"
  assert_ready_value trigger_source "${lpwm_trigger_source}"
  assert_ready_value trigger_mode "1"
  assert_ready_value period_us "${lpwm_period_us}"
  assert_ready_value offset_us "${lpwm_offset_us}"
  assert_ready_value duty_us "${lpwm_duty_us}"
  assert_ready_process

  local row expected_period expected_duty
  row="$(awk -v selected="${lpwm_channel}" '$1 == selected {print; exit}' \
    "${lpwm_config_info}")"
  [[ -n "${row}" ]] || die "未找到LPWM通道${lpwm_channel}状态"
  read -r _ actual_source actual_offset actual_period actual_duty \
    actual_threshold actual_adjust occupied <<<"${row}"
  expected_period=$((lpwm_period_us - 1))
  expected_duty=$((lpwm_duty_us - 1))
  [[ "${actual_source}" == "${lpwm_trigger_source}" ]] || die "LPWM触发源不符合配置"
  [[ "${actual_offset}" == "${lpwm_offset_us}" ]] || die "LPWM相位偏移不符合配置"
  [[ "${actual_period}" == "${expected_period}" ]] || die "LPWM周期不符合配置"
  [[ "${actual_duty}" == "${expected_duty}" ]] || die "LPWM高电平时间不符合配置"
  [[ "${actual_threshold}" == "${lpwm_threshold_us}" ]] || die "LPWM同步阈值不符合配置"
  [[ "${actual_adjust}" == "${lpwm_adjust_step}" ]] || die "LPWM调整步长不符合配置"
  [[ "${occupied}" == "CAMSYS" ]] || die "LPWM通道未被CAMSYS占用"

  local register_value
  register_value="$(read_pinmux_register)"
  (( ((register_value >> lpwm_pinmux_shift) & \
    ((1 << lpwm_pinmux_width) - 1)) == lpwm_pinmux_value )) || \
    die "物理Pin 33当前不是TIME_SYNC2输入"

  echo "当前后端：j14_lpwm"
  echo "相机触发输出：J14 Pin 18（CAM1_TRIG_3V3）"
  echo "PPS硬件输入：物理Pin 33（TIME_SYNC2，LPWM source 6）"
  echo "驱动状态：${row}"
}

prepare() {
  require_root_for_hardware
  rm -f "${ready_file}" "${ready_file}.tmp"
  if [[ "${backend}" == "pin32_pwm" ]]; then
    prepare_pin32_pwm
  else
    prepare_j14_lpwm
  fi
}

show_status() {
  if [[ "${backend}" == "pin32_pwm" ]]; then
    show_pin32_status
  else
    show_j14_status
  fi
}

cleanup() {
  require_root_for_hardware
  rm -f "${ready_file}" "${ready_file}.tmp"
  disable_pin32_pwm
  echo "相机触发输出已停止"
}

validate_backend
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
  *)
    die "用法：$0 {prepare|status|cleanup}"
    ;;
esac
