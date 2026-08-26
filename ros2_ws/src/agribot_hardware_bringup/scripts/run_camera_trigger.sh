#!/usr/bin/env bash
set -euo pipefail

backend="${CAMERA_TRIGGER_BACKEND:-pin32_pwm}"
pps_device="${CAMERA_TRIGGER_PPS_DEVICE:-/dev/pps-rtk}"
ready_file="${CAMERA_TRIGGER_READY_FILE:-/run/agribot-camera-trigger/ready}"
timeout_sec="${CAMERA_TRIGGER_PPS_TIMEOUT_SEC:-2.5}"

case "${backend}" in
  pin32_pwm)
    pwm_path="${AGRIBOT_PWM_SYSFS_ROOT:-/sys/class/pwm}/${PWM_CHIP:-pwmchip0}/pwm${PWM_CHANNEL:-0}"
    exec /usr/local/sbin/agribot-camera-trigger-pps-lock \
      --pps-device "${pps_device}" \
      --pwm-enable-path "${pwm_path}/enable" \
      --ready-file "${ready_file}" \
      --edge-gpio-chip "${PWM_EDGE_GPIO_CHIP:-/dev/gpiochip5}" \
      --edge-gpio-offset "${PWM_EDGE_GPIO_OFFSET:-10}" \
      --edge-buffer-path "${PWM_EDGE_BUFFER_PATH:-/run/agribot-camera-trigger/physical_edges.bin}" \
      --period-ns "${PWM_PERIOD_NS:-100000000}" \
      --duty-cycle-ns "${PWM_DUTY_CYCLE_NS:-1000000}" \
      --polarity "${PWM_POLARITY:-normal}" \
      --timeout-sec "${timeout_sec}" \
      --maximum-latency-ms "${PWM_PPS_MAXIMUM_LATENCY_MS:-5.0}" \
      --rearm-guard-ms "${PWM_PPS_REARM_GUARD_MS:-5.0}"
    ;;
  j14_lpwm)
    exec /usr/local/sbin/agribot-camera-trigger-lpwm \
      --device "${LPWM_DEVICE:-/dev/hobot-lpwm1}" \
      --pps-device "${pps_device}" \
      --ready-file "${ready_file}" \
      --channel-id "${LPWM_CHANNEL_ID:-4}" \
      --trigger-source "${LPWM_TRIGGER_SOURCE:-6}" \
      --period-us "${LPWM_PERIOD_US:-100000}" \
      --offset-us "${LPWM_OFFSET_US:-10}" \
      --duty-us "${LPWM_DUTY_US:-1000}" \
      --threshold-us "${LPWM_THRESHOLD_US:-0}" \
      --adjust-step "${LPWM_ADJUST_STEP:-0}" \
      --pps-timeout-sec "${timeout_sec}"
    ;;
  *)
    echo "错误：未知相机触发后端${backend}" >&2
    exit 1
    ;;
esac
