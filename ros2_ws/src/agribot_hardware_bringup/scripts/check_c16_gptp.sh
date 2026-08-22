#!/usr/bin/env bash
set -euo pipefail

interface="${1:-eth0}"
minimum_valid_epoch=1704067200
maximum_phc_offset_sec=0.1
maximum_cloud_offset_sec=2.0

die() {
  echo "错误：$*" >&2
  exit 1
}

systemctl is-active --quiet agribot-c16-phc.service
systemctl is-active --quiet agribot-c16-gptp.service

system_epoch="$(date +%s)"
(( system_epoch >= minimum_valid_epoch )) || \
  die "RDK系统时间尚未取得有效UTC：$(date -u +'%Y-%m-%dT%H:%M:%S%z')"

echo "System UTC: $(date -u +'%Y-%m-%dT%H:%M:%S.%N%z')"
phc_output="$(sudo phc_ctl "${interface}" get 2>&1)"
echo "${phc_output}"
phc_epoch="$(sed -n 's/.*clock time is \([0-9.]*\).*/\1/p' <<<"${phc_output}")"
[[ -n "${phc_epoch}" ]] || die "无法解析${interface}的PHC时间"
system_now="$(date +%s.%N)"
phc_offset="$(awk -v sys="${system_now}" -v phc="${phc_epoch}" \
  'BEGIN { d = sys - phc; if (d < 0) d = -d; printf "%.9f", d }')"
echo "System-PHC absolute offset: ${phc_offset} s"
awk -v value="${phc_offset}" -v limit="${maximum_phc_offset_sec}" \
  'BEGIN { exit !(value <= limit) }' || \
  die "网卡PHC与RDK系统时间偏差超过${maximum_phc_offset_sec}秒"
chronyc tracking

command -v ros2 >/dev/null 2>&1 || \
  die "未找到ros2，请先source ROS和工作区环境"

echo "C16 cloud header:"
cloud_header="$(timeout 8 ros2 topic echo /lidar/points --once --field header)" || \
  die "8秒内没有收到C16点云"
echo "${cloud_header}"
cloud_epoch="$(awk '/sec:/ { print $2; exit }' <<<"${cloud_header}")"
[[ "${cloud_epoch}" =~ ^[0-9]+$ ]] || die "无法解析C16点云时间戳"
cloud_now="$(date +%s.%N)"
cloud_offset="$(awk -v sys="${cloud_now}" -v cloud="${cloud_epoch}" \
  'BEGIN { d = sys - cloud; if (d < 0) d = -d; printf "%.6f", d }')"
echo "System-cloud absolute offset: ${cloud_offset} s"
awk -v value="${cloud_offset}" -v limit="${maximum_cloud_offset_sec}" \
  'BEGIN { exit !(value <= limit) }' || \
  die "C16点云时间戳与RDK系统时间偏差超过${maximum_cloud_offset_sec}秒"

echo "C16 cloud delay (5 second sample):"
delay_output="$(timeout 5 ros2 topic delay /lidar/points --window 30 2>&1)" || {
  result=$?
  [[ ${result} -eq 124 ]] || exit "${result}"
}
echo "${delay_output}"
grep -q "average delay:" <<<"${delay_output}" || die "没有得到C16点云延迟统计"
