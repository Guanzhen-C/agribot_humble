#!/usr/bin/env bash
set -euo pipefail

interface="${1:-eth0}"

systemctl is-active --quiet agribot-c16-phc.service
systemctl is-active --quiet agribot-c16-gptp.service

echo "System UTC: $(date -u +'%Y-%m-%dT%H:%M:%S.%N%z')"
sudo phc_ctl "${interface}" get
chronyc tracking

if command -v ros2 >/dev/null 2>&1; then
  echo "C16 cloud header:"
  timeout 5 ros2 topic echo /lidar/points --once --field header
  echo "C16 cloud delay (5 second sample):"
  timeout 5 ros2 topic delay /lidar/points --window 30 || test $? -eq 124
fi
