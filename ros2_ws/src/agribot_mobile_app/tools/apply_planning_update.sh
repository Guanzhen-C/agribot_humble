#!/usr/bin/env bash
set -euo pipefail
bundle=$(cd "$(dirname "$0")" && pwd)
cd "$bundle"
sha256sum --check SHA256SUMS
if [[ ${1:-} == --check ]]; then
  echo "Update archive checksums passed; no files changed."
  exit 0
fi
if [[ ${1:-} != --apply || $(uname -m) != aarch64 ]]; then
  echo "On the parked RDK X5, run: bash apply_planning_update.sh --apply" >&2
  exit 1
fi
workspace=${AGRIBOT_WORKSPACE:-$HOME/agribot_ws/ros2_ws}
map_root=${AGRIBOT_MAP_ROOT:-$HOME/agribot_maps/test_site}
map_id=$(python3 -c 'import json; print(json.load(open("manifest.json"))["map_id"])')
pcd_sha=$(python3 -c 'import json; print(json.load(open("manifest.json"))["required_existing_pcd_sha256"])')
[[ -f "$workspace/src/agribot_mobile_app/package.xml" && -f "$map_root/$map_id.pcd" ]]
[[ $(sha256sum "$map_root/$map_id.pcd" | cut -d' ' -f1) == "$pcd_sha" ]] || { echo "Existing PCD mismatch; not installing." >&2; exit 1; }
umask 077
backup=$HOME/agribot_backups/planning_$(date +%Y%m%d_%H%M%S)
mkdir -p "$backup/maps"
if systemctl is-active --quiet agribot-mobile-app.service; then
  curl --noproxy '*' --fail --silent --show-error --max-time 10 http://127.0.0.1:8088/api/v1/state -o "$backup/state_before.json"
  python3 - "$backup/state_before.json" <<'PY'
import json, sys
state = json.load(open(sys.argv[1]))
if any(state.get("processes", {}).get(name, {}).get("running") for name in ("collection", "processing")):
    sys.exit("Stop collection/processing and save data before deploying.")
if state.get("navigation", {}).get("status") in ("sending", "accepted", "executing", "canceling"):
    sys.exit("Stop the active navigation task and park the vehicle before deploying.")
if any(abs(state.get("command", {}).get(name, 0)) > .01 for name in ("linear", "angular")):
    sys.exit("Nonzero command: park the vehicle before deploying.")
PY
  curl --noproxy '*' --fail --silent --show-error --max-time 40 -X POST -H 'Content-Type: application/json' --data '{}' http://127.0.0.1:8088/api/v1/runtime/stop
  sudo systemctl stop agribot-mobile-app.service
fi
tar --exclude=node_modules --exclude=android --exclude=__pycache__ -czf "$backup/mobile_source.tgz" -C "$workspace/src" agribot_mobile_app
cp -a "$workspace/src/agribot_hardware_bringup/differential/config/nav2_params_differential_planner_only.yaml" "$backup/"
cp -a "$workspace/src/agribot_hardware_bringup/differential/config/nav2_params_differential_fastlivo_mapped.yaml" "$backup/"
for file in maps/*; do
  [[ ! -f "$map_root/$(basename "$file")" ]] || cp -a "$map_root/$(basename "$file")" "$backup/maps/"
done
cp -a code/agribot_mobile_app/. "$workspace/src/agribot_mobile_app/"
cp -a code/agribot_hardware_bringup/differential/config/. "$workspace/src/agribot_hardware_bringup/differential/config/"
cp -a maps/. "$map_root/"
cd "$workspace"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
colcon build --packages-select agribot_mobile_app --symlink-install
for name in nav2_params_differential_planner_only.yaml nav2_params_differential_fastlivo_mapped.yaml; do
  source_file=$workspace/src/agribot_hardware_bringup/differential/config/$name
  installed_file=$workspace/install/agribot_hardware_bringup/share/agribot_hardware_bringup/differential/config/$name
  [[ "$source_file" -ef "$installed_file" ]] || cp -a "$source_file" "$installed_file"
done
sudo systemctl start agribot-mobile-app.service
for attempt in {1..20}; do
  if curl --noproxy '*' --fail --silent --max-time 3 "http://127.0.0.1:8088/api/v1/maps/$map_id/routes" -o "$backup/routes_after.json"; then
    python3 - "$backup/routes_after.json" <<'PY'
import json, sys
routes = json.load(open(sys.argv[1]))["routes"]
assert len(routes) == 2 and all(len(route["poses"]) == 53 for route in routes)
print("53-waypoint alternatives are available. No vehicle runtime was started.")
PY
    echo "Backup: $backup"
    exit 0
  fi
  sleep 1
done
echo "Gateway not ready; inspect systemctl/journal. Backup: $backup" >&2
exit 1
