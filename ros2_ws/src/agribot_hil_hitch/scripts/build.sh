#!/usr/bin/env bash
set -eo pipefail
package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${AGRIBOT_WS:-$HOME/agribot_ws/ros2_ws}"
scene_flags=()
for option in "$@"; do
  if [[ "$option" == --powertrain ]]; then scene_flags=(--powertrain); fi
done
if (( ${#scene_flags[@]} )); then
  state="${HITCH_HIL_STATE:-$HOME/.local/state/agribot-powertrain-hil-20261111}"
else
  state="${HITCH_HIL_STATE:-$HOME/.local/state/agribot-hitch-hil-20261111}"
fi
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
export PATH=/usr/bin:/bin
export CMAKE_PREFIX_PATH="/opt/ros/humble:${CMAKE_PREFIX_PATH:-}"
bash "$workspace/src/agribot_hil_apple/scripts/fetch_isobus_stack.sh" >/dev/null
cmake -S "$package/protocol" -B "$state/build-protocol" -DBUILD_ISOBUS=ON
cmake --build "$state/build-protocol" --parallel 2
ctest --test-dir "$state/build-protocol" --output-on-failure
cmake -S "$package" -B "$state/build-plugin" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_INSTALL_PREFIX="$state/install"
cmake --build "$state/build-plugin" --parallel 2
cmake --install "$state/build-plugin"
for action in "$@"; do
  case "$action" in
    --firmware)
      export HPM_SDK_BASE="${HPM_SDK_BASE:-$workspace/hpm6750evk2/.hpm/hpm_sdk}"
      export GNURISCV_TOOLCHAIN_PATH="${GNURISCV_TOOLCHAIN_PATH:-$workspace/hpm6750evk2/.hpm/toolchain-shim}"
      cmake -S "$package/firmware" -B "$state/build-hpm" -DBOARD=hpm6750evk2 -DHPM_BUILD_TYPE=flash_xip
      cmake --build "$state/build-hpm" --parallel 2
      ;;
    --prepare)
      meshes="${HITCH_MESH_ROOT:-$state/meshes}"
      scene="${HITCH_SCENE_ROOT:-$state/scenario}"
      blender -b -t 2 --python-exit-code 1 --python "$package/scripts/prepare_meshes.py" -- \
        "$workspace/src/agribot_vehicle_description/generated/car4_harrow/models" "$meshes"
      python3 "$package/scripts/prepare_scene.py" --description "$workspace/src/agribot_vehicle_description" \
        --meshes "$meshes" --world "$HOME/.cache/agribot/scenarios/wide_headland/world.sdf" \
        --farm-assets "$workspace/src/agribot_farm_sim/assets/wide_headland" --output "$scene" "${scene_flags[@]}"
      python3 "$package/scripts/prepare_runtime.py" --description "$workspace/src/agribot_vehicle_description" --scene "$scene"
      ;;
    --powertrain) ;;
    *) printf 'Unknown argument: %s\n' "$action" >&2; exit 2 ;;
  esac
done
