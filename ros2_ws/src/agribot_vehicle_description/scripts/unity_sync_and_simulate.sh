#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE="${AGRIBOT_ROS_WORKSPACE:-$HOME/agribot_ws/ros2_ws}"
EXPORT_DIR=""
GUI="${AGRIBOT_SIM_GUI:-true}"
RVIZ="${AGRIBOT_SIM_RVIZ:-true}"
RUN_WAYPOINTS="${AGRIBOT_SIM_RUN_WAYPOINTS:-true}"
ALGORITHM="${AGRIBOT_SIM_ALGORITHM:-fastlivo_rtk_mppi}"
PERCEPTION="${AGRIBOT_SIM_PERCEPTION:-stvl}"
VISION="${AGRIBOT_SIM_VISION:-off}"
LOCALIZATION="${AGRIBOT_SIM_LOCALIZATION:-fastlivo_rtk}"
PLANNER="${AGRIBOT_SIM_PLANNER:-smac_hybrid}"
CONTROLLER="${AGRIBOT_SIM_CONTROLLER:-mppi}"
PREPARE_ONLY=false
STOP_ONLY=false
LIST_ALGORITHMS=false
LIST_COMPONENTS=false
VALIDATE_ALGORITHMS=false
RESOLVE_ONLY=false
ALGORITHM_EXPLICIT=false
COMPONENTS_EXPLICIT=false

[[ -n "${AGRIBOT_SIM_ALGORITHM+x}" ]] && ALGORITHM_EXPLICIT=true
if [[ -n "${AGRIBOT_SIM_PERCEPTION+x}" ||
      -n "${AGRIBOT_SIM_VISION+x}" ||
      -n "${AGRIBOT_SIM_LOCALIZATION+x}" ||
      -n "${AGRIBOT_SIM_PLANNER+x}" ||
      -n "${AGRIBOT_SIM_CONTROLLER+x}" ]]; then
  COMPONENTS_EXPLICIT=true
fi

PERCEPTION_COMPONENTS=(stvl voxel)
VISION_COMPONENTS=(off canny orb optical_flow)
LOCALIZATION_COMPONENTS=(fastlivo_rtk fast_lio navsat kiss_icp)
PLANNER_COMPONENTS=(smac_hybrid navfn theta_star smac_2d direct)
CONTROLLER_COMPONENTS=(mppi rpp dwb)

component_is_available() {
  local requested="$1"
  shift
  local candidate
  for candidate in "$@"; do
    [[ "$requested" == "$candidate" ]] && return 0
  done
  return 1
}

perception_name() {
  case "$1" in
    stvl) NAME_RESULT="STVL时空体素" ;;
    voxel) NAME_RESULT="Nav2三维体素" ;;
  esac
}

vision_name() {
  case "$1" in
    off) NAME_RESULT="关闭视觉感知" ;;
    canny) NAME_RESULT="Canny边缘感知" ;;
    orb) NAME_RESULT="ORB特征感知" ;;
    optical_flow) NAME_RESULT="Farneback光流感知" ;;
  esac
}

localization_name() {
  case "$1" in
    fastlivo_rtk) NAME_RESULT="FAST-LIVO2 + RTK" ;;
    fast_lio) NAME_RESULT="FAST-LIO2" ;;
    navsat) NAME_RESULT="NavSat ESKF" ;;
    kiss_icp) NAME_RESULT="KISS-ICP" ;;
  esac
}

planner_name() {
  case "$1" in
    smac_hybrid) NAME_RESULT="Smac Hybrid-A*" ;;
    navfn) NAME_RESULT="NavFn Dijkstra" ;;
    theta_star) NAME_RESULT="Theta*" ;;
    smac_2d) NAME_RESULT="Smac 2D A*" ;;
    direct) NAME_RESULT="必经点直连" ;;
  esac
}

controller_name() {
  case "$1" in
    mppi) NAME_RESULT="MPPI" ;;
    rpp) NAME_RESULT="RPP" ;;
    dwb) NAME_RESULT="DWB" ;;
  esac
}

list_algorithms() {
  local perception vision localization planner controller id name
  local perception_label vision_label localization_label planner_label controller_label
  for perception in "${PERCEPTION_COMPONENTS[@]}"; do
    perception_name "$perception"; perception_label="$NAME_RESULT"
    for vision in "${VISION_COMPONENTS[@]}"; do
      vision_name "$vision"; vision_label="$NAME_RESULT"
      for localization in "${LOCALIZATION_COMPONENTS[@]}"; do
        localization_name "$localization"; localization_label="$NAME_RESULT"
        for planner in "${PLANNER_COMPONENTS[@]}"; do
          planner_name "$planner"; planner_label="$NAME_RESULT"
          for controller in "${CONTROLLER_COMPONENTS[@]}"; do
            controller_name "$controller"; controller_label="$NAME_RESULT"
            id="${perception}_${vision}_${localization}_${planner}_${controller}"
            name="$perception_label / $vision_label / $localization_label / $planner_label / $controller_label"
            printf '%s|%s|统一接口自由组合：几何感知=%s，视觉感知=%s，定位=%s，规划=%s，控制=%s\n' \
              "$id" "$name" \
              "$perception_label" \
              "$vision_label" \
              "$localization_label" \
              "$planner_label" \
              "$controller_label"
          done
        done
      done
    done
  done
}

list_components() {
  printf 'perception|stvl|STVL时空体素层|C16 PointCloud2输入，带体素衰减和三维清除\n'
  printf 'perception|voxel|Nav2三维体素层|C16 PointCloud2输入，Nav2官方VoxelLayer\n'
  printf 'vision|off|关闭视觉感知|相机仍可供定位使用，不启动额外视觉节点\n'
  printf 'vision|canny|Canny边缘感知|发布独立边缘叠加图，不接入运动控制\n'
  printf 'vision|orb|ORB特征感知|发布独立ORB特征图，不接入运动控制\n'
  printf 'vision|optical_flow|Farneback光流感知|发布独立稠密光流图，不接入运动控制\n'
  printf 'localization|fastlivo_rtk|FAST-LIVO2 + RTK|视觉激光惯性里程计与固定解RTK因子融合\n'
  printf 'localization|fast_lio|FAST-LIO2|C16与IMU激光惯性里程计\n'
  printf 'localization|navsat|NavSat ESKF|RTK与IMU组合导航\n'
  printf 'localization|kiss_icp|KISS-ICP|轻量纯激光里程计\n'
  printf 'planner|smac_hybrid|Smac Hybrid-A*|考虑阿克曼运动学与最小转弯半径\n'
  printf 'planner|navfn|NavFn Dijkstra|经典二维栅格最短路\n'
  printf 'planner|theta_star|Theta*|任意角二维栅格规划\n'
  printf 'planner|smac_2d|Smac 2D A*|带平滑器的二维A*\n'
  printf 'planner|direct|必经点直连|按顺序连接全部预设必经点的基线\n'
  printf 'controller|mppi|MPPI|采样式模型预测控制与实时避障\n'
  printf 'controller|rpp|RPP|调节纯跟踪控制\n'
  printf 'controller|dwb|DWB|动态窗口轨迹评分控制\n'
}

resolve_legacy_algorithm() {
  PERCEPTION=stvl
  VISION=off
  case "$ALGORITHM" in
    fastlivo_rtk_mppi) LOCALIZATION=fastlivo_rtk; PLANNER=smac_hybrid; CONTROLLER=mppi ;;
    fastlio_mppi) LOCALIZATION=fast_lio; PLANNER=smac_hybrid; CONTROLLER=mppi ;;
    navsat_mppi) LOCALIZATION=navsat; PLANNER=smac_hybrid; CONTROLLER=mppi ;;
    fastlivo_rtk_rpp) LOCALIZATION=fastlivo_rtk; PLANNER=smac_hybrid; CONTROLLER=rpp ;;
    fastlio_rpp) LOCALIZATION=fast_lio; PLANNER=smac_hybrid; CONTROLLER=rpp ;;
    navsat_rpp) LOCALIZATION=navsat; PLANNER=smac_hybrid; CONTROLLER=rpp ;;
    kiss_icp_mppi) LOCALIZATION=kiss_icp; PLANNER=smac_hybrid; CONTROLLER=mppi ;;
    fastlivo_rtk_navfn_mppi) LOCALIZATION=fastlivo_rtk; PLANNER=navfn; CONTROLLER=mppi ;;
    fastlivo_rtk_theta_mppi) LOCALIZATION=fastlivo_rtk; PLANNER=theta_star; CONTROLLER=mppi ;;
    fastlivo_rtk_smac2d_mppi) LOCALIZATION=fastlivo_rtk; PLANNER=smac_2d; CONTROLLER=mppi ;;
    fastlivo_rtk_navfn_dwb) LOCALIZATION=fastlivo_rtk; PLANNER=navfn; CONTROLLER=dwb ;;
    fastlivo_rtk_direct_mppi) LOCALIZATION=fastlivo_rtk; PLANNER=direct; CONTROLLER=mppi ;;
    *) return 1 ;;
  esac
}

resolve_canonical_algorithm() {
  local perception vision localization planner controller candidate legacy_candidate
  for perception in "${PERCEPTION_COMPONENTS[@]}"; do
    for vision in "${VISION_COMPONENTS[@]}"; do
      for localization in "${LOCALIZATION_COMPONENTS[@]}"; do
        for planner in "${PLANNER_COMPONENTS[@]}"; do
          for controller in "${CONTROLLER_COMPONENTS[@]}"; do
            candidate="${perception}_${vision}_${localization}_${planner}_${controller}"
            legacy_candidate="${perception}_${localization}_${planner}_${controller}"
            if [[ "$ALGORITHM" == "$candidate" ||
                  ( "$vision" == "off" && "$ALGORITHM" == "$legacy_candidate" ) ]]; then
              PERCEPTION="$perception"
              VISION="$vision"
              LOCALIZATION="$localization"
              PLANNER="$planner"
              CONTROLLER="$controller"
              return 0
            fi
          done
        done
      done
    done
  done
  return 1
}

resolve_components() {
  if [[ "$ALGORITHM_EXPLICIT" == "true" ]]; then
    if ! resolve_legacy_algorithm && ! resolve_canonical_algorithm; then
      echo "ERROR: unknown simulation algorithm: $ALGORITHM" >&2
      return 2
    fi
  fi

  component_is_available "$PERCEPTION" "${PERCEPTION_COMPONENTS[@]}" || {
    echo "ERROR: unsupported perception component: $PERCEPTION" >&2; return 2; }
  component_is_available "$VISION" "${VISION_COMPONENTS[@]}" || {
    echo "ERROR: unsupported visual perception component: $VISION" >&2; return 2; }
  component_is_available "$LOCALIZATION" "${LOCALIZATION_COMPONENTS[@]}" || {
    echo "ERROR: unsupported localization component: $LOCALIZATION" >&2; return 2; }
  component_is_available "$PLANNER" "${PLANNER_COMPONENTS[@]}" || {
    echo "ERROR: unsupported planner component: $PLANNER" >&2; return 2; }
  component_is_available "$CONTROLLER" "${CONTROLLER_COMPONENTS[@]}" || {
    echo "ERROR: unsupported controller component: $CONTROLLER" >&2; return 2; }

  PERCEPTION_MODE="$PERCEPTION"
  VISION_MODE="$VISION"
  LOCALIZATION_MODE="$LOCALIZATION"
  CONTROLLER_MODE="$CONTROLLER"
  ROUTE_MODE=planned
  if [[ "$PLANNER" == "direct" ]]; then
    PLANNER_MODE=smac_hybrid
    ROUTE_MODE=direct
  else
    PLANNER_MODE="$PLANNER"
  fi
  ALGORITHM="${PERCEPTION}_${VISION}_${LOCALIZATION}_${PLANNER}_${CONTROLLER}"
}

validate_algorithm_catalog() {
  local count unique id saved_algorithm="$ALGORITHM"
  count="$(list_algorithms | wc -l)"
  unique="$(list_algorithms | cut -d'|' -f1 | sort -u | wc -l)"
  [[ "$count" -eq 480 ]] || {
    echo "ERROR: expected 480 combinations, got $count" >&2; return 1; }
  [[ "$unique" -eq "$count" ]] || {
    echo "ERROR: duplicate algorithm IDs in catalog" >&2; return 1; }
  while IFS='|' read -r id _; do
    ALGORITHM="$id"
    ALGORITHM_EXPLICIT=true
    resolve_components >/dev/null
  done < <(list_algorithms)
  ALGORITHM="$saved_algorithm"
  printf 'VALIDATED_ALGORITHM_COMBINATIONS=%s\n' "$count"
}

usage() {
  cat <<'EOF'
Usage:
  unity_sync_and_simulate.sh --export-dir PATH [options]
  unity_sync_and_simulate.sh --stop

Options:
  --workspace PATH       ROS 2 workspace (default: ~/agribot_ws/ros2_ws)
  --gui true|false       Open Gazebo GUI (default: true)
  --rviz true|false      Open RViz (default: true)
  --run-waypoints BOOL   Run the preset orchard route (default: true)
  --perception ID        Select perception plugin: stvl or voxel
  --vision ID            Select independent visual perception algorithm
  --localization ID      Select localization adapter
  --planner ID           Select planner plugin or direct path baseline
  --controller ID        Select controller plugin
  --algorithm ID         Backward-compatible combined profile ID
  --list-components      Print independently selectable components
  --list-algorithms      Print all generated component combinations
  --validate-algorithms  Exhaustively validate all generated combinations
  --resolve-only         Resolve selected components without building or running
  --prepare-only         Import and build without restarting simulation
  --stop                 Stop the simulation managed by this script
  -h, --help             Show this help
EOF
}

while (($#)); do
  case "$1" in
    --export-dir)
      EXPORT_DIR="${2:?missing value for --export-dir}"
      shift 2
      ;;
    --workspace)
      WORKSPACE="${2:?missing value for --workspace}"
      shift 2
      ;;
    --gui)
      GUI="${2:?missing value for --gui}"
      shift 2
      ;;
    --rviz)
      RVIZ="${2:?missing value for --rviz}"
      shift 2
      ;;
    --run-waypoints)
      RUN_WAYPOINTS="${2:?missing value for --run-waypoints}"
      shift 2
      ;;
    --algorithm)
      ALGORITHM="${2:?missing value for --algorithm}"
      ALGORITHM_EXPLICIT=true
      shift 2
      ;;
    --perception)
      PERCEPTION="${2:?missing value for --perception}"
      COMPONENTS_EXPLICIT=true
      shift 2
      ;;
    --vision)
      VISION="${2:?missing value for --vision}"
      COMPONENTS_EXPLICIT=true
      shift 2
      ;;
    --localization)
      LOCALIZATION="${2:?missing value for --localization}"
      COMPONENTS_EXPLICIT=true
      shift 2
      ;;
    --planner)
      PLANNER="${2:?missing value for --planner}"
      COMPONENTS_EXPLICIT=true
      shift 2
      ;;
    --controller)
      CONTROLLER="${2:?missing value for --controller}"
      COMPONENTS_EXPLICIT=true
      shift 2
      ;;
    --list-components)
      LIST_COMPONENTS=true
      shift
      ;;
    --list-algorithms)
      LIST_ALGORITHMS=true
      shift
      ;;
    --validate-algorithms)
      VALIDATE_ALGORITHMS=true
      shift
      ;;
    --resolve-only)
      RESOLVE_ONLY=true
      shift
      ;;
    --prepare-only)
      PREPARE_ONLY=true
      shift
      ;;
    --stop)
      STOP_ONLY=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$ALGORITHM_EXPLICIT" == "true" && "$COMPONENTS_EXPLICIT" == "true" ]]; then
  echo "ERROR: --algorithm cannot be combined with individual component options." >&2
  exit 2
fi

if [[ "$LIST_COMPONENTS" == "true" ]]; then
  list_components
  exit 0
fi

if [[ "$LIST_ALGORITHMS" == "true" ]]; then
  list_algorithms
  exit 0
fi

if [[ "$VALIDATE_ALGORITHMS" == "true" ]]; then
  validate_algorithm_catalog
  exit 0
fi

resolve_components

if [[ "$RESOLVE_ONLY" == "true" ]]; then
  printf 'algorithm=%s\nperception=%s\nvision=%s\nlocalization=%s\nplanner=%s\ncontroller=%s\nroute=%s\n' \
    "$ALGORITHM" "$PERCEPTION_MODE" "$VISION_MODE" "$LOCALIZATION_MODE" \
    "$PLANNER_MODE" "$CONTROLLER_MODE" "$ROUTE_MODE"
  exit 0
fi

for value in "$GUI" "$RVIZ" "$RUN_WAYPOINTS"; do
  if [[ "$value" != "true" && "$value" != "false" ]]; then
    echo "ERROR: boolean arguments must be true or false: $value" >&2
    exit 2
  fi
done

STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/agribot-unity-sim"
PID_FILE="$STATE_DIR/simulation.pid"
SID_FILE="$STATE_DIR/simulation.sid"
BOOT_ID_FILE="$STATE_DIR/simulation.boot_id"
LOCK_FILE="$STATE_DIR/sync.lock"
STATUS_FILE="$STATE_DIR/status"
LOG_FILE="$STATE_DIR/latest.log"
mkdir -p "$STATE_DIR"
: >"$LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

set_status() {
  printf '%s\n' "$1" >"$STATUS_FILE"
}

current_boot_id() {
  cat /proc/sys/kernel/random/boot_id
}

managed_session_pids() {
  local sid="$1"
  ps -eo pid=,sid= | awk -v target="$sid" '$2 == target { print $1 }'
}

managed_session_is_running() {
  [[ -s "$SID_FILE" && -s "$BOOT_ID_FILE" ]] || return 1
  local sid stored_boot_id
  sid="$(cat "$SID_FILE")"
  stored_boot_id="$(cat "$BOOT_ID_FILE")"
  [[ "$sid" =~ ^[0-9]+$ ]] || return 1
  [[ "$stored_boot_id" == "$(current_boot_id)" ]] || return 1
  [[ -n "$(managed_session_pids "$sid")" ]]
}

signal_managed_session() {
  local sid="$1"
  local signal="$2"
  local -a pids=()
  mapfile -t pids < <(managed_session_pids "$sid")
  ((${#pids[@]} > 0)) || return 0
  kill -s "$signal" -- "${pids[@]}" 2>/dev/null || true
}

remove_managed_state() {
  rm -f "$PID_FILE" "$SID_FILE" "$BOOT_ID_FILE"
}

terminate_managed_session() {
  local sid="$1"
  local quiet="${2:-false}"

  if [[ -z "$(managed_session_pids "$sid")" ]]; then
    return 0
  fi

  [[ "$quiet" == "true" ]] || log "正在停止上一轮 Unity 受管仿真（会话 $sid）..."
  signal_managed_session "$sid" INT
  for _ in {1..50}; do
    [[ -z "$(managed_session_pids "$sid")" ]] && return 0
    sleep 0.2
  done

  [[ "$quiet" == "true" ]] || log "上一轮仿真仍有子进程，发送 TERM。"
  signal_managed_session "$sid" TERM
  for _ in {1..25}; do
    [[ -z "$(managed_session_pids "$sid")" ]] && return 0
    sleep 0.2
  done

  [[ "$quiet" == "true" ]] || log "上一轮仿真仍未退出，发送 KILL。"
  signal_managed_session "$sid" KILL
  for _ in {1..10}; do
    [[ -z "$(managed_session_pids "$sid")" ]] && return 0
    sleep 0.1
  done

  return 1
}

stop_managed_simulation() {
  if ! managed_session_is_running; then
    remove_managed_state
    log "没有正在运行的 Unity 受管仿真。"
    return 0
  fi

  local sid
  sid="$(cat "$SID_FILE")"
  terminate_managed_session "$sid"
  remove_managed_state
}

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: another Unity-to-ROS synchronization is already running." >&2
  exit 3
fi

if [[ "$STOP_ONLY" == "true" ]]; then
  stop_managed_simulation
  set_status "stopped"
  exit 0
fi

if [[ -z "$EXPORT_DIR" ]]; then
  echo "ERROR: --export-dir is required." >&2
  usage >&2
  exit 2
fi

WORKSPACE="$(realpath "$WORKSPACE")"
EXPORT_DIR="$(realpath "$EXPORT_DIR")"
CONFIG_JSON="$EXPORT_DIR/vehicle_config.json"
CONFIG_MESH="$EXPORT_DIR/vehicle_visual.obj"
TOOLS_SOURCE="$WORKSPACE/src/agribot_vehicle_config_tools"
DESCRIPTION_SOURCE="$WORKSPACE/src/agribot_vehicle_description"
CANONICAL_CONFIG="$DESCRIPTION_SOURCE/config/vehicles/ackermann_current/vehicle_config.json"
GENERATED_OUTPUT="$DESCRIPTION_SOURCE/generated/ackermann_current"

for required in \
  /opt/ros/humble/setup.bash \
  /usr/bin/cmake \
  "$CONFIG_JSON" \
  "$CONFIG_MESH" \
  "$TOOLS_SOURCE/agribot_vehicle_config_tools/cli.py" \
  "$DESCRIPTION_SOURCE/launch/configured_ackermann_sim.launch.py"; do
  if [[ ! -e "$required" ]]; then
    echo "ERROR: required file is missing: $required" >&2
    set_status "failed: missing $required"
    exit 4
  fi
done

set_status "preparing"
log "Unity 导出目录：$EXPORT_DIR"
log "ROS 工作区：$WORKSPACE"
log "仿真算法：$ALGORITHM（几何感知=$PERCEPTION_MODE，视觉感知=$VISION_MODE，定位=$LOCALIZATION_MODE，规划=$PLANNER_MODE，控制=$CONTROLLER_MODE，路径=$ROUTE_MODE）"

set +u
source /opt/ros/humble/setup.bash
set -u
cd "$WORKSPACE"

log "校验 Unity 配置并生成 ROS 文件..."
PYTHONPATH="$TOOLS_SOURCE${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m agribot_vehicle_config_tools.cli import-unity \
    "$EXPORT_DIR" \
    --canonical "$CANONICAL_CONFIG" \
    --workspace-src "$WORKSPACE/src" \
    --output "$GENERATED_OUTPUT"

log "增量编译车辆配置工具和描述包..."
export PATH="/usr/bin:/bin:$PATH"
hash -r
log "编译使用 CMake：$(command -v cmake)"
set +e
colcon build \
  --symlink-install \
  --parallel-workers "${AGRIBOT_BUILD_WORKERS:-4}" \
  --packages-select agribot_vehicle_config_tools agribot_visual_perception agribot_vehicle_description
BUILD_RESULT=$?
set -e
if ((BUILD_RESULT != 0)); then
  set_status "failed: colcon build ($BUILD_RESULT)"
  log "ERROR: ROS 增量编译失败（状态 $BUILD_RESULT）。"
  exit "$BUILD_RESULT"
fi

set +u
source "$WORKSPACE/install/setup.bash"
set -u
ros2 pkg prefix agribot_vehicle_description >/dev/null
ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  --show-args >/dev/null

if [[ "$PREPARE_ONLY" == "true" ]]; then
  set_status "prepared"
  log "配置导入和编译完成；按要求未启动仿真。"
  exit 0
fi

# The old simulation is stopped only after validation and build succeed.
stop_managed_simulation

export ROS_DOMAIN_ID="${AGRIBOT_SIM_ROS_DOMAIN_ID:-37}"
export ROS_LOCALHOST_ONLY="${AGRIBOT_SIM_ROS_LOCALHOST_ONLY:-1}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

set_status "starting"
log "启动 Gazebo 和 RViz（ROS_DOMAIN_ID=$ROS_DOMAIN_ID）..."
setsid ros2 launch agribot_vehicle_description configured_ackermann_sim.launch.py \
  gui:="$GUI" \
  rviz:="$RVIZ" \
  run_waypoints:="$RUN_WAYPOINTS" \
  perception_mode:="$PERCEPTION_MODE" \
  vision_mode:="$VISION_MODE" \
  localization_mode:="$LOCALIZATION_MODE" \
  planner_mode:="$PLANNER_MODE" \
  controller_mode:="$CONTROLLER_MODE" \
  route_mode:="$ROUTE_MODE" 9>&- &
SIM_PID=$!
SIM_SID=""
for _ in {1..20}; do
  SIM_SID="$(ps -o sid= -p "$SIM_PID" 2>/dev/null | tr -d '[:space:]')"
  [[ "$SIM_SID" =~ ^[0-9]+$ ]] && break
  sleep 0.05
done
if [[ ! "$SIM_SID" =~ ^[0-9]+$ ]]; then
  log "ERROR: 无法读取仿真会话 ID。"
  kill -TERM "$SIM_PID" 2>/dev/null || true
  wait "$SIM_PID" 2>/dev/null || true
  set_status "failed: no simulation session"
  exit 5
fi
printf '%s\n' "$SIM_PID" >"$PID_FILE"
printf '%s\n' "$SIM_SID" >"$SID_FILE"
current_boot_id >"$BOOT_ID_FILE"
set_status "running: $SIM_PID algorithm=$ALGORITHM"
log "仿真已启动（PID $SIM_PID，会话 $SIM_SID，算法 $ALGORITHM）。再次从 Unity 导出时会自动切换到新配置。"
flock -u 9
exec 9>&-

cleanup_managed_session() {
  exec 8>"$LOCK_FILE"
  flock 8
  if [[ -s "$SID_FILE" && "$(cat "$SID_FILE")" == "$SIM_SID" ]]; then
    terminate_managed_session "$SIM_SID" true || true
    if [[ -s "$SID_FILE" && "$(cat "$SID_FILE")" == "$SIM_SID" ]]; then
      remove_managed_state
    fi
  fi
  flock -u 8
  exec 8>&-
}

forward_signal() {
  signal_managed_session "$SIM_SID" INT
}

trap cleanup_managed_session EXIT
trap forward_signal HUP INT TERM
set +e
wait "$SIM_PID"
result=$?
set -e
set_status "stopped: $result"
log "仿真进程已退出（状态 $result）。"
exit "$result"
