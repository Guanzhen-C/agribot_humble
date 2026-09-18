#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE="${AGRIBOT_ROS_WORKSPACE:-$HOME/agribot_ws/ros2_ws}"
EXPORT_DIR=""
GUI="${AGRIBOT_SIM_GUI:-true}"
RVIZ="${AGRIBOT_SIM_RVIZ:-true}"
RUN_WAYPOINTS="${AGRIBOT_SIM_RUN_WAYPOINTS:-true}"
ALGORITHM="${AGRIBOT_SIM_ALGORITHM:-fastlivo_rtk_mppi}"
PREPARE_ONLY=false
STOP_ONLY=false
LIST_ALGORITHMS=false

list_algorithms() {
  cat <<'EOF'
fastlivo_rtk_mppi|FAST-LIVO2 + RTK / Smac Hybrid-A* / MPPI|真机同构融合定位方案（默认）
fastlio_mppi|FAST-LIO2 / Smac Hybrid-A* / MPPI|激光雷达与IMU定位，MPPI控制
navsat_mppi|NavSat ESKF / Smac Hybrid-A* / MPPI|RTK与IMU融合定位，MPPI控制
fastlivo_rtk_rpp|FAST-LIVO2 + RTK / Smac Hybrid-A* / RPP|融合定位，纯跟踪控制器对照
fastlio_rpp|FAST-LIO2 / Smac Hybrid-A* / RPP|激光雷达与IMU定位，RPP控制
navsat_rpp|NavSat ESKF / Smac Hybrid-A* / RPP|RTK与IMU融合定位，RPP控制
kiss_icp_mppi|KISS-ICP / Smac Hybrid-A* / MPPI|轻量级纯激光里程计对照
fastlivo_rtk_navfn_mppi|FAST-LIVO2 + RTK / NavFn / MPPI|经典栅格Dijkstra规划对照
fastlivo_rtk_theta_mppi|FAST-LIVO2 + RTK / Theta* / MPPI|任意角栅格规划对照
fastlivo_rtk_smac2d_mppi|FAST-LIVO2 + RTK / Smac 2D / MPPI|二维A*规划对照
fastlivo_rtk_navfn_dwb|FAST-LIVO2 + RTK / NavFn / DWB|经典Nav2规划控制组合
fastlivo_rtk_direct_mppi|FAST-LIVO2 + RTK / 必经点直连 / MPPI|不调用全局规划器的直线路径基线
EOF
}

resolve_algorithm() {
  PLANNER_MODE=smac_hybrid
  ROUTE_MODE=planned
  case "$ALGORITHM" in
    fastlivo_rtk_mppi)
      LOCALIZATION_MODE=fastlivo_rtk
      CONTROLLER_MODE=mppi
      ;;
    fastlio_mppi)
      LOCALIZATION_MODE=fast_lio
      CONTROLLER_MODE=mppi
      ;;
    navsat_mppi)
      LOCALIZATION_MODE=navsat
      CONTROLLER_MODE=mppi
      ;;
    fastlivo_rtk_rpp)
      LOCALIZATION_MODE=fastlivo_rtk
      CONTROLLER_MODE=rpp
      ;;
    fastlio_rpp)
      LOCALIZATION_MODE=fast_lio
      CONTROLLER_MODE=rpp
      ;;
    navsat_rpp)
      LOCALIZATION_MODE=navsat
      CONTROLLER_MODE=rpp
      ;;
    kiss_icp_mppi)
      LOCALIZATION_MODE=kiss_icp
      CONTROLLER_MODE=mppi
      ;;
    fastlivo_rtk_navfn_mppi)
      LOCALIZATION_MODE=fastlivo_rtk
      CONTROLLER_MODE=mppi
      PLANNER_MODE=navfn
      ;;
    fastlivo_rtk_theta_mppi)
      LOCALIZATION_MODE=fastlivo_rtk
      CONTROLLER_MODE=mppi
      PLANNER_MODE=theta_star
      ;;
    fastlivo_rtk_smac2d_mppi)
      LOCALIZATION_MODE=fastlivo_rtk
      CONTROLLER_MODE=mppi
      PLANNER_MODE=smac_2d
      ;;
    fastlivo_rtk_navfn_dwb)
      LOCALIZATION_MODE=fastlivo_rtk
      CONTROLLER_MODE=dwb
      PLANNER_MODE=navfn
      ;;
    fastlivo_rtk_direct_mppi)
      LOCALIZATION_MODE=fastlivo_rtk
      CONTROLLER_MODE=mppi
      ROUTE_MODE=direct
      ;;
    *)
      echo "ERROR: unknown simulation algorithm: $ALGORITHM" >&2
      echo "Available algorithms:" >&2
      list_algorithms >&2
      exit 2
      ;;
  esac
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
  --algorithm ID         Select a validated localization/controller profile
  --list-algorithms      Print available profile IDs and descriptions
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
      shift 2
      ;;
    --list-algorithms)
      LIST_ALGORITHMS=true
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

if [[ "$LIST_ALGORITHMS" == "true" ]]; then
  list_algorithms
  exit 0
fi

resolve_algorithm

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
log "仿真算法：$ALGORITHM（定位=$LOCALIZATION_MODE，规划=$PLANNER_MODE，控制=$CONTROLLER_MODE，路径=$ROUTE_MODE）"

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
  --packages-select agribot_vehicle_config_tools agribot_vehicle_description
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
